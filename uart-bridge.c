// SPDX-License-Identifier: MIT
/*
 * Copyright 2021 Álvaro Fernández Rojas <noltari@gmail.com>
 *
 * microDOS development additions:
 * - CDC 0 remains the Raspberry Pi UART console bridge on GP16/GP17.
 * - CDC 1 is a control channel.
 * - Sending 'R' or 'r' on CDC 1 pulses GP2 high for 100 ms.
 * - GP2 drives an external transistor that pulls Raspberry Pi RUN to GND.
 */

#include <hardware/irq.h>
#include <hardware/structs/sio.h>
#include <hardware/uart.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <stdio.h>
#include <string.h>
#include <tusb.h>

#if !defined(MIN)
#define MIN(a, b) ((a > b) ? b : a)
#endif

#define LED_PIN 25

#define BUFFER_SIZE 2560

#define DEF_BIT_RATE 115200
#define DEF_STOP_BITS 1
#define DEF_PARITY 0
#define DEF_DATA_BITS 8

#define CONSOLE_ITF 0
#define CONTROL_ITF 1

/* Pico GPIO connected through a transistor to the Pi Zero 2 W RUN pad.
 *
 * GPIO LOW  -> transistor off  -> RUN released
 * GPIO HIGH -> transistor on   -> RUN pulled to GND
 */
#define PI_RUN_RESET_PIN 2
#define PI_RUN_RESET_PULSE_US 100000ull

typedef struct {
	uart_inst_t *const inst;
	uint irq;
	void *irq_fn;
	uint8_t tx_pin;
	uint8_t rx_pin;
} uart_id_t;

typedef struct {
	cdc_line_coding_t usb_lc;
	cdc_line_coding_t uart_lc;
	mutex_t lc_mtx;
	uint8_t uart_buffer[BUFFER_SIZE];
	uint32_t uart_pos;
	mutex_t uart_mtx;
	uint8_t usb_buffer[BUFFER_SIZE];
	uint32_t usb_pos;
	mutex_t usb_mtx;
} uart_data_t;

void uart0_irq_fn(void);
void uart1_irq_fn(void);

const uart_id_t UART_ID[CFG_TUD_CDC] = {
	{
		.inst = uart0,
		.irq = UART0_IRQ,
		.irq_fn = &uart0_irq_fn,
		.tx_pin = 16,
		.rx_pin = 17,
	}, {
		.inst = uart1,
		.irq = UART1_IRQ,
		.irq_fn = &uart1_irq_fn,
		.tx_pin = 4,
		.rx_pin = 5,
	}
};

uart_data_t UART_DATA[CFG_TUD_CDC];

typedef struct {
	volatile uint32_t rx_bytes;
	volatile uint32_t rx_dropped;
	volatile uint32_t rx_polls;
	volatile uint32_t actual_baud;
	volatile uint8_t last_byte;
} uart_diag_t;

static uart_diag_t UART_DIAG[CFG_TUD_CDC];

static volatile bool pi_reset_active;
static volatile uint64_t pi_reset_release_at_us;
static bool control_announced;

static inline uint databits_usb2uart(uint8_t data_bits)
{
	switch (data_bits) {
		case 5:
			return 5;
		case 6:
			return 6;
		case 7:
			return 7;
		default:
			return 8;
	}
}

static inline uart_parity_t parity_usb2uart(uint8_t usb_parity)
{
	switch (usb_parity) {
		case 1:
			return UART_PARITY_ODD;
		case 2:
			return UART_PARITY_EVEN;
		default:
			return UART_PARITY_NONE;
	}
}

static inline uint stopbits_usb2uart(uint8_t stop_bits)
{
	switch (stop_bits) {
		case 2:
			return 2;
		default:
			return 1;
	}
}

static void control_write(const char *text)
{
	const size_t len = strlen(text);

	if (!tud_cdc_n_connected(CONTROL_ITF))
		return;

	(void)tud_cdc_n_write(CONTROL_ITF, text, (uint32_t)len);
	tud_cdc_n_write_flush(CONTROL_ITF);
}

static void pi_reset_begin(void)
{
	/*
	 * Never drive the Pi RUN signal itself high.
	 * GP2 only drives the external transistor.
	 */
	gpio_put(PI_RUN_RESET_PIN, 1);
	pi_reset_release_at_us = time_us_64() + PI_RUN_RESET_PULSE_US;
	pi_reset_active = true;

	control_write("RESET: Pi RUN asserted\r\n");
}

static void pi_reset_poll(void)
{
	if (!pi_reset_active)
		return;

	if ((int64_t)(time_us_64() - pi_reset_release_at_us) >= 0) {
		gpio_put(PI_RUN_RESET_PIN, 0);
		pi_reset_active = false;
		control_write("RESET: Pi RUN released\r\n");
	}
}

static void control_diag(void)
{
	char text[256];
	uart_data_t *ud = &UART_DATA[CONSOLE_ITF];
	const uart_id_t *ui = &UART_ID[CONSOLE_ITF];
	uart_diag_t *dg = &UART_DIAG[CONSOLE_ITF];
	uint32_t buffered;
	uint32_t bit_rate;
	uint8_t data_bits;
	uint8_t parity;
	uint8_t stop_bits;

	mutex_enter_blocking(&ud->uart_mtx);
	buffered = ud->uart_pos;
	mutex_exit(&ud->uart_mtx);

	mutex_enter_blocking(&ud->lc_mtx);
	bit_rate = ud->uart_lc.bit_rate;
	data_bits = ud->uart_lc.data_bits;
	parity = ud->uart_lc.parity;
	stop_bits = ud->uart_lc.stop_bits;
	mutex_exit(&ud->lc_mtx);

	snprintf(text, sizeof(text),
		 "UART0 diag: GP17=%u readable=%u cfg=%lu/%u/%u/%u "
		 "actual=%lu rx=%lu drop=%lu buf=%lu polls=%lu last=%02X\r\n",
		 gpio_get(ui->rx_pin) ? 1u : 0u,
		 uart_is_readable(ui->inst) ? 1u : 0u,
		 (unsigned long)bit_rate,
		 (unsigned)data_bits,
		 (unsigned)parity,
		 (unsigned)stop_bits,
		 (unsigned long)dg->actual_baud,
		 (unsigned long)dg->rx_bytes,
		 (unsigned long)dg->rx_dropped,
		 (unsigned long)buffered,
		 (unsigned long)dg->rx_polls,
		 (unsigned)dg->last_byte);

	control_write(text);
}

static void control_diag_clear(void)
{
	uart_diag_t *dg = &UART_DIAG[CONSOLE_ITF];

	dg->rx_bytes = 0;
	dg->rx_dropped = 0;
	dg->rx_polls = 0;
	dg->last_byte = 0;

	control_write("UART0 diagnostics cleared\r\n");
}

static void control_process(void)
{
	uint8_t buffer[64];
	uint32_t available;

	if (!tud_cdc_n_connected(CONTROL_ITF)) {
		control_announced = false;
		return;
	}

	if (!control_announced) {
		control_write(
			"pico-uart-bridge microDOS control\r\n"
			"  R = hard reset Raspberry Pi RUN\r\n"
			"  D = show UART0 RX diagnostics\r\n"
			"  C = clear UART0 RX diagnostics\r\n"
			"  ? = show this help\r\n");
		control_announced = true;
	}

	available = tud_cdc_n_available(CONTROL_ITF);
	while (available != 0u) {
		uint32_t take = MIN(available, (uint32_t)sizeof(buffer));
		uint32_t got = tud_cdc_n_read(CONTROL_ITF, buffer, take);
		uint32_t i;

		for (i = 0; i < got; ++i) {
			switch (buffer[i]) {
				case 'R':
				case 'r':
					if (!pi_reset_active)
						pi_reset_begin();
					break;

				case 'D':
				case 'd':
					control_diag();
					break;

				case 'C':
				case 'c':
					control_diag_clear();
					break;

				case '?':
					control_write(
						"microDOS Pi control\r\n"
						"  R = hard reset Raspberry Pi RUN\r\n"
						"  D = show UART0 RX diagnostics\r\n"
						"  C = clear UART0 RX diagnostics\r\n");
					break;

				default:
					/* Ignore CR/LF and unknown bytes. */
					break;
			}
		}

		available = tud_cdc_n_available(CONTROL_ITF);
	}

	pi_reset_poll();
}

void update_uart_cfg(uint8_t itf)
{
	const uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

	mutex_enter_blocking(&ud->lc_mtx);

	if (ud->usb_lc.bit_rate != ud->uart_lc.bit_rate) {
		UART_DIAG[itf].actual_baud =
			uart_set_baudrate(ui->inst, ud->usb_lc.bit_rate);
		ud->uart_lc.bit_rate = ud->usb_lc.bit_rate;
	}

	if ((ud->usb_lc.stop_bits != ud->uart_lc.stop_bits) ||
	    (ud->usb_lc.parity != ud->uart_lc.parity) ||
	    (ud->usb_lc.data_bits != ud->uart_lc.data_bits)) {
		uart_set_format(ui->inst,
				databits_usb2uart(ud->usb_lc.data_bits),
				stopbits_usb2uart(ud->usb_lc.stop_bits),
				parity_usb2uart(ud->usb_lc.parity));
		ud->uart_lc.data_bits = ud->usb_lc.data_bits;
		ud->uart_lc.parity = ud->usb_lc.parity;
		ud->uart_lc.stop_bits = ud->usb_lc.stop_bits;
	}

	mutex_exit(&ud->lc_mtx);
}

void usb_read_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];
	uint32_t len = tud_cdc_n_available(itf);

	if (len &&
	    mutex_try_enter(&ud->usb_mtx, NULL)) {
		len = MIN(len, BUFFER_SIZE - ud->usb_pos);
		if (len) {
			uint32_t count;

			count = tud_cdc_n_read(itf, &ud->usb_buffer[ud->usb_pos], len);
			ud->usb_pos += count;
		}

		mutex_exit(&ud->usb_mtx);
	}
}

void usb_write_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->uart_pos &&
	    mutex_try_enter(&ud->uart_mtx, NULL)) {
		uint32_t count;

		count = tud_cdc_n_write(itf, ud->uart_buffer, ud->uart_pos);
		if (count < ud->uart_pos)
			memmove(ud->uart_buffer, &ud->uart_buffer[count],
			       ud->uart_pos - count);
		ud->uart_pos -= count;

		mutex_exit(&ud->uart_mtx);

		if (count)
			tud_cdc_n_write_flush(itf);
	}
}

void usb_cdc_process(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];

	mutex_enter_blocking(&ud->lc_mtx);
	tud_cdc_n_get_line_coding(itf, &ud->usb_lc);
	mutex_exit(&ud->lc_mtx);

	usb_read_bytes(itf);
	usb_write_bytes(itf);
}

void core1_entry(void)
{
	tusb_init();

	while (1) {
		int con = 0;

		tud_task();

		if (tud_cdc_n_connected(CONSOLE_ITF)) {
			con = 1;
			usb_cdc_process(CONSOLE_ITF);
		}

		if (tud_cdc_n_connected(CONTROL_ITF)) {
			con = 1;
			control_process();
		} else {
			control_announced = false;
			pi_reset_poll();
		}

		gpio_put(LED_PIN, con);
	}
}

static inline void uart_read_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];
	const uart_id_t *ui = &UART_ID[itf];
	uart_diag_t *dg = &UART_DIAG[itf];

	dg->rx_polls++;

	if (uart_is_readable(ui->inst)) {
		mutex_enter_blocking(&ud->uart_mtx);

		/*
		 * Always drain the hardware UART, even if the software buffer is
		 * temporarily full.  The old IRQ path stopped reading when uart_pos
		 * reached BUFFER_SIZE, leaving RX pending and allowing the interrupt
		 * to remain asserted indefinitely.
		 */
		while (uart_is_readable(ui->inst)) {
			uint8_t ch = (uint8_t)uart_getc(ui->inst);

			dg->rx_bytes++;
			dg->last_byte = ch;

			if (ud->uart_pos < BUFFER_SIZE) {
				ud->uart_buffer[ud->uart_pos] = ch;
				ud->uart_pos++;
			} else {
				dg->rx_dropped++;
			}
		}

		mutex_exit(&ud->uart_mtx);
	}
}

void uart0_irq_fn(void)
{
	uart_read_bytes(0);
}

void uart1_irq_fn(void)
{
	uart_read_bytes(1);
}

void uart_write_bytes(uint8_t itf)
{
	uart_data_t *ud = &UART_DATA[itf];

	if (ud->usb_pos &&
	    mutex_try_enter(&ud->usb_mtx, NULL)) {
		const uart_id_t *ui = &UART_ID[itf];
		uint32_t count = 0;

		while (uart_is_writable(ui->inst) &&
		       count < ud->usb_pos) {
			uart_putc_raw(ui->inst, ud->usb_buffer[count]);
			count++;
		}

		if (count < ud->usb_pos)
			memmove(ud->usb_buffer, &ud->usb_buffer[count],
			       ud->usb_pos - count);
		ud->usb_pos -= count;

		mutex_exit(&ud->usb_mtx);
	}
}

void init_uart_data(uint8_t itf)
{
	const uart_id_t *ui = &UART_ID[itf];
	uart_data_t *ud = &UART_DATA[itf];

	/* Pinmux */
	gpio_set_function(ui->tx_pin, GPIO_FUNC_UART);
	gpio_set_function(ui->rx_pin, GPIO_FUNC_UART);

	/* USB CDC LC */
	ud->usb_lc.bit_rate = DEF_BIT_RATE;
	ud->usb_lc.data_bits = DEF_DATA_BITS;
	ud->usb_lc.parity = DEF_PARITY;
	ud->usb_lc.stop_bits = DEF_STOP_BITS;

	/* UART LC */
	ud->uart_lc.bit_rate = DEF_BIT_RATE;
	ud->uart_lc.data_bits = DEF_DATA_BITS;
	ud->uart_lc.parity = DEF_PARITY;
	ud->uart_lc.stop_bits = DEF_STOP_BITS;

	/* Buffer */
	ud->uart_pos = 0;
	ud->usb_pos = 0;

	/* Mutex */
	mutex_init(&ud->lc_mtx);
	mutex_init(&ud->uart_mtx);
	mutex_init(&ud->usb_mtx);

	/* UART start */
	UART_DIAG[itf].actual_baud =
		uart_init(ui->inst, ud->usb_lc.bit_rate);
	uart_set_hw_flow(ui->inst, false, false);
	uart_set_format(ui->inst, databits_usb2uart(ud->usb_lc.data_bits),
			stopbits_usb2uart(ud->usb_lc.stop_bits),
			parity_usb2uart(ud->usb_lc.parity));

	/*
	 * Use the hardware FIFO and poll RX from core 0.
	 *
	 * At 115200 baud this is extremely light work for the RP2040 and avoids
	 * the previous RX interrupt/full-buffer failure mode.  CDC/TinyUSB still
	 * runs independently on core 1.
	 */
	uart_set_fifo_enabled(ui->inst, true);
	uart_set_irq_enables(ui->inst, false, false);
	irq_set_enabled(ui->irq, false);
}

int main(void)
{
	usbd_serial_init();

	/*
	 * CDC 0 is the only UART bridge now.
	 * CDC 1 is intentionally reserved for Pi reset/control commands.
	 */
	init_uart_data(CONSOLE_ITF);

	/*
	 * Safe startup state for the external reset transistor.
	 * Set output value before enabling output to avoid an accidental pulse.
	 */
	gpio_init(PI_RUN_RESET_PIN);
	gpio_put(PI_RUN_RESET_PIN, 0);
	gpio_set_dir(PI_RUN_RESET_PIN, GPIO_OUT);

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	multicore_launch_core1(core1_entry);

	while (1) {
		update_uart_cfg(CONSOLE_ITF);
		uart_read_bytes(CONSOLE_ITF);
		uart_write_bytes(CONSOLE_ITF);
		tight_loop_contents();
	}

	return 0;
}
