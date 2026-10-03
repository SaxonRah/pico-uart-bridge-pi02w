// SPDX-License-Identifier: MIT
/*
 * microDOS Raspberry Pi Zero 2 W bridge - v28 HID transport
 *
 * This version abandons USB CDC entirely.
 *
 * USB:
 *   TinyUSB generic HID IN/OUT
 *   64-byte interrupt reports, 1 ms interval
 *
 * UART:
 *   UART0
 *   GP16 TX
 *   GP17 RX
 *   115200 8N1
 *
 * Pi reset:
 *   GP2 -> external transistor -> Pi RUN
 *
 * Host OUT commands:
 *   P = PONG
 *   S = STATUS
 *   C = clear stats/ring
 *   H = hold Pi reset
 *   L = release Pi reset
 *
 * Device IN frames, fixed 64 bytes:
 *
 *   byte 0 = 0x01 : UART data
 *   byte 1 = payload length 0..62
 *   byte 2.. = UART bytes
 *
 *   byte 0 = 0x02 : control response
 *   byte 1 = response code
 *
 * Control response codes:
 *   1 PONG
 *   2 CLEARED
 *   3 HELD
 *   4 RELEASED
 *   5 STATUS
 *
 * STATUS payload, little endian:
 *   +2  uint32 rx_bytes
 *   +6  uint32 ring_drops
 *   +10 uint32 queued
 *   +14 uint32 hid_reports_sent
 *   +18 uint32 hid_report_failures
 *   +22 uint32 actual_baud
 *   +26 uint8  GP17
 *   +27 uint8  uart_readable
 *   +28 uint8  last_byte
 *   +29 uint8  mounted
 */

#include <hardware/uart.h>
#include <pico/stdlib.h>
#include <stdint.h>
#include <string.h>

#include "tusb.h"

#define LED_PIN 25u

#define UART_ID uart0
#define UART_TX_PIN 16u
#define UART_RX_PIN 17u
#define UART_BAUD 115200u

#define PI_RUN_RESET_PIN 2u

#define RING_SIZE 8192u
#define HID_REPORT_SIZE 64u
#define HID_UART_PAYLOAD 62u

enum {
    RESP_PONG = 1,
    RESP_CLEARED = 2,
    RESP_HELD = 3,
    RESP_RELEASED = 4,
    RESP_STATUS = 5
};

typedef struct {
    uint8_t data[RING_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
} byte_ring_t;

static byte_ring_t ringbuf;

static uint32_t actual_baud;
static uint32_t rx_bytes;
static uint32_t ring_drops;
static uint32_t hid_reports_sent;
static uint32_t hid_report_failures;
static uint8_t last_byte;
static volatile bool mounted;

static uint8_t control_frame[HID_REPORT_SIZE];
static bool control_pending;

static void put_u32le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 0);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void ring_clear(void)
{
    ringbuf.head = 0u;
    ringbuf.tail = 0u;
    ringbuf.count = 0u;
}

static void ring_push(uint8_t ch)
{
    if (ringbuf.count == RING_SIZE) {
        ringbuf.tail = (ringbuf.tail + 1u) % RING_SIZE;
        ringbuf.count--;
        ring_drops++;
    }

    ringbuf.data[ringbuf.head] = ch;
    ringbuf.head = (ringbuf.head + 1u) % RING_SIZE;
    ringbuf.count++;
}

static uint32_t ring_peek_payload(uint8_t *dst, uint32_t max_count)
{
    uint32_t n = ringbuf.count;
    uint32_t pos = ringbuf.tail;

    if (n > max_count)
        n = max_count;

    for (uint32_t i = 0; i < n; ++i) {
        dst[i] = ringbuf.data[pos];
        pos = (pos + 1u) % RING_SIZE;
    }

    return n;
}

static void ring_consume(uint32_t n)
{
    if (n > ringbuf.count)
        n = ringbuf.count;

    ringbuf.tail = (ringbuf.tail + n) % RING_SIZE;
    ringbuf.count -= n;
}

static void discard_uart_fifo(void)
{
    while (uart_is_readable(UART_ID))
        (void)uart_getc(UART_ID);
}

static void clear_runtime_stats(void)
{
    ring_clear();
    discard_uart_fifo();

    rx_bytes = 0u;
    ring_drops = 0u;
    hid_reports_sent = 0u;
    hid_report_failures = 0u;
    last_byte = 0u;
}

static void queue_simple_response(uint8_t code)
{
    memset(control_frame, 0, sizeof(control_frame));
    control_frame[0] = 0x02;
    control_frame[1] = code;
    control_pending = true;
}

static void queue_status_response(void)
{
    memset(control_frame, 0, sizeof(control_frame));

    control_frame[0] = 0x02;
    control_frame[1] = RESP_STATUS;

    put_u32le(&control_frame[2], rx_bytes);
    put_u32le(&control_frame[6], ring_drops);
    put_u32le(&control_frame[10], ringbuf.count);
    put_u32le(&control_frame[14], hid_reports_sent);
    put_u32le(&control_frame[18], hid_report_failures);
    put_u32le(&control_frame[22], actual_baud);

    control_frame[26] = gpio_get(UART_RX_PIN) ? 1u : 0u;
    control_frame[27] = uart_is_readable(UART_ID) ? 1u : 0u;
    control_frame[28] = last_byte;
    control_frame[29] = mounted ? 1u : 0u;

    control_pending = true;
}

static void handle_command(uint8_t command)
{
    switch (command) {
        case 'P':
        case 'p':
            queue_simple_response(RESP_PONG);
            break;

        case 'S':
        case 's':
            queue_status_response();
            break;

        case 'C':
        case 'c':
            clear_runtime_stats();
            queue_simple_response(RESP_CLEARED);
            break;

        case 'H':
        case 'h':
            gpio_put(PI_RUN_RESET_PIN, 1);
            queue_simple_response(RESP_HELD);
            break;

        case 'L':
        case 'l':
            gpio_put(PI_RUN_RESET_PIN, 0);
            clear_runtime_stats();
            queue_simple_response(RESP_RELEASED);
            break;

        default:
            break;
    }
}

static void drain_uart(void)
{
    while (uart_is_readable(UART_ID)) {
        const uint8_t ch = (uint8_t)uart_getc(UART_ID);

        rx_bytes++;
        last_byte = ch;
        ring_push(ch);
    }
}

static void hid_tx_poll(void)
{
    if (!tud_hid_ready())
        return;

    if (control_pending) {
        if (tud_hid_report(0, control_frame, sizeof(control_frame))) {
            control_pending = false;
            hid_reports_sent++;
        }
        return;
    }

    if (ringbuf.count != 0u) {
        uint8_t report[HID_REPORT_SIZE];
        memset(report, 0, sizeof(report));

        report[0] = 0x01;

        const uint32_t n =
            ring_peek_payload(&report[2], HID_UART_PAYLOAD);

        report[1] = (uint8_t)n;

        if (tud_hid_report(0, report, sizeof(report))) {
            ring_consume(n);
            hid_reports_sent++;
        } else {
            /*
             * Keep the bytes queued and retry later. Unlike the old CDC
             * bridge, a failed USB submit cannot silently discard UART data.
             */
            hid_report_failures++;
        }
    }
}

int main(void)
{
    ring_clear();

    actual_baud = 0u;
    rx_bytes = 0u;
    ring_drops = 0u;
    hid_reports_sent = 0u;
    hid_report_failures = 0u;
    last_byte = 0u;
    mounted = false;
    control_pending = false;

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    gpio_init(PI_RUN_RESET_PIN);
    gpio_set_dir(PI_RUN_RESET_PIN, GPIO_OUT);
    gpio_put(PI_RUN_RESET_PIN, 0);

    actual_baud = uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    if (!tud_init(0)) {
        while (1) {
            gpio_put(LED_PIN, 1);
            sleep_ms(100);
            gpio_put(LED_PIN, 0);
            sleep_ms(100);
        }
    }

    while (1) {
        /*
         * Non-blocking TinyUSB service. Do not use tud_task() here because its
         * default API may wait for an event. UART service must remain prompt.
         */
        tud_task_ext(0, false);

        drain_uart();
        hid_tx_poll();

        gpio_put(LED_PIN, mounted ? 1 : 0);

        tight_loop_contents();
    }
}

void tud_mount_cb(void)
{
    mounted = true;
}

void tud_umount_cb(void)
{
    mounted = false;
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
}

void tud_resume_cb(void)
{
}

uint16_t tud_hid_get_report_cb(
    uint8_t instance,
    uint8_t report_id,
    hid_report_type_t report_type,
    uint8_t *buffer,
    uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(
    uint8_t instance,
    uint8_t report_id,
    hid_report_type_t report_type,
    uint8_t const *buffer,
    uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)report_type;

    if (bufsize == 0u)
        return;

    handle_command(buffer[0]);
}

void tud_hid_report_complete_cb(
    uint8_t instance,
    uint8_t const *report,
    uint16_t len)
{
    (void)instance;
    (void)report;
    (void)len;
}

void tud_hid_report_failed_cb(
    uint8_t instance,
    hid_report_type_t report_type,
    uint8_t const *report,
    uint16_t xferred_bytes)
{
    (void)instance;
    (void)report_type;
    (void)report;
    (void)xferred_bytes;
    hid_report_failures++;
}
