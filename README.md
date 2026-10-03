# Raspberry Pi Pico microDOS USB-UART + Pi Reset Bridge

This is a microDOS-focused fork of the Raspberry Pi Pico USB-UART bridge.

The original project exposed the Pico hardware UARTs as USB CDC serial devices. This fork changes that design for Raspberry Pi Zero 2 W bare-metal development:

- **CDC 0** is the Raspberry Pi UART console bridge.
- **CDC 1** is a microDOS control channel.
- **GPIO16 / GPIO17** provide the UART connection to the Raspberry Pi.
- **GPIO2** drives an external transistor that can pulse the Raspberry Pi **RUN** pad for reliable automated resets.
- The bridge is intended to work with the microDOS Raspberry Pi Zero 2 W USB-boot/run workflow.

This makes one Pico act as both:

1. a USB-to-UART console adapter for the Raspberry Pi, and
2. a USB-controlled hardware reset controller.

---

## Disclaimer

This software is provided without warranty under the MIT License.

Do not use it in applications where failure could endanger life, cause financial loss, or create other safety-critical consequences.

---

# USB interfaces

The Pico enumerates as two USB CDC serial devices.

| USB CDC interface | Purpose |
|---|---|
| CDC 0 | Raspberry Pi UART console |
| CDC 1 | Raspberry Pi reset/control channel |

On Windows these normally appear as two separate COM ports.

For the current microDOS development setup:

```text
COM3 = Pi UART console
COM8 = Pi reset/control
```

The actual COM numbers are assigned by Windows and may differ on another computer.

---

# Raspberry Pi Pico pinout

## microDOS fork pin usage

| Raspberry Pi Pico GPIO | Physical pin | Function |
|:---:|:---:|---|
| GPIO16 | Pin 21 | UART0 TX -> Raspberry Pi GPIO15 / RX |
| GPIO17 | Pin 22 | UART0 RX <- Raspberry Pi GPIO14 / TX |
| GPIO2 | Pin 4 | Raspberry Pi RUN reset transistor drive |
| GND | Any GND pin | Shared ground with Raspberry Pi |

The original second UART on GPIO4/GPIO5 is **not used as a second USB-UART bridge in the microDOS configuration**. CDC 1 is reserved for control/reset commands.

---

# Raspberry Pi Zero 2 W connections

The Raspberry Pi UART uses:

| Raspberry Pi Zero 2 W | Physical pin | Function |
|---|:---:|---|
| GPIO14 | Pin 8 | UART TX |
| GPIO15 | Pin 10 | UART RX |
| GND | Pin 6 or another GND | Ground |
| RUN pad | Board test/reset pad | Hardware reset input |

UART is crossed in the normal way:

```text
Raspberry Pi Zero 2 W                 Raspberry Pi Pico
---------------------------------------------------------------
GPIO14 / Pin 8   TX  ------------->   GPIO17 / Pin 22   RX
GPIO15 / Pin 10  RX  <-------------   GPIO16 / Pin 21   TX
GND              ------------------   GND
```

Both boards use **3.3 V UART logic**. No level shifter is required.

Do **not** connect either UART signal to 5 V.

---

# Raspberry Pi RUN reset circuit

The Raspberry Pi RUN pad must **not** be driven directly high by the Pico.

This fork uses an external **2N3904 NPN transistor** so the Pico only controls a transistor that momentarily pulls the Raspberry Pi RUN pad to ground.

## Required parts

- 1 x **2N3904** NPN transistor
- 1 x **4.7 kΩ** resistor
- hookup wire

## Wiring

```text
Pico GPIO2
    |
    |
   4.7k
    |
    v
  BASE
   |
  2N3904
   |
   +---- COLLECTOR ---- Raspberry Pi RUN pad
   |
 EMITTER
   |
  GND ---------------- Raspberry Pi GND
```

Equivalent connection table:

| Connection | Goes to |
|---|---|
| Pico GPIO2 | 4.7 kΩ resistor |
| Other side of 4.7 kΩ resistor | 2N3904 base |
| 2N3904 collector | Raspberry Pi RUN pad |
| 2N3904 emitter | Ground |
| Pico ground | Raspberry Pi ground |

The firmware uses the following logic:

```text
GPIO2 LOW  -> transistor off -> RUN released
GPIO2 HIGH -> transistor on  -> RUN pulled to ground
```

A reset command pulses GPIO2 high for approximately **100 ms**, then returns it low.

The transistor isolates the Pico from the Raspberry Pi RUN signal and prevents the Pico from actively driving RUN high.

---

# Control channel

CDC 1 is the microDOS control interface.

Current commands:

| Command | Function |
|:---:|---|
| `R` or `r` | Pulse Raspberry Pi RUN and perform a hardware reset |
| `?` | Show control help |

The diagnostic bridge build may also provide:

| Command | Function |
|:---:|---|
| `D` or `d` | Show UART0 RX diagnostics |
| `C` or `c` | Clear UART0 RX diagnostic counters |

Example with Python / pyserial:

```python
import serial

s = serial.Serial("COM8", 115200, timeout=1)
s.write(b"R")
s.flush()
s.close()
```

Expected response:

```text
RESET: Pi RUN asserted
RESET: Pi RUN released
```

---

# UART configuration

The default UART configuration is:

```text
115200 baud
8 data bits
no parity
1 stop bit
no hardware flow control
```

CDC 0 forwards data between USB and Pico UART0.

```text
USB CDC 0
    |
    v
Pico UART0
    |
    +-- GPIO16 TX
    |
    +-- GPIO17 RX
    |
    v
Raspberry Pi GPIO15 / GPIO14
```

The current microDOS fork uses the Pico UART FIFO and a polling RX path on core 0. TinyUSB runs on core 1.

This avoids the previous interrupt-driven RX failure mode that could occur during sustained Raspberry Pi UART output.

---

# Building

## Prerequisites

- CMake
- Ninja
- ARM GCC toolchain (`arm-none-eabi-gcc`)
- Python 3
- Raspberry Pi Pico SDK
- picotool

Initialize submodules first:

```bash
git submodule update --init --recursive
```

---

## Linux / macOS build

The original build script can be used:

```bash
./build.sh
```

The default target is Raspberry Pi Pico.

The UF2 is produced at:

```text
build/uart_bridge.uf2
```

For Pico 2:

```bash
PICO_BOARD=pico2 ./build.sh
```

---

## Windows PowerShell build

Example configuration used by the microDOS development environment:

```powershell
cd C:\pico-uart-bridge

Remove-Item -Recurse -Force .\build -ErrorAction SilentlyContinue

Remove-Item Env:INCLUDE -ErrorAction SilentlyContinue
Remove-Item Env:LIB -ErrorAction SilentlyContinue
Remove-Item Env:LIBPATH -ErrorAction SilentlyContinue

cmake -S . -B build -G Ninja `
    -DCMAKE_MAKE_PROGRAM="C:\Users\Jupiter\.pico-sdk\ninja\v1.12.1\ninja.exe" `
    -DPICO_BOARD=pico `
    -DPICO_TOOLCHAIN_PATH="C:\Users\Jupiter\.pico-sdk\toolchain\14_2_Rel1" `
    -Dpicotool_DIR="C:\Users\Jupiter\.pico-sdk\picotool\2.1.1\picotool"

cmake --build build
```

Output:

```text
C:\pico-uart-bridge\build\uart_bridge.uf2
```

For Pico 2, change:

```text
-DPICO_BOARD=pico
```

to:

```text
-DPICO_BOARD=pico2
```

---

# Flashing

Put the Pico into BOOTSEL mode:

1. Disconnect USB.
2. Hold **BOOTSEL**.
3. Connect USB.
4. Release **BOOTSEL**.
5. Copy `uart_bridge.uf2` to the mounted `RPI-RP2` drive.

The Pico will reboot and enumerate the two CDC serial interfaces.

---

# Testing the UART bridge

## Pico local loopback test

Disconnect the Raspberry Pi UART wires temporarily.

Connect:

```text
Pico GPIO16 -> Pico GPIO17
```

Then run:

```powershell
@'
import serial
import time

s = serial.Serial("COM3", 115200, timeout=1)

time.sleep(0.2)
s.reset_input_buffer()

test = b"microDOS-PICO-UART-LOOPBACK-1234567890\r\n"

print("TX:", test)

s.write(test)
s.flush()

time.sleep(0.2)
data = s.read(len(test) + 32)

print("RX:", data)
print("PASS" if test in data else "FAIL")

s.close()
'@ | python -
```

Expected result:

```text
PASS
```

Reconnect the Raspberry Pi UART after the test.

---

# Testing Raspberry Pi UART reception

With the Raspberry Pi continuously transmitting on GPIO14:

```powershell
@'
import serial
import time

s = serial.Serial("COM3", 115200, timeout=2)
s.reset_input_buffer()

time.sleep(0.5)
data = s.read(64)

print("bytes:", len(data))
print("hex:", data.hex(" "))
print("raw:", repr(data))

s.close()
'@ | python -
```

For the microDOS continuous `0x55` UART diagnostic, a successful result looks like:

```text
bytes: 64
hex: 55 55 55 55 55 55 ...
raw: b'UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU'
```

---

# microDOS Raspberry Pi Zero 2 W workflow

This bridge was created to support the microDOS Raspberry Pi Zero 2 W bare-metal development loop.

The Pico provides:

```text
CDC 0 / COM3
    Raspberry Pi UART console

CDC 1 / COM8
    Raspberry Pi RUN reset control
```

That allows the microDOS runner to perform:

```text
build
  ->
stage kernel8.img
  ->
reset Raspberry Pi through Pico GPIO2 + 2N3904
  ->
rpiboot USB boot
  ->
capture Raspberry Pi UART
```

No Linux installation is required on the Raspberry Pi target.

The Raspberry Pi can run microDOS directly as a bare-metal `kernel8.img`.

---

# Notes

- Raspberry Pi GPIO14 is **TX**.
- Raspberry Pi GPIO15 is **RX**.
- Pico GPIO17 is **RX**.
- Pico GPIO16 is **TX**.
- TX must connect to RX.
- RX must connect to TX.
- Pico and Raspberry Pi grounds must be connected.
- The Raspberry Pi UART is 3.3 V logic.
- The RUN pad is pulled low through the 2N3904 transistor; it is never driven high by the Pico.
- CDC 1 is reserved for control and should not be treated as a second UART bridge in the microDOS fork.

---

# License

MIT License.

See the repository license file for the complete license text.

---

# Credits

Based on the Raspberry Pi Pico USB-UART bridge project by Álvaro Fernández Rojas, with earlier Raspberry Pi Pico example work by Raspberry Pi Ltd. and Damien P. George.

microDOS fork additions include:

- dedicated Raspberry Pi UART console bridge,
- dedicated USB reset/control CDC interface,
- GPIO2 Raspberry Pi RUN reset control,
- 2N3904 reset transistor interface,
- sustained UART RX handling improvements,
- development diagnostics for Raspberry Pi bare-metal bring-up.
