# RP2040-Zero USB-UART Bridge w ESP32 flashing support

This program uses a Waveshare RP2040-Zero as a USB->UART converter, also bringing out DTR and RTS pins to allow easy programming of ESP32 devices. This code is based heavily on https://github.com/Noltari/pico-uart-bridge, with various modifications and improvements rolled in.

Uses PIO to wiggle the onboard neopixel as data is read and written. UART reads use random values for Blue and Green (look for cool tones on the neopixel) and UART writes use random values for Red and Green (look for warm tones). The Neopixel will be red on initial boot, and purple when reset by the USB side.

# RP2040-Zero Pinout

| Raspberry Pi Pico GPIO | Function |
|:----------------------:|:--------:|
| GPIO4 (Pin 4)        | UART0 TX |
| GPIO5 (Pin 5)        | UART0 RX |
| GPIO6 (Pin 6)        | DTR (to IO0 pin) |
| GPIO7 (Pin 7)        | RTS (to EN pin) |

# Caveats

The DTR and RTS pin logic to auto-reset the ESP32 when flashing is marginally cursed. If hardware flow control is enabled in your terminal emulator (putty, minicom, etc) then the EN pin will be pulled low and the ESP32 turned off. Make sure flow control is turned off.

# Building

Git clone this repo, then:

```
mkdir build
cd build
export PICO_SDK_PATH=../../pico-sdk/ # or wherever you keep pico-sdk...
cmake ..
make -j`nproc`
```
