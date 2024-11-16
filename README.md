# Rust on XIAO RP2040 with WS2812

## [setup](https://github.com/tutoduino/xiao_rp2040_rs/) 

Rust program that blinks the USER LED and RGB LED on a Seeed Studio XIAO RP2040 board using ws2812-pio

![alt text](https://files.seeedstudio.com/wiki/XIAO-RP2040/img/xinfront.jpg)

## mutlicore

built after adjusting the memory.x. but the code does not seem to run.

### second try

The dependencies are a lot more tricky.

* check ```Cargo.toml```: it uses ```seeeduino-xiao-rp2040``` package and other dependencies.

* check ```.cargo/config.toml```: it uses ```elf2uf2-rs -d``` as the runner to flash the code to the storage drive set up by the bootloader

* check the library for XIAO RP2040 board

[The repo of rp-hal-boards](https://github.com/rp-rs/rp-hal-boards) has some code for XIAO RP2040. It uses rp2040_hal. It sets up boot2. 

| pin | function |
| --- | -------- |
| Gpio0 | FunctionUart UartTx |
| Gpio1 | FunctionUart UartRx |
| Gpio2 | FunctionSpi Sck |
| Gpio3 | FunctionSpi Mosi |
| Gpio4 | FunctionSpi Miso |
| Gpio6 | FunctionI2C Sda |
| Gpio7 | FunctionI2C Scl |
| Gpio11 | neopixel_power |
| Gpio12 | neopixel_data |
| Gpio16 | led_green |
| Gpio17 | led_red |
| Gpio25 | led_blue |
| Gpio26 | a0 |
| Gpio27 | a1 |
| Gpio28 | a2 |
| Gpio29 | a3 |

```
pub const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;
```

### back to running v1

there is a package, critical-section, its version is changed from 1.2.0 to ```1.1.1```. this may be critical to make the code to run.

### multicore

The version of critical-section is changed from 1.2.0 to 1.1.1.

In the example code running on the XIAO RP2040, there are two additional dependencies for WS2812B RGB LED.

```
ws2812-pio = "0.4.0"
smart-leds = "0.3.0"
```

However, with these two, cargo has problems with rp2040_hal version 0.10. Update both to the latest as follows to see if the build is OK.

```
ws2812-pio = "0.8.0"
smart-leds = "0.4.0"
```

Still, there are a lot of problems to use Neopixel LED.

