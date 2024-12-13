//!
//! XIAO RP2040 code for SF
//! 2024-12-12
//! 
//! [1. mapping first]
//! for SF, there are 65 WS2812B-2020 LEDs.
//! the LEDs are arranged in 2D-fibonacci
//!

#![no_std]
#![no_main]

#![allow(unused_variables)]

//#[cfg(test)]
//#[macro_use]
//extern crate std;

//use non_std::prelude::*;

// The macro for our start-up function
use rp_pico::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// Import the Timer for Ws2812:
use rp_pico::hal::timer::Timer;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

// PIOExt for the split() method that is needed to bring
// PIO0 into useable form for Ws2812:
use rp_pico::hal::pio::PIOExt;

// Import useful traits to handle the ws2812 LEDs:
use smart_leds::{brightness, SmartLedsWrite, RGB8};

// Import the actual crate to handle the Ws2812 protocol:
use ws2812_pio::Ws2812;

// Currently 3 consecutive LEDs are driven by this example
// to keep the power draw compatible with USB:
const LED_NUM: usize = 65; // number of LEDs

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Setup a delay for the LED blink signals:
    let mut frame_delay =
        cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Import the `sin` function for a smooth hue animation from the
    // Pico rp2040 ROM:
    let sin = hal::rom_data::float_funcs::fsin::ptr();

    // Create a count down timer for the Ws2812 instance:
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // Split the PIO state machine 0 into individual objects, so that
    // Ws2812 can use it:
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    // Instanciate a Ws2812 LED strip
    // GPIO27 is the data input to the LEDs of WS2812B-2020
    let mut ws = Ws2812::new(
        pins.gpio27.into_function(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );

    let mut leds: [RGB8; LED_NUM] = [(0, 0, 0).into(); LED_NUM];
    let mut t: f32 = 0.0;
    let mut hue_offs: f32;

    // Bring down the overall brightness of the strip to not blow
    // the USB power supply: every LED draws ~60mA, RGB means 3 LEDs per
    // ws2812 LED, for 3 LEDs that would be: 3 * 3 * 60mA, which is
    // already 540mA for just 3 white LEDs!
    let strip_brightness = 64u8; // Limit brightness to 64/256

    // Slow down timer by this factor (0.1 will result in 10 seconds):
    let animation_speed = 0.1;

    let mut fcnt = 0;
    loop {
        fcnt += 1;

        for (i, led) in leds.iter_mut().enumerate() {
            if fcnt > 0 && fcnt <= 2000 {
              hue_offs = match i % 32 {
                1 => 0.03125,
                2 => 0.0625,
                3 => 0.09375,
                4 => 0.125,
                5 => 0.15625,
                6 => 0.1875,
                7 => 0.21875,
                8 => 0.25,
                9 => 0.28125,
                10 => 0.3125,
                11 => 0.34375,
                12 => 0.375,
                13 => 0.40625,
                14 => 0.4375,
                15 => 0.46875,
                16 => 0.5,
                17 => 0.53125,
                18 => 0.5625,
                19 => 0.59375,
                20 => 0.625,
                21 => 0.65625,
                22 => 0.6875,
                23 => 0.71875,
                24 => 0.75,
                25 => 0.78125,
                26 => 0.8125,
                27 => 0.84375,
                28 => 0.875,
                29 => 0.90625,
                30 => 0.9375,
                31 => 0.96875,
                _ => 0.0,
              };
            } else if fcnt > 2000 && fcnt <= 4000 {
              hue_offs = match i % 16 {
                1 => 0.0625,
                2 => 0.125,
                3 => 0.1875,
                4 => 0.25,
                5 => 0.3125,
                6 => 0.375,
                7 => 0.4375,
                8 => 0.5,
                9 => 0.5625,
                10 => 0.625,
                11 => 0.6875,
                12 => 0.75,
                13 => 0.8125,
                14 => 0.875,
                15 => 0.9375,
                _ => 0.00,
              };
            } else if fcnt > 4000 && fcnt <= 6000 {
              hue_offs = match i % 8 {
                0 => 0.125,
                1 => 0.25,
                2 => 0.375,
                3 => 0.5,
                4 => 0.625,
                5 => 0.75,
                6 => 0.875,
                _ => 0.0,
              };
            } else if fcnt > 6000 && fcnt <= 8000 {
              hue_offs = match i % 4 {
                0 => 0.083,
                1 => 0.167,
                2 => 0.250,
                3 => 0.333,
                _ => 0.499,
              };
            } else if fcnt > 8000 && fcnt <= 10000 {
              hue_offs = match i % 3 {
                0 => 0.21,
                1 => 0.42,
                _ => 0.63,
              };
            } else {
              hue_offs = match i % LED_NUM {
                64 => 0.1,
                3 => 0.1,
                6 => 0.1,
                8 => 0.1,

                4 => 0.2,
                5 => 0.2,
                9 => 0.2,
                12 => 0.2,
                14 => 0.2,

                10 => 0.3,
                11 => 0.3,
                15 => 0.3,
                19 => 0.3,
                20 => 0.3,
                
                16 => 0.4,
                18 => 0.4,
                21 => 0.4,
                25 => 0.4,
                
                17 => 0.5,
                22 => 0.5,
                24 => 0.5,
                27 => 0.5,
                31 => 0.5,

                37 => 0.6,
                32 => 0.6,
                30 => 0.6,
                28 => 0.6,
                23 => 0.6,

                _ => 0.7,
              };

              if fcnt > 12000 {
                fcnt = 0;
              }
            }

            if fcnt == 0 {
              let hue = 360.0 * hue_offs;
              let sat = 1.0;
              let val = 1.0;

              let rgb = hsv2rgb_u8(hue, sat, val);
              *led = rgb.into();
            } else {
              let sin_11 = sin((t + hue_offs) * 2.0 * core::f32::consts::PI);
              // Bring -1..1 sine range to 0..1 range:
              let sin_01 = (sin_11 + 1.0) * 0.5;

              let hue = 360.0 * sin_01;
              let sat = 1.0;
              let val = 1.0;

              let rgb = hsv2rgb_u8(hue, sat, val);
              *led = rgb.into();
            }
        }
        
        //pattern1(leds, t);

        // Here the magic happens and the `leds` buffer is written to the
        // ws2812 LEDs:
        ws.write(brightness(leds.iter().copied(), strip_brightness))
            .unwrap();

        // Wait a bit until calculating the next frame:
        frame_delay.delay_ms(12); // 16 => ~60 FPS

        // Increase the time counter variable and make sure it
        // stays inbetween 0.0 to 1.0 range:
        t += (12.0 / 1000.0) * animation_speed;
        while t > 1.0 {
            t -= 1.0;
        }
    }
}

pub fn hsv2rgb(hue: f32, sat: f32, val: f32) -> (f32, f32, f32) {
    let c = val * sat;
    let v = (hue / 60.0) % 2.0 - 1.0;
    let v = if v < 0.0 { -v } else { v };
    let x = c * (1.0 - v);
    let m = val - c;
    let (r, g, b) = if hue < 60.0 {
        (c, x, 0.0)
    } else if hue < 120.0 {
        (x, c, 0.0)
    } else if hue < 180.0 {
        (0.0, c, x)
    } else if hue < 240.0 {
        (0.0, x, c)
    } else if hue < 300.0 {
        (x, 0.0, c)
    } else {
        (c, 0.0, x)
    };
    (r + m, g + m, b + m)
}

pub fn hsv2rgb_u8(h: f32, s: f32, v: f32) -> (u8, u8, u8) {
    let r = hsv2rgb(h, s, v);

    (
        (r.0 * 255.0) as u8,
        (r.1 * 255.0) as u8,
        (r.2 * 255.0) as u8,
    )
}
