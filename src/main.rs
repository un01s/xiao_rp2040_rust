//! # Multicore Blinking Example
//!
//! This application blinks two LEDs on GPIOs 2 and 3 at different rates (3Hz
//! and 4Hz respectively.)
//!
//! blink two leds, red and blue, by core1
//! blink led_green by core0
//!
 
#![no_std]
#![no_main]

use cortex_m::delay::Delay;

use hal::clocks::Clock;
use hal::gpio::Pins;
use hal::multicore::{Multicore, Stack};
use hal::sio::Sio;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

//use hal::pio::PIOExt;
//use hal::Timer;

//use smart_leds::SmartLedsWrite;
//use smart_leds::RGB8;
//use ws2812_pio::Ws2812;

//const LUMINOSITY: u8 = 10;
//const RED: RGB8 = RGB8::new(LUMINOSITY, 0, 0);
//const GREEN: RGB8 = RGB8::new(0, LUMINOSITY, 0);
//const BLUE: RGB8 = RGB8::new(0, 0, LUMINOSITY);

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_hal::digital::StatefulOutputPin;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// The frequency at which core 0 will blink its LED (Hz).
const CORE0_FREQ: u32 = 3;
/// The frequency at which core 1 will blink its LED (Hz).
const CORE1_FREQ: u32 = 4;
/// The delay between each toggle of core 0's LED (us).
const CORE0_DELAY: u32 = 1_000_000 / CORE0_FREQ;
/// The delay between each toggle of core 1's LED (us).
const CORE1_DELAY: u32 = 1_000_000 / CORE1_FREQ;

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static
/// values is reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything separately
/// and modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte
/// alignment, which allows the stack guard to take up the least amount of
/// usable RAM.
static mut CORE1_STACK: Stack<4096> = Stack::new();

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // Set up the GPIO pins
    let mut sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // change it to XIAO rp2040
    let mut led_green = pins.gpio16.into_push_pull_output();
    let mut led_red = pins.gpio17.into_push_pull_output();
    let mut led_blue = pins.gpio25.into_push_pull_output();

    // Set up the delay for the first core.
    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = Delay::new(core.SYST, sys_freq);

/*
    // Set up for neopixel
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    // PIO for neopixel
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    // RGB neopixel LED
    let mut ws = Ws2812::new(
        pins.neopixel_data.into_mode(),
        &mut pio,
        sm0,
        clocks.peripheral_clock.freq(),
        timer.count_down(),
    );
    // Power on Neopixel RGB LED
    let mut neopixel_power = pins.gpio11.into_push_pull_output();
    neopixel_power.set_high().unwrap();
*/
    // Start up the second core to blink the second LED
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    #[allow(static_mut_refs)]
    core1
        .spawn(unsafe { &mut CORE1_STACK.mem }, move || {
            // Get the second core's copy of the `CorePeripherals`, which are per-core.
            // Unfortunately, `cortex-m` doesn't support this properly right now,
            // so we have to use `steal`.
            let core = unsafe { pac::CorePeripherals::steal() };
            // Set up the delay for the second core.
            let mut delay = Delay::new(core.SYST, sys_freq);
            // Blink the second LED.
            loop {
                led_red.toggle().unwrap();
                led_blue.toggle().unwrap();
                delay.delay_us(CORE1_DELAY)
            }
        })
        .unwrap();

    // Blink the first LED.
    loop {
        led_green.toggle().unwrap();
        delay.delay_us(CORE0_DELAY)
    }
}

