#![no_std]
#![no_main]

use defmt::println;
use defmt_rtt as _;

use panic_probe as _;

use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};

use rp2040_hal::{
    Timer,
    gpio::{self, Interrupt::EdgeLow},
    pac::Peripherals,
};

#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {
    println!("hi!");
    let mut pac = Peripherals::take().unwrap();

    let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);

    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();
    let _timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let sio = rp2040_hal::Sio::new(pac.SIO);

    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.gpio25.into_push_pull_output();

    let mut in_pin = pins.gpio16.into_pull_up_input();
    in_pin.set_slew_rate(gpio::OutputSlewRate::Slow);
    in_pin.set_schmitt_enabled(true);

    in_pin.set_interrupt_enabled(EdgeLow, true);

    loop {
        if in_pin.is_low().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }
    }
}
