#![no_std]
#![no_main]

use defmt::println;
use defmt_rtt as _;

use panic_probe as _;

#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[rp2040_hal::entry]
fn main() -> ! {
    println!("hi!");
    loop {}
}
