#![no_std]
#![no_main]

use defmt::{println, warn};
use defmt_rtt as _;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use fugit::MicrosDurationU32;

use panic_probe as _;

use embedded_hal::digital::OutputPin;

use rp2040_hal::{
    Timer,
    gpio::{self, Interrupt::EdgeLow},
    pac::{NVIC, Peripherals, interrupt},
    timer::{Alarm, Alarm0},
};

use crate::interrupt::{IO_IRQ_BANK0, TIMER_IRQ_0};

use core::cell::RefCell;
use critical_section::Mutex;

#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

type LedPin = gpio::Pin<gpio::bank0::Gpio25, gpio::FunctionSioOutput, gpio::PullNone>;

type ButtonPin = gpio::Pin<gpio::bank0::Gpio16, gpio::FunctionSioInput, gpio::PullUp>;

static GLOBAL_LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static GLOBAL_BUTTON: Mutex<RefCell<Option<ButtonPin>>> = Mutex::new(RefCell::new(None));

static GLOBAL_ALARM: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

static CHANNEL: Channel<CriticalSectionRawMutex, Event, 10> = Channel::new();

enum Event {
    ButtonPressed,
    AlarmExpired,
}

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
    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let alarm = timer.alarm_0().unwrap();
    critical_section::with(|cs| {
        GLOBAL_ALARM.borrow(cs).replace(Some(alarm));
    });

    let sio = rp2040_hal::Sio::new(pac.SIO);

    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let led = pins.gpio25.reconfigure();

    let mut in_pin = pins.gpio16.into_pull_up_input();
    in_pin.set_slew_rate(gpio::OutputSlewRate::Slow);
    in_pin.set_schmitt_enabled(true);

    in_pin.set_interrupt_enabled(EdgeLow, true);

    critical_section::with(|cs| {
        GLOBAL_LED.borrow(cs).replace(Some(led));
        GLOBAL_BUTTON.borrow(cs).replace(Some(in_pin));
    });

    unsafe {
        NVIC::unmask(IO_IRQ_BANK0);
        NVIC::unmask(TIMER_IRQ_0);
    }

    let mut counter = 0;

    loop {
        println!("going to sleep");
        cortex_m::asm::wfi();
        println!("woke up");

        let Ok(event) = CHANNEL.try_receive() else {
            continue;
        };
        match event {
            Event::ButtonPressed => {
                println!("Button pressed, scheduling alarm");
                critical_section::with(|cs| {
                    schedule_alarm(cs);
                    led_on(cs);
                });
                println!("New alarm scheduled");
            }
            Event::AlarmExpired => {
                critical_section::with(led_off);
                counter += 1;
                println!("Alarm expired {}", counter);
            }
        };
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    println!("gpio interrupt");

    let edge = critical_section::with(|cs| {
        let mut guard = GLOBAL_BUTTON.borrow(cs).borrow_mut();
        let button = guard.as_mut().unwrap();
        if button.interrupt_status(EdgeLow) {
            button.clear_interrupt(EdgeLow);
            true
        } else {
            false
        }
    });
    if edge {
        if let Err(_e) = CHANNEL.try_send(Event::ButtonPressed) {
            warn!("Channel full, skipping event");
        }
    }
    println!("end gpio interrupt");
}

#[interrupt]
fn TIMER_IRQ_0() {
    println!("timer interrupt");

    critical_section::with(|cs| {
        let mut guard = GLOBAL_ALARM.borrow(cs).borrow_mut();
        guard.as_mut().unwrap().clear_interrupt();
    });
    if let Err(_e) = CHANNEL.try_send(Event::AlarmExpired) {
        warn!("Channel full, skipping event");
    }
    println!("end timer interrupt");
}

fn schedule_alarm(cs: critical_section::CriticalSection<'_>) {
    const DURATION: MicrosDurationU32 = MicrosDurationU32::secs(1);
    let mut alarm = GLOBAL_ALARM.borrow(cs).borrow_mut();
    alarm.as_mut().unwrap().schedule(DURATION).unwrap();
    alarm.as_mut().unwrap().enable_interrupt();
}

fn led_on(cs: critical_section::CriticalSection) {
    GLOBAL_LED
        .borrow(cs)
        .borrow_mut()
        .as_mut()
        .unwrap()
        .set_high()
        .unwrap();
}

fn led_off(cs: critical_section::CriticalSection<'_>) {
    GLOBAL_LED
        .borrow(cs)
        .borrow_mut()
        .as_mut()
        .unwrap()
        .set_low()
        .unwrap();
}
