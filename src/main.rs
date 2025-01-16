#![no_std]
#![no_main]

use rtic_monotonics::rp2040::prelude::*;

use defmt::{println, warn};
use defmt_rtt as _;

use panic_probe as _;

rp2040_timer_monotonic!(Mono);

#[derive(Debug, defmt::Format)]
pub enum Event {
    ButtonPressed,
    AlarmExpired,
}

#[rtic::app(device = rp_pico::hal::pac)]
mod app {
    use super::*;

    use fugit::MicrosDurationU32;
    use heapless::mpmc::Q16;
    use rp_pico::hal::gpio::bank0::Gpio16;
    use rp_pico::hal::gpio::{Pin, PullDown};
    use rp_pico::hal::timer::{Alarm, Alarm1};
    use rp_pico::hal::{self, Timer, Watchdog, clocks};
    use rp_pico::hal::{
        gpio::{self, FunctionSio, PullNone, SioOutput, bank0::Gpio25},
        sio::Sio,
    };
    use rp_pico::pac::NVIC;

    use embedded_hal::digital::v2::OutputPin;
    use panic_probe as _;
    use rp_pico::XOSC_CRYSTAL_FREQ;

    type LedPin = Pin<Gpio25, FunctionSio<SioOutput>, PullNone>;
    type ButtonPin = Pin<Gpio16, gpio::FunctionSioInput, PullDown>;

    #[shared]
    struct Shared {
        queue: heapless::mpmc::Q16<Event>,
        alarm: Alarm1,
    }

    #[local]
    struct Local {
        led: LedPin,
        button: ButtonPin,
        counter: u32,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Init LED pin
        let sio = Sio::new(ctx.device.SIO);
        let pins = rp_pico::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );
        let mut led = pins
            .led
            .into_pull_type::<PullNone>()
            .into_push_pull_output();
        led.set_low().unwrap();

        let button = pins.gpio16.into_pull_down_input();
        button.set_interrupt_enabled(gpio::Interrupt::EdgeHigh, true);

        let mut timer = Timer::new(ctx.device.TIMER, &mut ctx.device.RESETS, &clocks);

        let mut alarm = timer.alarm_1().unwrap();
        alarm.enable_interrupt();

        unsafe {
            NVIC::unmask(hal::pac::Interrupt::IO_IRQ_BANK0);
            NVIC::unmask(hal::pac::Interrupt::TIMER_IRQ_1);
        }

        ctx.core.SCB.set_sleepdeep();

        (
            Shared {
                queue: Q16::new(),
                alarm,
            },
            Local {
                led,
                button,
                counter: 0,
            },
        )
    }

    #[idle(shared = [queue, alarm], local = [led, counter])]
    fn idle(mut ctx: idle::Context) -> ! {
        const DURATION: MicrosDurationU32 = MicrosDurationU32::secs(1);
        let mut on = false;

        loop {
            println!("going to sleep");
            cortex_m::asm::wfi();
            println!("woke up");

            let Some(event) = ctx.shared.queue.lock(|q| q.dequeue()) else {
                continue;
            };

            match event {
                Event::ButtonPressed => {
                    println!("Button pressed");
                    ctx.shared
                        .alarm
                        .lock(|alarm| alarm.schedule(DURATION))
                        .unwrap();
                    if !on {
                        ctx.local.led.set_high().unwrap();
                        on = true;
                    }
                }
                Event::AlarmExpired => {
                    *ctx.local.counter += 1;
                    println!("Alarm expired {}", ctx.local.counter);
                    ctx.local.led.set_low().unwrap();
                    on = false;
                }
            };
        }
    }

    #[task(shared = [queue], local = [button], binds = IO_IRQ_BANK0)]
    fn on_gpio(mut ctx: on_gpio::Context) {
        defmt::println!("button irq");
        if let Err(e) = ctx.shared.queue.lock(|q| q.enqueue(Event::ButtonPressed)) {
            warn!("skipped button press: {}", e);
        }
        ctx.local.button.clear_interrupt(gpio::Interrupt::EdgeHigh);
    }

    #[task(shared = [queue, alarm], binds = TIMER_IRQ_1)]
    fn on_timer_expiration(mut ctx: on_timer_expiration::Context) {
        println!("timer interrupt");
        if let Err(e) = ctx
            .shared
            .queue
            .lock(|queue| queue.enqueue(Event::AlarmExpired))
        {
            warn!("skipped alarm expiration: {}", e);
        }
        ctx.shared.alarm.lock(|alarm| alarm.clear_interrupt());
    }
}
