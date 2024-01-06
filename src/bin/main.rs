// main.rs

#![no_std]
#![no_main]

// #![allow(dead_code)]
// #![allow(unused_imports)]
// #![allow(unused_mut)]
// #![allow(unused_variables)]

use suomipeli::*;

use panic_halt as _;

use core::fmt::Write;
use cortex_m::asm;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::*;
use port_expander_multi::{dev::pca9555::Driver, Direction, PortDriver, PortDriverTotemPole};
use rand::{prelude::*, SeedableRng};
use shared_bus::BusMutex;
use systick_monotonic::Systick;

use rp_pico::{
    hal::{self, gpio::bank0::*, gpio::*, pac, uart, Clock},
    XOSC_CRYSTAL_FREQ,
};

// Some type porn
type GPIO12 = Pin<Gpio12, FunctionI2c, PullDown>;
type GPIO13 = Pin<Gpio13, FunctionI2c, PullDown>;
type GPIO18 = Pin<Gpio18, FunctionI2c, PullDown>;
type GPIO19 = Pin<Gpio19, FunctionI2c, PullDown>;
type GPIO20 = Pin<Gpio20, FunctionSio<SioInput>, PullUp>;
type LedPin = Pin<Gpio25, FunctionSio<SioOutput>, PullDown>;

type MyI2C0 = hal::I2C<pac::I2C0, (GPIO12, GPIO13)>;
type MyI2C1 = hal::I2C<pac::I2C1, (GPIO18, GPIO19)>;

type PortExpInner0 = shared_bus::NullMutex<Driver<MyI2C0>>;
type PortExpInner1 = shared_bus::NullMutex<Driver<MyI2C1>>;
type IoE0 = port_expander_multi::Pca9555<PortExpInner0>;
type IoE1 = port_expander_multi::Pca9555<PortExpInner1>;

type MyUart = uart::UartPeripheral<
    uart::Enabled,
    pac::UART0,
    (
        Pin<Gpio0, FunctionUart, PullDown>,
        Pin<Gpio1, FunctionUart, PullDown>,
    ),
>;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OutTestState {
    Idle,
    TestQuiz,
    TestMapS,
    TestMapN,
    TestSocket,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EasterEggState {
    Start,
    Phase1,
    Phase2,
    Phase3,
    Go,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GameMode {
    MapQuizA,
    MapQuizB,
    Quiz,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GameDifficulty {
    Novice,
    Expert,
}
#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [RTC_IRQ, XIP_IRQ, TIMER_IRQ_3, TIMER_IRQ_2])]
mod app {
    use crate::*;

    #[shared]
    struct Shared {
        led: LedPin,
        led_on: bool,
        uart: MyUart,

        irqc: u32,
        sw_pin: GPIO20,

        ioe0: IoE0,
        ioe1: IoE1,

        bits: [u16; 8],
        rise: [u16; 8],
        fall: [u16; 8],
        output: [u16; 8],

        tstate: OutTestState,
        estate: EasterEggState,
        gmode: GameMode,
        gdiff: GameDifficulty,
        game_on: bool,
        socket: Option<MyPin>,
        socket_i: Option<usize>,
        answers_a: [Option<MyPin>; 24], // for GameMode::MapQuizA and GameMode::Quiz
        answers_b: [Option<MyPin>; 24], // for GameMode::MapQuizB
        rand: bool,
        rng: Option<StdRng>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SystickMono = Systick<100>;

    // time intervals for different blinkenlichten
    //
    pub const VERY_SHORT_TIME: u64 = 50u64;
    pub const SHORT_TIME: u64 = 200u64;
    pub const MEDIUM_TIME: u64 = 500u64;
    pub const LONG_TIME: u64 = 1000u64;
    pub const VERY_LONG_TIME: u64 = 7500u64;

    pub const ALIVE_INTERVAL: u64 = 5000u64;
    pub const LED_BLINK_INTERVAL: u64 = 420u64;

    #[local]
    struct Local {}

    #[init()]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = c.device;
        let mut resets = dp.RESETS;
        let mut watchdog = hal::watchdog::Watchdog::new(dp.WATCHDOG);

        // Configure the clocks - The default is to generate a 125 MHz system clock
        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            dp.XOSC,
            dp.CLOCKS,
            dp.PLL_SYS,
            dp.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let systick = c.core.SYST;
        let mono = Systick::new(systick, 125_000_000);

        let sio = hal::Sio::new(dp.SIO);
        let pins = rp_pico::Pins::new(dp.IO_BANK0, dp.PADS_BANK0, sio.gpio_bank0, &mut resets);

        // Initialize the LED GPIO
        let mut led = pins.led.into_push_pull_output();
        led.set_low().ok();

        // Initialize the UART
        let uart_pins = (
            // UART TX on pin 1 (GPIO0)
            pins.gpio0.into_function::<FunctionUart>(),
            // UART RX on pin 2 (GPIO1)
            pins.gpio1.into_function::<FunctionUart>(),
        );
        let uart = hal::uart::UartPeripheral::new(dp.UART0, uart_pins, &mut resets)
            .enable(
                uart::UartConfig::new(
                    115200u32.Hz(),
                    uart::DataBits::Eight,
                    None,
                    uart::StopBits::One,
                ),
                clocks.peripheral_clock.freq(),
            )
            .unwrap();
        let mut buf = [0u8; 80];
        write!(
            Wrapper::new(&mut buf),
            "\r\n\r\n*** Starting up Suomipeli\r\n\
            \x20   firmware v{}\r\n\r\n",
            env!("CARGO_PKG_VERSION"),
        )
        .ok();
        uart.write_full_blocking(&buf);

        // Configure these pins as being I²C, not GPIO
        let sda0_pin = pins.gpio12.into_function::<FunctionI2C>();
        let scl0_pin = pins.gpio13.into_function::<FunctionI2C>();
        let i2c0 = hal::I2C::i2c0(
            dp.I2C0,
            sda0_pin,
            scl0_pin,
            100u32.kHz(),
            &mut resets,
            &clocks.system_clock,
        );
        // Create ioe0, port expander with output pins
        let ioe0 = port_expander_multi::Pca9555::new_m(i2c0);

        // Configure these pins as being I²C, not GPIO
        let sda1_pin = pins.gpio18.into_function::<FunctionI2C>();
        let scl1_pin = pins.gpio19.into_function::<FunctionI2C>();
        let i2c1 = hal::I2C::i2c1(
            dp.I2C1,
            sda1_pin,
            scl1_pin,
            100u32.kHz(),
            &mut resets,
            &clocks.system_clock,
        );
        // Create ioe1, port expander with input pins
        let ioe1 = port_expander_multi::Pca9555::new_m(i2c1);

        // Internal data structures to track pin states & changes
        let mut bits = [0; 8];
        let rise = [0; 8];
        let fall = [0; 8];
        let output = [0; 8];

        // Setup ioe0 for outputs only with LOW state
        (0..=7).for_each(|i| {
            ioe0.0.lock(|drv| {
                drv.set_direction(i, 0xFFFF, Direction::Output, false).ok();
            })
        });

        // Setup ioe1 for inputs only
        (0..=7).for_each(|i| {
            ioe1.0.lock(|drv| {
                drv.set_direction(i, 0xFFFF, Direction::Input, false).ok();
            })
        });

        // read all input pins on ioe1 to clear _INT pin
        (0..=7).for_each(|i| {
            ioe1.0.lock(|drv| {
                bits[i as usize] = drv.read_u16(i).unwrap();
            })
        });

        // We connect the interrupt line _INT from PCA9555s into gpio20
        let sw_pin = pins.gpio20.into_pull_up_input();

        #[cfg(feature = "test_output")]
        test_output::spawn_after(MEDIUM_TIME.millis()).ok();

        #[cfg(feature = "io_irq")]
        enable_io_irq::spawn_after(VERY_LONG_TIME.millis()).ok();

        #[cfg(feature = "io_noirq")]
        io_noirq::spawn_after(VERY_LONG_TIME.millis()).ok();

        // double-click the relay when we are ready for input
        //
        click_relay01::spawn_after(VERY_LONG_TIME.millis()).ok();
        click_relay01::spawn_after((VERY_LONG_TIME + SHORT_TIME).millis()).ok();
        gamemode_map::spawn_after((VERY_LONG_TIME + SHORT_TIME).millis()).ok();

        led_blink::spawn().ok();
        alive::spawn().ok();

        (
            Shared {
                led,
                led_on: false,
                uart,

                irqc: 0,
                sw_pin,

                ioe0,
                ioe1,
                bits,
                rise,
                fall,
                output,

                tstate: OutTestState::TestQuiz,
                estate: EasterEggState::Start,
                gmode: GameMode::MapQuizA,
                gdiff: GameDifficulty::Novice,
                game_on: false,
                answers_a: [None; 24],
                answers_b: [None; 24],
                socket: None,
                socket_i: None,
                rand: false,
                rng: None,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    fn which_bit(bits: &[u16; 8]) -> (u8, u8) {
        for chip in 0..=7 {
            let input_bits = bits[chip as usize];
            if input_bits != 0 {
                for bit in 0..=15 {
                    if input_bits & (1 << bit) != 0 {
                        return (chip, bit as u8);
                    }
                }
            }
        }
        (0, 0)
    }

    // Task with least priority that only runs when nothing else is running.
    #[idle(local = [x: u32 = 0])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            asm::wfi();
            *cx.local.x += 1;
        }
    }

    #[task(priority = 1, shared = [led, led_on])]
    fn led_blink(cx: led_blink::Context) {
        let led_blink::SharedResources { led, led_on, .. } = cx.shared;

        (led, led_on).lock(|led, led_on| {
            if *led_on {
                led.set_high().ok();
                *led_on = false;
            } else {
                led.set_low().ok();
                *led_on = true;
            }
        });
        led_blink::spawn_after(LED_BLINK_INTERVAL.millis()).ok();
    }

    #[task(priority = 1, shared = [irqc, sw_pin, uart])]
    fn alive(cx: alive::Context) {
        let alive::SharedResources {
            irqc, sw_pin, uart, ..
        } = cx.shared;

        let mut buf = [0u8; 64];

        (irqc, sw_pin).lock(|irqc, sw_pin| {
            write!(
                Wrapper::new(&mut buf),
                "{}: irqc = {}, swpin = {}\r\n",
                monotonics::now().duration_since_epoch().to_secs(),
                *irqc,
                sw_pin.is_high().unwrap() as u8
            )
            .ok();
        });

        (uart,).lock(|uart| {
            uart.write_full_blocking(&buf);
        });

        alive::spawn_after(ALIVE_INTERVAL.millis()).ok();
    }

    #[task(priority = 2)]
    fn io_noirq(_cx: io_noirq::Context) {
        io_poll::spawn().ok();
        io_noirq::spawn_after(VERY_SHORT_TIME.millis()).ok();
    }

    #[task(binds = IO_IRQ_BANK0, priority = 3, shared = [irqc, sw_pin])]
    fn io_irq(cx: io_irq::Context) {
        let io_irq::SharedResources { irqc, sw_pin, .. } = cx.shared;

        // Clear & disable our interrupt immediately to avoid repeated firing
        (irqc, sw_pin).lock(|irqc, sw_pin| {
            sw_pin.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, false);
            sw_pin.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
            *irqc += 1;
        });

        // We are using 200ms debounce period
        enable_io_irq::spawn_after(SHORT_TIME.millis()).ok();

        // Here we actually check what had changed in input pins
        io_poll::spawn().ok();
    }

    #[task(priority = 2, capacity = 4, shared = [sw_pin])]
    fn enable_io_irq(cx: enable_io_irq::Context) {
        let enable_io_irq::SharedResources { sw_pin, .. } = cx.shared;

        io_poll::spawn().ok();

        (sw_pin,).lock(|sw_pin| {
            sw_pin.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
            sw_pin.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe1, bits, rise, fall])]
    fn io_poll(cx: io_poll::Context) {
        let io_poll::SharedResources {
            ioe1,
            bits,
            rise,
            fall,
            ..
        } = cx.shared;

        let mut changed = false;
        let mut fallen = false;
        let mut risen = false;

        // We read all input pin states and mark changes since last check'
        // Note: reading all pins will also clear _INT state on PCA9555
        (ioe1, bits, rise, fall).lock(|ioe1, bits, rise, fall| {
            (0..=7).for_each(|i| {
                ioe1.0.lock(|drv| {
                    let before = bits[i];
                    let after = drv.read_u16(i as u8).unwrap();
                    bits[i] = after;
                    rise[i] = 0;
                    fall[i] = 0;

                    if after != before {
                        changed = true;

                        let risen_bits = !before & after;
                        if risen_bits != 0 {
                            risen = true;
                            rise[i] = risen_bits;
                        }

                        let fallen_bits = before & !after;
                        if fallen_bits != 0 {
                            fallen = true;
                            fall[i] = fallen_bits;
                        }
                    }
                });
            });
        });

        if fallen {
            input_event_fall::spawn().ok();
        }
        if risen {
            input_event_rise::spawn().ok();
        }

        #[cfg(feature = "io_debug")]
        if changed {
            io_debug::spawn().ok();
        }
    }

    #[task(priority = 1, capacity = 4, shared = [bits, rise, fall, uart])]
    fn io_debug(cx: io_debug::Context) {
        let io_debug::SharedResources {
            bits,
            rise,
            fall,
            uart,
            ..
        } = cx.shared;

        let mut buf = [0u8; 512];
        let mut w = Wrapper::new(&mut buf);
        write!(w, "bits0: ").ok();

        (bits, rise, fall).lock(|bits, rise, fall| {
            (0..=3).for_each(|i| {
                write!(w, " {:016b}", bits[i]).ok();
            });
            write!(w, "\r\nfall0: ").ok();
            (0..=3).for_each(|i| {
                write!(w, " {:016b}", fall[i]).ok();
            });
            write!(w, "\r\nrise0: ").ok();
            (0..=3).for_each(|i| {
                write!(w, " {:016b}", rise[i]).ok();
            });

            write!(w, "\r\n\r\nbits1: ").ok();
            (4..=7).for_each(|i| {
                write!(w, " {:016b}", bits[i]).ok();
            });
            write!(w, "\r\nfall1: ").ok();
            (4..=7).for_each(|i| {
                write!(w, " {:016b}", fall[i]).ok();
            });
            write!(w, "\r\nrise1: ").ok();
            (4..=7).for_each(|i| {
                write!(w, " {:016b}", rise[i]).ok();
            });
        });
        write!(w, "\r\n\r\n").ok();

        (uart,).lock(|uart| {
            uart.write_full_blocking(&buf);
        });
    }

    #[task(priority = 1, capacity = 8, shared = [fall, estate, gmode, gdiff, game_on, socket, socket_i, answers_a, answers_b, irqc, uart])]
    fn input_event_fall(cx: input_event_fall::Context) {
        let input_event_fall::SharedResources {
            fall,
            estate,
            gmode,
            gdiff,
            game_on,
            socket,
            socket_i,
            answers_a,
            answers_b,
            irqc,
            uart,
            ..
        } = cx.shared;

        let mut edge_low = false;
        let (mut chip, mut bit) = (0, 0);

        (fall,).lock(|fall| {
            (0..=7).for_each(|i| {
                if fall[i] != 0 {
                    edge_low = true;
                    (chip, bit) = which_bit(fall);
                }
            });
        });

        // Have any contacts been touched?
        if edge_low {
            let pin = pin_input_ident(chip, bit);

            #[cfg(feature = "input_debug")]
            {
                let mut buf = [0u8; 80];
                let mut w = Wrapper::new(&mut buf);

                (irqc,).lock(|irqc| {
                    write!(w, "Falling edge: c{chip} b{bit} c{} {pin:?}\r\n\r\n", *irqc).ok();
                });
                (uart,).lock(|uart| {
                    uart.write_full_blocking(&buf);
                });
            }

            (estate,).lock(|estate| match *estate {
                EasterEggState::Start => {
                    if let MyPin::Map05_Turku = pin {
                        *estate = EasterEggState::Phase1;
                    }
                }
                EasterEggState::Phase1 => {
                    if let MyPin::Map01_Tammisaari = pin {
                        *estate = EasterEggState::Phase2;
                    } else {
                        *estate = EasterEggState::Start;
                    }
                }
                EasterEggState::Phase2 => {
                    if let MyPin::Map05_Turku = pin {
                        *estate = EasterEggState::Phase3;
                    } else {
                        *estate = EasterEggState::Start;
                    }
                }
                EasterEggState::Phase3 => {
                    if let MyPin::Map01_Tammisaari = pin {
                        *estate = EasterEggState::Go;
                    } else {
                        *estate = EasterEggState::Start;
                    }
                }
                EasterEggState::Go => {
                    out_zero::spawn().ok();
                    easter_egg::spawn(pin).ok();
                }
            });

            (
                gmode, gdiff, game_on, socket, socket_i, answers_a, answers_b,
            )
                .lock(
                    |gmode, gdiff, game_on, socket, socket_i, answers_a, answers_b| {
                        match pin {
                            MyPin::Mode01 | MyPin::UnknownPin => {
                                // ignore these (Mode01 is handled at rising edge)
                            }
                            _ => {
                                /*
                                // useful when debugging
                                set_output::spawn(pin.clone()).ok();
                                clear_output::spawn_after((3*LONG_TIME).millis(), pin.clone()).ok();
                                */

                                // Make audible feedback
                                click_relay01::spawn().ok();

                                if let Some(i) = socket_index(pin) {
                                    // a socket was connected
                                    *socket = Some(pin);
                                    *socket_i = Some(i);

                                    // blink the light shortly
                                    toggle_output::spawn(pin).ok();
                                    toggle_output::spawn_after(SHORT_TIME.millis(), pin).ok();
                                } else if let Some(i) = socket_i {
                                    // if we already have a socket connected
                                    let socket = socket.unwrap_or(MyPin::UnknownPin);

                                    if !*game_on {
                                        // Difficulty level selection only available before starting the game
                                        match *gmode {
                                            GameMode::MapQuizA | GameMode::MapQuizB => {
                                                if pin == MyPin::Quiz03 {
                                                    match *gdiff {
                                                        GameDifficulty::Novice => {
                                                            *gdiff = GameDifficulty::Expert;
                                                            set_output::spawn(MyPin::Quiz03).ok();
                                                        }
                                                        GameDifficulty::Expert => {
                                                            *gdiff = GameDifficulty::Novice;
                                                            clear_output::spawn(MyPin::Quiz03).ok();
                                                        }
                                                    }
                                                    return;
                                                }
                                            }
                                            GameMode::Quiz => {
                                                if pin == MyPin::Map47_Utsjoki {
                                                    match *gdiff {
                                                        GameDifficulty::Novice => {
                                                            *gdiff = GameDifficulty::Expert;
                                                            set_output::spawn(MyPin::Map47_Utsjoki)
                                                                .ok();
                                                        }
                                                        GameDifficulty::Expert => {
                                                            *gdiff = GameDifficulty::Novice;
                                                            clear_output::spawn(
                                                                MyPin::Map47_Utsjoki,
                                                            )
                                                            .ok();
                                                        }
                                                    }
                                                    return;
                                                }
                                            }
                                        }
                                    }

                                    *game_on = true;
                                    match *gmode {
                                        GameMode::Quiz => {
                                            if pin == answers_a[*i].unwrap_or(MyPin::UnknownPin) {
                                                // we got the right answer!
                                                ting_bell01::spawn().ok();
                                                clear_output::spawn(pin).ok();
                                                clear_output::spawn(socket).ok();
                                                answers_a[*i] = None;
                                            } else if *gdiff == GameDifficulty::Expert {
                                                // no mistakes allowed in expert mode
                                                click_relay01::spawn().ok();
                                                gamemode_quiz::spawn().ok();
                                            }
                                        }
                                        GameMode::MapQuizA => {
                                            if pin == answers_a[*i].unwrap_or(MyPin::UnknownPin) {
                                                // we got the right answer!
                                                ting_bell01::spawn().ok();
                                                clear_output::spawn(pin).ok();
                                                clear_output::spawn(socket).ok();
                                                answers_a[*i] = None;
                                            } else if pin == MyPin::Quiz02 {
                                                // Switch to Map B
                                                gamemode_mapB::spawn().ok();
                                            } else if *gdiff == GameDifficulty::Expert {
                                                // no mistakes allowed in expert mode
                                                click_relay01::spawn().ok();
                                                gamemode_map::spawn().ok();
                                            }
                                        }
                                        GameMode::MapQuizB => {
                                            if pin == answers_b[*i].unwrap_or(MyPin::UnknownPin) {
                                                // we got the right answer!
                                                ting_bell01::spawn().ok();
                                                clear_output::spawn(pin).ok();
                                                clear_output::spawn(socket).ok();
                                                answers_b[*i] = None;
                                            } else if pin == MyPin::Quiz01 {
                                                // Switch to Map A
                                                gamemode_mapA::spawn().ok();
                                            } else if *gdiff == GameDifficulty::Expert {
                                                // no mistakes allowed in expert mode
                                                click_relay01::spawn().ok();
                                                gamemode_map::spawn().ok();
                                            }
                                        }
                                    }
                                }
                            }
                        };
                    },
                );
        }
    }

    #[task(priority = 1, capacity = 8, shared = [rise, gmode, game_on, socket, socket_i, irqc, uart])]
    fn input_event_rise(cx: input_event_rise::Context) {
        let input_event_rise::SharedResources {
            rise,
            gmode,
            game_on,
            socket,
            socket_i,
            irqc,
            uart,
            ..
        } = cx.shared;

        let mut edge_high = false;
        let (mut chip, mut bit) = (0, 0);

        (rise,).lock(|rise| {
            (0..=7).for_each(|i| {
                if rise[i] != 0 {
                    edge_high = true;
                    (chip, bit) = which_bit(rise);
                }
            });
        });

        if edge_high {
            let pin = pin_input_ident(chip, bit);

            #[cfg(feature = "input_debug")]
            {
                let mut buf = [0u8; 80];
                let mut w = Wrapper::new(&mut buf);

                (irqc,).lock(|irqc| {
                    write!(w, "Rising edge: c{chip} b{bit} c{} {pin:?}\r\n\r\n", *irqc).ok();
                });
                (uart,).lock(|uart| {
                    uart.write_full_blocking(&buf);
                });
            }

            // socket was disconnected?
            if socket_index(pin).is_some() {
                (socket, socket_i).lock(|socket, socket_i| {
                    *socket = None;
                    *socket_i = None;

                    // blink the light shortly
                    toggle_output::spawn(pin).ok();
                    toggle_output::spawn_after(SHORT_TIME.millis(), pin).ok();
                });
            }

            if let MyPin::Mode01 = pin {
                (gmode, game_on).lock(|gmode, game_on| {
                    // Has the MODE switch been pressed?
                    // We have a state machine for mode change.
                    if *game_on {
                        // The game is on: RESTART the current game mode
                        match *gmode {
                            GameMode::MapQuizA | GameMode::MapQuizB => {
                                ting_bell01::spawn().ok();
                                gamemode_map::spawn_after(MEDIUM_TIME.millis()).ok();
                            }
                            GameMode::Quiz => {
                                ting_bell01::spawn().ok();
                                gamemode_quiz::spawn_after(MEDIUM_TIME.millis()).ok();
                            }
                        }
                    } else {
                        // The game is not yet on: switch between MAP/QUIZ
                        match *gmode {
                            GameMode::Quiz => {
                                gamemode_map::spawn_after(MEDIUM_TIME.millis()).ok();
                            }
                            GameMode::MapQuizA | GameMode::MapQuizB => {
                                gamemode_quiz::spawn_after(MEDIUM_TIME.millis()).ok();
                            }
                        }
                    }
                });
            }
        }
    }

    #[task(priority = 1, shared = [ioe0, output, tstate])]
    fn test_output(cx: test_output::Context) {
        #[cfg(feature = "test_output")]
        {
            let test_output::SharedResources {
                ioe0,
                output,
                tstate,
                ..
            } = cx.shared;

            (ioe0, output, tstate).lock(|ioe0, output, tstate| {
                let (test, active) = match *tstate {
                    OutTestState::TestQuiz => {
                        *tstate = OutTestState::TestMapS;
                        (&OUT_QUIZ, true)
                    }
                    OutTestState::TestMapS => {
                        *tstate = OutTestState::TestMapN;
                        (&OUT_MAP_S, true)
                    }
                    OutTestState::TestMapN => {
                        *tstate = OutTestState::TestSocket;
                        (&OUT_MAP_N, true)
                    }
                    OutTestState::TestSocket => {
                        *tstate = OutTestState::Idle;
                        (&OUT_SOCKET, true)
                    }
                    OutTestState::Idle => (&OUT_SOCKET, false),
                };

                if active {
                    click_relay01::spawn().ok();
                    (0..24).for_each(|i| {
                        let pin_bits = test[i] as u32;
                        let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                        let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                        output[chip] = out;
                        ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
                    });
                    out_zero::spawn_after((LONG_TIME / 10 * 9).millis()).ok();
                    test_output::spawn_after(LONG_TIME.millis()).ok();
                } else {
                    ting_bell01::spawn().ok();
                }
            });
        }
    }

    #[task(priority = 1, shared = [rand, rng])]
    fn rand_output(cx: rand_output::Context) {
        let rand_output::SharedResources { rand, rng, .. } = cx.shared;

        (rng,).lock(|rng| {
            if rng.is_none() {
                *rng = Some(StdRng::seed_from_u64(monotonics::now().ticks()));
            }

            let mut r: [u8; 2] = [0; 2];
            rng.as_mut().unwrap().fill_bytes(&mut r);
            let rpin = pin_input_ident(r[0] & 0x07, r[1] & 0x0F);

            match rpin {
                MyPin::UnknownPin => {}
                MyPin::Mode01 => {} // Mode01 is input only
                p => {
                    // Blink the lamp for a short time
                    set_output::spawn(p).ok();
                    clear_output::spawn_after(MEDIUM_TIME.millis(), p).ok();
                }
            }
        });
        (rand,).lock(|rand| {
            if *rand {
                rand_output::spawn_after(VERY_SHORT_TIME.millis()).ok();
            }
        });
    }

    #[task(priority = 1, shared = [rand, ioe0, output])]
    fn easter_egg(cx: easter_egg::Context, pin: MyPin) {
        let easter_egg::SharedResources {
            rand, ioe0, output, ..
        } = cx.shared;

        let (test, test_active) = match pin {
            MyPin::Quiz01 => (&OUT_QUIZ, true),
            MyPin::Quiz02 => (&OUT_MAP_S, true),
            MyPin::Quiz03 => (&OUT_MAP_N, true),
            MyPin::Quiz04 => (&OUT_SOCKET, true),
            MyPin::Quiz05 => {
                out_zero::spawn().ok();
                (&OUT_QUIZ, false)
            }
            MyPin::Quiz23 => {
                (rand,).lock(|rand| {
                    *rand = true;
                    rand_output::spawn().ok();
                });
                (&OUT_QUIZ, false)
            }
            MyPin::Quiz24 => {
                (rand,).lock(|rand| {
                    *rand = false;
                });
                (&OUT_QUIZ, false)
            }
            _ => (&OUT_QUIZ, false),
        };

        if test_active {
            (output, ioe0).lock(|output, ioe0| {
                (0..24).for_each(|i| {
                    let pin_bits = test[i] as u32;
                    let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                    let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                    output[chip] = out;
                    ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
                });
            });
        }
    }

    #[task(priority = 1, capacity = 32, shared = [ioe0, output])]
    fn set_output(cx: set_output::Context, pin: MyPin) {
        let set_output::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            let pin_bits = pin as u32;
            let chip = ((pin_bits & 0xFF00) >> 8) as usize;
            let out = output[chip] | (1u16 << ((pin_bits & 0xFF) as u8));
            output[chip] = out;
            ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
        });
    }

    #[task(priority = 1, capacity = 8, shared = [ioe0, output])]
    fn toggle_output(cx: toggle_output::Context, pin: MyPin) {
        let toggle_output::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            let pin_bits = pin as u32;
            let chip = ((pin_bits & 0xFF00) >> 8) as usize;
            let out = output[chip] ^ (1u16 << ((pin_bits & 0xFF) as u8));
            output[chip] = out;
            ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
        });
    }

    #[task(priority = 1, capacity = 16, shared = [ioe0, output])]
    fn clear_output(cx: clear_output::Context, pin: MyPin) {
        let clear_output::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            let pin_bits = pin as u32;
            let chip = ((pin_bits & 0xFF00) >> 8) as usize;
            let out = output[chip] & (!(1u16 << ((pin_bits & 0xFF) as u8)));
            output[chip] = out;
            ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn out_zero(cx: out_zero::Context) {
        let out_zero::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..=7u8).for_each(|i| {
                ioe0.0.lock(|drv| drv.write_u16(i, 0).ok());
                output[i as usize] = 0;
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn out_ones(cx: out_ones::Context) {
        let out_ones::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..=7).for_each(|i| {
                ioe0.0.lock(|drv| drv.write_u16(i, 0xFFFF).ok());
                output[i as usize] = 0xFFFF;
            });
        });
    }

    #[task(priority = 1, capacity = 4)]
    fn click_relay01(_cx: click_relay01::Context) {
        set_output::spawn(MyPin::Relay01).ok();
        clear_output::spawn_after(VERY_SHORT_TIME.millis(), MyPin::Relay01).ok();
    }

    #[task(priority = 1, capacity = 4)]
    fn ting_bell01(_cx: ting_bell01::Context) {
        set_output::spawn(MyPin::Bell01).ok();
        clear_output::spawn_after(VERY_SHORT_TIME.millis(), MyPin::Bell01).ok();
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn sockets_on(cx: sockets_on::Context) {
        let sockets_on::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = OUT_SOCKET[i] as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn sockets_off(cx: sockets_off::Context) {
        let sockets_off::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = OUT_SOCKET[i] as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] & !(1u16 << ((pin_bits & 0xFF) as u8));
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn quiz_on(cx: quiz_on::Context) {
        let quiz_on::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = OUT_QUIZ[i] as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn quiz_off(cx: quiz_off::Context) {
        let quiz_off::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = OUT_QUIZ[i] as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] & !(1u16 << ((pin_bits & 0xFF) as u8));
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn map_a_on(cx: map_a_on::Context) {
        let map_a_on::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = ANSWERS_MAP_A[i].unwrap() as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn map_a_off(cx: map_a_off::Context) {
        let map_a_off::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = ANSWERS_MAP_A[i].unwrap() as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] & !(1u16 << ((pin_bits & 0xFF) as u8));
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn map_b_on(cx: map_b_on::Context) {
        let map_b_on::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = ANSWERS_MAP_B[i].unwrap() as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn map_b_off(cx: map_b_off::Context) {
        let map_b_off::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = ANSWERS_MAP_B[i].unwrap() as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] & !(1u16 << ((pin_bits & 0xFF) as u8));
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn map_s_on(cx: map_s_on::Context) {
        let map_s_on::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = OUT_MAP_S[i] as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn map_s_off(cx: map_s_off::Context) {
        let map_s_off::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = OUT_MAP_S[i] as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] & !(1u16 << ((pin_bits & 0xFF) as u8));
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn map_n_on(cx: map_n_on::Context) {
        let map_n_on::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = OUT_MAP_N[i] as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn map_n_off(cx: map_n_off::Context) {
        let map_n_off::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..24).for_each(|i| {
                let pin_bits = OUT_MAP_N[i] as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output[chip] & !(1u16 << ((pin_bits & 0xFF) as u8));
                output[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        });
    }

    #[task(priority = 1, capacity = 4)]
    fn map_all_on(_cx: map_all_on::Context) {
        map_s_on::spawn().ok();
        map_n_on::spawn().ok();
    }

    #[task(priority = 1, capacity = 4)]
    fn map_all_off(_cx: map_all_off::Context) {
        map_s_off::spawn().ok();
        map_n_off::spawn().ok();
    }

    // When starting a NEW Quiz Game
    #[task(priority = 1, capacity = 4, shared = [answers_a, gmode, gdiff, game_on])]
    fn gamemode_quiz(cx: gamemode_quiz::Context) {
        let gamemode_quiz::SharedResources {
            answers_a,
            gmode,
            gdiff,
            game_on,
        } = cx.shared;

        (answers_a, gmode, game_on).lock(|answers_a, gmode, game_on| {
            *gmode = GameMode::Quiz;
            *game_on = false;
            *answers_a = ANSWERS_QUIZ;
        });

        debug::spawn("Gamemode QUIZ starting NEW game.\r\n".as_bytes()).ok();

        out_zero::spawn().ok();
        quiz_on::spawn().ok();
        click_relay01::spawn().ok();
        quiz_off::spawn_after(MEDIUM_TIME.millis()).ok();

        click_relay01::spawn_after(LONG_TIME.millis()).ok();
        quiz_on::spawn_after(LONG_TIME.millis()).ok();
        quiz_off::spawn_after((LONG_TIME + MEDIUM_TIME).millis()).ok();

        quiz_on::spawn_after((2 * LONG_TIME).millis()).ok();
        sockets_on::spawn_after((2 * LONG_TIME).millis()).ok();

        (gdiff,).lock(|gdiff| {
            if let GameDifficulty::Expert = *gdiff {
                set_output::spawn_after((2 * LONG_TIME).millis(), MyPin::Map47_Utsjoki).ok();
            }
        });
    }

    // When starting a NEW Map game
    //
    #[task(priority = 1, capacity = 4, shared = [answers_a, answers_b, gmode, gdiff, game_on])]
    fn gamemode_map(cx: gamemode_map::Context) {
        let gamemode_map::SharedResources {
            answers_a,
            answers_b,
            gmode,
            gdiff,
            game_on,
        } = cx.shared;

        (answers_a, answers_b, gmode, game_on).lock(|answers_a, answers_b, gmode, game_on| {
            *gmode = GameMode::MapQuizA;
            *game_on = false;
            *answers_a = ANSWERS_MAP_A;
            *answers_b = ANSWERS_MAP_B;
        });

        debug::spawn("Gamemode MAP starting new game.\r\n".as_bytes()).ok();

        out_zero::spawn().ok();
        map_a_on::spawn().ok();
        click_relay01::spawn().ok();
        map_a_off::spawn_after(MEDIUM_TIME.millis()).ok();

        click_relay01::spawn_after(LONG_TIME.millis()).ok();
        map_b_on::spawn_after(LONG_TIME.millis()).ok();
        map_b_off::spawn_after((LONG_TIME + MEDIUM_TIME).millis()).ok();

        set_output::spawn_after((2 * LONG_TIME).millis(), MyPin::Quiz01).ok();
        map_all_on::spawn_after((2 * LONG_TIME).millis()).ok();
        sockets_on::spawn_after((2 * LONG_TIME).millis()).ok();

        (gdiff,).lock(|gdiff| {
            if let GameDifficulty::Expert = *gdiff {
                set_output::spawn_after((2 * LONG_TIME).millis(), MyPin::Quiz03).ok();
            }
        });
    }

    // Switch between map B -> A
    #[task(priority = 1, capacity = 4, shared = [gmode, gdiff, answers_a])]
    fn gamemode_mapA(cx: gamemode_mapA::Context) {
        let gamemode_mapA::SharedResources {
            gmode,
            gdiff,
            answers_a,
            ..
        } = cx.shared;

        debug::spawn("Switching to map A.\r\n".as_bytes()).ok();

        quiz_off::spawn().ok();
        sockets_off::spawn().ok();
        click_relay01::spawn_after(MEDIUM_TIME.millis()).ok();
        click_relay01::spawn_after((2 * MEDIUM_TIME).millis()).ok();
        set_output::spawn_after((2 * LONG_TIME).millis(), MyPin::Quiz01).ok();

        (gmode, gdiff, answers_a).lock(|gmode, gdiff, answers_a| {
            *gmode = GameMode::MapQuizA;
            for socket in &OUT_SOCKET {
                let i = socket_index(*socket);

                if i.is_some() {
                    if answers_a[i.unwrap()].is_some() {
                        set_output::spawn(*socket).ok();
                    }
                }
            }
            if let GameDifficulty::Expert = *gdiff {
                set_output::spawn(MyPin::Quiz03).ok();
            }
        });
    }

    // Switch between map A -> B
    #[task(priority = 1, capacity = 4, shared = [gmode, gdiff, answers_b])]
    fn gamemode_mapB(cx: gamemode_mapB::Context) {
        let gamemode_mapB::SharedResources {
            gmode,
            gdiff,
            answers_b,
            ..
        } = cx.shared;

        debug::spawn("Switching to map B\r\n".as_bytes()).ok();

        quiz_off::spawn().ok();
        sockets_off::spawn().ok();
        click_relay01::spawn_after(MEDIUM_TIME.millis()).ok();
        click_relay01::spawn_after((2 * MEDIUM_TIME).millis()).ok();
        set_output::spawn_after((2 * LONG_TIME).millis(), MyPin::Quiz02).ok();

        (gmode, gdiff, answers_b).lock(|gmode, gdiff, answers_b| {
            *gmode = GameMode::MapQuizB;
            for socket in &OUT_SOCKET {
                let i = socket_index(*socket);

                if i.is_some() {
                    if answers_b[i.unwrap()].is_some() {
                        set_output::spawn(*socket).ok();
                    }
                }
            }
            if let GameDifficulty::Expert = *gdiff {
                set_output::spawn(MyPin::Quiz03).ok();
            }
        });
    }

    #[task(priority = 1, shared = [uart])]
    fn debug(cx: debug::Context, buf: &'static [u8]) {
        let debug::SharedResources { uart, .. } = cx.shared;

        (uart,).lock(|uart| {
            uart.write_full_blocking(buf);
        });
    }
}

// EOF
