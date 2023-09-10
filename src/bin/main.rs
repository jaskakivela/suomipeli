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
    Test1,
    Test2,
    Test3,
    Test4,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EasterEggState {
    Start,
    Phase1,
    Phase2,
    Phase3,
    Go,
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

        rand: bool,
        rng: Option<StdRng>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SystickMono = Systick<100>;

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
        writeln!(
            Wrapper::new(&mut buf),
            "*** Starting up Suomipeli\r\n\
            \x20   firmware v{}\r\n\r",
            env!("CARGO_PKG_VERSION")
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
        test_output::spawn_after(1_000u64.millis()).ok();

        #[cfg(feature = "io_irq")]
        enable_io_irq::spawn_after(10_000u64.millis()).ok();

        #[cfg(feature = "io_noirq")]
        io_noirq::spawn_after(10_000u64.millis()).ok();

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

                tstate: OutTestState::Test1,
                estate: EasterEggState::Start,

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
        led_blink::spawn_after(1000u64.millis()).ok();
    }

    #[task(priority = 1, shared = [irqc, sw_pin, uart])]
    fn alive(cx: alive::Context) {
        let alive::SharedResources {
            irqc, sw_pin, uart, ..
        } = cx.shared;

        let mut buf = [0u8; 64];

        (irqc, sw_pin).lock(|irqc, sw_pin| {
            writeln!(
                Wrapper::new(&mut buf),
                "irqc = {}, swpin = {}\r",
                *irqc,
                sw_pin.is_high().unwrap() as u8
            )
            .ok();
        });

        (uart,).lock(|uart| {
            uart.write_full_blocking(&buf);
        });

        alive::spawn_after(5000u64.millis()).ok();
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

        let mut buf = [0u8; 320];
        let mut change0 = false;
        let mut change1 = false;

        (bits, rise, fall).lock(|_bits, rise, fall| {
            (0..=3).for_each(|i| {
                if rise[i] != 0 || fall[i] != 0 {
                    change0 = true;
                }
            });
            (4..=7).for_each(|i| {
                if rise[i] != 0 || fall[i] != 0 {
                    change1 = true;
                }
            });

            let mut w = Wrapper::new(&mut buf);
            if change0 {
                writeln!(
                    w,
                    "Rise0: {:016b} {:016b} {:016b} {:016b}\r",
                    rise[0], rise[1], rise[2], rise[3]
                )
                .ok();
                writeln!(
                    w,
                    "Fall0: {:016b} {:016b} {:016b} {:016b}\r\n\r",
                    fall[0], fall[1], fall[2], fall[3]
                )
                .ok();
            }
            if change1 {
                writeln!(
                    w,
                    "Rise1: {:016b} {:016b} {:016b} {:016b}\r",
                    rise[4], rise[5], rise[6], rise[7]
                )
                .ok();
                writeln!(
                    w,
                    "Fall1: {:016b} {:016b} {:016b} {:016b}\r\n\r",
                    fall[4], fall[5], fall[6], fall[7]
                )
                .ok();
            }
        });

        if !buf.is_empty() {
            (uart,).lock(|uart| {
                uart.write_full_blocking(&buf);
            });
        }
    }

    #[task(priority = 1, capacity = 4, shared = [fall, estate, irqc, uart])]
    fn input_event(cx: input_event::Context) {
        let input_event::SharedResources {
            fall,
            estate,
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

        if edge_low {
            let pin = pin_input_ident(chip, bit);

            #[cfg(feature = "input_debug")]
            {
                let mut buf = [0u8; 80];
                let mut w = Wrapper::new(&mut buf);

                (irqc,).lock(|irqc| {
                    writeln!(
                        w,
                        "Input: chip {chip} bit {bit} irqc: {} ident: {pin:?}\r\n\r",
                        *irqc
                    )
                    .ok();
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
                    *estate = EasterEggState::Start;
                    easter_egg::spawn(pin.clone()).ok();
                }
            });

            if let MyPin::UnknownPin = pin {
                // ignore unknowns
            } else {
                set_output::spawn(pin.clone()).ok();
                clear_output::spawn_after(5000u64.millis(), pin.clone()).ok();
            }
        }
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

        // We read all input pin states and mark changes since last check
        (ioe1, bits, rise, fall).lock(|ioe1, bits, rise, fall| {
            (0..=7).for_each(|i| {
                ioe1.0.lock(|drv| {
                    let before = bits[i as usize];
                    let after = drv.read_u16(i).unwrap();
                    if after != before {
                        changed = true;
                        let risen_bits = !before & after;
                        let fallen_bits = before & !after;
                        if fallen_bits != 0 {
                            fallen = true;
                        }
                        rise[i as usize] = risen_bits;
                        fall[i as usize] = fallen_bits;
                        bits[i as usize] = after;
                    }
                });
            });
        });

        #[cfg(feature = "io_debug")]
        if changed {
            io_debug::spawn().ok();
        }

        if fallen {
            // We only react to falling edges, since inputs are pull-up
            input_event::spawn().ok();
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
                    OutTestState::Test1 => {
                        *tstate = OutTestState::Test2;
                        out_zero::spawn_after(900u64.millis()).ok();
                        (&OUT_TEST1, true)
                    }
                    OutTestState::Test2 => {
                        *tstate = OutTestState::Test3;
                        out_zero::spawn_after(900u64.millis()).ok();
                        (&OUT_TEST2, true)
                    }
                    OutTestState::Test3 => {
                        *tstate = OutTestState::Test4;
                        out_zero::spawn_after(900u64.millis()).ok();
                        (&OUT_TEST3, true)
                    }
                    OutTestState::Test4 => {
                        *tstate = OutTestState::Idle;
                        out_zero::spawn_after(900u64.millis()).ok();
                        (&OUT_TEST4, true)
                    }
                    OutTestState::Idle => (&OUT_TEST4, false),
                };

                if active {
                    (0..24).for_each(|i| {
                        let pin_bits = test[i].clone() as u32;
                        let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                        let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                        output[chip] = out;
                        ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
                    });
                    test_output::spawn_after(1000u64.millis()).ok();
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
                p => {
                    // Blink the lamp for 0.5 seconds
                    set_output::spawn(p.clone()).ok();
                    clear_output::spawn_after(500u64.millis(), p).ok();
                }
            }
        });
        (rand,).lock(|rand| {
            if *rand {
                rand_output::spawn_after(50u64.millis()).ok();
            }
        });
    }

    #[task(priority = 2)]
    fn io_noirq(_cx: io_noirq::Context) {
        io_poll::spawn().ok();
        io_noirq::spawn_after(100u64.millis()).ok();
    }

    #[task(priority = 2, capacity = 4, shared = [sw_pin])]
    fn enable_io_irq(cx: enable_io_irq::Context) {
        let enable_io_irq::SharedResources { sw_pin, .. } = cx.shared;

        io_poll::spawn().ok();

        (sw_pin,).lock(|sw_pin| {
            sw_pin.clear_interrupt(hal::gpio::Interrupt::LevelLow);
            sw_pin.set_interrupt_enabled(hal::gpio::Interrupt::LevelLow, true);
        });
    }

    #[task(binds = IO_IRQ_BANK0, priority = 3, shared = [irqc, sw_pin])]
    fn io_irq(cx: io_irq::Context) {
        let io_irq::SharedResources { irqc, sw_pin, .. } = cx.shared;

        // Clear & disable our interrupt immediately to avoid repeated firing
        (irqc, sw_pin).lock(|irqc, sw_pin| {
            sw_pin.clear_interrupt(hal::gpio::Interrupt::LevelLow);
            sw_pin.set_interrupt_enabled(hal::gpio::Interrupt::LevelLow, false);
            sw_pin.clear_interrupt(hal::gpio::Interrupt::LevelLow);
            *irqc += 1;
        });

        // We are using 200ms debounce period
        enable_io_irq::spawn_after(200u64.millis()).ok();

        // Here we actually check what had changed in input pins
        io_poll::spawn().ok();
    }

    #[task(priority = 1, capacity = 4, shared = [rand, ioe0, output])]
    fn easter_egg(cx: easter_egg::Context, pin: MyPin) {
        let easter_egg::SharedResources {
            rand, ioe0, output, ..
        } = cx.shared;

        let (test, test_active) = match pin {
            MyPin::Quiz01 => (&OUT_TEST1, true),
            MyPin::Quiz02 => (&OUT_TEST2, true),
            MyPin::Quiz03 => (&OUT_TEST3, true),
            MyPin::Quiz04 => (&OUT_TEST4, true),
            MyPin::Quiz05 => {
                out_zero::spawn().ok();
                (&OUT_TEST1, false)
            }
            MyPin::Quiz23 => {
                (rand,).lock(|rand| {
                    *rand = true;
                    rand_output::spawn().ok();
                });
                (&OUT_TEST1, false)
            }
            MyPin::Quiz24 => {
                (rand,).lock(|rand| {
                    *rand = false;
                });
                (&OUT_TEST1, false)
            }
            _ => (&OUT_TEST1, false),
        };

        if test_active {
            (output, ioe0).lock(|output, ioe0| {
                (0..24).for_each(|i| {
                    let pin_bits = test[i].clone() as u32;
                    let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                    let out = output[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                    output[chip] = out;
                    ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
                });
            });
        }
    }

    #[task(priority = 1, capacity = 8, shared = [ioe0, output])]
    fn set_output(cx: set_output::Context, pin: MyPin) {
        let set_output::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            let pin_bits = pin.clone() as u32;
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
            let pin_bits = pin.clone() as u32;
            let chip = ((pin_bits & 0xFF00) >> 8) as usize;
            let out = output[chip] ^ (1u16 << ((pin_bits & 0xFF) as u8));
            output[chip] = out;
            ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
        });
    }

    #[task(priority = 1, capacity = 64, shared = [ioe0, output])]
    fn clear_output(cx: clear_output::Context, pin: MyPin) {
        let clear_output::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            let pin_bits = pin.clone() as u32;
            let chip = ((pin_bits & 0xFF00) >> 8) as usize;
            let out = output[chip] & (!(1u16 << ((pin_bits & 0xFF) as u8)));
            output[chip] = out;
            ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
        });
    }

    #[task(priority = 1, capacity = 2, shared = [ioe0, output])]
    fn out_zero(cx: out_zero::Context) {
        let out_zero::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..=7u8).for_each(|i| {
                ioe0.0.lock(|drv| drv.write_u16(i, 0).ok());
                output[i as usize] = 0;
            });
        });
    }

    #[task(priority = 1, capacity = 2, shared = [ioe0, output])]
    fn out_ones(cx: out_ones::Context) {
        let out_ones::SharedResources { ioe0, output, .. } = cx.shared;
        (ioe0, output).lock(|ioe0, output| {
            (0..=7).for_each(|i| {
                ioe0.0.lock(|drv| drv.write_u16(i, 0xFFFF).ok());
                output[i as usize] = 0xFFFF;
            });
        });
    }
}

// EOF
