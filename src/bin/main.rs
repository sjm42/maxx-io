// main.rs
#![no_std]
#![no_main]
#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_mut)]
#![allow(unused_variables)]

use defmt_rtt as _;
use panic_halt as _;

use core::fmt::{self, Write};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::*;
use port_expander_multi::{dev::pca9555::Driver, Direction, PortDriver, PortDriverTotemPole};
use rp_pico::hal::{
    self,
    gpio::{bank0::*, Function},
    timer::Alarm,
};
use rp_pico::{pac, XOSC_CRYSTAL_FREQ};
use shared_bus::BusMutex;
use systick_monotonic::Systick;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};
// USB Communications Class Device support
use usbd_serial::SerialPort;

use maxx_io::*;

// Blinck time
const SCAN_TIME_US: u32 = 2_000_000;

// Some type porn
type IO12 = hal::gpio::pin::Pin<Gpio12, Function<hal::gpio::pin::I2C>>;
type IO13 = hal::gpio::pin::Pin<Gpio13, Function<hal::gpio::pin::I2C>>;
type IO18 = hal::gpio::pin::Pin<Gpio18, Function<hal::gpio::pin::I2C>>;
type IO19 = hal::gpio::pin::Pin<Gpio19, Function<hal::gpio::pin::I2C>>;
type IO20 = hal::gpio::Pin<hal::gpio::pin::bank0::Gpio20, hal::gpio::Input<hal::gpio::PullUp>>;

type MyI2C0 = hal::I2C<pac::I2C0, (IO12, IO13)>;
type MyI2C1 = hal::I2C<pac::I2C1, (IO18, IO19)>;

type PortExpInner0 = shared_bus::NullMutex<Driver<MyI2C0>>;
type PortExpInner1 = shared_bus::NullMutex<Driver<MyI2C1>>;
type IoE0 = port_expander_multi::Pca9555<PortExpInner0>;
type IoE1 = port_expander_multi::Pca9555<PortExpInner1>;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OutTestState {
    Idle,
    Test1,
    Test2,
    Test3,
    Test4,
}

// type EParts = port_expander_multi::dev::pca9555::Parts<'static, MyI2C>;
// type IPin = port_expander_multi::Pin<'static, mode::Input, PortExpInner>;
// type OPin = port_expander_multi::Pin<'static, mode::Output, PortExpInner>;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [RTC_IRQ, XIP_IRQ, TIMER_IRQ_3, TIMER_IRQ_2])]
mod app {
    use crate::*;

    // IMPORTANT: The USB-Serial with RTIC github project example that I'm following.
    //            I tried to use the Pico board examples of USB-Serial (without interrupts
    //            and with interrupts with success, but when using with RTIC I could not make
    //            it work when merged with the RTIC example.) So I asked some questions
    //            in the in Matrix chat and received links to examples of there github
    //            project where it was working, then a used and adapted some parts there
    //            in this project template.
    //            This were the kind folks that helped me in the Matrix chat, the 3 projects
    //            that they suggest me to study are good examples of programs made with RTIC
    //            and USB and should be studied.
    //
    // Paul Daniel Faria
    // https://github.com/Nashenas88/dactyl-manuform-kb2040-rs/blob/main/src/main.rs#L80
    //
    // see also:
    // korken89
    // https://github.com/korken89/pico-probe/tree/master/src
    //
    // see also:
    // Mathias
    // https://github.com/mgottschlag/rp2040-usb-sound-card/blob/b8078b57361c1b08755e5ab5f9992c56457ec18b/src/main.rs#L188
    //
    //
    // Global Static variable, has to be written inside unsafe blocks.
    // A reference can be obtained with as_ref() method.

    #[shared]
    struct Shared {
        alarm: hal::timer::Alarm0,
        led: hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
        led_blink_enable: bool,

        serial: SerialPort<'static, hal::usb::UsbBus>,
        usb_dev: usb_device::device::UsbDevice<'static, hal::usb::UsbBus>,

        counter: Counter,

        irqc: u32,
        sw_pin: IO20,

        ioe0: Option<IoE0>,
        ioe1: Option<IoE1>,

        bits: [u16; 8],
        rise: [u16; 8],
        fall: [u16; 8],
        output: [u16; 8],
        tstate: OutTestState,
        zpend: bool,
        iopend: bool,
    }

    #[monotonic(binds = SysTick, default = true)]
    type SystickMono = Systick<100>;

    #[local]
    struct Local {}

    #[init(local = [usb_bus: Option<usb_device::bus::UsbBusAllocator<hal::usb::UsbBus>> = None])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        //*******
        // Initialization of the system clock.

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

        //*******
        // Initialization of the USB and Serial and USB Device ID.

        // USB
        //
        // Set up the USB driver
        // The bus that is used to manage the device and class below.
        let usb_bus: &'static _ =
            c.local
                .usb_bus
                .insert(UsbBusAllocator::new(hal::usb::UsbBus::new(
                    dp.USBCTRL_REGS,
                    dp.USBCTRL_DPRAM,
                    clocks.usb_clock,
                    true,
                    &mut resets,
                )));

        // Set up the USB Communications Class Device driver.
        let serial = SerialPort::new(usb_bus);

        // Create a USB device with a fake VID and PID
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();

        //*******
        // Initialization of the LED GPIO and the timer.

        let sio = hal::Sio::new(dp.SIO);
        let pins = rp_pico::Pins::new(dp.IO_BANK0, dp.PADS_BANK0, sio.gpio_bank0, &mut resets);
        let mut led = pins.led.into_push_pull_output();
        led.set_low().ok();

        // Enable led_blink.
        let led_blink_enable = false;

        // Reset the counter
        let counter = Counter::new();

        // Configure these pins as being I²C, not GPIO
        #[cfg(feature = "ioe0")]
        let sda0_pin = pins.gpio12.into_mode::<hal::gpio::FunctionI2C>();
        #[cfg(feature = "ioe0")]
        let scl0_pin = pins.gpio13.into_mode::<hal::gpio::FunctionI2C>();

        #[cfg(feature = "ioe0")]
        let i2c0 = hal::I2C::i2c0(
            dp.I2C0,
            sda0_pin,
            scl0_pin,
            100u32.kHz(),
            &mut resets,
            &clocks.system_clock,
        );

        #[cfg(feature = "ioe0")]
        let ioe0 = Some(port_expander_multi::Pca9555::new_m(i2c0));
        #[cfg(not(feature = "ioe0"))]
        let ioe0 = None;

        // Configure these pins as being I²C, not GPIO
        #[cfg(feature = "ioe1")]
        let sda1_pin = pins.gpio18.into_mode::<hal::gpio::FunctionI2C>();
        #[cfg(feature = "ioe1")]
        let scl1_pin = pins.gpio19.into_mode::<hal::gpio::FunctionI2C>();

        #[cfg(feature = "ioe1")]
        let i2c1 = hal::I2C::i2c1(
            dp.I2C1,
            sda1_pin,
            scl1_pin,
            100u32.kHz(),
            &mut resets,
            &clocks.system_clock,
        );
        #[cfg(feature = "ioe1")]
        let ioe1 = Some(port_expander_multi::Pca9555::new_m(i2c1));
        #[cfg(not(feature = "ioe1"))]
        let ioe1 = None;

        let mut bits = [0; 8];
        let rise = [0; 8];
        let fall = [0; 8];
        let output = [0; 8];

        // Setup ioe0 for outputs only with LOW state
        #[cfg(feature = "ioe0")]
        (0..=7).for_each(|i| {
            ioe0.as_ref().unwrap().0.lock(|drv| {
                drv.set_direction(i, 0xFFFF, Direction::Output, false).ok();
            })
        });

        // Setup ioe1 for inputs only
        #[cfg(feature = "ioe1")]
        (0..=7).for_each(|i| {
            ioe1.as_ref().unwrap().0.lock(|drv| {
                drv.set_direction(i, 0xFFFF, Direction::Input, false).ok();
            })
        });

        // read all input pins on ioe1 to clear _INT pin
        #[cfg(feature = "ioe1")]
        (0..=7).for_each(|i| {
            ioe1.as_ref().unwrap().0.lock(|drv| {
                bits[i as usize] = drv.read_u16(i).unwrap();
            })
        });

        // Deliver the interrupt from PCA9555s into gpio20
        let sw_pin = pins.gpio20.into_pull_up_input();

        let mut timer = hal::Timer::new(dp.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();

        /*
        alarm.schedule(SCAN_TIME_US.micros()).ok();
        alarm.enable_interrupt();
        */

        #[cfg(all(feature = "ioe0", feature = "test_output"))]
        test_output::spawn_after(1_000u64.millis()).ok();

        #[cfg(all(feature = "ioe1", feature = "io_irq"))]
        enable_io_irq::spawn_after(10_000u64.millis()).ok();

        #[cfg(all(feature = "ioe1", feature = "io_noirq"))]
        io_noirq::spawn_after(10_000u64.millis()).ok();

        alive::spawn().ok();

        //********
        // Return the Shared variables struct, the Local variables struct and the XPTO Monitonics
        //    (Note: Read again the RTIC book in the section of Monotonics timers)
        (
            Shared {
                alarm,
                led,
                led_blink_enable,

                serial,
                usb_dev,
                counter,

                irqc: 0,
                sw_pin,

                ioe0,
                ioe1,

                bits,
                rise,
                fall,
                output,
                tstate: OutTestState::Test1,
                zpend: false,
                iopend: false,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    // Task with least priority that only runs when nothing else is running.
    #[idle(local = [x: u32 = 0])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            *cx.local.x += 1;
        }
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

    #[task(
        priority = 1,
        shared = [serial, counter, irqc, sw_pin],
    )]
    fn alive(cx: alive::Context) {
        let mut buf = [0u8; 64];

        let alive::SharedResources {
            mut serial,
            counter,
            irqc,
            sw_pin,
            ..
        } = cx.shared;
        (counter, irqc, sw_pin, serial).lock(|counter_a, irqc_a, sw_pin_a, serial_a| {
            writeln!(
                Wrapper::new(&mut buf),
                "counter = {}, irqc = {}, swpin = {}\r",
                counter_a.get(),
                *irqc_a,
                sw_pin_a.is_high().unwrap() as u8
            )
            .ok();
            write_serial(
                serial_a,
                unsafe { core::str::from_utf8_unchecked(&buf) },
                false,
            );

            if counter_a.enabled() {
                counter_a.increment();
            }
        });
        alive::spawn_after(2000u64.millis()).ok();
    }

    #[task(priority = 1, capacity = 4, shared = [bits, rise, fall, serial])]
    fn io_report(cx: io_report::Context) {
        #[cfg(feature = "io_report")]
        {
            let mut buf = [0u8; 320];
            let mut change0 = false;
            let mut change1 = false;

            let io_report::SharedResources {
                bits,
                rise,
                fall,
                serial,
                ..
            } = cx.shared;

            (bits, rise, fall).lock(|_bits_a, rise_a, fall_a| {
                (0..=3).for_each(|i| {
                    if rise_a[i] != 0 || fall_a[i] != 0 {
                        change0 = true;
                    }
                });
                (4..=7).for_each(|i| {
                    if rise_a[i] != 0 || fall_a[i] != 0 {
                        change1 = true;
                    }
                });

                let mut w = Wrapper::new(&mut buf);
                if change0 {
                    writeln!(
                        w,
                        "Rise0: {:016b} {:016b} {:016b} {:016b}\r",
                        rise_a[0], rise_a[1], rise_a[2], rise_a[3]
                    )
                    .ok();
                    writeln!(
                        w,
                        "Fall0: {:016b} {:016b} {:016b} {:016b}\r\n\r",
                        fall_a[0], fall_a[1], fall_a[2], fall_a[3]
                    )
                    .ok();
                }
                if change1 {
                    writeln!(
                        w,
                        "Rise1: {:016b} {:016b} {:016b} {:016b}\r",
                        rise_a[4], rise_a[5], rise_a[6], rise_a[7]
                    )
                    .ok();
                    writeln!(
                        w,
                        "Fall1: {:016b} {:016b} {:016b} {:016b}\r\n\r",
                        fall_a[4], fall_a[5], fall_a[6], fall_a[7]
                    )
                    .ok();
                }
            });
            if !buf.is_empty() {
                (serial,).lock(|s| {
                    write_serial(s, unsafe { core::str::from_utf8_unchecked(&buf) }, false);
                });
            }
        }
    }

    #[task(priority = 1, capacity = 4, shared = [fall, irqc, serial])]
    fn input_id(cx: input_id::Context) {
        #[cfg(feature = "input_id")]
        {
            let mut buf = [0u8; 80];
            let mut change0 = false;
            let input_id::SharedResources {
                fall, irqc, serial, ..
            } = cx.shared;

            (fall, irqc).lock(|fall_a, irqc_a| {
                (0..=7).for_each(|i| {
                    if fall_a[i] != 0 {
                        change0 = true;
                    }
                });
                if change0 {
                    let mut w = Wrapper::new(&mut buf);
                    let (chip, bit) = which_bit(fall_a);
                    let pin = pin_input_ident(chip, bit);
                    writeln!(
                        w,
                        "Input: chip {chip} bit {bit} irqc: {} ident: {pin:?}\r\n\r",
                        *irqc_a
                    )
                    .ok();
                }
            });

            if !buf.is_empty() {
                (serial,).lock(|s| {
                    write_serial(s, unsafe { core::str::from_utf8_unchecked(&buf) }, false);
                });
            }
        }
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, ioe1, fall, serial, zpend])]
    fn in_to_out(cx: in_to_out::Context) {
        #[cfg(all(feature = "ioe0", feature = "ioe1", feature = "in_to_out"))]
        {
            let in_to_out::SharedResources {
                ioe0,
                ioe1,
                fall,
                zpend,
                serial,
                ..
            } = cx.shared;
            let mut fall0 = false;
            let mut buf = [0u8; 100];

            (ioe0, ioe1, fall).lock(|ioe0_a, ioe1_a, fall_a| {
                let ioe0 = ioe0_a.as_ref().unwrap();
                (0..=7).for_each(|i| {
                    if fall_a[i] != 0 {
                        fall0 = true;
                    }
                });
                if fall0 {
                    let mut w = Wrapper::new(&mut buf);
                    let (chip, bit) = which_bit(fall_a);
                    let pin = pin_input_ident(chip, bit);
                    if let MyPin::UnknownPin = pin {
                        // ignore unknowns
                    } else {
                        let pin_bits = pin.clone() as u32;
                        let chip = ((pin_bits & 0xFF00) >> 8) as u8;
                        let out = 1u16 << ((pin_bits & 0xFF) as u8);
                        writeln!(
                            w,
                            "i2o chip {chip} out {out:#016b} ident: {:#08x} {pin:?}\r\n\r",
                            pin_bits
                        )
                        .ok();
                        (serial,).lock(|s| {
                            write_serial(s, unsafe { core::str::from_utf8_unchecked(&buf) }, false);
                        });

                        // ioe0.0.lock(|drv| drv.write_u16(chip, out).ok());
                        set_output::spawn(pin.clone()).ok();
                        clear_output::spawn_after(3000u64.millis(), pin).ok();
                    }
                }
            });
        }
    }

    #[task(priority = 1, capacity = 4, shared = [ioe0, output])]
    fn set_output(cx: set_output::Context, pin: MyPin) {
        #[cfg(feature = "ioe0")]
        {
            let set_output::SharedResources { ioe0, output, .. } = cx.shared;
            let mut buf = [0u8; 100];

            (ioe0, output).lock(|ioe0_a, output_a| {
                let ioe0 = ioe0_a.as_ref().unwrap();
                let pin_bits = pin.clone() as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output_a[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                output_a[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        }
    }

    #[task(priority = 1, capacity = 16, shared = [ioe0, output])]
    fn clear_output(cx: clear_output::Context, pin: MyPin) {
        #[cfg(feature = "ioe0")]
        {
            let clear_output::SharedResources { ioe0, output, .. } = cx.shared;
            let mut buf = [0u8; 100];

            (ioe0, output).lock(|ioe0_a, output_a| {
                let ioe0 = ioe0_a.as_ref().unwrap();
                let pin_bits = pin.clone() as u32;
                let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                let out = output_a[chip] & ((1u16 << ((pin_bits & 0xFF) as u8)) ^ 0xFFFFu16);
                output_a[chip] = out;
                ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
            });
        }
    }

    #[task(priority = 1, capacity = 2, shared = [ioe0, output])]
    fn out_zero(cx: out_zero::Context) {
        #[cfg(feature = "ioe0")]
        {
            let out_zero::SharedResources { ioe0, output, .. } = cx.shared;
            (ioe0, output).lock(|ioe0_a, output_a| {
                let ioe0 = ioe0_a.as_ref().unwrap();
                (0..=7u8).for_each(|i| {
                    ioe0.0.lock(|drv| drv.write_u16(i, 0).ok());
                    output_a[i as usize] = 0;
                });
            });
        }
    }

    #[task(priority = 1, capacity = 2, shared = [ioe0, output])]
    fn out_ones(cx: out_ones::Context) {
        #[cfg(feature = "ioe0")]
        {
            let out_ones::SharedResources { ioe0, output, .. } = cx.shared;
            (ioe0, output).lock(|ioe0_a, output_a| {
                let ioe0 = ioe0_a.as_ref().unwrap();
                (0..=7).for_each(|i| {
                    ioe0.0.lock(|drv| drv.write_u16(i, 0xFFFF).ok());
                    output_a[i as usize] = 0xFFFF;
                });
            });
        }
    }

    #[task(priority = 1, capacity = 4, shared = [ioe1, bits, rise, fall])]
    fn io_poll(cx: io_poll::Context) {
        #[cfg(feature = "ioe1")]
        {
            let io_poll::SharedResources {
                ioe1,
                bits,
                rise,
                fall,
                ..
            } = cx.shared;
            let mut changed = false;
            let mut fallen = false;
            (ioe1, bits, rise, fall).lock(|ioe1_a, bits_a, rise_a, fall_a| {
                let ioe1 = ioe1_a.as_ref().unwrap();
                (0..=7).for_each(|i| {
                    ioe1.0.lock(|drv| {
                        let before = bits_a[i as usize];
                        let after = drv.read_u16(i).unwrap();
                        if after != before {
                            changed = true;
                            let fallen_bits = before & !after;
                            if fallen_bits != 0 {
                                fallen = true;
                            }
                            rise_a[i as usize] = !before & after;
                            fall_a[i as usize] = fallen_bits;
                            bits_a[i as usize] = after;
                        }
                    });
                });
            });

            if changed {
                #[cfg(feature = "io_report")]
                io_report::spawn().ok();
            }
            if fallen {
                #[cfg(feature = "input_id")]
                input_id::spawn().ok();
                #[cfg(feature = "in_to_out")]
                in_to_out::spawn().ok();
            }
        }
    }

    #[task(priority = 1, shared = [ioe0, output, tstate])]
    fn test_output(cx: test_output::Context) {
        #[cfg(all(feature = "ioe0", feature = "test_output"))]
        {
            let test_output::SharedResources {
                ioe0,
                output,
                tstate,
                ..
            } = cx.shared;
            let mut active = true;
            (ioe0, output, tstate).lock(|ioe0_a, output_a, tstate_a| {
                let ioe0 = ioe0_a.as_ref().unwrap();
                let test = match *tstate_a {
                    OutTestState::Idle => {
                        &OUT_TEST1 // not used
                    }
                    OutTestState::Test1 => &OUT_TEST1,
                    OutTestState::Test2 => &OUT_TEST2,
                    OutTestState::Test3 => &OUT_TEST3,
                    OutTestState::Test4 => &OUT_TEST3,
                };
                match *tstate_a {
                    OutTestState::Idle => {
                        active = false;
                    }
                    OutTestState::Test1 => {
                        *tstate_a = OutTestState::Test2;
                        out_zero::spawn_after(2900u64.millis()).ok();
                    }
                    OutTestState::Test2 => {
                        *tstate_a = OutTestState::Test3;
                        out_zero::spawn_after(2900u64.millis()).ok();
                    }
                    OutTestState::Test3 => {
                        *tstate_a = OutTestState::Test4;
                        out_zero::spawn_after(2900u64.millis()).ok();
                    }
                    OutTestState::Test4 => {
                        *tstate_a = OutTestState::Idle;
                        // out_zero::spawn_after(2900u64.millis()).ok();
                    }
                }

                if active {
                    (0..24).for_each(|i| {
                        let pin_bits = test[i].clone() as u32;
                        let chip = ((pin_bits & 0xFF00) >> 8) as usize;
                        let out = output_a[chip] | 1u16 << ((pin_bits & 0xFF) as u8);
                        output_a[chip] = out;
                        ioe0.0.lock(|drv| drv.write_u16(chip as u8, out).ok());
                    });
                    test_output::spawn_after(3000u64.millis()).ok();
                }
            });
        }
    }

    #[task(priority = 2)]
    fn io_noirq(cx: io_noirq::Context) {
        #[cfg(feature = "ioe1")]
        io_poll::spawn().ok();

        io_noirq::spawn_after(100u64.millis()).ok();
    }

    #[task(priority = 2, capacity = 4, shared = [sw_pin])]
    fn enable_io_irq(cx: enable_io_irq::Context) {
        #[cfg(feature = "ioe1")]
        io_poll::spawn().ok();
        let enable_io_irq::SharedResources { sw_pin, .. } = cx.shared;
        (sw_pin,).lock(|sw_pin_a| {
            sw_pin_a.clear_interrupt(hal::gpio::Interrupt::LevelLow);
            sw_pin_a.set_interrupt_enabled(hal::gpio::Interrupt::LevelLow, true);
        });
    }

    #[task(
        binds = IO_IRQ_BANK0,
        priority = 3,
        shared = [irqc, sw_pin],
    )]
    fn io_irq(cx: io_irq::Context) {
        let io_irq::SharedResources { irqc, sw_pin, .. } = cx.shared;

        (irqc, sw_pin).lock(|irqc_a, sw_pin_a| {
            sw_pin_a.clear_interrupt(hal::gpio::Interrupt::LevelLow);
            sw_pin_a.set_interrupt_enabled(hal::gpio::Interrupt::LevelLow, false);
            sw_pin_a.clear_interrupt(hal::gpio::Interrupt::LevelLow);
            *irqc_a += 1;
        });
        enable_io_irq::spawn_after(200u64.millis()).ok();

        #[cfg(feature = "ioe1")]
        io_poll::spawn().ok();
    }

    // Task that blinks the rp-pico onboard LED and that send a message "LED ON!" and "LED OFF!" do USB-Serial.
    #[task(
        binds = TIMER_IRQ_0,
        priority = 2,
        shared = [alarm, led, led_blink_enable, serial, counter, irqc, sw_pin],
        local = [tog: bool = true],
    )]
    fn timer_irq(cx: timer_irq::Context) {
        let mut buf = [0u8; 64];

        let timer_irq::SharedResources {
            mut alarm,
            led,
            led_blink_enable,
            mut serial,
            counter,
            irqc,
            sw_pin,
            ..
        } = cx.shared;
        let tog = cx.local.tog;

        // Blinks the LED ON / OFF.
        (led, led_blink_enable, counter, irqc, sw_pin).lock(
            |led_a, led_blink_enable_a, counter_a, irqc_a, sw_pin_a| {
                let led_state_str: &str;
                if *led_blink_enable_a {
                    if *tog {
                        led_a.set_high().ok();
                        led_state_str = "ON ";
                    } else {
                        led_a.set_low().ok();
                        led_state_str = "OFF";
                    }
                    writeln!(
                        Wrapper::new(&mut buf),
                        "LED {}!   counter = {}, irqc = {}, swpin = {}\r",
                        led_state_str,
                        counter_a.get(),
                        *irqc_a,
                        sw_pin_a.is_high().unwrap() as u8
                    )
                    .ok();
                }
                if counter_a.enabled() {
                    counter_a.increment();
                }

                if *led_blink_enable_a {
                    *tog = !*tog;
                }
            },
        );

        // Clears the timer interrupt and Set's the new delta_time in the future.
        (alarm).lock(|a| {
            a.clear_interrupt();
            let _ = a.schedule(SCAN_TIME_US.micros());
        });

        serial.lock(|s| {
            write_serial(s, unsafe { core::str::from_utf8_unchecked(&buf) }, true);
        });
    }

    // Usb interrupt handler. Runs every time the host requests new data.
    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [led, led_blink_enable, serial, usb_dev, counter])]
    fn usb_rx(cx: usb_rx::Context) {
        let usb_rx::SharedResources {
            led,
            led_blink_enable,
            serial,
            usb_dev,
            counter,
            ..
        } = cx.shared;

        (led, led_blink_enable, usb_dev, serial, counter).lock(
            |led_a, led_blink_enable_a, usb_dev_a, serial_a, counter_a| {
                // Check for new data
                if usb_dev_a.poll(&mut [serial_a]) {
                    let mut buf = [0u8; 64];
                    match serial_a.read(&mut buf) {
                        Err(_e) => {
                            // Do nothing
                        }
                        Ok(0) => {
                            // Do nothing
                            // serial_a.write(b"Didn't receive data.").ok();
                            // serial_a.flush().ok();
                        }
                        Ok(_count) => {
                            match_usb_serial_buf(
                                &buf,
                                led_a,
                                led_blink_enable_a,
                                serial_a,
                                counter_a,
                            );
                        }
                    }
                }
            },
        );
    }

    /* New Tasks */

    // This function come from the github with USB-Serial example (see link above).
    // Helper function to ensure all data is written across the serial interface.
    fn write_serial(serial: &mut SerialPort<'static, hal::usb::UsbBus>, buf: &str, block: bool) {
        let write_ptr = buf.as_bytes();

        // Because the buffer is of constant size and initialized to zero (0) we here
        // add a test to determine the size that's really occupied by the str that we
        // wan't to send. From index zero to first byte that is as the zero byte value.
        let mut index = 0;
        while index < write_ptr.len() && write_ptr[index] != 0 {
            index += 1;
        }
        let mut write_ptr = &write_ptr[0..index];

        while !write_ptr.is_empty() {
            match serial.write(write_ptr) {
                Ok(len) => write_ptr = &write_ptr[len..],
                // Meaning the USB write buffer is full
                Err(UsbError::WouldBlock) => {
                    if !block {
                        break;
                    }
                }
                // On error, just drop unwritten data.
                Err(_) => break,
            }
        }
        serial.flush().ok();
    }

    fn match_usb_serial_buf(
        buf: &[u8; 64],
        led: &mut hal::gpio::Pin<hal::gpio::pin::bank0::Gpio25, hal::gpio::PushPullOutput>,
        led_blink_enable: &mut bool,
        serial: &mut SerialPort<'static, hal::usb::UsbBus>,
        counter: &mut Counter,
    ) {
        let _buf_len = buf.len();
        match buf[0] {
            // Print Menu
            b'M' | b'm' => {
                write_serial(serial, "M - Print Menu\r\n", false);
                print_menu(serial);
            }
            // 0 - Reset counter
            b'0' => {
                write_serial(serial, "M - Print Menu\r\n", false);
                counter.reset();
            }
            // 1 - Increment counter
            b'1' => {
                write_serial(serial, "1 - Increment counter\r\n", false);
                counter.increment();
            }
            // 2 - Start continues counter
            b'2' => {
                write_serial(serial, "2 - Start continues counter\r\n", false);
                counter.set_enable(true);
            }
            // 3 - Stop continues counter
            b'3' => {
                write_serial(serial, "3 - Stop continues counter\r\n", false);
                counter.set_enable(false);
            }
            // 4 - Get switch and LED state
            b'4' => {
                write_serial(serial, "4 - Get switch and LED state\r\n", false);

                // GPIO 25 onboard LED, we are going to read the bit 8 of the gpio_status register.
                //  OUTFROMPERI - output signal from selected peripheral, before register
                //                override is applied.
                // See pag 272 of the Pico Datasets:
                // https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf#_gpio_functions

                let led_status_reg =
                    unsafe { (*pac::IO_BANK0::ptr()).gpio[25].gpio_status.read().bits() };

                // Reserved bit.
                // let sio_pin_value = unsafe { (*pac::SIO::ptr()).gpio_out.read().bits() };

                let (led_bool, led_status) = if ((led_status_reg & 1 << 8) >> 8) == 1_u32 {
                    (true, "ON")
                } else {
                    (false, "OFF")
                };

                let mut buf = [0u8; 64];
                let _ = writeln!(
                    Wrapper::new(&mut buf),
                    "LED Status {:b}, {}   LED {}\r",
                    led_status_reg,
                    led_bool,
                    led_status
                );
                write_serial(
                    serial,
                    unsafe { core::str::from_utf8_unchecked(&buf) },
                    true,
                );

                // unsafe { (*pac::TIMER::ptr()).timerawh.read().bits() };
            }
            // 5 - Set LED on
            b'5' => {
                write_serial(serial, "5 - Set LED on\r\n", false);
                *led_blink_enable = false;
                let _ = led.set_high();
            }
            // 6 - Set LED off
            b'6' => {
                write_serial(serial, "6 - Set LED off\r\n", false);
                *led_blink_enable = false;
                let _ = led.set_low();
            }
            // 7 - Set LED blink enable
            b'7' => {
                write_serial(serial, "7 - Set LED blink enable\r\n", false);
                *led_blink_enable = true;
            }
            b'8' => {
                write_serial(serial, "8 - Display data rate\r\n", false);

                let data_rate = serial.line_coding().data_rate();
                let mut buf = [0u8; 64];
                let _ = writeln!(Wrapper::new(&mut buf), "Data rate: {} bit/s\r", data_rate);
                write_serial(
                    serial,
                    unsafe { core::str::from_utf8_unchecked(&buf) },
                    true,
                );
            }
            _ => {
                write_serial(serial, unsafe { core::str::from_utf8_unchecked(buf) }, true);
                write_serial(serial, "Invalid option!\r\n", true);
            }
        }
    }

    fn print_menu(serial: &mut SerialPort<'static, hal::usb::UsbBus>) {
        // Create the Menu.
        let menu_str = "*****************\r
*  Menu:\r
*\r
*  M / m - Print menu\r
*    0   - Reset counter\r
*    1   - Increment counter\r
*    2   - Start continues counter\r
*    3   - Stop continues counter\r
*    4   - Get switch and LED state\r
*    5   - Set LED on\r
*    6   - Set LED off\r
*    7   - Set LED blink enable\r
*    8   - Display data rate\r
*****************\r
Enter option: ";

        write_serial(serial, menu_str, true);
    }
}

// EOF
