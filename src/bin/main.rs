// main.rs

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_halt as _;

use core::fmt::{self, Write};
use embedded_hal::digital::v2::{InputPin, OutputPin};
use fugit::*;
use port_expander_multi::{dev::pca9555::Driver, Direction, PortDriver, PortDriverTotemPole};
use rp_pico::hal::{
    self,
    gpio::{bank0::Gpio18, bank0::Gpio19, Function},
    timer::Alarm,
};
use rp_pico::{pac, XOSC_CRYSTAL_FREQ};
use shared_bus::BusMutex;
use systick_monotonic::Systick;

// USB Device support
use usb_device::{class_prelude::*, prelude::*};
// USB Communications Class Device support
use usbd_serial::SerialPort;

// Blinck time
const SCAN_TIME_US: u32 = 2_000_000;

// Some type porn
type IO18 = hal::gpio::pin::Pin<Gpio18, Function<hal::gpio::pin::I2C>>;
type IO19 = hal::gpio::pin::Pin<Gpio19, Function<hal::gpio::pin::I2C>>;
type MyI2C = hal::I2C<pac::I2C1, (IO18, IO19)>;

type PortExpInner = shared_bus::NullMutex<Driver<MyI2C>>;
type IoE = port_expander_multi::Pca9555<PortExpInner>;

type IO20 = hal::gpio::Pin<hal::gpio::pin::bank0::Gpio20, hal::gpio::Input<hal::gpio::PullUp>>;

// type EParts = port_expander_multi::dev::pca9555::Parts<'static, MyI2C>;
// type IPin = port_expander_multi::Pin<'static, mode::Input, PortExpInner>;
// type OPin = port_expander_multi::Pin<'static, mode::Output, PortExpInner>;

pub struct Wrapper<'a> {
    buf: &'a mut [u8],
    offset: usize,
}

impl<'a> Wrapper<'a> {
    pub(crate) fn new(buf: &'a mut [u8]) -> Self {
        Wrapper { buf, offset: 0 }
    }
}

impl<'a> fmt::Write for Wrapper<'a> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();

        // Skip over already-copied data.
        let remainder = &mut self.buf[self.offset..];
        // Check if there is space remaining (return error instead of panicking).
        if remainder.len() < bytes.len() {
            return Err(core::fmt::Error);
        }

        // Make the two slices the same length.
        let remainder = &mut remainder[..bytes.len()];
        // Copy.
        remainder.copy_from_slice(bytes);

        // Update offset to avoid overwriting.
        self.offset += bytes.len();

        Ok(())
    }
}

pub struct Counter {
    counter: u32,
    enable: bool,
}

impl Counter {
    fn new() -> Self {
        Counter {
            counter: 0_u32,
            enable: true,
        }
    }
    fn get(&self) -> u32 {
        self.counter
    }
    fn reset(&mut self) {
        self.counter = 0_u32;
    }
    fn increment(&mut self) {
        self.counter += 1_u32;
    }
    fn enable(&mut self, state: bool) {
        self.enable = state;
    }
}

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
        ioe: IoE,
        bits: [u16; 8],
        rise: [u16; 8],
        fall: [u16; 8],
        zpend: bool,
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
        let led_blink_enable = true;

        // Reset the counter
        let counter = Counter::new();

        // Configure two pins as being I²C, not GPIO
        let sda_pin = pins.gpio18.into_mode::<hal::gpio::FunctionI2C>();
        let scl_pin = pins.gpio19.into_mode::<hal::gpio::FunctionI2C>();

        let i2c = hal::I2C::i2c1(
            dp.I2C1,
            sda_pin,
            scl_pin,
            100u32.kHz(),
            &mut resets,
            &clocks.system_clock,
        );
        let ioe = port_expander_multi::Pca9555::new_m(i2c);

        let bits = [0; 8];
        let rise = [0; 8];
        let fall = [0; 8];
        /*
        (0..=7).for_each(|i| {
            ioe.0.lock(|drv| {
                drv.set_direction(i, 0xFFFF, Direction::Input, false).ok();
                bits[i as usize] = drv.read_u16(i).unwrap();
            })
        });
        */

        // Deliver the interrupt from PCA9555s into gpio20
        let sw_pin = pins.gpio20.into_pull_up_input();

        let mut timer = hal::Timer::new(dp.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        alarm.schedule(SCAN_TIME_US.micros()).ok();
        alarm.enable_interrupt();

        test_input::spawn_after(5000u64.millis()).ok();
        // test_output::spawn_after(1000u64.millis()).ok();
        // test_in_out::spawn_after(1000u64.millis()).ok();

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
                ioe,
                bits,
                rise,
                fall,
                zpend: false,
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
            // cortex_m::asm::wfe();
        }
    }

    #[task(priority = 1, capacity = 4, shared = [bits, rise, fall, serial])]
    fn io_report(cx: io_report::Context) {
        let mut buf = [0u8; 320];
        let mut change0 = false;
        let mut change1 = false;

        let io_report::SharedResources {
            bits,
            rise,
            fall,
            serial,
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
            /*
            if change0 || change1 {
                let o = w.offset;
                writeln!(w, "{}\r", o).ok();
            }
            */
        });
        if !buf.is_empty() {
            (serial,).lock(|s| {
                write_serial(s, unsafe { core::str::from_utf8_unchecked(&buf) }, true);
            });
        }
        // cortex_m::asm::sev();
    }

    #[task(priority = 1, capacity = 4, shared = [ioe, bits, rise, fall, zpend])]
    fn io_poll(cx: io_poll::Context) {
        let io_poll::SharedResources {
            ioe,
            bits,
            rise,
            fall,
            zpend,
        } = cx.shared;
        (ioe, bits, rise, fall).lock(|ioe_a, bits_a, rise_a, fall_a| {
            (0..=7).for_each(|i| {
                ioe_a.0.lock(|drv| {
                    let before = bits_a[i as usize];
                    let after = drv.read_u16(i).unwrap();
                    rise_a[i as usize] = !before & after;
                    fall_a[i as usize] = before & !after;
                    bits_a[i as usize] = after;
                });
            });

            // Indicate input findings on the last 2 chips outputs
            /*
            let mut out = 0;
            (0..=5).for_each(|i| {
                let b = rise_a[i];
                if b != 0 {
                    out = if b & 0xFF00 != 0 {
                        b & 0xFF00 | b >> 8
                    } else {
                        b << 8 | b & 0x00FF
                    };
                }
            });
            if out != 0 {
                (6..=7).for_each(|i| {
                    ioe_a.0.lock(|drv| drv.write_u16(i, out).unwrap());
                });
            }
            */
        });

        // only have one spawn pending
        (zpend,).lock(|zpend_a| {
            if !*zpend_a {
                *zpend_a = true;
                // out_zero::spawn_after(1000u64.millis()).ok();
            }
        });

        io_report::spawn().ok();
    }

    #[task(priority = 1, capacity = 4, shared = [ioe, bits, rise, fall, zpend])]
    fn out_zero(cx: out_zero::Context) {
        let out_zero::SharedResources {
            ioe,
            bits,
            rise,
            fall,
            zpend,
        } = cx.shared;
        (ioe, bits, rise, fall).lock(|ioe_a, bits_a, rise_a, fall_a| {
            (6..=7).for_each(|i| {
                ioe_a.0.lock(|drv| drv.write_u16(i as u8, 0).unwrap());
                bits_a[i] = 0;
                rise_a[i] = 0;
                fall_a[i] = 0;
            });
        });
        (zpend,).lock(|zpend_a| {
            *zpend_a = false;
        });
        // cortex_m::asm::sev();
    }

    #[task(priority = 1, shared = [ioe, bits])]
    fn test_input(cx: test_input::Context) {
        let test_input::SharedResources { ioe, bits } = cx.shared;
        (ioe, bits).lock(|ioe_a, bits_a| {
            (0..=7).for_each(|i| {
                ioe_a.0.lock(|drv| {
                    drv.set_direction(i, 0xFFFF, Direction::Input, false).ok();
                })
            });
            (0..=7).for_each(|i| {
                ioe_a.0.lock(|drv| {
                    bits_a[i as usize] = drv.read_u16(i).unwrap();
                })
            });
        });
        enable_irq::spawn().ok();
    }

    #[task(priority = 1, shared = [ioe, bits])]
    fn test_in_out(cx: test_in_out::Context) {
        let test_in_out::SharedResources { ioe, bits } = cx.shared;
        (ioe, bits).lock(|ioe_a, bits_a| {
            (0..=5).for_each(|i| {
                ioe_a.0.lock(|drv| {
                    drv.set_direction(i, 0xFFFF, Direction::Input, false).ok();
                })
            });
            (6..=7).for_each(|i| {
                ioe_a.0.lock(|drv| {
                    drv.set_direction(i, 0xFFFF, Direction::Output, false).ok();
                })
            });
            (0..=7).for_each(|i| {
                ioe_a.0.lock(|drv| {
                    bits_a[i as usize] = drv.read_u16(i).unwrap();
                })
            });
        });
        enable_irq::spawn().ok();
    }

    #[task(priority = 1, shared = [ioe], local = [init: bool = true, index: u8 = 0, bit: u8 = 0])]
    fn test_output(cx: test_output::Context) {
        let test_output::SharedResources { ioe } = cx.shared;
        let test_output::LocalResources { init, index, bit } = cx.local;

        let data0 = !(1u16 << *bit);
        let data1 = !(0x8000u16 >> *bit);
        (ioe,).lock(|ioe_a| {
            if *init {
                (0..=7).for_each(|i| {
                    ioe_a.0.lock(|drv| {
                        drv.set_direction(i, 0xFFFF, Direction::Output, false).ok();
                    })
                });
                *init = false;
            }

            ioe_a.0.lock(|drv| {
                drv.write_u16(0, data0).ok();
                drv.write_u16(1, data1).ok();
                drv.write_u16(2, data0).ok();
                drv.write_u16(3, data1).ok();
                drv.write_u16(4, data0).ok();
                drv.write_u16(5, data1).ok();
                drv.write_u16(6, data0).ok();
                drv.write_u16(7, data1).ok();
            });
        });

        if *bit < 15 {
            *bit += 1;
        } else {
            *bit = 0;
            *index += 1;
        }
        if *index == 8 {
            *index = 0;
        }

        test_output::spawn_after(200u64.millis()).ok();
        // cortex_m::asm::sev();
    }

    #[task(priority = 1, capacity = 2, shared = [sw_pin])]
    fn enable_irq(cx: enable_irq::Context) {
        let enable_irq::SharedResources { sw_pin } = cx.shared;
        (sw_pin,).lock(|sw_pin_a| {
            sw_pin_a.clear_interrupt(hal::gpio::Interrupt::LevelLow);
            sw_pin_a.set_interrupt_enabled(hal::gpio::Interrupt::LevelLow, true);
        });
        // cortex_m::asm::sev();
    }

    #[task(
        binds = IO_IRQ_BANK0,
        priority = 2,
        shared = [irqc, sw_pin],
    )]
    fn io_irq(cx: io_irq::Context) {
        let io_irq::SharedResources { irqc, sw_pin } = cx.shared;

        (irqc, sw_pin).lock(|irqc_a, sw_pin_a| {
            *irqc_a += 1;
            sw_pin_a.clear_interrupt(hal::gpio::Interrupt::LevelLow);
            sw_pin_a.set_interrupt_enabled(hal::gpio::Interrupt::LevelLow, false);
        });

        enable_irq::spawn_after(100u64.millis()).ok();
        io_poll::spawn().ok();

        // cortex_m::asm::sev();
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
        } = cx.shared;
        let tog = cx.local.tog;

        // Blinks the LED ON / OFF.
        (led, led_blink_enable, counter, irqc, sw_pin).lock(
            |led_a, led_blink_enable_a, counter_a, irqc_a, sw_pin_a| {
                let led_state_str: &str;
                if *led_blink_enable_a {
                    if *tog {
                        led_a.set_high().unwrap();
                        led_state_str = "ON ";
                    } else {
                        led_a.set_low().unwrap();
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
                if counter_a.enable {
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
        // cortex_m::asm::sev();
    }

    /// Usb interrupt handler. Runs every time the host requests new data.
    #[task(binds = USBCTRL_IRQ, priority = 3, shared = [led, led_blink_enable, serial, usb_dev, counter])]
    fn usb_rx(cx: usb_rx::Context) {
        let usb_rx::SharedResources {
            led,
            led_blink_enable,
            serial,
            usb_dev,
            counter,
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
        // cortex_m::asm::sev();
    }

    /* New Tasks */

    /// This function come from the github with USB-Serial example (see link above).
    ///
    /// Helper function to ensure all data is written across the serial interface.
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
                counter.enable(true);
            }
            // 3 - Stop continues counter
            b'3' => {
                write_serial(serial, "3 - Stop continues counter\r\n", false);
                counter.enable(false);
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
