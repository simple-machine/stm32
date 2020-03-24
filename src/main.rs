#![no_std]
#![no_main]

extern crate panic_semihosting;

use cortex_m::asm;
use cortex_m_semihosting::hprintln;
use embedded_hal::digital::v2::OutputPin;
use rtfm::app;
use stm32f1xx_hal::{
    gpio::{gpioc::PC13, GpioExt, Output, PushPull},
    pac,
    prelude::*,
    timer::{CountDownTimer, Event, Timer},
    usb::{self, UsbBus, UsbBusType},
};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

/// The protocol state machine (for initialization of the device)
#[derive(Debug, PartialEq, Eq)]
pub enum State {
    Start,
    Magic,
    Version,
    Confirmation,
    Normal,
}

/// Error processed by the machine
#[derive(Debug, PartialEq, Eq)]
pub enum ErrorState {
    /// Everything normal
    Ok,
    /// Device initialization error (fatal, led turns on and all further communication is stopped)
    Fatal,
    /// The request failed, this is recoverable and the led starts blinking
    Request,
}

const VERSION: u16 = 0;

pub struct Error {
    error: ErrorState,
    led: PC13<Output<PushPull>>,
    timer: CountDownTimer<pac::TIM1>,
}

#[app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        // usb
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,

        // Protocol state
        #[init(State::Start)]
        state: State,

        // Error state
        error: Error,

        // detect disconnects
        disconnect_timer: CountDownTimer<pac::TIM2>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        let p = cx.device;

        let mut rcc = p.RCC.constrain();
        let mut flash = p.FLASH.constrain();
        // Init clock
        let clock = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);
        assert!(clock.usbclk_valid());

        let mut gpioa = p.GPIOA.split(&mut rcc.apb2);
        let mut gpioc = p.GPIOC.split(&mut rcc.apb2);

        // Turn off the led (low = lit)
        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high().unwrap();

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        // This forced reset is needed only for development, without it host
        // will not reset your device when you upload new firmware.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);

        usb_dp.set_low().unwrap();
        asm::delay(clock.sysclk().0 / 100);

        *USB_BUS = Some(UsbBus::new(usb::Peripheral {
            usb: p.USB,
            pin_dm: gpioa.pa11,
            pin_dp: usb_dp.into_floating_input(&mut gpioa.crh),
        }));
        let usb_bus = USB_BUS.as_ref().unwrap();

        // Serial Port
        let serial = SerialPort::new(&usb_bus);
        // Custom device identifier
        // IMPORTANT: This device identifier is not valid and further development would
        //   need to buy a proper license from the USB forum
        let usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xAABC, 0x0001))
            .manufacturer("Simple machines")
            .product("OpenCV Ventilator")
            .serial_number("smov")
            .device_class(USB_CLASS_CDC)
            .build();

        // Blinking led rate
        let timer = Timer::tim1(p.TIM1, &clock, &mut rcc.apb2).start_count_down(10.hz());
        let disconnect_timer = Timer::tim2(p.TIM2, &clock, &mut rcc.apb1).start_count_down(1.hz());

        init::LateResources {
            usb_dev,
            serial,
            error: Error {
                led,
                timer,
                error: ErrorState::Ok,
            },
            disconnect_timer,
        }
    }

    #[task(
        binds = USB_HP_CAN_TX,
        priority = 2,
        resources = [usb_dev, serial, error, state, disconnect_timer]
    )]
    fn usb_tx(mut cx: usb_tx::Context) {
        // If the initialization failed, don't even process usb
        if cx.resources.usb_dev.poll(&mut [cx.resources.serial]) {
            // same as above
            if cx.resources.error.error != ErrorState::Fatal {
                cx.resources.error.error = usb(
                    &mut cx.resources.serial,
                    &mut cx.resources.state,
                    &mut cx.resources.disconnect_timer,
                );
                // Set the led
                match cx.resources.error.error {
                    // Turn it off
                    ErrorState::Ok => cx.resources.error.led.set_high().unwrap(),
                    // Turn it on
                    ErrorState::Request => cx.resources.error.led.set_low().unwrap(),
                    // Start the blinking
                    ErrorState::Fatal => cx.resources.error.timer.listen(Event::Update),
                }
            } else {
                let mut buf = [0; 4];
                let _ = cx.resources.serial.read(&mut buf);
                let _ = cx.resources.serial.write(&[0xFD]);
            }
        }
    }

    #[task(
        binds = USB_LP_CAN_RX0,
        priority = 2,
        resources = [usb_dev, serial, state, error, disconnect_timer]
    )]
    fn usb_rx0(mut cx: usb_rx0::Context) {
        if cx.resources.usb_dev.poll(&mut [cx.resources.serial]) {
            // same as above
            if cx.resources.error.error != ErrorState::Fatal {
                cx.resources.error.error = usb(
                    &mut cx.resources.serial,
                    &mut cx.resources.state,
                    &mut cx.resources.disconnect_timer,
                );
                match cx.resources.error.error {
                    ErrorState::Ok => cx.resources.error.led.set_high().unwrap(),
                    ErrorState::Request => cx.resources.error.led.set_low().unwrap(),
                    ErrorState::Fatal => cx.resources.error.timer.listen(Event::Update),
                }
            } else {
                let mut buf = [0; 4];
                let _ = cx.resources.serial.read(&mut buf);
                let _ = cx.resources.serial.write(&[0xFD]);
            }
        }
    }

    #[task(binds = TIM1_UP, priority = 1, resources = [error])]
    fn tick(mut cx: tick::Context) {
        cx.resources.error.lock(|e| {
            // Toggle led
            e.led.toggle().unwrap();

            // Clears the update flag (else the timer will trigger right after)
            e.timer.clear_update_interrupt_flag();
        });
    }

    #[task(binds = TIM2, priority = 1, resources = [error, disconnect_timer, state])]
    fn disconnect(mut cx: disconnect::Context) {
        cx.resources.disconnect_timer.lock(|t| {
            t.clear_update_interrupt_flag();
            t.unlisten(Event::Update);
        });
        cx.resources.error.lock(|e| {
            e.timer.listen(Event::Update);
            e.error = ErrorState::Fatal;
        });
    }
};

fn usb<B: usb_device::bus::UsbBus>(
    serial: &mut SerialPort<'static, B>,
    state: &mut State,
    disconnect_timer: &mut CountDownTimer<pac::TIM2>,
) -> ErrorState {
    loop {
        match state {
            State::Start => {
                let mut buf = [0u8; 4];
                match serial.read(&mut buf) {
                    Ok(4) if buf == [0x73, 0x6d, 0x6f, 0x76] => *state = State::Magic,
                    Err(UsbError::WouldBlock) => return ErrorState::Ok,
                    _ => return ErrorState::Fatal,
                }
            }
            State::Magic => match serial.write(&[0x73, 0x6d, 0x6f, 0x76]) {
                Ok(4) => *state = State::Version,
                Err(UsbError::WouldBlock) => return ErrorState::Ok,
                _ => return ErrorState::Fatal,
            },
            State::Version => match serial.write(&VERSION.to_be_bytes()) {
                Ok(2) => *state = State::Confirmation,
                Err(UsbError::WouldBlock) => return ErrorState::Ok,
                _ => return ErrorState::Fatal,
            },
            State::Confirmation => {
                let mut buf = [0u8; 1];
                match serial.read(&mut buf) {
                    Ok(1) if buf == [0x00] => *state = State::Normal,
                    Err(UsbError::WouldBlock) => return ErrorState::Ok,
                    _ => return ErrorState::Fatal,
                }
                disconnect_timer.clear_update_interrupt_flag();
                disconnect_timer.listen(Event::Update);
            }
            State::Normal => {
                let mut error = false;
                let mut buf = [0u8; 64];
                match serial.read(&mut buf) {
                    Ok(o) => {
                        let response = match buf[..o] {
                            [0x00] => 0x00,
                            [0x01, speed_high, speed_low] => {
                                let speed = u16::from_be_bytes([speed_high, speed_low]);
                                // Set motor speed
                                0x00
                            }
                            [0x01, ..] => 0x01,
                            _ => 0xFF,
                        };
                        if response != 0x00 {
                            error = true;
                        }
                        disconnect_timer.reset();
                        serial.write(&[response]).unwrap();
                    }
                    Err(UsbError::WouldBlock) => {
                        return ErrorState::Ok;
                    }
                    _ => return ErrorState::Fatal,
                }
                return if error {
                    ErrorState::Request
                } else {
                    ErrorState::Ok
                };
            }
        }
    }
}
