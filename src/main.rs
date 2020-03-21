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

/// A ring buffer to hold tokens to eventually send to the device
#[derive(Default, Debug, PartialEq, Eq)]
pub struct WriteBuf {
    buf: [u8; 32],
    start: usize,
    end: usize,
}

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

#[app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        // usb
        usb_dev: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBusType>,

        // Bytes to send as messages
        send_backlog: WriteBuf,

        // Protocol state
        #[init(State::Start)]
        state: State,

        // Error state
        #[init(ErrorState::Ok)]
        error: ErrorState,

        // led blink
        led: PC13<Output<PushPull>>,
        timer: CountDownTimer<pac::TIM1>,
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
        init::LateResources {
            usb_dev,
            serial,
            timer,
            led,
            send_backlog: WriteBuf::default(),
        }
    }

    #[task(binds = USB_HP_CAN_TX, priority = 2, resources = [usb_dev, serial, send_backlog, led, timer, state, error])]
    fn usb_tx(mut cx: usb_tx::Context) {
        // If the initialization failed, don't even process usb
        if *cx.resources.error != ErrorState::Fatal {
            *cx.resources.error = usb(
                &mut cx.resources.usb_dev,
                &mut cx.resources.serial,
                &mut cx.resources.send_backlog,
                &mut cx.resources.state,
            );
            // Set the led
            match *cx.resources.error {
                // Turn it off
                ErrorState::Ok => cx.resources.led.set_high().unwrap(),
                // Turn it on
                ErrorState::Request => cx.resources.led.set_low().unwrap(),
                // Start the blinking
                ErrorState::Fatal => cx.resources.timer.listen(Event::Update),
            }
        }
    }

    #[task(binds = USB_LP_CAN_RX0, priority = 2, resources = [usb_dev, serial, send_backlog, led, timer, state, error])]
    fn usb_rx0(mut cx: usb_rx0::Context) {
        // same as above
        if *cx.resources.error != ErrorState::Fatal {
            *cx.resources.error = usb(
                &mut cx.resources.usb_dev,
                &mut cx.resources.serial,
                &mut cx.resources.send_backlog,
                &mut cx.resources.state,
            );
            match *cx.resources.error {
                ErrorState::Ok => cx.resources.led.set_high().unwrap(),
                ErrorState::Request => cx.resources.led.set_low().unwrap(),
                ErrorState::Fatal => cx.resources.timer.listen(Event::Update),
            }
        }
    }

    #[task(binds = TIM1_UP, priority = 1, resources = [led, timer])]
    fn tick(mut cx: tick::Context) {
        // Toggle led
        cx.resources.led.lock(|led| led.toggle()).unwrap();

        // Clears the update flag (else the timer will trigger right after)
        cx.resources
            .timer
            .lock(|timer| timer.clear_update_interrupt_flag());
    }
};

fn usb<B: usb_device::bus::UsbBus>(
    usb_dev: &mut UsbDevice<'static, B>,
    serial: &mut SerialPort<'static, B>,
    send_backlog: &mut WriteBuf,
    state: &mut State,
) -> ErrorState {
    if usb_dev.poll(&mut [serial]) {
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
                }
                State::Normal => {
                    let mut error = false;
                    let mut buf = [0u8; 64];
                    if let Ok(o) = serial.read(&mut buf) {
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
                        send_backlog.buf[send_backlog.end] = response;
                        send_backlog.end += 1;
                        send_backlog.end %= send_backlog.buf.len();
                    }
                    if send_backlog.start <= send_backlog.end {
                        if let Ok(written) =
                            serial.write(&send_backlog.buf[send_backlog.start..send_backlog.end])
                        {
                            send_backlog.start += written;
                        }
                    } else if send_backlog.start > send_backlog.end {
                        if let Ok(written) = serial.write(&send_backlog.buf[send_backlog.start..]) {
                            send_backlog.start += written;
                        }
                        if let Ok(written) = serial.write(&send_backlog.buf[..send_backlog.end]) {
                            send_backlog.start += written;
                        }
                        send_backlog.start %= send_backlog.buf.len();
                    }
                    return if error {
                        ErrorState::Request
                    } else {
                        ErrorState::Ok
                    };
                }
            }
        }
    } else {
        ErrorState::Ok
    }
}
