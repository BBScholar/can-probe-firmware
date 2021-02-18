#![no_std]
#![no_main]
#![allow(dead_code, unused_imports, unused_variables, unused_mut)]

mod usb_class;

use usb_class::{UsbBusType, USB};

// Plans
// stm32f103
// https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf

pub use stm32f1xx_hal as hal;
pub use usb_device as usb;
use usbd_serial;

use core::prelude::*;
use embedded_hal::prelude::*;
use hal::prelude::*;
use usb::prelude::*;

use defmt;
use defmt_rtt as _;

use rtic::app;
use rtic::cyccnt::{Instant, U32Ext as _};

//
use usbd_serial::{SerialPort, USB_CLASS_CDC};

#[defmt::timestamp]
fn timestamp() -> u64 {
    cortex_m::peripheral::DWT::get_cycle_count() as u64
}

#[app(device=stm32f1xx_hal::stm32, peripherals=true, monotonic=rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        settings: adaptor_common::AdaptorSettings,

        usb_device: usb::device::UsbDevice<'static, UsbBusType>,
        usb_can_class: usb_class::CanProbeClass<'static, UsbBusType, heapless::consts::U32>,

        #[cfg(debug_assertions)]
        #[init(0)]
        frames_processed: usize,
    }

    #[init()]
    fn init(cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<usb::bus::UsbBusAllocator<UsbBusType>> = None;
        let device = cx.device;

        device.DBGMCU.cr.modify(|_, w| w.dbg_sleep().set_bit());
        device.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);
        let mut debug = device.DBGMCU;

        let clocks = rcc.cfgr.use_hse(8.mhz()).freeze(&mut flash.acr);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);

        // let usb_dm = gpioa.pa11;
        // let usb_dp = gpioa.pa12.into_floating_input(&mut gpioa.crh);

        // usb bus setup
        let usb = USB {
            usb_global: device.USB_OTG_GLOBAL,
            usb_device: device.USB_OTG_DEVICE,
            usb_pwrclk: device.USB_OTG_PWRCLK,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
            hclk: clocks.hclk(),
        };

        let USB_MEM: &'static mut [u32; 1024] =
            cortex_m::singleton!(: [u32; 1024] = [0; 1024]).unwrap();

        *USB_BUS = Some(synopsys_usb_otg::UsbBus::new(usb, USB_MEM));

        let usb_device = usb::device::UsbDeviceBuilder::new(
            USB_BUS.as_ref().unwrap(),
            usb::device::UsbVidPid(0x69, 0x420),
        )
        .manufacturer("Scholarly Systems")
        .product("Scholarly CAN Probe")
        .supports_remote_wakeup(false)
        .self_powered(false)
        .max_power(100)
        .device_class(USB_CLASS_CDC)
        .device_release(0x0010) // 0.1
        .build();

        let usb_can_class = usb_class::CanProbeClass::new(USB_BUS.as_ref().unwrap());
        let settings = adaptor_common::AdaptorSettings::default();

        init::LateResources {
            settings,
            usb_device,
            usb_can_class,
        }
    }

    #[idle()]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = USB_HP_CAN_TX, resources=[usb_device, usb_can_class, settings])]
    fn usb_rx(cx: usb_rx::Context) {
        use usb::class::UsbClass;
        let usb_device = cx.resources.usb_device;
        let class = cx.resources.usb_can_class;

        if usb_device.poll(&mut [class]) {
            class.update();

            // transmit frames to can
            for frame in class.read_frames() {}
            class.clear_read_buffer();

            match class.update_settings_if_new(cx.resources.settings) {
                true => {
                    // change can settings
                }
                false => {}
            }
        }
    }

    #[task(binds = EXTI1, resources=[usb_can_class])]
    fn can_rx(cx: can_rx::Context) {
        use adaptor_common::CANFrame;
        let class = cx.resources.usb_can_class;

        let vec: heapless::Vec<CANFrame, heapless::consts::U16> = heapless::Vec::new();

        // TODO: recieve frames

        // write frames to usb
        let _ = class.write_frames(vec.into_iter());
    }

    extern "C" {
        fn SPI1();
        fn SPI2();
        fn SPI3();
    }
};

#[panic_handler]
fn my_panic(info: &core::panic::PanicInfo) -> ! {
    loop {}
}
