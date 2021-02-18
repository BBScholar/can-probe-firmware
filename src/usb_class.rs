use stm32f1xx_hal as hal;
use usb_device as usb;

use adaptor_common::AdaptorSettings;
use adaptor_common::CANFrame;

use synopsys_usb_otg::UsbPeripheral;

use usb::bus::{InterfaceNumber, UsbBus, UsbBusAllocator};
use usb::class::UsbClass;
use usb::endpoint::{Endpoint, EndpointIn, EndpointOut};

use heapless::{consts, Vec};

use hal::gpio::{Alternate, Floating, PushPull};

pub struct USB<A, B> {
    pub usb_global: hal::stm32::USB_OTG_GLOBAL,
    pub usb_device: hal::stm32::USB_OTG_DEVICE,
    pub usb_pwrclk: hal::stm32::USB_OTG_PWRCLK,
    pub pin_dm: hal::gpio::gpioa::PA11<A>,
    pub pin_dp: hal::gpio::gpioa::PA12<B>,
    pub hclk: hal::time::Hertz,
}

unsafe impl<A, B> Sync for USB<A, B> {}

unsafe impl UsbPeripheral for USB {
    const REGISTERS: *const () = hal::stm32::USB_OTG_GLOBAL::ptr() as *const ();

    const HIGH_SPEED: bool = true;

    const FIFO_DEPTH_WORDS: usize = 160;

    const ENDPOINT_COUNT: usize = 4;

    fn enable() {
        let rcc = unsafe { &*hal::stm32::RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            rcc.ahbenr.write(|w| w.otgfsen().set_bit());

            rcc.ahbrstr.write(|w| w.otgfsrst().set_bit());
            rcc.ahbrstr.write(|w| w.otgfsrst().clear_bit());
        });
    }

    fn ahb_frequency_hz(&self) -> u32 {
        self.hclk.0
    }
}

pub type UsbBusType = synopsys_usb_otg::UsbBus<USB>;

pub struct CanProbeClass<'a, B, S>
where
    B: usb::bus::UsbBus,
    S: heapless::ArrayLength<CANFrame>,
{
    //
    buffered_settings: Option<AdaptorSettings>,

    write_buffer: heapless::Vec<CANFrame, S>,
    read_buffer: heapless::Vec<CANFrame, S>,

    // interfaces
    comm_if: InterfaceNumber,
    data_if: InterfaceNumber,

    // endpoints
    write_ep: EndpointIn<'a, B>,
    read_ep: EndpointOut<'a, B>,
}

impl<B, S> CanProbeClass<'_, B, S>
where
    B: UsbBus + 'static,
    S: heapless::ArrayLength<CANFrame>,
{
    pub const DATA_PACKET_SIZE: u16 = 32;

    pub fn new(alloc: &UsbBusAllocator<B>) -> CanProbeClass<'_, B, S> {
        let read_buffer = Vec::<CANFrame, S>::new();
        let write_buffer = Vec::<CANFrame, S>::new();
        CanProbeClass {
            comm_if: alloc.interface(),
            data_if: alloc.interface(),
            read_ep: alloc.bulk(Self::DATA_PACKET_SIZE),
            write_ep: alloc.bulk(Self::DATA_PACKET_SIZE),
            read_buffer,
            write_buffer,
            buffered_settings: None,
        }
    }

    pub fn write_frame(&mut self, frame: CANFrame) -> Result<(), CANFrame> {
        self.write_buffer.push(frame)
    }

    pub fn write_frames<I>(&mut self, frames: I) -> Result<u32, CANFrame>
    where
        I: Iterator<Item = CANFrame>,
    {
        let mut written_frames = 0;
        for frame in frames {
            match self.write_frame(frame).err() {
                Some(ret_frame) => return Err(ret_frame),
                None => {
                    written_frames += 1;
                }
            }
        }

        Ok(written_frames)
    }

    pub fn read_frames(&self) -> impl core::iter::Iterator<Item = &CANFrame> {
        self.read_buffer.iter()
    }

    pub fn clear_read_buffer(&mut self) {
        self.read_buffer.clear();
    }

    pub fn update_settings_if_new(&mut self, settings: &mut AdaptorSettings) -> bool {
        match self.buffered_settings {
            Some(s) => {
                *settings = s;
                self.buffered_settings = None;
                true
            }
            None => false,
        }
    }

    pub fn update(&mut self) {
        let mut byte_buf = Vec::<u8, consts::U64>::new();

        let mut bytes_written = 0_u32;
        let mut bytes_read = 0_32;

        // write outgoing frames
        for f in self.write_buffer.iter() {
            postcard::to_slice(&f, &mut byte_buf[..]).unwrap();
            let bytes = self.write_ep.write(&byte_buf);
            byte_buf.clear();
            bytes_written += 1;
        }

        self.write_buffer.clear();

        // get incoming frames
        loop {
            match self.read_ep.read(&mut byte_buf[..]) {
                Ok(bytes) => {
                    // TODO: Handle this error
                    let frame: CANFrame = postcard::from_bytes(&byte_buf[..bytes]).unwrap();
                    self.read_buffer.push(frame).err();
                    bytes_read += 1;
                }
                Err(usb::UsbError::WouldBlock) => break,
                _ => defmt::unreachable!(),
            }
            byte_buf.clear();
        }

        defmt::info!(
            "Polled -- Wrote {:u32} frames, Read {:u32} frames",
            bytes_written,
            bytes_read,
        );
    }
}

impl<B, S> UsbClass<B> for CanProbeClass<'_, B, S>
where
    B: usb::bus::UsbBus,
    S: heapless::ArrayLength<CANFrame>,
{
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb::class_prelude::DescriptorWriter,
    ) -> usb::Result<()> {
        writer.iad(self.comm_if, 2, 0x0, 0x0, 0x0)?;
        writer.interface(self.comm_if, 0x0a, 0x00, 0x00)?;
        writer.interface(self.data_if, 0x0a, 0x00, 0x00)?;
        writer.endpoint(&self.write_ep)?;
        writer.endpoint(&self.read_ep)?;
        Ok(())
    }

    fn reset(&mut self) {
        self.write_buffer.clear();
        self.read_buffer.clear();
    }

    // we can handle other stuff here
    fn control_in(&mut self, xfer: usb::class::ControlIn<B>) {
        let _ = xfer;
    }

    /// This method will handle settings
    fn control_out(&mut self, xfer: usb::class::ControlOut<B>) {
        use usb::control;
        let req = xfer.request();
        let data = xfer.data();

        if !(req.request_type == control::RequestType::Class
            && req.recipient == control::Recipient::Interface
            && req.index == u8::from(self.comm_if) as u16)
        {
            return;
        }

        match req.request {
            _ => {
                xfer.reject().ok();
            }
        }
    }
}
