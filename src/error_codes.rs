use defmt;
use num_enum;

#[repr(u8)]
#[derive(
    defmt::Format,
    num_enum::IntoPrimitive,
    num_enum::TryFromPrimitive,
    Eq,
    PartialEq,
    Ord,
    PartialOrd,
)]
pub(crate) enum ErrorCode {
    None = 0x0,
    UsbError,
    SpiError,
    SerdeError,
}
