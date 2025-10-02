#[cfg(not(feature = "std"))]
#[cfg(feature = "no-std")]
pub(crate) use crate::bsp;

#[cfg(feature = "std")]
use crate::tests;
#[cfg(feature = "std")]
pub use tests::stubs::fake_bsp as bsp;

#[allow(unused)]
pub mod log {
    #[cfg(feature = "std")]
    pub use core::assert;
    #[cfg(feature = "std")]
    pub use core::assert_eq;
    #[cfg(feature = "std")]
    pub use core::unreachable;
    #[cfg(not(feature = "std"))]
    pub use defmt::{assert, assert_eq, error, info, trace, unreachable, warn};
    #[cfg(feature = "std")]
    pub use log::{error, info, trace, warn};
}

#[cfg(not(feature = "std"))]
pub use cortex_m;

#[cfg(feature = "std")]
pub mod cortex_m {
    pub mod asm {
        pub fn wfi() {}
        pub fn nop() {}
    }
    pub mod prelude {}
}
