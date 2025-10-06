#[cfg(feature = "no-std")]
pub use crate::bsp;
#[cfg(feature = "std")]
use crate::tests;
#[cfg(not(feature = "std"))]
pub use cortex_m;
#[cfg(not(feature = "std"))]
pub use critical_section;
#[cfg(feature = "std")]
pub use tests::stubs::bsp;
#[cfg(feature = "std")]
pub use tests::stubs::fake_critical_section as critical_section;

#[allow(unused)]
pub mod log {
    #[cfg(feature = "std")]
    pub use core::assert;
    #[cfg(feature = "std")]
    pub use core::assert_eq;
    #[cfg(feature = "std")]
    pub use core::unreachable;
    #[cfg(not(feature = "std"))]
    pub use defmt::{assert, assert_eq, debug, error, info, trace, unreachable, warn};
    #[cfg(feature = "std")]
    pub use log::{debug, error, info, trace, warn};
}

#[cfg(feature = "std")]
pub mod cortex_m {
    pub mod asm {
        pub fn wfi() {}
        pub fn nop() {}
    }
    pub mod prelude {}
}
