#[cfg(feature = "std")]
#[cfg(not(target_arch = "thumbv6m"))]
pub mod stubs;

#[cfg(feature = "std")]
pub(crate) mod test_state;

#[cfg(feature = "std")]
extern crate std;
