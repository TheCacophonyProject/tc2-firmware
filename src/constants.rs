// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
// TODO Check against minor version also.
// Update the version in src/tests/mocks/fake_shared_spi.rs so the tests will pass

#[cfg(not(feature = "erase-flash"))]
pub const FIRMWARE_VERSION: u32 = 38;
// For the erase flash version we set it to zero so when tc2-agent starts it will re-program the the RP2040
#[cfg(feature = "erase-flash")]
pub const FIRMWARE_VERSION: u32 = 0;
pub const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1; // Checking against the attiny Major version.

pub const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 125_000_000;
pub const FFC_INTERVAL_MS: u32 = 60 * 1000 * 10; // 10 mins between FFCs
