// NOTE: The version number here isn't important.  What's important is that we increment it
//  when we do a release, so the tc2-agent can match against it and see if the version is correct
//  for the agent software.
// TODO Check against minor version also.
pub const FIRMWARE_VERSION: u32 = 38;
pub const EXPECTED_ATTINY_FIRMWARE_VERSION: u8 = 1; // Checking against the attiny Major version.

pub const ROSC_TARGET_CLOCK_FREQ_HZ: u32 = 125_000_000;
pub const FFC_INTERVAL_MS: u32 = 60 * 1000 * 10; // 10 mins between FFCs
