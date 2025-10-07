# Testing rationale and background

Testing TC2 firmware is challenging because the firmware is a no_std firmware crate targeting custom hardware based
around the RP2040 microcontroller.
It isn't possible to run as-is using the target architecture and bespoke hardware on
x86_64 continuous integration machines, or local development machines.

To work around this, we have created a "std" cargo feature, and this is turned on whenever we
run tests.
All the RP2040/TC2 (aka DOC AI Cam) camera peripherals the firmware references are
mocked out, so we can test the high-level control flow logic of the firmware without
needing a real device.
The `rp2040-hal` crate is re-exported under `re_exports`, and the mocks
located in `tests/stubs` take the place of the real HAL code when `std` is enabled.

With the TC2 firmware we're primarily interested in the high-level control flow logic,
so we're not testing the low-level details of the camera peripherals. The reason for this is that the control flow logic
is quite hard to reason about, because we support so many different user configurations.

# Testing

Tests are run by passing the firmware simulation a `config.toml` file, which is the device configuration that is stored
on the accompanying SD card used by the raspberry pi.

The main features that affect the operation of the firmware are:

### Camera location lat/lng coordinates.

The camera must always be configured with a location. In the case where the thermal recording window is relative to dusk
or dawn, the location is used to calculate sunrise and sunset times throughout the year.

### Power mode

The camera has two power modes: `high-power` and `low-power`.

#### High power mode

In `high-power` mode, the raspberry pi is powered on during the thermal recording window, and thermal sensor frame data
captured by the RP2040 is streamed to the raspberry pi via an SPI interface.
The `tc2-agent` daemon running on the raspberry pi then makes these raw thermal frames available to other processes,
such and onboard realtime AI classification. In this mode battery life is limited, but if the user wants close to
real-time classification of predators this can be a reasonable tradeoff.

#### Low power mode

In `low-power` mode, the raspberry pi is powered off during the thermal recording window, and the thermal frame data is
processed on the RP2040 microcontroller. When thermal motion triggers recording, CPTV recordings are created, and stored
to an onboard 2Gb (256MB) SPI NAND flash chip.
At the end of the thermal recording window, the RP2040 will power on the raspberry pi, and the CPTV recordings will be
offloaded to the raspberry pi via an SPI interface. In devices with a cellular modem, the CPTV recordings will be
uploaded to the cloud.
Once offloading is complete, the RP2040 will power off the raspberry pi again.

### Thermal recording window

This window can be an absolute time range, or a relative time range (relative to dawn and dusk).
If not specified, the window is calculated as starting 30mins before dusk and ending 30mins after dawn, since
historically
the project has been primarily interested in capturing nocturnal predators of New Zealand birds.
It's possible to configure the window to be a fixed time range, i.e. 10am to 10pm.
It's also technically possible to configure the window to be a mixture of the two, i.e. 10am until 30mins before
dusk, but in practice this is not used.
Finally, many users want the camera to be active 24/7, so the window can be configured to be something like 00:00
to 00:00, or 12:00 to 12:00.

### Audio recording mode

- `AudioOnly` (only records birdsong audio at various randomized intervals throughout the day)
- `AudioAndThermal` (records birdsong audio and thermal data, interrupting readiness of thermal recording to make
  scheduled audio recordings)
- `AudioOrThermal` (records birdsong audio *outside* the configured thermal recording window, otherwise inside
  the window is ready to record thermal recordings)
- `Disabled` (no audio recording, will just be ready to trigger thermal recordings during the configured thermal
  recording window)

## Testing matrix

| Power mode | Thermal recording window | Audio recording mode | Test file                                                    |
|------------|--------------------------|----------------------|--------------------------------------------------------------|
| High power | Relative: Dusk-Dawn      | Disabled             | `./testing/high_power__default_window__audio_disabled.rs`    | 
| High power | Absolute: 24/7           | Disabled             | `./testing/high_power__always_on__audio_disabled.rs`         | 
| High power | Absolute: 10am - 10pm    | Disabled             | `./testing/high_power__fixed_window__audio_disabled.rs`      | 
| High power | Relative: Dusk-Dawn      | AudioOrThermal       | `./testing/high_power__default_window__audio_or_thermal.rs`  | 
| High power | Absolute: 10am - 10pm    | AudioOrThermal       | `./testing/high_power__fixed_window__audio_or_thermal.rs`    | 
| High power | Relative: Dusk-Dawn      | AudioAndThermal      | `./testing/high_power__default_window__audio_and_thermal.rs` | 
| High power | Absolute: 10am - 10pm    | AudioAndThermal      | `./testing/high_power__fixed_window__audio_and_thermal.rs`   | 
| High power | Absolute: 24/7           | AudioAndThermal      | `./testing/high_power__always_on__audio_and_thermal.rs`      | 
| High power | N/A                      | AudioOnly            | `./testing/high_power__audio_only.rs`                        | 
| Low power  | Relative: Dusk-Dawn      | Disabled             | `./testing/low_power__default_window__audio_disabled.rs`     | 
| Low power  | Absolute: 24/7           | Disabled             | `./testing/low_power__always_on__audio_disabled.rs`          | 
| Low power  | Absolute: 10am - 10pm    | Disabled             | `./testing/low_power__fixed_window__audio_disabled.rs`       | 
| Low power  | Relative: Dusk-Dawn      | AudioOrThermal       | `./testing/low_power__default_window__audio_or_thermal.rs`   | 
| Low power  | Absolute: 10am - 10pm    | AudioOrThermal       | `./testing/low_power__fixed_window__audio_or_thermal.rs`     | 
| Low power  | Relative: Dusk-Dawn      | AudioAndThermal      | `./testing/low_power__default_window__audio_and_thermal.rs`  | 
| Low power  | Absolute: 10am - 10pm    | AudioAndThermal      | `./testing/low_power__fixed_window__audio_and_thermal.rs`    | 
| Low power  | Absolute: 24/7           | AudioAndThermal      | `./testing/low_power__always_on__audio_and_thermal.rs`       | 
| Low power  | N/A                      | AudioOnly            | `./testing/low_power__audio_only.rs`                         | 