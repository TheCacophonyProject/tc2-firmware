# Thermal Camera 2 (tc2) firmware P2

This firmware is designed to take frames from a flir lepton 3 or lepton 3.5 sensor, and relay them to an attached
raspberry pi running the `tc2-agent` software. It works as a drop-in replacement for `leptond` on 'classic'
Cacophony thermal cameras.

This firmware is driven off of the vsync signal from the lepton module, and makes the rp2040 dormant whenever
possible in between frame data being sent from the lepton module.

It connects to the lepton via one of the rp2040's built in SPI peripherals, but uses a PIO defined SPI slave
to communicate with the raspberry pi, leaving the second hardware SPI peripheral for future communications
with the onboard 2Gbit flash module.

The firmware/rp2040 is able to be restarted by the raspberry pi toggling it's `RUN` pin, and it's also able to be
put into dormant mode via i2c by either the raspberry pi or the onboard attiny1616. It can then be woken by an
interrupt on the `WAKE` pin on the board.

Note that currently this firmware includes a copied snapshot of rp2040-hal with some changes made to allow setting
the frequency of the rp2040s Ring Oscillator, which is the main system clock used for the firmware due to its low
power characteristics, and due to the fact that we don't require super-precise timings.  
Once these changes are upstreamed, we should be able to switch to using the HAL as an external dependency.

The rosc also has the added
benefit that it can go dormant and resume very quickly in between lepton frames - the onboard crystal oscillator is
not able to do this.

### Flashing this firmware via an attached raspberry pi

Build the firmware by running

```sh
cargo build --release
```

Copy the binary from `./target/thumbv6m-none-eabi/release/tc2-firmware` to your raspberry pi.

With openocd installed on the pi, run

```sh
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program ./tc2-firmware verify reset exit"
```

This firmware is based off of rp2040-hal-template.

`probe-run` is configured as the default runner, so you can start your program as easy as

```sh
cargo run --release
```

If you aren't using a debugger (or want to use cargo-embed/probe-rs-debugger), check out [alternative runners](#alternative-runners) for other options

<!-- TABLE OF CONTENTS -->
<details open="open">
  
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li><a href="#requirements">Requirements</a></li>
    <li><a href="#installation-of-development-dependencies">Installation of development dependencies</a></li>
    <li><a href="#running">Running</a></li>
    <li><a href="#alternative-runners">Alternative runners</a></li>
    <li><a href="#medium-power-mode">Medium Power Mode</a></li>
  </ol>
</details>

<!-- Requirements -->
<details open="open">
  <summary><h2 style="display: inline-block" id="requirements">Requirements</h2></summary>
  
- The standard Rust tooling (cargo, rustup) which you can install from https://rustup.rs/

- Toolchain support for the cortex-m0+ processors in the rp2040 (thumbv6m-none-eabi)

- flip-link - this allows you to detect stack-overflows on the first core, which is the only supported target for now.

- probe-run. Upstream support for RP2040 was added with version 0.3.1.

- A CMSIS-DAP probe. (J-Link and other probes will not work with probe-run)

  You can use a second Pico as a CMSIS-DAP debug probe by installing the following firmware on it:
  https://github.com/majbthrd/DapperMime/releases/download/20210225/raspberry_pi_pico-DapperMime.uf2

  More details on supported debug probes can be found in [debug_probes.md](debug_probes.md)

</details>

<!-- Installation of development dependencies -->
<details open="open">
  <summary><h2 style="display: inline-block" id="installation-of-development-dependencies">Installation of development dependencies</h2></summary>

```sh
rustup target install thumbv6m-none-eabi
cargo install flip-link
# This is our suggested default 'runner'
cargo install probe-run
# If you want to use elf2uf2-rs instead of probe-run, instead do...
cargo install elf2uf2-rs --locked
```

</details>

<!-- Running -->
<details open="open">
  <summary><h2 style="display: inline-block" id="running">Running</h2></summary>
  
For a debug build
```sh
cargo run
```
For a release build
```sh
cargo run --release
```

If you do not specify a DEFMT_LOG level, it will be set to `debug`.
That means `println!("")`, `info!("")` and `debug!("")` statements will be printed.
If you wish to override this, you can change it in `.cargo/config.toml`

```toml
[env]
DEFMT_LOG = "off"
```

You can also set this inline (on Linux/MacOS)

```sh
DEFMT_LOG=trace cargo run
```

or set the _environment variable_ so that it applies to every `cargo run` call that follows:

#### Linux/MacOS/unix

```sh
export DEFMT_LOG=trace
```

Setting the DEFMT_LOG level for the current session  
for bash

```sh
export DEFMT_LOG=trace
```

#### Windows

Windows users can only override DEFMT_LOG through `config.toml`
or by setting the environment variable as a separate step before calling `cargo run`

- cmd

```cmd
set DEFMT_LOG=trace
```

- powershell

```ps1
$Env:DEFMT_LOG = trace
```

```cmd
cargo run
```

</details>
<!-- ALTERNATIVE RUNNERS -->
<details open="open">
  <summary><h2 style="display: inline-block" id="alternative-runners">Alternative runners</h2></summary>

If you don't have a debug probe or if you want to do interactive debugging you can set up an alternative runner for cargo.

Some of the options for your `runner` are listed below:

- **cargo embed**  
  _Step 1_ - Install [`cargo embed`](https://github.com/probe-rs/cargo-embed):

  ```console
  $ cargo install cargo-embed
  ```

  _Step 2_ - Make sure your .cargo/config contains the following

  ```toml
  [target.thumbv6m-none-eabi]
  runner = "cargo embed"
  ```

  _Step 3_ - Update settings in [Embed.toml](Embed.toml)

  - The defaults are to flash, reset, and start a defmt logging session
    You can find all the settings and their meanings [in the cargo-embed repo](https://github.com/probe-rs/cargo-embed/blob/master/src/config/default.toml)

  _Step 4_ - Use `cargo run`, which will compile the code and start the
  specified 'runner'. As the 'runner' is cargo embed, it will flash the device
  and start running immediately

  ```console
  $ cargo run --release
  ```

- **probe-rs-debugger**

  _Step 1_ - Download [`probe-rs-debugger VSCode plugin 0.4.0`](https://github.com/probe-rs/vscode/releases/download/v0.4.0/probe-rs-debugger-0.4.0.vsix)

  _Step 2_ - Install `probe-rs-debugger VSCode plugin`

  ```console
  $ code --install-extension probe-rs-debugger-0.4.0.vsix
  ```

  _Step 3_ - Install `probe-rs-debugger`

  ```console
  $ cargo install probe-rs-debugger
  ```

  _Step 4_ - Open this project in VSCode

  _Step 5_ - Launch a debug session by choosing `Run`>`Start Debugging` (or press F5)

- **Loading a UF2 over USB**  
  _Step 1_ - Install [`elf2uf2-rs`](https://github.com/JoNil/elf2uf2-rs):

  ```console
  $ cargo install elf2uf2-rs --locked
  ```

  _Step 2_ - Make sure your .cargo/config contains the following

  ```toml
  [target.thumbv6m-none-eabi]
  runner = "elf2uf2-rs -d"
  ```

  The `thumbv6m-none-eabi` target may be replaced by the all-Arm wildcard
  `'cfg(all(target_arch = "arm", target_os = "none"))'`.

  _Step 3_ - Boot your RP2040 into "USB Bootloader mode", typically by rebooting
  whilst holding some kind of "Boot Select" button. On Linux, you will also need
  to 'mount' the device, like you would a USB Thumb Drive.

  _Step 4_ - Use `cargo run`, which will compile the code and start the
  specified 'runner'. As the 'runner' is the elf2uf2-rs tool, it will build a UF2
  file and copy it to your RP2040.

  ```console
  $ cargo run --release
  ```

- **Loading with picotool**  
  As ELF files produced by compiling Rust code are completely compatible with ELF
  files produced by compiling C or C++ code, you can also use the Raspberry Pi
  tool [picotool](https://github.com/raspberrypi/picotool). The only thing to be
  aware of is that picotool expects your ELF files to have a `.elf` extension, and
  by default Rust does not give the ELF files any extension. You can fix this by
  simply renaming the file.

  This means you can't easily use it as a cargo runner - yet.

  Also of note is that the special
  [pico-sdk](https://github.com/raspberrypi/pico-sdk) macros which hide
  information in the ELF file in a way that `picotool info` can read it out, are
  not supported in Rust. An alternative is TBC.

</details>

<!-- Medium Power Mode -->
<details open="open">
  <summary><h2 style="display: inline-block" id="medium-power-mode">Medium Power Mode</h2></summary>

In low power mode the PI will be woken up as soon as a non status recording is made.

Once the PI is awake, the RP2040 will begin offloading the currently recording CPTV File.

- Frames will be read from the onboard flash and will be sent over the PIO SPI to the PI.
- As to not slow down the processing & writing of the lepton frames, the frames wont start being
  sent until after 20 frames have been written on the RP2040 flash.
- At most 5 flash pages of data will be sent each transfer. Again to not slow down the processing of lepton frames.

Since we have to wait for the PI to turn on, sometimes the RP2040 has finished the current recording and a new recording has started. The following rules apply

1. When transferring a recording, Rec 1 and a new recording starts Rec 2 starts:

- If no frames of Rec 1 have been transferred to the PI. Rec 2 will be sent instead and a MissedClassification event will be logged
- If Rec 1 has started transferring it will finish transferring and then Rec 2 will be sent.

2. Once a recording occurs the PI will stay on for 5 minutes in case of new recordings.

3. If the RP2040 fails to send a frame after 270 retries ~30 seconds, the offload will be aborted and a CouldNotTransfer event will be logged

Note: The PIO SPI is the same interface that is used to send
the current frame to the PI, so while a recording is being made the management interface
preview will not work.

</details>
