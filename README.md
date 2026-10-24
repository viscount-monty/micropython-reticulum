# µReticulum for RP2040

This fork is very much a stage 0 work in progress.

## Raspberry Pi Pico (RP2040) with Waveshare Pico-LoRa-SX1262 module

This fork focuses solely on developing a minimal `lxmf` implementation with the
following functions:

- [ ] Announce `lxmf` destination
- [ ] Receive `lxmf` messages and print to serial
- [ ] Send `lxmf` messages to a hard-coded destination

Firmware: `MicroPython v1.29.0 on 2026-08-24; Raspberry Pi Pico with RP2040`

## Detailed Objectives

- [x] Determine size in bytes of original `./firmware/` directory
  - `du -sb firmware/`
    - `1116683 firmware/`
- [x] List free file system space on RP2040
  - `mpremote rm -r :.` to erase all files
  - `mpremote df` to show available storage

    ```text
    filesystem     size     used    avail use% mounted on
    <VfsLfs2>   1441792     8192  1433600    0 /
    ```

  - `1,433,600 bytes`

## Motivation & RP2040 Issues

- The work in the upstream repository focuses on ESP32 based devices with PSRAM.
- The RP2040 has significantly less RAM and flash space, requiring a different approach.
- The instructions in the upstream repo result in the following errors:
  - When trying to copy the firmware files `mpremote cp -r firmware/ :`
    - This results in `firmware` as the root directory on the pico
    - Causes all imports (which are looking for a root directory `lib`) to fail
  - Correct command:
    - `mpremote cp -r firmware/. :`, (note the `.`)
  - When corrected, it completes without error, but,
  - Results in a silent out of space condition, as revealed by `mpremote df`:

    ```text
    filesystem     size     used    avail use% mounted on
    <VfsLfs2>   1441792  1495040   -53248  103 /
    ```

  - Resolution: remove examples/libs not intended for pico
