# V15.21A — J6 DM-G6220 read-only bring-up

## Authority result

| Authority | Status |
|---|---|
| Windows reference motion | `OPERATOR_VERIFIED_PASS` |
| Ubuntu J6 communication | `PASS` |
| Ubuntu J6 100 Hz feedback | `PASS` |
| Ubuntu J6 motion | `NOT_TESTED` |
| Current J6 motion authority | `FALSE` |

This revision establishes a reproducible Ubuntu read-only transport and feedback authority for J6. It does not establish motion, calibration, zero, limit, READY-pose, ROS 2, or MoveIt authority.

## Frozen source gate

- Source branch: `agent/v15-20a-j345-shared-bus-ready-pose`
- Source HEAD: `6040c1d5c35c4e67efaa9cb5729970a7b674fb36`
- Work branch: `agent/v15-21a-j6-dmg6220-readonly`
- The source worktree was clean before branching.

The supplied `DM.zip` was preserved under `tools/hardware/j6_dm_g6220/reference_windows_known_good/`. Its ZIP SHA-256 is:

`b6329bb0c3f1dea74a856b460e360ac86515e02a50820d72ffc62d69395ded89`

The extracted-file hashes are recorded in `SOURCE_SHA256.txt`. The original Windows scripts were reviewed and compiled only; they were not executed on Ubuntu because they contain mode switching, enable/control cycles, and motion. The GUI also exposes a set-zero command. Their status is therefore operator-provided known-good reference, not permission to repeat those actions.

## Physical and transport identity

The operator confirmed the physical gate exactly as:

`J6_DMG6220_POWER_AND_CAN_CONNECTED=YES`

The adapter was ultimately connected directly to the host after the dock path repeatedly enumerated unreliably. The confirmed native adapter identity is:

- Product: `DaMiao-Tech DM-USB2FDCAN`
- USB VID:PID: `34b7:6877`
- USB serial: `EEE8D71AB573449FCAFE7B39BD222C75`
- Adapter application version: `app v1.0.0.5`
- CAN channel: `0`
- CAN bitrate: `1,000,000 bit/s`
- CAN-FD: `false`

Linux also exposes a `/dev/ttyACM0` CDC node. That node is diagnostic enumeration only for this native adapter; production communication uses the vendor `dmcan`/`libusb` SDK path. The Unitree FTDI device with serial `FTASQA6F` is explicitly rejected by the J6 transport tool.

The read-only transport verifies the existing adapter mode and bitrate and blocks if they differ; it does not rewrite adapter channel configuration.

## Reproducible runtime

The verified isolated runtime is described by `tools/hardware/j6_dm_g6220/environment.yml`:

- Python `3.13.15`
- `dmcan-sdk` `1.0.4`
- `libusb` `1.0.29`
- `pyserial` `3.5`
- `pyusb`

The Python package requires the native `libdm_device.so` backend, which its wheel did not contain. The run used the upstream dmBots `dm-device-sdk` binary at commit `36a8b3971dc35608701d128f353a32cad3ee6cdb`, path `C&C++/lib/v1.1.0/linux/x86_64/libdm_device.so`, SHA-256 `bec4ca4421f5366c90fa11cc34f2f683ed71a6372e15c265e7e4b17e720607dd`.

Because that upstream repository did not provide a license file establishing redistribution authority, the binary is not committed here. Exact provenance is recorded in `dm_device_runtime_SOURCE.txt`. A udev rule for VID:PID `34b7:6877` is versioned under `tools/hardware/j6_dm_g6220/udev/`.

## Read-only discovery

Read-only register queries over candidate IDs found exactly one motor:

- Motor ID: `1`
- Control mode readback: `1` (`MIT`)
- PMAX readback: `12.5 rad`
- Initial feedback state: `0` (`DISABLED`)
- Initial position: approximately `1.05421 rad`
- Initial velocity: approximately `-0.01099 rad/s`
- Initial protocol torque value: approximately `-0.002442`
- MOS temperature: `39 °C`
- Coil temperature: `35 °C`

PMAX and the configured velocity/torque mappings are protocol encoding ranges. They are not commissioned J6 physical limits.

The motor was already `DISABLED`, so the optional safety FD-disable command was not sent.

## 100 Hz feedback evidence

The authoritative run is:

- CSV: `hardware/v15_21a/j6_feedback100_20260820T081810Z.csv`
- SHA-256: `d82d52c6c432e976db472bf2d551a13af890a2685d6950afb87801d76a712d96`
- Planned/valid: `500/500`
- Valid rate: `1.0`
- Request send duration: `5.000191860 s`
- Feedback span: `4.913044719 s`
- Feedback rate: `101.566346 Hz`
- Mean/p95/max host-observed request-to-callback latency: `50.505 / 95.317 / 100.459 ms`
- Maximum consecutive timeout: `0`
- Extra unmatched feedback: `0`
- State distribution: `DISABLED = 500`
- Position range/span: `1.053444724 .. 1.054589151 rad`, span `0.001144427 rad`
- MOS temperature: constant `39 °C`
- Coil temperature: constant `35 °C`

The relatively large host-observed latency reflects SDK callback batching plus FIFO matching in this probe; it is preserved as measured and must not be reinterpreted as a per-frame deterministic control-loop latency claim.

An earlier run is deliberately preserved as failed evidence:

- CSV: `hardware/v15_21a/j6_feedback100_20260820T081327Z.csv`
- SHA-256: `f0b8bbab16aca0240278f4e67a9e90d6b8fd111135e27c233a8ee32eee0ded75`
- Result: `42/500`

The failure was caused by the first probe implementation clearing queued SDK callbacks for each request. A diagnostic request batch returned all requested valid disabled feedback, which isolated the issue to host-side callback handling rather than the motor or bus. The corrected probe timestamps every callback, drains the batch within a bounded window, and FIFO-matches responses. No motor state change was needed to diagnose or correct it.

## Safety accounting

The entire Ubuntu bring-up used no active enable, motor motion, parameter write, set-zero, mode switch, ID write, or EEPROM write. It did not modify READY_POSE_V1, J1 authority, J3/J4/J5 authority, or any frozen simulation authority.

`dm_g6220_transport.py` and `j6_readonly_probe.py` intentionally expose no motion, enable, set-zero, mode-write, parameter-write, or ID-write path. The only state-changing method in the transport is a narrowly scoped safety disable, and the authoritative run records that it was not used because feedback was already `DISABLED`.

## Still pending

- J6 motion commissioning
- Joint direction/sign
- CAD zero and ROS zero
- Physical joint limits
- Motor internal zero remains unchanged
- READY reference and startup integration
- ROS 2 / ros2_control / MoveIt integration
- Loaded and gravity-dependent behavior

No item in this section is authorized by V15.21A.
