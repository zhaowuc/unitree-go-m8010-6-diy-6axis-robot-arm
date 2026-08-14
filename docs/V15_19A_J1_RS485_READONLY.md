# V15.19A-1 — J1 GO-M8010-6 RS485 Read-Only Communication

## Result

V15.19A-1 J1_RS485_COMMUNICATION = PASS

MOTOR_MOTION_CONTROL = NOT_STARTED

This task validates only transport, the Unitree actuator protocol, and J1 state feedback. No active motion command was used.

## Execution and wiring

- Execution host: car, Ubuntu 22.04.5 LTS
- Repository branch: agent/v15-19a-j1-rs485-readonly
- Wiring: Ubuntu → FTDI FT4232H four-port USB/RS485 adapter → interface 03 / CHANNEL_3 → J1 GO-M8010-6
- Only J1 was connected during probing, confirmed by the operator before the first frame.

## USB and serial inventory

- USB VID:PID: 0403:6011
- Product: FT4232H Quad HS USB-UART/FIFO IC
- Manufacturer / serial: FTDI / FTASQA6F
- Linux driver: ftdi_sio
- User car belongs to dialout; all four ports were readable and writable without sudo.
- ModemManager classified this FTDI device as unsupported and released all ports.
- No process held a candidate tty during preflight.

| Channel | USB interface | Diagnostic tty | Stable by-id suffix |
|---|---:|---|---|
| CHANNEL_0 | 00 | /dev/ttyUSB0 | FTASQA6F-if00-port0 |
| CHANNEL_1 | 01 | /dev/ttyUSB1 | FTASQA6F-if01-port0 |
| CHANNEL_2 | 02 | /dev/ttyUSB2 | FTASQA6F-if02-port0 |
| CHANNEL_3 | 03 | /dev/ttyUSB3 | FTASQA6F-if03-port0 |

Selected stable port:

/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if03-port0

All four interfaces accepted and reported 4,000,000 baud. The probe explicitly uses a 16-byte response, 20,000 µs timeout, 8 data bits, no parity, one stop bit, and no flow control.

## Official SDK authority

- Repository: https://github.com/unitreerobotics/unitree_actuator_sdk.git
- Commit: 5b79a42d81cd69adac1367db79efdb972a898fd1
- Build: PASS with GCC/G++ 11.4.0 and CMake 3.22.1
- Motor type: GO_M8010_6
- queryMotorMode(GO_M8010_6, BRAKE) returned protocol value 0

The official C++ and Python GO-M8010-6 examples use FOC with non-zero velocity. They were reviewed but never executed; direct execution remains forbidden.

## Safety command

The dedicated probe is tools/hardware/v15_19a_j1_rs485_probe.cpp. Without --execute it performs a no-I/O dry-run. Every executed request used:

- Mode: BRAKE
- Motor ID: 0
- kp=0
- kd=0
- q=0
- dq=0
- tau=0

FOC, calibration, ID changes, EEPROM/parameter writes, zero setting, homing, ROS2, ros2_control, and MoveIt were not used.

## First valid feedback

CHANNEL_0 through CHANNEL_2 each received no response to their single ID-0 frame. CHANNEL_3 returned the first valid frame, after which probing stopped.

- data.correct=true
- motor_id=0
- mode=0
- raw motor position q=5.0669422149658203
- dq=-0.24543750286102295
- tau=0
- temperature=33 °C
- merror=0
- timestamp=2026-08-14T08:57:23Z

The operator confirmed no rotation, vibration, abnormal sound, or alarm after the first frame.

## 20 Hz stability test

- Requested duration: 10 seconds
- Sent / received / valid: 200 / 200 / 200
- Invalid / timeout / bad-correct / wrong-ID: 0 / 0 / 0 / 0
- Non-zero merror: 0
- Valid response rate: 100%
- q_first=5.067133903503418
- q_last=5.0669422149658203
- q_min=5.066750526428223
- q_max=5.067325592041016
- Maximum single-step q change: 0.0003833770751953125 rad
- Temperature range: 33–33 °C

The operator confirmed no motion or abnormal condition. No encoder jump exceeded the 0.05 rad suspicion threshold.

## Explicitly not validated

The reported q is RAW_MOTOR_POSITION, not yet a ROS J1 angle. This task does not validate:

- ROS joint sign or motor direction mapping
- mechanical zero or gear/output convention
- position, velocity, torque, or current control
- current or thermal limits
- homing or calibration
- ros2_control HardwareInterface, /joint_states, MoveIt, or trajectory control

Those topics are outside V15.19A-1 and require separately authorized work.
