# pico-build


This repo is the source code, and provides instructions, for building the .elf file that is loaded to the RP2040 chip (its RAM) on the Pioreactor HAT.

### Building locally on macOS

Smallest setup:

- `brew install cmake arm-none-eabi-gcc`
- `make build`

This uses the vendored `pico_sdk_import.cmake` file to fetch the Pico SDK during CMake configure, so you do not need to clone `pico-sdk` separately for a local build.

The build output is:

- `build/main.elf`


### Building on a Raspberry Pi

 - on a Raspberry Pi, follow the setup instructions in Chapter 1. Quick Pico Setup of [the datasheet](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf).
 - `mkdir ~/pico/pico-examples/pioreactor && cd ~/pico/pico-examples/pioreactor`
 - move `main.c` and `CMakeLists.txt` from this repo to the cwd
 - `cp ../../pico-sdk/external/pico_sdk_import.cmake .`
 - `mkdir pioreactor/build && cd build`
 - `export PICO_SDK_PATH=../../../pico-sdk`
 - `cmake .. && make`
 - if a RP2040 is connected over SWD, you can upload with `openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "init" -c "reset halt" -c "load_image main.elf" -c "resume 0x20000000" -c "exit"`


### I2C API

The RP2040 exposes a register-based I2C slave at address `0x2C` on `GPIO14/15`.
The register pointer auto-increments during reads and writes. Multi-byte telemetry values are little-endian.

On a Raspberry Pi running Debian 13 (Trixie), the usual host tools are:

```sh
sudo apt install i2c-tools python3-smbus
```

The examples below assume:

- the HAT is visible on bus `1`
- the RP2040 slave address is `0x2C`
- commands are run as a user with access to `/dev/i2c-1` or via `sudo`

#### Identity and status

| Register | Width | Access | Description |
| --- | --- | --- | --- |
| `0x00` | 1 | R | API version major |
| `0x01` | 1 | R | API version minor |
| `0x02` | 1 | R | Firmware version major |
| `0x03` | 1 | R | Firmware version minor |
| `0x04` | 1 | R | Status bitfield |
| `0x05` | 1 | R | Capability bitfield, low byte |
| `0x06` | 1 | R | Capability bitfield, high byte |

Status bits in `0x04`:

- bit `0`: TMP1075 `ALERT` active
- bit `1`: current raw `HALL_SNS` pin state
- bit `2`: hall/RPM value is currently valid
- bit `3`: TMP1075 temperature value is currently valid

Capabilities bits in `0x05/0x06`:

- bit `0`: H-bridge outputs
- bit `1`: heater PWM output
- bit `2`: IR PWM outputs
- bit `3`: `HW_VERS` ADC
- bit `4`: `VIN_SNS_MSR` ADC
- bit `5`: hall RPM telemetry
- bit `6`: TMP1075 temperature telemetry
- bit `7`: alert input

#### Control registers

| Register | Width | Access | Description |
| --- | --- | --- | --- |
| `0x08` | 1 | R/W | H-bridge channel 0 drive byte |
| `0x09` | 1 | R/W | H-bridge channel 1 drive byte |
| `0x0A` | 1 | R/W | H-bridge channel 2 drive byte |
| `0x0B` | 1 | R/W | H-bridge channel 3 drive byte |
| `0x0C` | 1 | R/W | Heater PWM duty |
| `0x0D` | 1 | R/W | IR PWM channel 0 duty |
| `0x0E` | 1 | R/W | IR PWM channel 1 duty |
| `0x0F` | 1 | R/W | IR PWM channel 2 duty |
| `0x10` | 1 | R/W | IR PWM channel 3 duty |

#### Telemetry registers

| Register | Width | Access | Description |
| --- | --- | --- | --- |
| `0x11` | 2 | R | `HW_VERS` raw ADC counts |
| `0x13` | 2 | R | `VIN_SNS_MSR` raw ADC counts |
| `0x15` | 2 | R | Hall RPM, unsigned little-endian |
| `0x17` | 2 | R | TMP1075 temperature in signed `1/16 °C` |

#### Examples

Probe the device:

```sh
i2cdetect -y 1
```

Read the API and firmware versions:

```sh
i2cget -y 1 0x2C 0x00
i2cget -y 1 0x2C 0x01
i2cget -y 1 0x2C 0x02
i2cget -y 1 0x2C 0x03
```

Read status and capabilities:

```sh
i2cget -y 1 0x2C 0x04
i2cget -y 1 0x2C 0x05
i2cget -y 1 0x2C 0x06
```

Interpret the status byte in shell:

```sh
status=$(i2cget -y 1 0x2C 0x04)
printf 'status=%s\n' "$status"
```

Write H-bridge outputs:

```sh
i2cset -y 1 0x2C 0x08 0x40
i2cset -y 1 0x2C 0x09 0x80
i2cset -y 1 0x2C 0x0A 0xFF
i2cset -y 1 0x2C 0x0B 0x00
```

Read back the H-bridge output bytes:

```sh
i2cget -y 1 0x2C 0x08
i2cget -y 1 0x2C 0x09
i2cget -y 1 0x2C 0x0A
i2cget -y 1 0x2C 0x0B
```

Set heater PWM duty:

```sh
i2cset -y 1 0x2C 0x0C 0x80
```

Set IR PWM channel 2 to 50%:

```sh
i2cset -y 1 0x2C 0x0F 0x80
```

Set all four IR PWM channels in one transaction:

```sh
i2ctransfer -y 1 w5@0x2C 0x0D 0x10 0x20 0x40 0x80
```

Read all control registers in one block:

```sh
i2ctransfer -y 1 w1@0x2C 0x08 r9
```

Read `HW_VERS` raw ADC counts:

```sh
i2ctransfer -y 1 w1@0x2C 0x11 r2
```

Read `VIN_SNS_MSR` raw ADC counts:

```sh
i2ctransfer -y 1 w1@0x2C 0x13 r2
```

Read the hall RPM telemetry:

```sh
i2ctransfer -y 1 w1@0x2C 0x15 r2
```

Read the TMP1075 temperature telemetry:

```sh
i2ctransfer -y 1 w1@0x2C 0x17 r2
```

Read the full telemetry block in one transaction:

```sh
i2ctransfer -y 1 w1@0x2C 0x11 r8
```

Python example using `python3-smbus` on Raspberry Pi:

```python
import smbus

I2C_BUS = 1
ADDR = 0x2C

REG_STATUS = 0x04
REG_HW_VERS = 0x11
REG_VIN_SNS = 0x13
REG_HALL_RPM = 0x15
REG_HEATER_TEMP = 0x17


def read_u16_le(bus: smbus.SMBus, reg: int) -> int:
    data = bus.read_i2c_block_data(ADDR, reg, 2)
    return data[0] | (data[1] << 8)


def read_s16_le(bus: smbus.SMBus, reg: int) -> int:
    value = read_u16_le(bus, reg)
    if value >= 0x8000:
        value -= 0x10000
    return value


with smbus.SMBus(I2C_BUS) as bus:
    status = bus.read_byte_data(ADDR, REG_STATUS)
    hw_vers_counts = read_u16_le(bus, REG_HW_VERS)
    vin_counts = read_u16_le(bus, REG_VIN_SNS)
    hall_rpm = read_u16_le(bus, REG_HALL_RPM)
    heater_temp_raw = read_s16_le(bus, REG_HEATER_TEMP)
    heater_temp_c = heater_temp_raw / 16.0

    print(f"status=0x{status:02x}")
    print(f"hw_vers_counts={hw_vers_counts}")
    print(f"vin_counts={vin_counts}")
    print(f"hall_rpm={hall_rpm}")
    print(f"heater_temp_c={heater_temp_c:.4f}")
```

Python example to set outputs:

```python
import smbus

I2C_BUS = 1
ADDR = 0x2C

REG_HEATER_PWM = 0x0C
REG_IR_PWM_0 = 0x0D
REG_IR_PWM_1 = 0x0E
REG_IR_PWM_2 = 0x0F
REG_IR_PWM_3 = 0x10


with smbus.SMBus(I2C_BUS) as bus:
    bus.write_byte_data(ADDR, REG_HEATER_PWM, 0x80)
    bus.write_byte_data(ADDR, REG_IR_PWM_0, 0x10)
    bus.write_byte_data(ADDR, REG_IR_PWM_1, 0x20)
    bus.write_byte_data(ADDR, REG_IR_PWM_2, 0x40)
    bus.write_byte_data(ADDR, REG_IR_PWM_3, 0x80)
```
