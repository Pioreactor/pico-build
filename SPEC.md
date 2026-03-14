# HAT-V2 Notes

## VIN sense formula

The `VIN_SENSE` / `vin_sns_msr` path measures the board input rail coming from the USB-C power stage (`+VOUT_USBC`), before the LM25148 buck converter.

Divider values on the MCU sheet:

- `R101 = 64.9k`
- `R102 = 10k`
- `R103 = 1k` series resistor into the ADC

Voltage conversion:

```text
Vin = Vadc * (64.9k + 10k) / 10k
Vin = Vadc * 7.49
```

Equivalent divider ratio:

```text
Vadc = Vin * 0.133511348
```

Examples:

- `5.0 V` in -> `0.668 V` at ADC
- `9.0 V` in -> `1.202 V` at ADC
- `12.0 V` in -> `1.602 V` at ADC
- `20.0 V` in -> `2.670 V` at ADC

## HW_VERS formula

The `HW_VERS` path is a fixed resistor divider into RP2040 `GPIO26` / `ADC0`. It appears to be intended as an analog hardware-revision strap.

Divider values on the board:

- `R91 = 10k` from `+3V3_RPi` to `HW_VERS`
- `R90 = 649R` from `HW_VERS` to `GND`

Voltage at the ADC pin:

```text
Vhw_vers = V3V3 * 649 / (10000 + 649)
```

For a nominal `3.3 V` rail:

```text
Vhw_vers = 3.3V * 649 / 10649
Vhw_vers = 3.3V * 0.0609447
Vhw_vers ≈ 0.201 V
```

Inverse form:

```text
V3V3 = Vhw_vers * (10000 + 649) / 649
```

Expected RP2040 ADC reading, assuming `Vref = 3.3 V` and 12-bit conversion:

```text
Vhw_vers = 3.3V * adc_counts / 4095
adc_counts ≈ 4095 * 0.201 / 3.3 ≈ 250
```

Expected value on the currently stuffed board:

- `HW_VERS` should read about `0.201 V`
- This corresponds to roughly `250` ADC counts on `ADC0`

## RP2040 pinout

This section is based on the `U1` RP2040 footprint net assignments in `HAT-V2.kicad_pcb`, not just symbol labels. Use this as the firmware-facing source of truth for board pin assignments.

### Summary

- MCU: `RP2040` / `SC0914`
- Designator: `U1`
- Package: `QFN-56`
- Logic rail: `+3V3_RPi`
- Core rail: `+1V1_DVDD`
- External USB data pins are not used on this board.
- External QSPI pins are not used on this board.
- `GPIO26` and `GPIO27` are used as analog inputs.
- `GPIO28` and `GPIO29` are unused.

### Firmware-visible GPIO assignments

| RP2040 GPIO | Package pad | Net / label | Board function | Firmware notes |
| --- | ---: | --- | --- | --- |
| `GPIO2` | `4` | `/MCU_RP2040/HEATER_PWM_R` | Heater PWM output | PWM/control output for heater stage. |
| `GPIO3` | `5` | `/HALL_SNS` | Hall sensor input | Sensor input from heater sheet. |
| `GPIO4` | `6` | `/ALERT` | Alert input | Alert/fault-style input from heater sheet. |
| `GPIO5` | `7` | `/MCU_RP2040/I2C_SCL_HEATER_RP2040` | Heater I2C SCL | Dedicated heater-side I2C clock. |
| `GPIO6` | `8` | `/MCU_RP2040/I2C_SDA_HEATER_RP2040` | Heater I2C SDA | Dedicated heater-side I2C data. |
| `GPIO8` | `11` | `/MCU_RP2040/IR_PWM_0_R` | IR PWM channel 0 | Output to IR LED driver. |
| `GPIO9` | `12` | `/MCU_RP2040/IR_PWM_1_R` | IR PWM channel 1 | Output to IR LED driver. |
| `GPIO10` | `13` | `/MCU_RP2040/IR_PWM_2_R` | IR PWM channel 2 | Output to IR LED driver. |
| `GPIO11` | `14` | `/MCU_RP2040/IR_PWM_3_R` | IR PWM channel 3 | Output to IR LED driver. |
| `GPIO14` | `17` | `/MCU_RP2040/I2C_SDA_RP2040` | Main I2C SDA | Main board I2C data line. |
| `GPIO15` | `18` | `/MCU_RP2040/I2C_SCL_RP2040` | Main I2C SCL | Main board I2C clock line. |
| `GPIO16` | `27` | `/MCU_RP2040/H_BRIDGE2_IN1_R` | H-bridge 2 IN1 | Motor/bridge control output. |
| `GPIO17` | `28` | `/MCU_RP2040/H_BRIDGE2_IN2_R` | H-bridge 2 IN2 | Motor/bridge control output. |
| `GPIO18` | `29` | `/MCU_RP2040/H_BRIDGE1_IN1_R` | H-bridge 1 IN1 | Motor/bridge control output. |
| `GPIO19` | `30` | `/MCU_RP2040/H_BRIDGE1_IN2_R` | H-bridge 1 IN2 | Motor/bridge control output. |
| `GPIO20` | `31` | `/MCU_RP2040/H_BRIDGE4_IN1_R` | H-bridge 4 IN1 | Motor/bridge control output. |
| `GPIO21` | `32` | `/MCU_RP2040/H_BRIDGE4_IN2_R` | H-bridge 4 IN2 | Motor/bridge control output. |
| `GPIO22` | `34` | `/MCU_RP2040/H_BRIDGE3_IN1_R` | H-bridge 3 IN1 | Motor/bridge control output. |
| `GPIO23` | `35` | `/MCU_RP2040/H_BRIDGE3_IN2_R` | H-bridge 3 IN2 | Motor/bridge control output. |
| `GPIO26` | `38` | `/MCU_RP2040/HW_VERS` | Hardware version sense | Analog-capable pin. Used to read board hardware version encoding. RP2040 ADC channel `ADC0`. |
| `GPIO27` | `39` | `/MCU_RP2040/VIN_SNS_MSR` | Input voltage sense | Analog-capable pin. Measures board input voltage through divider described above. RP2040 ADC channel `ADC1`. |

### Debug, clock, and support pins

| RP2040 pin | Package pad | Net / label | Purpose | Firmware notes |
| --- | ---: | --- | --- | --- |
| `SWCLK` | `24` | `/SWD_CLK` | SWD debug clock | Debug/programming interface. |
| `SWDIO` | `25` | `/SWD_DATA` | SWD debug data | Debug/programming interface. |


### Firmware notes

- `GPIO26` (`HW_VERS`) and `GPIO27` (`VIN_SNS_MSR`) are the two active analog channels in use.
- `GPIO28` and `GPIO29` are available in silicon for ADC use, but unconnected on this board.
- `GPIO0` is the most likely firmware-controlled buck enable pin, but it is named `BUCK_EN_n`. The board also has optional DNP inversion parts around this path, so firmware should preserve configurability here.
- Main board I2C is on `GPIO14/15`.
- Heater-local I2C is on `GPIO5/6`.
- IR PWM outputs are grouped on `GPIO8-11`.
- H-bridge control outputs are grouped on `GPIO16-23`.
