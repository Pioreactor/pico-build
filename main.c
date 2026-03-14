// drv8231_quad_pwm.c  –  4-channel DRV8231 controller (RP2040 I²C slave)
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/* ─── version ─────────────────────────────────────────────────────────────── */
#define FW_VERSION_MAJOR   1
#define FW_VERSION_MINOR   0
#define API_VERSION_MAJOR  1
#define API_VERSION_MINOR  0

/* ─── I²C pins & address ──────────────────────────────────────────────────── */
#define I2C_SDA_PIN        14
#define I2C_SCL_PIN        15
#define I2C_SLAVE_ADDR     0x2C     // 7-bit
#define HEATER_PWM_PIN     2
#define HALL_PIN           3
#define ALERT_PIN          4
#define HEATER_I2C_SCL_PIN 5
#define HEATER_I2C_SDA_PIN 6
#define IR_PWM_0_PIN       8
#define IR_PWM_1_PIN       9
#define IR_PWM_2_PIN       10
#define IR_PWM_3_PIN       11
#define HW_VERS_ADC_INPUT  0
#define VIN_SNS_ADC_INPUT  1
#define HW_VERS_GPIO       26
#define VIN_SNS_GPIO       27

/* ─── I²C register map v2 ─────────────────────────────────────────────────── */
#define REG_API_VERSION_MAJOR 0x00
#define REG_API_VERSION_MINOR 0x01
#define REG_FW_VERSION_MAJOR  0x02
#define REG_FW_VERSION_MINOR  0x03
#define REG_STATUS            0x04
#define REG_CAPABILITIES_LO   0x05
#define REG_CAPABILITIES_HI   0x06

#define REG_HBRIDGE_FIRST     0x08
#define REG_HBRIDGE_LAST      0x0B
#define REG_HEATER_PWM        0x0C
#define REG_IR_PWM_FIRST      0x0D
#define REG_IR_PWM_LAST       0x10

#define REG_TELEMETRY_FIRST   0x11
#define REG_HW_VERS_LO        0x11
#define REG_HW_VERS_HI        0x12
#define REG_VIN_SNS_LO        0x13
#define REG_VIN_SNS_HI        0x14
#define REG_HALL_RPM_LO       0x15
#define REG_HALL_RPM_HI       0x16
#define REG_HEATER_TEMP_LO    0x17
#define REG_HEATER_TEMP_HI    0x18
#define REG_TELEMETRY_LAST    0x18

#define TMP1075_ADDR       0x4F
#define TMP1075_TEMP_PTR   0x00
#define TMP1075_POLL_US    100000u
#define TMP1075_TIMEOUT_US 2000u
#define HALL_MIN_PERIOD_US 1000u
#define HALL_TIMEOUT_US    1500000u
#define STATUS_ALERT_ACTIVE_BIT 0x01
#define STATUS_HALL_STATE_BIT   0x02
#define STATUS_HALL_VALID_BIT   0x04
#define STATUS_TEMP_VALID_BIT   0x08

#define CAPABILITY_HBRIDGE      0x0001
#define CAPABILITY_HEATER_PWM   0x0002
#define CAPABILITY_IR_PWM       0x0004
#define CAPABILITY_HW_VERS_ADC  0x0008
#define CAPABILITY_VIN_SNS_ADC  0x0010
#define CAPABILITY_HALL_RPM     0x0020
#define CAPABILITY_TMP1075_TEMP 0x0040
#define CAPABILITY_ALERT_INPUT  0x0080
#define CAPABILITIES_MASK ( \
    CAPABILITY_HBRIDGE | \
    CAPABILITY_HEATER_PWM | \
    CAPABILITY_IR_PWM | \
    CAPABILITY_HW_VERS_ADC | \
    CAPABILITY_VIN_SNS_ADC | \
    CAPABILITY_HALL_RPM | \
    CAPABILITY_TMP1075_TEMP | \
    CAPABILITY_ALERT_INPUT \
)

/* ─── Channel definitions ─────────────────────────────────────────────────── */
typedef struct {
    uint gpio_high;   // pin held at 100 % during drive
    uint gpio_pwm;    // pin that gets PWM  (drive=0, brake=255-duty)
} hbridge_chan_t;

static const hbridge_chan_t chans[4] = {
    {22, 23},   // CH-0 -> physical 0 -> H_BRIDGE3_IN1_R / H_BRIDGE3_IN2_R
    {20, 21},   // CH-1 -> physical 1 -> H_BRIDGE4_IN1_R / H_BRIDGE4_IN2_R
    {18, 19},   // CH-2 -> physical 2 -> H_BRIDGE1_IN1_R / H_BRIDGE1_IN2_R
    {16, 17},   // CH-3 -> physical 3 -> H_BRIDGE2_IN1_R / H_BRIDGE2_IN2_R
};

/* ─── globals ─────────────────────────────────────────────────────────────── */
static volatile uint8_t drive_byte[4] = {0};   // echoed to reads
static volatile uint8_t heater_pwm_duty = 0;
static volatile uint8_t ir_pwm_duty[4] = {0};
static volatile int16_t tmp1075_temperature_c16 = 0;
static volatile bool tmp1075_temperature_valid = false;
static volatile uint32_t hall_last_edge_us = 0;
static volatile uint32_t hall_period_us = 0;
static volatile bool hall_has_period = false;
static uint8_t telemetry_cache[REG_TELEMETRY_LAST - REG_TELEMETRY_FIRST + 1] = {0};
static volatile uint8_t reg_ptr       = 0;
static volatile bool    expect_ptr    = true;
static const uint8_t ir_pwm_pins[4] = {IR_PWM_0_PIN, IR_PWM_1_PIN, IR_PWM_2_PIN, IR_PWM_3_PIN};

/* ─── helper: configure one PWM pin pair ─────────────────────────────────── */
static void pwm_setup_output(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (20000.0f * 256.0f); // 20 kHz, 8-bit
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, 255);
    pwm_init(slice, &cfg, true);

    pwm_set_gpio_level(pin, 0);
}

static void pwm_setup_ir_output(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (325000.0f * 256.0f);
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, 256);
    pwm_init(slice, &cfg, true);

    pwm_set_gpio_level(pin, 0);
}

static void pwm_setup_pair(uint high_pin, uint pwm_pin)
{
    pwm_setup_output(high_pin);
    pwm_setup_output(pwm_pin);
}

/* ─── apply a drive byte to a channel ─────────────────────────────────────── */
static inline void apply_drive(uint idx, uint8_t v)
{
    uint high_pin = chans[idx].gpio_high;
    uint pwm_pin  = chans[idx].gpio_pwm;

    if (v == 0x00) {                          // Coast
        pwm_set_gpio_level(high_pin, 0);
        pwm_set_gpio_level(pwm_pin , 0);
        return;
    }
    if (v == 0xFF) {                          // Brake
        pwm_set_gpio_level(high_pin, 255);
        pwm_set_gpio_level(pwm_pin , 255);
        return;
    }

    bool rev   = v & 0x80;
    uint8_t duty   = (v & 0x7F) << 1;    // scale to 0-254
    uint brake = 255 - duty;             // drive/brake method

    if (rev) {
        // IN2 (pwm_pin) held high, IN1 (high_pin) gets PWM
        pwm_set_gpio_level(chans[idx].gpio_pwm , 255);
        pwm_set_gpio_level(chans[idx].gpio_high, brake);
    } else {
        pwm_set_gpio_level(chans[idx].gpio_high, 255);
        pwm_set_gpio_level(chans[idx].gpio_pwm , brake);
    }
}

static void apply_heater_pwm(uint8_t duty)
{
    heater_pwm_duty = duty;
    pwm_set_gpio_level(HEATER_PWM_PIN, duty);
}

static void apply_ir_pwm(uint idx, uint8_t duty)
{
    ir_pwm_duty[idx] = duty;
    pwm_set_gpio_level(ir_pwm_pins[idx], duty);
}

static uint16_t read_adc_input(uint input)
{
    adc_select_input(input);
    return adc_read();
}

static uint16_t calculate_hall_rpm(void)
{
    if (!hall_has_period || hall_period_us < HALL_MIN_PERIOD_US) {
        return 0;
    }

    uint32_t age_us = time_us_32() - hall_last_edge_us;
    if (age_us > HALL_TIMEOUT_US) {
        return 0;
    }

    return (uint16_t)(60000000u / hall_period_us);
}

static bool hall_rpm_is_valid(void)
{
    if (!hall_has_period || hall_period_us < HALL_MIN_PERIOD_US) {
        return false;
    }

    return (time_us_32() - hall_last_edge_us) <= HALL_TIMEOUT_US;
}

static uint8_t read_status_register(void)
{
    uint8_t status = 0;

    if (!gpio_get(ALERT_PIN)) {
        status |= STATUS_ALERT_ACTIVE_BIT;
    }
    if (gpio_get(HALL_PIN)) {
        status |= STATUS_HALL_STATE_BIT;
    }
    if (hall_rpm_is_valid()) {
        status |= STATUS_HALL_VALID_BIT;
    }
    if (tmp1075_temperature_valid) {
        status |= STATUS_TEMP_VALID_BIT;
    }

    return status;
}

static void write_telemetry_u16(uint8_t reg, uint16_t value)
{
    uint8_t offset = reg - REG_TELEMETRY_FIRST;
    telemetry_cache[offset] = (uint8_t)(value & 0xFFu);
    telemetry_cache[offset + 1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void snapshot_telemetry_registers(void)
{
    write_telemetry_u16(REG_HW_VERS_LO, read_adc_input(HW_VERS_ADC_INPUT));
    write_telemetry_u16(REG_VIN_SNS_LO, read_adc_input(VIN_SNS_ADC_INPUT));
    write_telemetry_u16(REG_HALL_RPM_LO, calculate_hall_rpm());
    write_telemetry_u16(REG_HEATER_TEMP_LO, (uint16_t)tmp1075_temperature_c16);
}

static bool reg_is_telemetry(uint8_t reg)
{
    return reg >= REG_TELEMETRY_FIRST && reg <= REG_TELEMETRY_LAST;
}

static uint8_t telemetry_byte_for_reg(uint8_t reg)
{
    return telemetry_cache[reg - REG_TELEMETRY_FIRST];
}

static bool reg_is_hbridge_control(uint8_t reg)
{
    return reg >= REG_HBRIDGE_FIRST && reg <= REG_HBRIDGE_LAST;
}

static bool reg_is_ir_pwm(uint8_t reg)
{
    return reg >= REG_IR_PWM_FIRST && reg <= REG_IR_PWM_LAST;
}

static bool update_tmp1075_temperature(void)
{
    uint8_t pointer = TMP1075_TEMP_PTR;
    uint8_t buffer[2] = {0};

    int written = i2c_write_timeout_us(i2c0, TMP1075_ADDR, &pointer, 1, true, TMP1075_TIMEOUT_US);
    if (written != 1) {
        return false;
    }

    int read = i2c_read_timeout_us(i2c0, TMP1075_ADDR, buffer, 2, false, TMP1075_TIMEOUT_US);
    if (read != 2) {
        tmp1075_temperature_c16 = 0;
        return false;
    }

    int16_t raw_temp = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    tmp1075_temperature_c16 = raw_temp / 16;
    return true;
}

static uint8_t read_register_value(uint8_t reg)
{
    if (reg == REG_API_VERSION_MAJOR) {
        return API_VERSION_MAJOR;
    }
    if (reg == REG_API_VERSION_MINOR) {
        return API_VERSION_MINOR;
    }
    if (reg == REG_FW_VERSION_MAJOR) {
        return FW_VERSION_MAJOR;
    }
    if (reg == REG_FW_VERSION_MINOR) {
        return FW_VERSION_MINOR;
    }
    if (reg == REG_STATUS) {
        return read_status_register();
    }
    if (reg == REG_CAPABILITIES_LO) {
        return (uint8_t)(CAPABILITIES_MASK & 0xFFu);
    }
    if (reg == REG_CAPABILITIES_HI) {
        return (uint8_t)((CAPABILITIES_MASK >> 8) & 0xFFu);
    }
    if (reg_is_hbridge_control(reg)) {
        return drive_byte[reg - REG_HBRIDGE_FIRST];
    }
    if (reg == REG_HEATER_PWM) {
        return heater_pwm_duty;
    }
    if (reg_is_ir_pwm(reg)) {
        return ir_pwm_duty[reg - REG_IR_PWM_FIRST];
    }
    if (reg_is_telemetry(reg)) {
        return telemetry_byte_for_reg(reg);
    }

    return 0xFF;
}

static void hall_gpio_irq_handler(uint gpio, uint32_t events)
{
    if (gpio != HALL_PIN || (events & GPIO_IRQ_EDGE_RISE) == 0) {
        return;
    }

    uint32_t now_us = time_us_32();
    if (hall_last_edge_us != 0) {
        uint32_t period_us = now_us - hall_last_edge_us;
        if (period_us >= HALL_MIN_PERIOD_US) {
            hall_period_us = period_us;
            hall_has_period = true;
        }
    }

    hall_last_edge_us = now_us;
}

/* ─── I²C1 interrupt ─────────────────────────────────────────────────────── */
void i2c1_irq_handler(void)
{
    uint32_t s = i2c1->hw->intr_stat;

    if (s & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) (void)i2c1->hw->clr_tx_abrt;

    /* master WRITE */
    if (s & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint8_t data = (uint8_t)i2c1->hw->data_cmd;

        if (expect_ptr) {           // first byte = register ptr
            reg_ptr    = data;
            expect_ptr = false;
            if (reg_is_telemetry(reg_ptr)) {
                snapshot_telemetry_registers();
            }
        } else {
            if (reg_is_hbridge_control(reg_ptr)) {
                uint8_t idx = reg_ptr - REG_HBRIDGE_FIRST;
                drive_byte[idx] = data;
                apply_drive(idx, data);
            } else if (reg_ptr == REG_HEATER_PWM) {
                apply_heater_pwm(data);
            } else if (reg_is_ir_pwm(reg_ptr)) {
                apply_ir_pwm(reg_ptr - REG_IR_PWM_FIRST, data);
            }
            reg_ptr++;              // auto-inc (like EEPROMs)
        }
    }

    /* master READ */
    if (s & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        if (reg_ptr == REG_TELEMETRY_FIRST) {
            snapshot_telemetry_registers();
        }
        i2c1->hw->data_cmd = read_register_value(reg_ptr);
        reg_ptr++;
        (void)i2c1->hw->clr_rd_req;
    }

    /* STOP detected → next byte will be a new pointer */
    if (s & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {
        (void)i2c1->hw->clr_stop_det;
        expect_ptr = true;
    }
}

/* ─── main ───────────────────────────────────────────────────────────────── */
int main(void)
{
    stdio_init_all();

    adc_init();
    adc_gpio_init(HW_VERS_GPIO);
    adc_gpio_init(VIN_SNS_GPIO);

    gpio_init(HALL_PIN);
    gpio_set_dir(HALL_PIN, GPIO_IN);
    gpio_disable_pulls(HALL_PIN);

    gpio_init(ALERT_PIN);
    gpio_set_dir(ALERT_PIN, GPIO_IN);
    gpio_pull_up(ALERT_PIN);

    i2c_init(i2c0, 100000);
    gpio_set_function(HEATER_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(HEATER_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(HEATER_I2C_SCL_PIN);
    gpio_pull_up(HEATER_I2C_SDA_PIN);

    /* PWM setup for all channels */
    for (int i = 0; i < 4; i++)
        pwm_setup_pair(chans[i].gpio_high, chans[i].gpio_pwm);
    pwm_setup_output(HEATER_PWM_PIN);
    for (int i = 0; i < 4; i++)
        pwm_setup_ir_output(ir_pwm_pins[i]);

    /* I²C-1 slave */
    i2c_init(i2c1, 100000);
    i2c_set_slave_mode(i2c1, true, I2C_SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    i2c1->hw->intr_mask =
        I2C_IC_INTR_MASK_M_RX_FULL_BITS |
        I2C_IC_INTR_MASK_M_RD_REQ_BITS  |
        I2C_IC_INTR_MASK_M_STOP_DET_BITS;

    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);
    gpio_set_irq_enabled_with_callback(HALL_PIN, GPIO_IRQ_EDGE_RISE, true, &hall_gpio_irq_handler);

    uint32_t next_tmp1075_poll_us = 0;
    while (true) {
        uint32_t now_us = time_us_32();
        if ((int32_t)(now_us - next_tmp1075_poll_us) >= 0) {
            tmp1075_temperature_valid = update_tmp1075_temperature();
            next_tmp1075_poll_us = now_us + TMP1075_POLL_US;
        }
        tight_loop_contents();
    }
}
