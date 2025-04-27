// drv8231_control.c  –  RP2040 slave that drives a DRV8231 H-bridge
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

// ──────────────────────────  user-editable constants  ─────────────────────────
#define I2C_SLAVE_ADDR  0x2C

// I²C-1 pins (match your wiring)
#define I2C_SDA_PIN     14
#define I2C_SCL_PIN     15

// H-bridge inputs          RP2040 pin      PWM slice / channel
#define IN1_PIN          2                 // PWM1_A
#define IN2_PIN          3                 // PWM1_B
// ───────────────────────────────────────────────────────────────────────────────

// internal “registers”
static volatile uint8_t reg_pointer = 0;
static volatile uint8_t reg_speed   = 0;   // 0-255
static volatile uint8_t reg_dir     = 0;   // bit0: 0=fwd,1=rev

static inline void motor_update(void)
{
    uint slice      = pwm_gpio_to_slice_num(IN1_PIN);
    uint16_t level_drive = 0;       // low = drive
    uint16_t level_brake = 255;     // high = brake

    if (reg_speed == 0) {                   // coast
        pwm_set_gpio_level(IN1_PIN, 0);
        pwm_set_gpio_level(IN2_PIN, 0);
        return;
    }

    uint16_t duty_brake = 255 - reg_speed;  // invert for drive/brake method

    if (reg_dir & 0x01) {                   // reverse
        pwm_set_gpio_level(IN2_PIN, level_brake);
        pwm_set_gpio_level(IN1_PIN, duty_brake);
    } else {                                // forward
        pwm_set_gpio_level(IN1_PIN, level_brake);
        pwm_set_gpio_level(IN2_PIN, duty_brake);
    }
}

// ───────────────────────────── I²C IRQ handler ────────────────────────────────
void i2c1_irq_handler(void)
{
    uint32_t status = i2c1->hw->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS)
        (void)i2c1->hw->clr_tx_abrt;

    // master WRITE
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint8_t data = (uint8_t)i2c1->hw->data_cmd;

        if (data & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {  // first byte = pointer
            reg_pointer = data & 0xFF;
        } else {
            switch (reg_pointer) {
                case 0x00: reg_speed = data;  motor_update(); break;
                case 0x01: reg_dir   = data;  motor_update(); break;
                default: break;
            }
            reg_pointer++;   // auto-increment like many EEPROMs
        }
    }

    // master READ
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        uint8_t out = 0xFF;
        switch (reg_pointer) {
            case 0x00: out = reg_speed; break;
            case 0x01: out = reg_dir  ; break;
        }
        i2c1->hw->data_cmd = out;
        (void)i2c1->hw->clr_rd_req;
        reg_pointer++;
    }

    if (status & I2C_IC_INTR_STAT_R_RX_DONE_BITS)
        (void)i2c1->hw->clr_rx_done;
}

// ────────────────────────────────  setup  ─────────────────────────────────────
static void setup_pwm_pin(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();

    // ~20 kHz PWM: f = clk_sys / (div * 256) ⇒ div = clk_sys / (20 k * 256)
    float div = (float)clock_get_hz(clk_sys) / (20000.0f * 256.0f);
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, 255);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pin, 0);
}

// ────────────────────────────────  main  ──────────────────────────────────────
int main(void)
{
    stdio_init_all();

    // I²C-1 slave
    i2c_init(i2c1, 100000);
    i2c_set_slave_mode(i2c1, true, I2C_SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // H-bridge pins
    setup_pwm_pin(IN1_PIN);
    setup_pwm_pin(IN2_PIN);

    // enable I²C interrupts
    i2c1->hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS |
                          I2C_IC_INTR_MASK_M_RD_REQ_BITS;
    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);

    while (true) tight_loop_contents();     // all work is interrupt-driven
}
