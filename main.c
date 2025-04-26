/*
 * RP2040 H-bridge controller
 *
 * I2C slave 0x2C
 *   Reg 0: duty (0-255)
 *   Reg 1: state (0-3)
 *   Reg 2: version  (read-only minor, major)
 *
 * H-bridge inputs
 *   GP2  → IN1
 *   GP3  → IN2
 */

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

// ───────────────────────────────────────────────────────────
// Configuration
// ───────────────────────────────────────────────────────────
#define I2C_ADDR          0x2C

// I2C1 on GP14 = SDA, GP15 = SCL (kept from your prior design)
#define I2C_SDA_PIN       14
#define I2C_SCL_PIN       15

// H-bridge pins
#define HBRIDGE_IN1_PIN   2   // GP2
#define HBRIDGE_IN2_PIN   3   // GP3

// Version
#define FW_VERSION_MAJOR  0
#define FW_VERSION_MINOR  1

// Registers
volatile uint8_t i2c_pointer = 0;
volatile uint8_t duty_reg    = 0;  // 0-255
volatile uint8_t state_reg   = 0;  // 0-3

// ───────────────────────────────────────────────────────────
// Helper: configure one pin for PWM (wrap = 255 for 8-bit duty)
// ───────────────────────────────────────────────────────────
static void pwm_init_8bit(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config cfg = pwm_get_default_config();
    // 40 kHz PWM (well above audible, and fine for common H-bridges)
    float div = (float)clock_get_hz(clk_sys) / (40000.0f * 256.0f);
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, 255);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(pin, 0);
}

// ───────────────────────────────────────────────────────────
// Apply H-bridge state + duty to pins
// ───────────────────────────────────────────────────────────
static void apply_hbridge_state() {
    uint slice1 = pwm_gpio_to_slice_num(HBRIDGE_IN1_PIN);
    uint slice2 = pwm_gpio_to_slice_num(HBRIDGE_IN2_PIN);

    switch (state_reg) {
        case 0:     // Coast  (unchanged)
            pwm_set_enabled(slice1, false);
            pwm_set_enabled(slice2, false);
            gpio_put(HBRIDGE_IN1_PIN, 0);
            gpio_put(HBRIDGE_IN2_PIN, 0);
            break;

        case 1:     // Forward, drive<->brake on IN2
            // IN1 held high
            pwm_set_enabled(slice1, false);
            gpio_put(HBRIDGE_IN1_PIN, 1);

            // IN2 PWM, duty = (255-duty_reg)  so 255→full drive, 0→full brake
            pwm_set_wrap(slice2, 255);
            pwm_set_gpio_level(HBRIDGE_IN2_PIN, 255 - duty_reg);
            pwm_set_enabled(slice2, true);
            break;

        case 2:     // Reverse, drive<->brake on IN1
            // IN2 held high
            pwm_set_enabled(slice2, false);
            gpio_put(HBRIDGE_IN2_PIN, 1);

            // IN1 PWM, inverted duty
            pwm_set_wrap(slice1, 255);
            pwm_set_gpio_level(HBRIDGE_IN1_PIN, 255 - duty_reg);
            pwm_set_enabled(slice1, true);
            break;

        case 3:     // Brake (both high, no PWM)  (unchanged)
            pwm_set_enabled(slice1, false);
            pwm_set_enabled(slice2, false);
            gpio_put(HBRIDGE_IN1_PIN, 1);
            gpio_put(HBRIDGE_IN2_PIN, 1);
            break;
    }
}


// ───────────────────────────────────────────────────────────
// I2C IRQ handler (slave mode)
// ───────────────────────────────────────────────────────────
void i2c1_irq_handler() {
    uint32_t status = i2c1->hw->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS)
        i2c1->hw->clr_tx_abrt;

    // Master write
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint8_t value = (uint8_t)i2c1->hw->data_cmd;
        if (value & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {
            i2c_pointer = value & 0xFF;
        } else {
            if (i2c_pointer == 0) {
                duty_reg = value;
            } else if (i2c_pointer == 1) {
                state_reg = value & 0x03;     // only 0-3 are valid
            }
            apply_hbridge_state();
        }
    }

    // Master read
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        switch (i2c_pointer) {
            case 0:  i2c1->hw->data_cmd = duty_reg;          break;
            case 1:  i2c1->hw->data_cmd = state_reg;         break;
            case 2:  i2c1->hw->data_cmd = FW_VERSION_MINOR;
                     i2c1->hw->data_cmd = FW_VERSION_MAJOR;  break;
            default: i2c1->hw->data_cmd = 0xFF;              break;
        }
        i2c1->hw->clr_rd_req;
    }

    if (status & I2C_IC_INTR_STAT_R_RX_DONE_BITS)
        i2c1->hw->clr_rx_done;
}

// ───────────────────────────────────────────────────────────
// Main
// ───────────────────────────────────────────────────────────
int main() {
    stdio_init_all();

    // GPIO for PWM
    pwm_init_8bit(HBRIDGE_IN1_PIN);
    pwm_init_8bit(HBRIDGE_IN2_PIN);

    // Default to coast
    apply_hbridge_state();

    // I2C-1, 100 kHz, slave 0x2C
    i2c_init(i2c1, 100000);
    i2c_set_slave_mode(i2c1, true, I2C_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // IRQs for RX_FULL + RD_REQ
    i2c1->hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS |
                          I2C_IC_INTR_MASK_M_RD_REQ_BITS;
    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);

    // Nothing to do in the foreground
    while (true) tight_loop_contents();
}
