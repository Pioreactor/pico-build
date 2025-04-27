#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// ──────────────────────────────────────────────────────────────────────────────
// Configuration
// ──────────────────────────────────────────────────────────────────────────────
#define I2C_SLAVE_ADDR    0x2C      // 7-bit I²C address
#define I2C_SDA_PIN       4         // I²C SDA on GPIO4 (I2C0 SDA)
#define I2C_SCL_PIN       5         // I²C SCL on GPIO5 (I2C0 SCL)

#define HBRIDGE_IN1_PIN   2         // DRV8231 IN1
#define HBRIDGE_IN2_PIN   3         // DRV8231 IN2

#define PWM_FREQ_HZ       1000      // PWM base frequency: 1 kHz
#define PWM_WRAP          255       // 8-bit resolution (0…255)

#define VERSION_MAJOR     1
#define VERSION_MINOR     0

// ──────────────────────────────────────────────────────────────────────────────
// “Registers” exposed over I²C:
//   0 → write/read 8-bit duty cycle (0…255)
//   1 → write/read mode (0=coast,1=forward,2=reverse,3=brake)
//   2 → read  version minor
//   3 → read  version major
// ──────────────────────────────────────────────────────────────────────────────
volatile uint8_t reg_pointer = 0xFF;
volatile uint8_t duty       = 0;
volatile uint8_t mode       = 0;    // 0: coast, 1: forward, 2: reverse, 3: brake

// ──────────────────────────────────────────────────────────────────────────────
// Helper: configure a GPIO for 8-bit PWM
// ──────────────────────────────────────────────────────────────────────────────
static void setup_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    // derive clock divider for PWM_FREQ_HZ with (PWM_WRAP+1) steps
    float div = (float) clock_get_hz(clk_sys) / (PWM_FREQ_HZ * (PWM_WRAP + 1));
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice, &config, true);
    pwm_set_gpio_level(pin, 0);
}

// ──────────────────────────────────────────────────────────────────────────────
// Update IN1/IN2 outputs based on (mode, duty):
//   coast:    IN1=0, IN2=0
//   forward:  IN1=duty, IN2=0
//   reverse:  IN1=0, IN2=duty
//   brake:    IN1=full, IN2=full
// ──────────────────────────────────────────────────────────────────────────────
static void apply_hbridge() {
    uint16_t level1 = 0, level2 = 0;
    switch (mode) {
        case 1: level1 = duty;        break;  // forward
        case 2: level2 = duty;        break;  // reverse
        case 3: level1 = PWM_WRAP;    // brake
                level2 = PWM_WRAP;
                break;
        default: /* mode=0 coast: both 0 */ break;
    }
    pwm_set_gpio_level(HBRIDGE_IN1_PIN, level1);
    pwm_set_gpio_level(HBRIDGE_IN2_PIN, level2);
}

// ──────────────────────────────────────────────────────────────────────────────
// I²C0 interrupt handler: handle writes to “registers” and read‐requests.
// ──────────────────────────────────────────────────────────────────────────────
void i2c0_irq_handler() {
    uint32_t status = i2c0->hw->intr_stat;

    // clear any TX abort
    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        (void)i2c0->hw->clr_tx_abrt;
    }

    // master WRITE → RX_FULL
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint32_t data = i2c0->hw->data_cmd;
        bool first = data & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS;
        uint8_t val = data & I2C_IC_DATA_CMD_DAT_BITS;
        if (first) {
            // first byte = register pointer
            reg_pointer = val;
            if (reg_pointer > 3) reg_pointer = 0xFF;
        } else {
            // subsequent byte = data for that register
            if (reg_pointer == 0) {
                duty = val;
                apply_hbridge();
            } else if (reg_pointer == 1) {
                mode = val & 0x03;
                apply_hbridge();
            }
        }
    }

    // master READ  → RD_REQ
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        uint8_t out = 0xFF;
        if      (reg_pointer == 0) out = duty;
        else if (reg_pointer == 1) out = mode;
        else if (reg_pointer == 2) out = VERSION_MINOR;
        else if (reg_pointer == 3) out = VERSION_MAJOR;
        i2c0->hw->data_cmd = out;
        (void)i2c0->hw->clr_rd_req;
    }

    // clear RX_DONE
    if (status & I2C_IC_INTR_STAT_R_RX_DONE_BITS) {
        (void)i2c0->hw->clr_rx_done;
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Main
// ──────────────────────────────────────────────────────────────────────────────
int main() {
    stdio_init_all();

    // Setup PWM on IN1/IN2
    setup_pwm_pin(HBRIDGE_IN1_PIN);
    setup_pwm_pin(HBRIDGE_IN2_PIN);
    apply_hbridge();  // start coast

    // Init I²C0 as slave
    i2c_init(i2c0, 100000);
    i2c_set_slave_mode(i2c0, true, I2C_SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Enable only the interrupts we need
    i2c0->hw->intr_mask = I2C_IC_INTR_MASK_M_RD_REQ_BITS
                        | I2C_IC_INTR_MASK_M_RX_FULL_BITS;
    irq_set_exclusive_handler(I2C0_IRQ, i2c0_irq_handler);
    irq_set_enabled(I2C0_IRQ, true);

    // forever idle
    while (1) {
        tight_loop_contents();
    }
    return 0;
}
