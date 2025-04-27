// drv8231_pwm.c  –  RP2040 I²C-slave → DRV8231 with drive/brake PWM
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/* ─── pins & address ─────────────────────────────────────────────────────── */
#define I2C_SDA_PIN       14
#define I2C_SCL_PIN       15
#define I2C_SLAVE_ADDR    0x2C           // 7-bit address

#define IN1_PIN           2              // → DRV8231 IN1  (PWM1_A)
#define IN2_PIN           3              // → DRV8231 IN2  (PWM1_B)
/* ─────────────────────────────────────────────────────────────────────────── */

static volatile uint8_t motor_byte = 0x00;       // echoed on reads

/* ─── PWM setup (20 kHz, 8-bit) ───────────────────────────────────────────── */
static void pwm_init_hbridge(void)
{
    gpio_set_function(IN1_PIN, GPIO_FUNC_PWM);
    gpio_set_function(IN2_PIN, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(IN1_PIN);     // GP2 & GP3 share slice 1
    pwm_config cfg = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (20000.0f * 256.0f);
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, 255);                  // 8-bit
    pwm_init(slice, &cfg, true);                     // start
}

/* ─── apply motor_byte to IN1/IN2 ─────────────────────────────────────────── */
static inline void apply_motor_byte(uint8_t v)
{
    if (v == 0x00) {                                // Coast
        pwm_set_gpio_level(IN1_PIN, 0);
        pwm_set_gpio_level(IN2_PIN, 0);
        return;
    }
    if (v == 0xFF) {                                // Brake
        pwm_set_gpio_level(IN1_PIN, 255);
        pwm_set_gpio_level(IN2_PIN, 255);
        return;
    }

    bool reverse   = v & 0x80;                      // MSB = direction
    uint8_t duty   = v & 0x7F;                      // 1–127
    uint8_t brake_level = 255 - duty;               // drive/brake scheme

    if (reverse) {                                  // Reverse: IN2 high
        pwm_set_gpio_level(IN2_PIN, 255);           // constant high
        pwm_set_gpio_level(IN1_PIN, brake_level);   // PWM
    } else {                                        // Forward: IN1 high
        pwm_set_gpio_level(IN1_PIN, 255);
        pwm_set_gpio_level(IN2_PIN, brake_level);
    }
}

/* ─── I²C1 interrupt ─────────────────────────────────────────────────────── */
void i2c1_irq_handler(void)
{
    uint32_t s = i2c1->hw->intr_stat;

    if (s & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) (void)i2c1->hw->clr_tx_abrt;

    /* master WRITE: every byte = new motor command */
    if (s & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        motor_byte = (uint8_t)i2c1->hw->data_cmd;
        apply_motor_byte(motor_byte);
    }

    /* master READ: echo last command */
    if (s & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        i2c1->hw->data_cmd = motor_byte;
        (void)i2c1->hw->clr_rd_req;
    }
}

/* ─── main ───────────────────────────────────────────────────────────────── */
int main(void)
{
    stdio_init_all();

    pwm_init_hbridge();                             // start PWM (both 0)

    /* I²C-1 slave */
    i2c_init(i2c1, 100000);
    i2c_set_slave_mode(i2c1, true, I2C_SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    /* Enable RX_FULL + RD_REQ interrupts */
    i2c1->hw->intr_mask =
        I2C_IC_INTR_MASK_M_RX_FULL_BITS |
        I2C_IC_INTR_MASK_M_RD_REQ_BITS;

    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);

    while (true) tight_loop_contents();             // ISR does the work
}
