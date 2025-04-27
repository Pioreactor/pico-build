// drv8231_ultra_simple.c ─ RP2040 I²C-slave, 1-byte protocol
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

// ─── user pins / address ──────────────────────────────────────────────────────
#define I2C_SDA_PIN       14
#define I2C_SCL_PIN       15
#define I2C_SLAVE_ADDR    0x2C        // 7-bit

#define IN1_PIN           2           // → DRV8231 IN1
#define IN2_PIN           3           // → DRV8231 IN2
// ──────────────────────────────────────────────────────────────────────────────

static volatile uint8_t hbridge_state = 0x00;      // echoes on reads

static inline void apply_state(uint8_t v)
{
    gpio_put(IN1_PIN, v & 0x01);
    gpio_put(IN2_PIN, (v & 0x02) >> 1);
}

/* ─── I²C1 interrupt ───────────────────────────────────────────────────────── */
void i2c1_irq_handler(void)
{
    uint32_t s = i2c1->hw->intr_stat;

    if (s & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) (void)i2c1->hw->clr_tx_abrt;

    /* master WRITE: every byte is the new state */
    if (s & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        hbridge_state = (uint8_t)i2c1->hw->data_cmd & 0x03;
        apply_state(hbridge_state);
    }

    /* master READ: just send the cached state */
    if (s & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        i2c1->hw->data_cmd = hbridge_state;
        (void)i2c1->hw->clr_rd_req;
    }
}

int main(void)
{
    stdio_init_all();

    /* DRV8231 input pins */
    gpio_init(IN1_PIN); gpio_set_dir(IN1_PIN, GPIO_OUT); gpio_put(IN1_PIN, 0);
    gpio_init(IN2_PIN); gpio_set_dir(IN2_PIN, GPIO_OUT); gpio_put(IN2_PIN, 0);

    /* I²C-1 slave */
    i2c_init(i2c1, 100000);
    i2c_set_slave_mode(i2c1, true, I2C_SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    /* enable RX_FULL + RD_REQ interrupts */
    i2c1->hw->intr_mask =
        I2C_IC_INTR_MASK_M_RX_FULL_BITS |
        I2C_IC_INTR_MASK_M_RD_REQ_BITS;

    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);

    while (true) tight_loop_contents();   // all work happens in ISR
}
