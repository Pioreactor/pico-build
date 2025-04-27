// drv8231_minimal.c  –  RP2040 I²C-slave → DRV8231 driver (no PWM)
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

// ─────────── user-editable pins and address ───────────
#define I2C_SLAVE_ADDR  0x2C
#define I2C_SDA_PIN     14
#define I2C_SCL_PIN     15

#define IN1_PIN         2      // RP2040 pin to DRV8231 IN1
#define IN2_PIN         3      // RP2040 pin to DRV8231 IN2
// ──────────────────────────────────────────────────────

// cached register value so we can echo it back on reads
static volatile uint8_t reg_state = 0x00;   // default = coast
static volatile uint8_t reg_pointer = 0x00; // auto-increment pointer

static inline void apply_state(uint8_t v)
{
    bool in1 = false, in2 = false;
    switch (v & 0x03) {          // only lowest two bits matter
        case 0x01: in1 = true;                 break;   // forward
        case 0x02:            in2 = true;      break;   // reverse
        case 0x03: in1 = true; in2 = true;     break;   // brake
        default: break;                        // 0x00 → both low (coast)
    }
    gpio_put(IN1_PIN, in1);
    gpio_put(IN2_PIN, in2);
}

// ─────────── I²C interrupt handler ───────────
void i2c1_irq_handler(void)
{
    uint32_t status = i2c1->hw->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS)
        (void)i2c1->hw->clr_tx_abrt;

    /* ---------- master WRITE ---------- */
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint8_t data = (uint8_t)i2c1->hw->data_cmd;
        if (data & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {
            reg_pointer = data & 0xFF;          // first byte = register address
        } else {
            if (reg_pointer == 0x00) {
                reg_state = data;
                apply_state(reg_state);
            }
            reg_pointer++;                      // EEPROM-style auto-increment
        }
    }

    /* ---------- master READ ---------- */
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        uint8_t out = 0xFF;
        if (reg_pointer == 0x00) out = reg_state;
        i2c1->hw->data_cmd = out;
        (void)i2c1->hw->clr_rd_req;
        reg_pointer++;
    }

    if (status & I2C_IC_INTR_STAT_R_RX_DONE_BITS)
        (void)i2c1->hw->clr_rx_done;
}

// ─────────── main ───────────
int main(void)
{
    stdio_init_all();

    /* --- GPIO for H-bridge inputs --- */
    gpio_init(IN1_PIN);  gpio_set_dir(IN1_PIN, GPIO_OUT);  gpio_put(IN1_PIN, 0);
    gpio_init(IN2_PIN);  gpio_set_dir(IN2_PIN, GPIO_OUT);  gpio_put(IN2_PIN, 0);

    /* --- I²C-1 slave setup --- */
    i2c_init(i2c1, 100000);                       // 100 kHz
    i2c_set_slave_mode(i2c1, true, I2C_SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    /* --- enable IRQs we care about --- */
    i2c1->hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS |
                          I2C_IC_INTR_MASK_M_RD_REQ_BITS;
    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);

    while (true) tight_loop_contents();           // all work is interrupt-driven
}
