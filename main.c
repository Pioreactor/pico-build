// drv8231_simple.c  –  I²C-slave driver, no PWM
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"

// ───────── User-configurable pins / address ──────────
#define I2C_SDA_PIN       14
#define I2C_SCL_PIN       15
#define I2C_SLAVE_ADDR    0x2C

#define IN1_PIN           2     // → DRV8231 IN1
#define IN2_PIN           3     // → DRV8231 IN2
// ─────────────────────────────────────────────────────

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
static volatile uint8_t reg_pointer      = 0x00;   // current register
static volatile bool    expecting_ptr    = true;   // first byte in a msg?
static volatile uint8_t hbridge_state    = 0x00;   // echo'd to reads

// -----------------------------------------------------------------------------
// Helper: drive the two GPIOs according to lower two bits of val
// -----------------------------------------------------------------------------
static inline void apply_state(uint8_t val)
{
    bool in1 = val & 0x01;
    bool in2 = val & 0x02;
    gpio_put(IN1_PIN, in1);
    gpio_put(IN2_PIN, in2 >> 1);          // bit1 ⇒ bool
}

// -----------------------------------------------------------------------------
// I²C1 interrupt handler
// -----------------------------------------------------------------------------
void i2c1_irq_handler(void)
{
    uint32_t status = i2c1->hw->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS)
        (void)i2c1->hw->clr_tx_abrt;

    // ── Master WRITE ──────────────────────────────────
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint8_t data = (uint8_t)i2c1->hw->data_cmd;

        if (expecting_ptr) {              // first byte = pointer
            reg_pointer   = data;
            expecting_ptr = false;
        } else {
            if (reg_pointer == 0x00) {
                hbridge_state = data & 0x03;   // only bits 0-1 matter
                apply_state(hbridge_state);
            }
            reg_pointer++;                // auto-inc like EEPROMs
        }
    }

    // ── Master READ ───────────────────────────────────
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        uint8_t out = 0xFF;
        if (reg_pointer == 0x00) out = hbridge_state;
        i2c1->hw->data_cmd = out;
        (void)i2c1->hw->clr_rd_req;
        reg_pointer++;
    }

    // ── End of I²C message (STOP seen) ────────────────
    if (status & I2C_IC_INTR_STAT_R_RX_DONE_BITS) {
        (void)i2c1->hw->clr_rx_done;
        expecting_ptr = true;             // next msg starts with pointer
    }
}

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(void)
{
    stdio_init_all();

    // GPIO for DRV8231 inputs
    gpio_init(IN1_PIN);  gpio_set_dir(IN1_PIN, GPIO_OUT); gpio_put(IN1_PIN, 0);
    gpio_init(IN2_PIN);  gpio_set_dir(IN2_PIN, GPIO_OUT); gpio_put(IN2_PIN, 0);

    // I²C-1 in slave mode
    i2c_init(i2c1, 100000);                       // 100 kHz
    i2c_set_slave_mode(i2c1, true, I2C_SLAVE_ADDR);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Enable RX_FULL (writes) and RD_REQ (reads) interrupts
    i2c1->hw->intr_mask =
        I2C_IC_INTR_MASK_M_RX_FULL_BITS |
        I2C_IC_INTR_MASK_M_RD_REQ_BITS  |
        I2C_IC_INTR_MASK_M_RX_DONE_BITS;   // detect STOP to reset state machine

    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);

    while (1) tight_loop_contents();      // all work is interrupt-driven
}
