// drv8231_quad_pwm.c  –  4-channel DRV8231 controller (RP2040 I²C slave)
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/* ─── version ─────────────────────────────────────────────────────────────── */
#define FW_VERSION_MAJOR   0
#define FW_VERSION_MINOR   1

/* ─── I²C pins & address ──────────────────────────────────────────────────── */
#define I2C_SDA_PIN        14
#define I2C_SCL_PIN        15
#define I2C_SLAVE_ADDR     0x2C     // 7-bit

/* ─── Channel definitions ─────────────────────────────────────────────────── */
typedef struct {
    uint gpio_high;   // pin held at 100 % during drive
    uint gpio_pwm;    // pin that gets PWM  (drive=0, brake=255-duty)
} hbridge_chan_t;

static const hbridge_chan_t chans[4] = {
    {2, 3},     // CH-0  slice 1
    {4, 5},     // CH-1  slice 2
    {6, 7},     // CH-2  slice 3
    {20, 21},   // CH-3  slice 5
};

/* ─── globals ─────────────────────────────────────────────────────────────── */
static volatile uint8_t drive_byte[4] = {0};   // echoed to reads
static volatile uint8_t reg_ptr       = 0;
static volatile bool    expect_ptr    = true;

/* ─── helper: configure one PWM pin pair ─────────────────────────────────── */
static void pwm_setup_pair(uint high_pin, uint pwm_pin)
{
    gpio_set_function(high_pin, GPIO_FUNC_PWM);
    gpio_set_function(pwm_pin , GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(high_pin);
    pwm_config cfg = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (20000.0f * 256.0f); // 20 kHz, 8-bit
    pwm_config_set_clkdiv(&cfg, div);
    pwm_config_set_wrap(&cfg, 255);
    pwm_init(slice, &cfg, true);

    pwm_set_gpio_level(high_pin, 0);
    pwm_set_gpio_level(pwm_pin , 0);
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
    uint duty  = v & 0x7F;                    // 1-127
    uint brake = 255 - duty;                  // drive/brake method

    if (rev) {
        // IN2 (pwm_pin) held high, IN1 (high_pin) gets PWM
        pwm_set_gpio_level(chans[idx].gpio_pwm , 255);
        pwm_set_gpio_level(chans[idx].gpio_high, brake);
    } else {
        pwm_set_gpio_level(chans[idx].gpio_high, 255);
        pwm_set_gpio_level(chans[idx].gpio_pwm , brake);
    }
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
        } else {
            if (reg_ptr < 4) {      // 0x00-0x03 → channels
                drive_byte[reg_ptr] = data;
                apply_drive(reg_ptr, data);
            }
            reg_ptr++;              // auto-inc (like EEPROMs)
        }
    }

    /* master READ */
    if (s & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        uint8_t out = 0xFF;
        if (reg_ptr < 4)                 out = drive_byte[reg_ptr];
        else if (reg_ptr == 0x10)        out = FW_VERSION_MINOR;
        else if (reg_ptr == 0x11)        out = FW_VERSION_MAJOR;
        i2c1->hw->data_cmd = out;
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

    /* PWM setup for all channels */
    for (int i = 0; i < 4; i++)
        pwm_setup_pair(chans[i].gpio_high, chans[i].gpio_pwm);

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

    while (true) tight_loop_contents();
}
