#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"


#define I2C1_PERIPHERAL_ADDR 0x30

#define GPIO_SDA0 14
#define GPIO_SCK0 15

#define LED_CHANNEL_A_PIN 16
#define LED_CHANNEL_B_PIN 17
#define LED_CHANNEL_C_PIN 18
#define LED_CHANNEL_D_PIN 19

#define VERSION_MAJOR 0
#define VERSION_MINOR 3

uint8_t pointer = 0;
uint8_t pwm_channels[4] = {LED_CHANNEL_A_PIN, LED_CHANNEL_B_PIN, LED_CHANNEL_C_PIN, LED_CHANNEL_D_PIN};
uint8_t pwm_duty_cycles[4];

const int randomArray[16] = {2, -1, 1, 0, -2, 2, -1, 0, 1, -2, 0, 2, -1, 1, 0, -2};


void set_up_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    float div = (float)clock_get_hz(clk_sys) / (325000 * 256);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 256);
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);
};

void reset_i2c_bus() {
    i2c_deinit(i2c1);
    i2c_init(i2c1, 100000);
    i2c_set_slave_mode(i2c1, true, I2C1_PERIPHERAL_ADDR);
}

void i2c1_irq_handler() {
    uint32_t status = i2c1->hw->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_RX_DONE_BITS || status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        pointer = 0xFF;  // Reset pointer on NACK or TX abort
        i2c1->hw->clr_rx_done;
        i2c1->hw->clr_tx_abrt;
    }

    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint32_t value = i2c1->hw->data_cmd;

        if (value & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {
            pointer = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);
            if (pointer > 8) {
                pointer = 0xFF;  // Ignore unexpected addresses
            }
        } else {
            if (pointer <= 3){
                pwm_duty_cycles[pointer] = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);
                pwm_set_gpio_level(pwm_channels[pointer], pwm_duty_cycles[pointer]);
            }
        }
    }

    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        if (pointer >= 4 && pointer <= 7){
            uint8_t adc_input = pointer - 4;
            adc_select_input(adc_input);

            const int n_samples = 16;
            const int samples_to_fill_2bytes = 16;
            uint32_t running_sum = 0;
            for (int j = 0; j < n_samples; ++j){
                for (int i = 0; i < samples_to_fill_2bytes; ++i){
                    running_sum = running_sum + adc_read();
                    sleep_us(5 + randomArray[j]);
                }
            }
            uint16_t average = (uint16_t)(running_sum / n_samples);

            i2c1->hw->data_cmd = (average & 0xFF); // Send the low-order byte
            i2c1->hw->data_cmd = (average >> 8);   // Send the high-order byte
        } else if (pointer == 8) {
            i2c1->hw->data_cmd = VERSION_MINOR;
            i2c1->hw->data_cmd = VERSION_MAJOR;
        } else {
            i2c1->hw->data_cmd = 0xFF; // return 0xFF as an error code
            i2c1->hw->data_cmd = 0xFF;
        }

        i2c1->hw->clr_rd_req;
    }

    i2c1->hw->clr_intr;  // Clear all interrupts
}

int main() {
    stdio_init_all();

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);

    adc_select_input(0);

    i2c_init(i2c1, 100000);
    i2c_set_slave_mode(i2c1, true, I2C1_PERIPHERAL_ADDR);

    gpio_set_function(GPIO_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_SCK0, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_SDA0);
    gpio_pull_up(GPIO_SCK0);

    set_up_pwm_pin(LED_CHANNEL_A_PIN);
    set_up_pwm_pin(LED_CHANNEL_B_PIN);
    set_up_pwm_pin(LED_CHANNEL_C_PIN);
    set_up_pwm_pin(LED_CHANNEL_D_PIN);

    i2c1->hw->intr_mask = (I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS);

    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);

    while (true) {
        tight_loop_contents();
    }
    return 0;
}