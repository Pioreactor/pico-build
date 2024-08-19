#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

/* The code implements the following i2c API
1. Write to any of the first 4 addresses (0, 1, 2, 3) to set the 8-bit duty cycle of the corresponding PWM
2. Read from any of the next 4 addresses (4, 5, 6, 7) to get the 16 bit ADC value of the corresponding ADC
2. Read from address 8 to return the version of this code.

Here are some examples of using i2cset and i2cget to interact with this program:
To set the duty cycle of the PWM channel corresponding to address 0 to 75%, you could use the following i2cset command:
> i2cset -y 1 0x30 0 0xC0
To read the 16-bit ADC value of the ADC channel corresponding to address 4, you could use the following i2cget command:
> i2cget -y 1 0x30 w 4
Note that the -y flag is used to automatically answer yes to any prompt from i2cset or i2cget.
The 1 after the -y flag specifies the i2c bus number to use, and the 0x30 specifies the i2c address of the peripheral.
The 0 and w in the i2cset and i2cget commands, respectively, specify the starting register address to be accessed.
The 4 in the i2cget command specifies the address of the ADC channel to read.
*/

// define I2C addresses to be used for this peripheral

#define I2C1_PERIPHERAL_ADDR 0x2C

#define GPIO_SDA0 14
#define GPIO_SCK0 15

// GPIO pins to use for PWM -> LED channels
#define LED_CHANNEL_A_PIN 16
#define LED_CHANNEL_B_PIN 17
#define LED_CHANNEL_C_PIN 18
#define LED_CHANNEL_D_PIN 19

#define VERSION_MAJOR 0
#define VERSION_MINOR 3

// first 4 addresses are for the PWM channels for LEDS  (A, B, C, D)
// second 4 addresses are for the ADCs (0, 1, 2, 3)
uint8_t pointer = 0;
// store the GPIO pins of the PWMs
uint8_t pwm_channels[4] = {LED_CHANNEL_A_PIN, LED_CHANNEL_B_PIN, LED_CHANNEL_C_PIN, LED_CHANNEL_D_PIN};
// store the duty cycles of the PWMs
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


void i2c1_irq_handler() {
    // Get interrupt status
    uint32_t status = i2c1->hw->intr_stat;

    // Check for NACK signals from the master or TX abort
    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        // Clear the TX abort interrupt
        i2c1->hw->clr_tx_abrt;
    }

    // Check to see if we have received data from the I2C controller
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint32_t value = i2c1->hw->data_cmd;
        if (value & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {
            pointer = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);
            if (pointer > 8) {
                pointer = 0xFF;  // invalid address
            }
        } else {
            if (pointer <= 3) {
                pwm_duty_cycles[pointer] = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);
                pwm_set_gpio_level(pwm_channels[pointer], (uint8_t) pwm_duty_cycles[pointer]);
            }
        }
    }

    // Check to see if the I2C controller is requesting data
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        if (pointer >= 4 && pointer <= 7) {
            uint8_t adc_input = pointer - 4;
            adc_select_input(adc_input);

            // since the adc_read will return maximum 2**12, and I can
            // send up to 2**16 data over two bytes in i2c, I can theoretically
            // read up to 2**4 = 16 times here.
            uint32_t running_sum = 0;
            for (int j = 0; j < 16; ++j) {
                for (int i = 0; i < 16; ++i) {
                    running_sum += adc_read();
                    sleep_us(5 + randomArray[j]);
                }
            }
            uint16_t average = (uint16_t)(running_sum / 16);
            i2c1->hw->data_cmd = (average & 0xFF);
            i2c1->hw->data_cmd = (average >> 8);
        } else if (pointer == 8) {
            i2c1->hw->data_cmd = VERSION_MINOR;
            i2c1->hw->data_cmd = VERSION_MAJOR;
        } else {
            i2c1->hw->data_cmd = 0xFF;
            i2c1->hw->data_cmd = 0xFF;
        }

        // Clear the read request interrupt
        i2c1->hw->clr_rd_req;
    }

    // Clear any other interrupts that might have been set
    if (status & I2C_IC_INTR_STAT_R_RX_DONE_BITS) {
        i2c1->hw->clr_rx_done;
    }
}


int main() {
    stdio_init_all();

    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);

    // Select ADC input 0 initially (GPIO26)
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

    // Enable the I2C interrupts we want to process
    i2c1->hw->intr_mask = (I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS);
    // Set up the interrupt handler to service I2C interrupts
    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    // Enable I2C interrupt
    irq_set_enabled(I2C1_IRQ, true);

    while (true) {
        tight_loop_contents();
    }
    return 0;
}