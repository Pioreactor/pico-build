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
#define I2C1_PERIPHERAL_ADDR 0x30

// GPIO pins to use for I2C
#define GPIO_SDA0 14
#define GPIO_SCK0 15

// GPIO pins to use for PWM -> LED channels
#define LED_CHANNEL_A_PIN 16
#define LED_CHANNEL_B_PIN 17
#define LED_CHANNEL_C_PIN 18
#define LED_CHANNEL_D_PIN 19

// define firmware version that can be read over i2c
#define VERSION_MAJOR 0
#define VERSION_MINOR 2

// first 4 addresses are for the PWM channels for LEDS  (A, B, C, D)
// second 4 addresses are for the ADCs (0, 1, 2, 3)
uint8_t pointer = 0;

// store the GPIO pins of the PWMs 
uint8_t pwm_channels[4] = {LED_CHANNEL_A_PIN, LED_CHANNEL_B_PIN, LED_CHANNEL_C_PIN, LED_CHANNEL_D_PIN};

// store the duty cycles of the PWMs
uint8_t pwm_duty_cycles[4];


void set_up_pwm_pin(uint pin) { 
    // starts at duty_cycle = 0, hz = 325k
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

    // Check to see if we have received data from the I2C controller
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {

        // Read the data (this will clear the interrupt)
        uint32_t value = i2c1->hw->data_cmd;

        // Check if this is the 1st byte we have received
        if (value & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {

            // If so treat it as the address to use
            pointer = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);

        } else {
            if (pointer <= 3){
                // If not 1st byte then store the data in the RAM
                // and increment the address to point to next byte
                pwm_duty_cycles[pointer] = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);
                pwm_set_gpio_level(pwm_channels[pointer], (uint8_t) pwm_duty_cycles[pointer]);
            }
        }
    }

    // Check to see if the I2C controller is requesting data from the RAM
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        if (pointer >= 4 && pointer <= 7){
            uint8_t adc_input = pointer - 4;
            adc_select_input(adc_input);

            // since the adc_read will return maximum 2**12, and I can
            // send up to 2**16 data over two bytes in i2c, I can theoretically
            // read up to 2**4 = 16 times here.
            uint16_t raw = 0;
            int i;
            for (i = 0; i < 16; ++i){
                raw = raw + adc_read();
            }
            i2c1->hw->data_cmd = (raw & 0xFF); // Send the low-order byte
            i2c1->hw->data_cmd = (raw >> 8);   // Send the high-order byte
        } else if (pointer == 8) {
            i2c1->hw->data_cmd = VERSION_MINOR;
            i2c1->hw->data_cmd = VERSION_MAJOR;
        } else {
            i2c1->hw->data_cmd = 0; // return 0
            i2c1->hw->data_cmd = 0;
        }

        // Clear the interrupt
        i2c1->hw->clr_rd_req;
    }
}


// Main loop - initializes system and then loops while interrupts get on with processing the data
int main() {

    // setup ADC
    stdio_init_all(); 

    // Initialise ADCs
    adc_init();
    // Make sure GPIO is high-impedance, no pull-ups etc
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);

    // Select ADC input 0 initially (GPIO26)
    adc_select_input(0);

    // Setup I2C1 as peripheral
    i2c_init(i2c1, 100000); 
    i2c_set_slave_mode(i2c1, true, I2C1_PERIPHERAL_ADDR);

    // Setup GPIO pins to use and add pull up resistors
    gpio_set_function(GPIO_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_SCK0, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_SDA0);
    gpio_pull_up(GPIO_SCK0);

    // set up PWMs
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

    // Do nothing in main loop
    while (true) {
        tight_loop_contents();
    }
    return 0;
}