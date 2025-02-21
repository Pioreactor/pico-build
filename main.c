#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <stdlib.h>  // for rand() and srand()

// I2C peripheral address for this device
#define I2C1_PERIPHERAL_ADDR 0x2C

// I2C GPIO pins
#define GPIO_SDA0 14
#define GPIO_SCK0 15

// PWM GPIO pins (LED channels A, B, C, D)
#define LED_CHANNEL_A_PIN 16
#define LED_CHANNEL_B_PIN 17
#define LED_CHANNEL_C_PIN 18
#define LED_CHANNEL_D_PIN 19

// Code version numbers
#define VERSION_MAJOR 0
#define VERSION_MINOR 5

// Global pointer for I2C register selection (addresses 0–8)
volatile uint8_t pointer = 0;

// PWM channel configuration:
// addresses 0–3 are used to set the 8-bit duty cycle for each PWM output.
uint8_t pwm_channels[4] = {LED_CHANNEL_A_PIN, LED_CHANNEL_B_PIN, LED_CHANNEL_C_PIN, LED_CHANNEL_D_PIN};
uint8_t pwm_duty_cycles[4] = {0};

// Cached ADC values for channels 0–3 (corresponding to I2C addresses 4–7)
// These values are updated continuously in the background.
volatile uint16_t adc_cache[4] = {0};
volatile uint32_t filtered_adc[4] = {0, 0, 0, 0};
#define ALPHA 0.1f

// -----------------------------------------------------------------------------
// Function: set_up_pwm_pin
// Configures a given GPIO for PWM output.
// -----------------------------------------------------------------------------
static inline void set_up_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    // Configure the PWM clock divider and wrap value.
    float div = (float) clock_get_hz(clk_sys) / (325000 * 256);
    pwm_config_set_clkdiv(&config, div);
    pwm_config_set_wrap(&config, 256);
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);
}


// Increase oversampling factor to 1024 samples for an extra bit of resolution.
uint16_t read_adc_oversampled(uint8_t adc_channel) {
    adc_select_input(adc_channel);
    const int num_samples = 1024;  // 4^5 samples to gain 5 extra bits
    uint32_t sum = 0;
    for (int i = 0; i < num_samples; i++) {
        sum += adc_read();
        sleep_us(1 + (rand() % 3));
    }
    // Shift right by 5 bits to account for the increased number of samples.
    return (uint16_t)(sum >> 5);
}

// -----------------------------------------------------------------------------
// I2C Interrupt Handler
//
// Implements the following I2C API:
//   - Write to addresses 0–3 to set the PWM duty cycle for the corresponding channel.
//   - Read from addresses 4–7 to return the cached ADC oversampled value for ADC inputs 0–3.
//   - Read from address 8 to return the version of this code.
// -----------------------------------------------------------------------------
void i2c1_irq_handler() {
    uint32_t status = i2c1->hw->intr_stat;

    // Clear any TX abort condition.
    if (status & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        i2c1->hw->clr_tx_abrt;
    }

    // Process incoming data (master write)
    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        uint32_t value = i2c1->hw->data_cmd;
        if (value & I2C_IC_DATA_CMD_FIRST_DATA_BYTE_BITS) {
            // The first byte received specifies the register pointer.
            pointer = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);
            if (pointer > 8) {
                pointer = 0xFF;  // Mark as an invalid address.
            }
        } else {
            // A subsequent byte: if pointer is 0–3, update the PWM duty cycle.
            if (pointer <= 3) {
                pwm_duty_cycles[pointer] = (uint8_t)(value & I2C_IC_DATA_CMD_DAT_BITS);
                pwm_set_gpio_level(pwm_channels[pointer], pwm_duty_cycles[pointer]);
            }
        }
    }

    // Process master read requests.
    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        if (pointer >= 4 && pointer <= 7) {
            // For addresses 4–7, return the cached ADC value.
            uint8_t adc_index = pointer - 4;
            uint16_t cached_value = adc_cache[adc_index];
            i2c1->hw->data_cmd = (cached_value & 0xFF);       // Send lower byte.
            i2c1->hw->data_cmd = ((cached_value >> 8) & 0xFF);  // Send upper byte.
        } else if (pointer == 8) {
            // Return version information.
            i2c1->hw->data_cmd = VERSION_MINOR;
            i2c1->hw->data_cmd = VERSION_MAJOR;
        } else {
            // For an invalid pointer, return 0xFF on both bytes.
            i2c1->hw->data_cmd = 0xFF;
            i2c1->hw->data_cmd = 0xFF;
        }
        // Clear the read request flag.
        i2c1->hw->clr_rd_req;
    }

    // Clear any RX done status.
    if (status & I2C_IC_INTR_STAT_R_RX_DONE_BITS) {
        i2c1->hw->clr_rx_done;
    }
}

// -----------------------------------------------------------------------------
// Main function: Initializes peripherals and then enters a background loop to
// update the cached ADC values for channels 0–3.
// -----------------------------------------------------------------------------
int main() {
    stdio_init_all();

    // Initialize the ADC and its GPIOs (channels 0–3 correspond to GPIO26–29).
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);
    adc_select_input(0);  // Start with channel 0.

    // Seed the random number generator (used for ADC sampling delay jitter).
    srand((unsigned) time_us_32());

    // Initialize I2C1 in slave mode.
    i2c_init(i2c1, 100000);
    i2c_set_slave_mode(i2c1, true, I2C1_PERIPHERAL_ADDR);
    gpio_set_function(GPIO_SDA0, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_SCK0, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_SDA0);
    gpio_pull_up(GPIO_SCK0);

    // Set up PWM outputs.
    set_up_pwm_pin(LED_CHANNEL_A_PIN);
    set_up_pwm_pin(LED_CHANNEL_B_PIN);
    set_up_pwm_pin(LED_CHANNEL_C_PIN);
    set_up_pwm_pin(LED_CHANNEL_D_PIN);

    // Enable I2C interrupts for RX (data received) and RD_REQ (master read request).
    i2c1->hw->intr_mask = I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS;
    irq_set_exclusive_handler(I2C1_IRQ, i2c1_irq_handler);
    irq_set_enabled(I2C1_IRQ, true);

    // Variables for scheduling updates
    uint32_t loop_counter = 0;
    while (true) {

        for (uint8_t channel = 2; channel < 4; channel++) {
            uint16_t new_sample = read_adc_oversampled(channel);
            // Update the filtered value with a heavy low-pass filter:
            filtered_adc[channel] = (uint32_t)((1.0f - ALPHA) * filtered_adc[channel] + ALPHA * new_sample);
            // Use the filtered value as the cached result for I2C reads.
            adc_cache[channel] = (uint16_t)filtered_adc[channel];
        }

        // Low-priority channels (0 and 1) are updated only once every N loops.
        if (loop_counter % 100 == 0) {

            adc_cache[0] = read_adc_oversampled(0);
            adc_cache[1] = read_adc_oversampled(1);

        }

        loop_counter++;
        // A short delay to yield; adjust as needed.
        sleep_ms(1);
    }
    return 0;
}