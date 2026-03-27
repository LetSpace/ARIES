// Obtain calibration values for the hx711 load cell amplifier.
// To use this program with VS code, the line ending must be set to LF in the serial monitor.

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h> 
#include <math.h>
#include "pico/stdlib.h"
#include "hx711-pico-c-main/include/common.h"
#include "pico/time.h"
#include "pico/flash.h"
#include "hardware/flash.h"


//HX711
#define ADC_DAT 14
#define ADC_CLK 15

#define FLASH_TARGET_OFFSET (512 * 1024)

typedef struct {
    float slope;
    float intercept;
} calibration_data_t;

char buffer[1024];

calibration_data_t calibration_data;
const uint8_t* flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

hx711_t loadCell;

static void call_flash_range_erase(void *param) {
    uint32_t offset = (uint32_t)param;
    flash_range_erase(offset, FLASH_SECTOR_SIZE);
}

static void call_flash_range_program(void *param) {
    uint32_t offset = ((uintptr_t*)param)[0];
    const uint8_t *data = (const uint8_t *)((uintptr_t*)param)[1];
    flash_range_program(offset, data, FLASH_PAGE_SIZE);
}

float read_averaged_value() {
    float sum = 0;
    int data_count = 10;
    int32_t sample_val;
    const uint timeout = 250000; //microseconds
    for (int i = 0; i < 10; i++) {
        if (hx711_get_value_timeout(&loadCell, &sample_val, timeout) == false) {
            printf("HX711 read timeout\n");
            data_count--;
        } else {
            printf("Calibration sample %d: %d\n", i, sample_val);
            sum += sample_val;
        }
    }
    return (sum / data_count);
}


int main()
{
    stdio_init_all();

    // HX711 Setup
    hx711_config_t hxconfig;
    hx711_get_default_config(&hxconfig);
    hxconfig.clock_pin = ADC_CLK;
    hxconfig.data_pin = ADC_DAT;

    // gpio_put(ADC_CLK, 0);
    // sleep_ms(50);

    hx711_init(&loadCell, &hxconfig);
    hx711_power_up(&loadCell, hx711_gain_128);
    hx711_wait_settle(hx711_rate_10);
    sleep_ms(500);

    int32_t test_val;
    const uint timeout = 250000; //microseconds
    if(hx711_get_value_timeout(&loadCell, &test_val, timeout)) {
        printf("\nBlocking HX711 value: %" PRId32 "\n", test_val);
    } else {
        printf("\nHX711 blocking read failed\n");
    }

    int32_t sensorVal1;
    int weight1;
    int32_t sensorVal2;
    int weight2;
    
    sleep_ms(5000);

    /*  The code below takes a number of data points and outputs (1) the measured value of the load cell and (2) the value entered in the terminal. 
        This can be used to calculate the slope and intercept using Excel. */

    float sensorVal = 0.0;
    float weight;
    for(int i = 0; i < 5; i++) {
        printf("\nTrial %d:", i+1);
        printf("\nPlace known weight on scale. Enter weight in grams and press enter:");
        scanf("%1024s", buffer);
        weight = atoi(buffer);
        printf("\nWeight recorded: %f", weight);
        sensorVal = read_averaged_value();
        printf("\nSensor value recorded: %f", sensorVal);
    }

    /* The code below can be used to enter a slope and intercept calculated externally by the user. */

    // printf("\nEnter calculated slope and press enter:");
    // scanf("%1024s", buffer);
    // calibration_data.slope = atof(buffer);
    // printf("\nEnter calculated intercept and press enter:");
    // scanf("%1024s", buffer);
    // calibration_data.intercept = atof(buffer);

    /*  The code below calculates a slope and intercept using two data points. This is less accurate than using more data points and finding the slope and intercept
        in Excel using regression. */

    // printf("\nPlace first known weight on scale. Enter weight in grams and press enter:");
    // scanf("%1024s", buffer);
    // weight1 = atoi(buffer);
    // printf("\nWeight recorded: %d", weight1);
    // sensorVal1 = read_averaged_value();
    // printf("\nSensor value recorded: %d", sensorVal1);
    // printf("\nPlace second known weight on scale. Enter weight in grams and press enter:");
    // scanf("%1024s", buffer);
    // weight2 = atoi(buffer);
    // printf("\nWeight recorded: %d", weight2);
    // sensorVal2 = read_averaged_value();
    // printf("\nSensor value recorded: %d", sensorVal2);
    // float slope = (float)(weight2 - weight1) / (float)(sensorVal2 - sensorVal1);
    // printf("\nCalculated slope: %f", slope);
    // float intercept = (float)weight1 - slope * (float)sensorVal1;
    // printf("\nCalculated intercept: %f", intercept);
    
    // calibration_data.slope = slope;
    // calibration_data.intercept = intercept;


    /*  The code below saves the calibration data to the flash memory, so the main ARIES program can access it later. */
    uint32_t interrupts = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, (uint8_t*) &calibration_data, FLASH_PAGE_SIZE);
    restore_interrupts(interrupts);

    printf("\nCalibration data saved to flash.\n");
    sleep_ms(2000);
}