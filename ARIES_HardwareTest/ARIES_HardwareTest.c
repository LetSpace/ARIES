#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/timer.h"


/*-------PIN DEFINES-------*/

// SPI
#define SPI_PORT spi1
#define SCK  10
#define MOSI 11
#define MISO 12
#define CS_NRF 13
#define CS_SD 8

#define EN_NRF 9

//HX711
#define ADC_DAT 14
#define ADC_CLK 15

//Pi ADC
#define ARM_SENSE 28
#define PYRO_SENSE_1 27
#define PYRO_SENSE_2 26

//MOSFETs
#define PYRO_1 20
#define PYRO_2 19
#define BUZZ 21
#define LED 18



int main()
{
    stdio_init_all();

    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(SCK,  GPIO_FUNC_SPI);
    gpio_set_function(MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MISO, GPIO_FUNC_SPI);
    gpio_set_function(CS_NRF,   GPIO_FUNC_SIO);
    gpio_set_function(CS_SD,   GPIO_FUNC_SIO);

    gpio_init(PYRO_1);
    gpio_init(PYRO_2);
    gpio_init(BUZZ);
    gpio_init(LED);
    gpio_set_dir(PYRO_1, GPIO_OUT);
    gpio_set_dir(PYRO_2, GPIO_OUT); 
    gpio_set_dir(BUZZ, GPIO_OUT);
    gpio_set_dir(LED, GPIO_OUT);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(CS_NRF, GPIO_OUT);
    gpio_put(CS_NRF, 1);
    gpio_set_dir(CS_SD, GPIO_OUT);
    gpio_put(CS_SD, 1);

    while (true) {
        printf("Testing outputs...\n");
        gpio_put(PYRO_1, 1);
        sleep_ms(2000);
        gpio_put(PYRO_1, 0);
        sleep_ms(2000);
        gpio_put(PYRO_2, 1);
        sleep_ms(2000);
        gpio_put(PYRO_2, 0);
        sleep_ms(2000);
        gpio_put(BUZZ, 1);
        sleep_ms(2000);
        gpio_put(BUZZ, 0);
        sleep_ms(2000);
        gpio_put(LED, 1);
        sleep_ms(2000);
        gpio_put(LED, 0);
        sleep_ms(2000);
    }
}
