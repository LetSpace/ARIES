// TODO 
// - Add HX711 library and initialise
// - Add NRF24L01 initialization
// - Add SD library and initialise
// - Add main loop functionality





#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico-nrf24/lib/nrf24l01/nrf24_driver.h"

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
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(CS_NRF, GPIO_OUT);
    gpio_put(CS_NRF, 1);
    gpio_set_dir(CS_SD, GPIO_OUT);
    gpio_put(CS_SD, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    /*------SETUP-------*/
    // initialize I/O
    // drive all MOSFET channels low
    // if arm switch is on, beep forever (to prohibit turning on while armed)
    // begin NRFL
    // begin SD, set up file structure
    // begin HX711
    
    // idle until NRFL connection is established
    //  if NRFL connection is ever lost, keep pyro channels low
    // once connection is established:
    //  - beep and flash LED intermittently


    /*------MAIN LOOP-------*/
    // send status regularly
    // - Normal/error
    // - Pyro channel measured resistances
    // - Sensor data
    // respond to channel ignition messages
    // - Start data logging
    // - Beep quickly and flash quickly for ten seconds
    // - Send channel ignition message
    // - Turn pyro channel on for 5s, then off
    // - Return to normal state

    // Transmitted data:
    // - Resistance 1 (int), mOhms
    // - Resistance 2 (int), mOhms
    // - Sensor data (int)
    // - Channel 1 ignition feedback (bit)
    // - Channel 2 ignition feedback (bit)
    // - Armed/Normal/Error (2 bits)

    // Recieved data:
    // - Armed/Normal/Error (2 bits)
    // - Channel 1 ignition code (int)      // ignition messages will be transmitted as a unique integer, so pyro channels aren't dependent on a single bit (change b/c CRC?)
    // - Channel 2 ignition code (int)

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}
