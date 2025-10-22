// Advanced Remote Ignition and Evaluation System (ARIES)
// Main Code
// This program handles radio transmission and receipt, pyro ignition, and data collection
//
// Authors: Cooper McAllister, Josh Hartman
// 9-9-2025 File created
// 10-2-2025 First minimally functional version completed 
// 10-11-2025 First full version completed 

// LIBRARIES USED
// NRF24L01 Radio (Andy Rids): https://github.com/AndyRids/pico-nrf24
// HX711 Load Cell (endail): https://github.com/endail/hx711-pico-c
// FatFS for SD card: https://elm-chan.org/fsw/ff/

// TODO
// Add channel 2 pyro ignition
// Make pyro ignition delay non-blocking (so radio transmissions and data collection continue during it)
// Put data structs in a separate header file for both ARIES and CRIS to declutter
// Fix radio bug of only receiving 3 transmissions


#define DEBUG

// Import necessary libraries
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h> 
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/adc.h"
#include "pico-nrf24-main/lib/nrf24l01/nrf24_driver.h"
#include "hx711-pico-c-main/include/common.h"
#include "pico/time.h"
#include "hw_config.h"
#include "f_util.h"
#include "ff.h"
#include "pico/flash.h"
#include "hardware/flash.h"

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
// Calibration values obtained with ARIES_Calibration
#define LOADCELL_CALIBRATION_SLOPE -16.666666
#define LOADCELL_CALIBRATION_INTERCEPT 13014150.000000

//Pi ADC
#define ARM_SENSE 28
#define PYRO_SENSE_1 27
#define PYRO_SENSE_2 26

//MOSFETs
#define PYRO_1 20
#define PYRO_2 19
#define BUZZ 21
#define LED 18

// NRFL Addresses
#define ARIES_ADDRESS {0x41, 0x52, 0x49, 0x45, 0x53} // "ARIES"
#define CRIS_ADDRESS {0x43, 0x52, 0x49, 0x53, 0x31} // "CRIS1"

// Flash Location for Calibration Data
#define FLASH_TARGET_OFFSET (512 * 1024)

/*-------DATA STRUCTURES-------*/
// Pyro commands (sent from CRIS)
typedef enum{
    NO_COMMAND,
    IGNITE,
    ABORT
} pyro_command_t;

// Pyro state
typedef enum {
    PYRO_OFF,
    PYRO_ON
} pyro_state_t;

// Status
typedef enum {
    STATUS_NORMAL,
    STATUS_ARMED,
    STATUS_ERROR
} status_t;

// Data sent from CRIS to ARIES
typedef struct {
    pyro_command_t pyro_1;
    pyro_command_t pyro_2;
    status_t ptx_status; // 0 = normal, 1 = armed, 2 = error
} arc_data_t;

// Data sent from ARIES to CRIS
typedef struct {
    int32_t resistance_1; // in milliohms
    int32_t resistance_2; // in milliohms
    float sensor_data;
    uint8_t pyro_1_feedback;
    uint8_t pyro_2_feedback;
    uint8_t prx_status; // 0 = normal, 1 = armed, 2 = error
} aries_data_t;

// Initialize data structs with default values
aries_data_t aries_data = {-1, -1, -1, PYRO_OFF, PYRO_OFF, STATUS_NORMAL};
arc_data_t arc_data = {NO_COMMAND, NO_COMMAND, STATUS_NORMAL};

// Data used to calibrate the load cell using a linear slope-intercept model
typedef struct {
    float slope;
    float intercept;
} calibration_data_t;

calibration_data_t calibration_data;
// The memory address where the calibration data is stored by the calibration program
const uint8_t* flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

// Load cell data
hx711_t loadCell;
volatile bool adc_ready = false;
volatile int32_t irq_sensor_data = 0;
float tare_value = 0.0;

// File used for the SD Card
FIL logfile;

// Callback tied to the "data ready" pin on the HX711
// This function reads the hx711 without blocking, saves the value, and sets a flag
void adc_callback(uint gpio, uint32_t events) {
    int32_t val;
    if (hx711_get_value_noblock(&loadCell, &val)) {
        irq_sensor_data = val;
        adc_ready = true;
    }
}

/* ----- SD CARD HARDWARE DEFINITION ----- */

// Configuration of hardware SPI object
static spi_t spi = {
    .hw_inst = SPI_PORT,
    .sck_gpio = SCK,
    .mosi_gpio = MOSI,
    .miso_gpio = MISO,
    .baud_rate = 1 * 1000 * 1000 // 1 MHz is used for both the sd card the nrfl radio
};

// SPI Interface
static sd_spi_if_t spi_if = {
    .spi = &spi,  // Pointer to the SPI driving this card
    .ss_gpio = CS_SD  // The SPI slave select GPIO for this SD card
};

// Configuration of the SD Card socket object
static sd_card_t sd_card = {
    .type = SD_IF_SPI,
    .spi_if_p = &spi_if  // Pointer to the SPI interface driving this card
};

size_t sd_get_num() { return 1; }

sd_card_t *sd_get_by_num(size_t num) {
    if (0 == num) {
        return &sd_card;
    } else {
        return NULL;
    }
}

/* ----- PI ADC HARDWARE DEFINITION ----- */

const float adc_conversion_factor = 3.3f / (1 << 12); // 12 bits at 3.3V

// Returns the voltage after the arm switch.
float get_arm_sense() {
    adc_select_input(2);
    return adc_read() * adc_conversion_factor * (99.1 + 300.5)/99.1; // Arm sense = (Battery Voltage) / 4, actual resistor values used
}

// Returns the resistance on the given channel (1 or 2) in miliohms.
int32_t get_resistance(int channel) {
    float v_arm = get_arm_sense();
    adc_select_input(2 - channel);
    float vd_conversion = (channel == 1) ? ((100 + 298.5)/100) : ((98.1 + 298.1)/98.1); // Voltage division conversion using actual resistor values
    float v_d = adc_read() * adc_conversion_factor * vd_conversion; // voltage at mosfet drain
    float resistance = (v_arm - v_d) / (v_d / 400);
    return resistance * 1000;
}

int main() {
    /*------SETUP-------*/

    stdio_init_all();
    //sleep_ms(5000);
    //printf("Hello");

    // Drive CS High (active low)
    gpio_set_dir(CS_SD, GPIO_OUT);
    gpio_put(CS_SD, 1);
    gpio_set_dir(CS_NRF, GPIO_OUT);
    gpio_put(CS_NRF, 1);

    // Initialize GPIO
    gpio_init(PYRO_1);
    gpio_init(PYRO_2);
    gpio_init(BUZZ);
    gpio_init(LED);
    gpio_set_dir(PYRO_1, GPIO_OUT);
    gpio_set_dir(PYRO_2, GPIO_OUT); 
    gpio_set_dir(BUZZ, GPIO_OUT);
    gpio_set_dir(LED, GPIO_OUT);

    // drive all MOSFET channels low
    gpio_put(PYRO_1, 0);
    gpio_put(PYRO_2, 0);
    gpio_put(BUZZ, 0);
    gpio_put(LED, 0);

    // Initialize ADC
    adc_init();
    adc_gpio_init(ARM_SENSE);
    adc_gpio_init(PYRO_SENSE_1);
    adc_gpio_init(PYRO_SENSE_2);

    // If arm switch is on, beep forever (to prohibit turning on while armed)
    if (get_arm_sense() > 5) {
        gpio_put(BUZZ, 1);
        while (true) {}
    }

    // Mount SD Card
    // See http://elm-chan.org/fsw/ff/00index_e.html
    FATFS fs;
    FRESULT fr = f_mount(&fs, "", 1);
    if (FR_OK != fr) {
        panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
    }

    // Find next free filename
    DIR dir;
    FILINFO finfo;
    int maxfileno = 0;

    fr = f_opendir(&dir, "");                   /* Open the directory */
    if (fr == FR_OK) {
        for (;;) {
            fr = f_readdir(&dir, &finfo);           /* Read a directory item */
            if (finfo.fname[0] == 0) break;          /* Error or end of dir */
            if (finfo.fattrib & AM_DIR) {            /* It is a directory */
                continue;
            } else {                               /* It is a file */
                // check filename for number
                int num;
                if (sscanf(finfo.fname, "log%d.csv", &num)) { // If match found...
                    maxfileno = MAX(num, maxfileno);
                }
            }
        }
        f_closedir(&dir);
    } else {
        printf("Failed to open root dir. %s (%d)\n", FRESULT_str(fr), fr);
    }

    FIL file;
    char filename[32];
    sprintf(filename, "log%d.csv", maxfileno + 1);
    printf("Next free filename: %s\n", filename);

    // Open log file and write column headers to it:
    //   time: current time since startup in ms.
    //   force: current load cell reading.
    //   pyro1r: resistance on channel 1.
    //   pyro2r: resistance on channel 2.
    //   event: usually empty, marks major state changes (arm, fire, etc).
    
    fr = f_open(&logfile, filename, FA_OPEN_APPEND | FA_WRITE);
    f_printf(&logfile, "time, force, pyro1r, pyro2r, event\n");
    fr = f_close(&logfile);
    
    // Initialize NRFL
    pin_manager_t nrfl_pins = {
        .ce = EN_NRF,
        .csn = CS_NRF,
        .sck = SCK,
        .copi = MOSI,
        .cipo = MISO
    };
    nrf_manager_t nrfl_config = {
        .channel = 120,
        .address_width = AW_5_BYTES,
        .dyn_payloads = DYNPD_ENABLE,
        .data_rate = RF_DR_1MBPS,
        .power = RF_PWR_0DBM,
        .retr_count = ARC_10RT,
        .retr_delay = ARD_500US
    };
    uint32_t nrfl_baudrate = 1000 * 1000; // 1 MHz
    uint8_t pipe_number = 0;
    fn_status_t tx_success;
    uint8_t aries_address[5] = ARIES_ADDRESS;
    uint8_t cris_address[5] = CRIS_ADDRESS;
    //sleep_ms(5000);
    nrf_client_t nrfl_client;
    
    if (nrf_driver_create_client(&nrfl_client) == ERROR) {
        while (true) {
            printf("\nNRFL client creation failure");
            sleep_ms(1000);
        }
    };
    if (nrfl_client.configure(&nrfl_pins, nrfl_baudrate) == ERROR) {
        while (true) {
            printf("\nNRFL pin configuration failure");
            sleep_ms(1000);
        }
    };
    if (nrfl_client.initialise(&nrfl_config) == ERROR) {
        while (true) {
            printf("\nNRFL init failure");
            sleep_ms(1000);
        }
    };  
    if (nrfl_client.rx_destination(DATA_PIPE_1, aries_address) == ERROR) {
        while (true) {
            printf("\nNRFL RX address failure");
            sleep_ms(1000);
        }
    }
    if (nrfl_client.tx_destination(cris_address) == ERROR) {
        while (true) {
            printf("\nNRFL TX address failure");
            sleep_ms(1000);
        }
    }
    nrfl_client.standby_mode();
    sleep_ms(1);
    printf("NRFL init success");

    // Initialize HX711
    hx711_config_t hxconfig;
    hx711_get_default_config(&hxconfig);
    hxconfig.clock_pin = ADC_CLK;
    hxconfig.data_pin = ADC_DAT;

    hx711_init(&loadCell, &hxconfig);
    hx711_power_up(&loadCell, hx711_gain_128);
    hx711_wait_settle(hx711_rate_10);
    sleep_ms(500);

    // Copy calibration data from flash
    memcpy(&calibration_data, flash_target_contents, sizeof(calibration_data));

    int32_t sum = 0;
    int32_t sample_val;
    const uint timeout = 250000; //microseconds
    for (int i = 0; i < 10; i++) {
        if (hx711_get_value_timeout(&loadCell, &sample_val, timeout) == false) {
            printf("HX711 initial read timeout\n");
        } else {
            printf("Calibration sample %d: %d\n", i, sample_val);
        }
        sum += sample_val * calibration_data.slope + calibration_data.intercept;
    }
    tare_value = sum / 10;
    
    printf("Calibration slope: %f\n", calibration_data.slope);
    printf("Calibration intercept: %f\n", calibration_data.intercept);
    printf("Tare Value: %f\n", tare_value);

    // Setup HX711 callback
    gpio_set_irq_enabled_with_callback(ADC_DAT, GPIO_IRQ_EDGE_FALL, true, &adc_callback);

    // Beep to indicate end of setup
    for (int i = 0; i < 2; i++) {
        gpio_put(BUZZ, 1);
        sleep_ms(100);
        gpio_put(BUZZ, 0);
        sleep_ms(100);
    }

    /*------MAIN LOOP-------*/

    int buzz_counter = 0;
    int packets_read = 0;
    while (true) {
        printf("Main Loop Start");

        absolute_time_t endTime = make_timeout_time_ms(1000); // 1 second loop

        // Beep every 5 seconds if armed
        if (aries_data.prx_status == STATUS_ARMED) { 
            if (buzz_counter >= 5) {  
                gpio_put(BUZZ, 1);
                sleep_ms(50);
                gpio_put(BUZZ, 0);
                buzz_counter = 0;
            }
            buzz_counter++;
        }
        
        // Get resistance of both channels
        aries_data.resistance_1 = get_resistance(1);
        aries_data.resistance_2 = get_resistance(2);

        // Transmit data
        if(nrfl_client.standby_mode() == ERROR) {
            printf("\nNRFL standby failure");
        }
        printf("\nTransmitting data...");
        printf("\n - Resistance 1: %d mOhms", aries_data.resistance_1);
        printf("\n - Resistance 2: %d mOhms", aries_data.resistance_2);
        printf("\n - Sensor data: %f", aries_data.sensor_data);
        printf("\n - Pyro 1 feedback: %d", aries_data.pyro_1_feedback);
        printf("\n - Pyro 2 feedback: %d", aries_data.pyro_2_feedback);
        printf("\n - PRX status: %d", aries_data.prx_status);
        printf("\n - ARMSENSE: %f", get_arm_sense());
        tx_success = nrfl_client.send_packet(&aries_data, sizeof(aries_data));
        if (tx_success == ERROR) {
            printf("\nTX failure\n");

            nrfl_client.standby_mode();
            nrfl_client.receiver_mode();
        } else {
            printf("\nTX success\n");
        }

        // Used to debug radio, doesn't work currently
        // fifo_status_t fifo;
        // if (nrfl_client.fifo_status(&fifo) == SPI_MNGR_OK) {
        //     printf("TX: empty=%d full=%d reuse=%d | RX: empty=%d full=%d\n",
        //         fifo.tx_empty, fifo.tx_full, fifo.tx_reuse,
        //         fifo.rx_empty, fifo.rx_full);
        // }

        nrfl_client.receiver_mode(); 
        //nrfl_client.flush_rx_fifo();
        printf("Listening for data...\n");
        
        // Until the 1s timer is up, run several different tasks while listening for radio transmissions from CRIS
        while(!time_reached(endTime)) {
            // Check for data from ARC
            while(nrfl_client.is_packet(&pipe_number)) {
                packets_read++;
                    printf("Data available...");
                if (nrfl_client.read_packet(&arc_data, sizeof(arc_data)) == ERROR) {
                    printf("RX Fail!");
                } else {
                    printf("RX Success!");
                    printf("\nRX data received:");
                    printf("\n     - Pyro 1 command: %d", arc_data.pyro_1);
                    printf("\n     - Pyro 2 command: %d", arc_data.pyro_2);
                    printf("\n     - PTX status: %d", arc_data.ptx_status);
                }
                if (arc_data.pyro_1 == IGNITE && aries_data.prx_status == STATUS_ARMED) {
                    // Ignite pyro 1
                    #ifdef DEBUG
                    printf("\nIGNITE PYRO 1");
                    #endif
                    absolute_time_t warningTimer = make_timeout_time_ms(10000);
                    while(!time_reached(warningTimer)) {
                        gpio_put(BUZZ, 1);
                        sleep_ms(100);
                        gpio_put(BUZZ, 0);
                        gpio_put(LED, 1);
                        sleep_ms(2);
                        gpio_put(LED, 0);
                        sleep_ms(100);
                    }
                    f_printf(&logfile, "%llu, %" PRId32 ", %" PRId32 ", %" PRId32 ", PYRO 1 IGNITION\n", to_ms_since_boot(get_absolute_time()), aries_data.sensor_data, aries_data.resistance_1, aries_data.resistance_2); // time, force, pyro1r, pyro2r, event
                    aries_data.pyro_1_feedback = PYRO_ON;
                    gpio_put(PYRO_1, 1);
                    sleep_ms(1000);
                    gpio_put(PYRO_1, 0);
                    aries_data.pyro_1_feedback = PYRO_OFF;
                    arc_data.pyro_1 = NO_COMMAND; // reset command
                }
                
            }

            // Update sensor data if ready
            if (adc_ready && (aries_data.prx_status == STATUS_ARMED)) {
                // disable interrupts briefly while copying volatile values
                uint32_t save = save_and_disable_interrupts();
                int32_t sensor_val = irq_sensor_data;
                adc_ready = false;
                restore_interrupts(save);
                // Update sensor data in aries_data
                aries_data.sensor_data = sensor_val * calibration_data.slope + calibration_data.intercept - tare_value;
                // Print data
                unsigned long long ms = to_ms_since_boot(get_absolute_time());
                f_printf(&logfile, "%llu, %f, %" PRId32 ", %" PRId32 "\n", ms, aries_data.sensor_data, aries_data.resistance_1, aries_data.resistance_2); // time, force, pyro1r, pyro2r, event
                printf("%llu, %f, %" PRId32 ", %" PRId32 "\n", ms, aries_data.sensor_data, aries_data.resistance_1, aries_data.resistance_2);
            }

            // Update arm status
            if (!(aries_data.prx_status == STATUS_ARMED) && get_arm_sense() > 5) {
                aries_data.prx_status = STATUS_ARMED;
                fr = f_open(&logfile, filename, FA_OPEN_APPEND | FA_WRITE);
                f_printf(&logfile, "%llu, %f, %" PRId32 ", %" PRId32 ", ARMED\n", to_ms_since_boot(get_absolute_time()), aries_data.sensor_data, aries_data.resistance_1, aries_data.resistance_2); // time, force, pyro1r, pyro2r, event
                printf("%llu, %f, %" PRId32 ", %" PRId32 ", ARMED\n", to_ms_since_boot(get_absolute_time()), aries_data.sensor_data, aries_data.resistance_1, aries_data.resistance_2);
                gpio_put(BUZZ, 1);
                sleep_ms(500);
                gpio_put(BUZZ, 0);
            }
            if ((aries_data.prx_status == STATUS_ARMED) && get_arm_sense() <= 5) {
                aries_data.prx_status = STATUS_NORMAL;
                f_printf(&logfile, "%llu, %f, %" PRId32 ", %" PRId32 ", DISARMED\n", to_ms_since_boot(get_absolute_time()), aries_data.sensor_data, aries_data.resistance_1, aries_data.resistance_2); // time, force, pyro1r, pyro2r, event
                printf("%llu, %f, %" PRId32 ", %" PRId32 ", DISARMED\n", to_ms_since_boot(get_absolute_time()), aries_data.sensor_data, aries_data.resistance_1, aries_data.resistance_2);
                fr = f_close(&logfile);
            }
        }
    }
}
