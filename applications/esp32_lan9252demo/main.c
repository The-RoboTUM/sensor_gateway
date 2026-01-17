#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "inttypes.h"

// SOES Includes
#include "ecat_slv.h"
#include "utypes.h"
#include "ecat_options.h"

// --------------------------------------------------------------------------
// PIN CONFIGURATION
// Define the ESP32 GPIOs you want to use for your application
// --------------------------------------------------------------------------
#define GPIO_LED_OUTPUT  2   // Example: Onboard LED on many ESP32 boards
#define GPIO_BTN_INPUT   0   // Example: Boot button

static const char *TAG = "SOES_APP";

/* Application variables declared in utypes.h */
_Objects Obj; 

// --------------------------------------------------------------------------
// HARDWARE HOOKS
// --------------------------------------------------------------------------

void cb_get_inputs(void)
{
    // 1. Update Input Variables (TxPDOs)
    // Values mapped here will be sent to the EtherCAT Master
    
    // Example: Read a button/sensor and update Encoder1
    // (In a real app, you would read your IMU/Encoder drivers here)
    static uint32_t counter = 0;
    counter++;

    Obj.Encoder1 = counter;                  // Dummy counter data
    Obj.Encoder2 = gpio_get_level(GPIO_BTN_INPUT); // Read button state
    Obj.IMU = 0xDEADBEEF;                    // Dummy hex data
}

void cb_set_outputs(void)
{
    // 2. Read Output Variables (RxPDOs)
    // Values received from the EtherCAT Master are used here
    
    // logic: If the master sends a non-zero value for LED, turn pin ON
    if (Obj.LED > 0) {
        gpio_set_level(GPIO_LED_OUTPUT, 1);
    } else {
        gpio_set_level(GPIO_LED_OUTPUT, 0);
    }
}

void GPIO_init(void)
{
    // Configure LED Output
    gpio_reset_pin(GPIO_LED_OUTPUT);
    gpio_set_direction(GPIO_LED_OUTPUT, GPIO_MODE_OUTPUT);

    // Configure Button Input
    gpio_reset_pin(GPIO_BTN_INPUT);
    gpio_set_direction(GPIO_BTN_INPUT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_BTN_INPUT, GPIO_PULLUP_ONLY);
    
    ESP_LOGI(TAG, "GPIO Initialized");
}

// --------------------------------------------------------------------------
// ETHERCAT TASK
// --------------------------------------------------------------------------

void ecat_task(void *pvParameters)
{
    // Configuration for the SOES stack
    static esc_cfg_t config = {
        .user_arg = NULL,
        .use_interrupt = 0, // Set to 1 if you implemented the ISR in HAL
        .watchdog_cnt = 150,
        .set_defaults_hook = NULL,
        .pre_state_change_hook = NULL,
        .post_state_change_hook = NULL,
        .application_hook = NULL,
        .safe_output_override = NULL,
        .pre_object_download_hook = NULL,
        .post_object_download_hook = NULL,
        .rxpdo_override = NULL,
        .txpdo_override = NULL,
        .esc_reset_hook = NULL,
        .esc_reset_isolation_hook = NULL,
        .esc_power_hook = NULL,
    };

    ESP_LOGI(TAG, "Initializing EtherCAT Slave Stack...");
    
    // Initialize Hardware (GPIOs)
    GPIO_init();
    
    // Initialize Stack (calls the ESC_init function in your HAL)
    ecat_slv_init(&config);
    
    ESP_LOGI(TAG, "Initialization complete. Entering loop.");

    while (1)
    {
        // Execute the EtherCAT stack engine
        ecat_slv();

        // Update application I/O only when in Operational State (OP)
        if (ESCvar.ALstatus == ESC_STATE_OP) 
        {
             cb_get_inputs();
             cb_set_outputs();
        }

        // Essential delay to feed the FreeRTOS watchdog
        // SOES is a polling stack, so we need to yield frequently.
        // 1ms (1000us) is standard for "Simple" Open Source EtherCAT.
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// --------------------------------------------------------------------------
// ESP-IDF ENTRY POINT
// --------------------------------------------------------------------------

extern "C" void app_main(void)
{
    // Initialize Logging
    esp_log_level_set("*", ESP_LOG_INFO);
    
    // Create the EtherCAT Task
    // Stack size: 4096 bytes, Priority: 5 (adjust based on your needs)
    xTaskCreate(ecat_task, "ecat_task", 4096, NULL, 5, NULL);
}