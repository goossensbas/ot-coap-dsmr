#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_openthread.h"
#include "esp_openthread_cli.h"
#include "esp_openthread_lock.h"
#include "esp_openthread_netif_glue.h"
#include "esp_openthread_types.h"
#include "esp_ot_config.h"
#include "esp_vfs_eventfd.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/uart_types.h"
#include "nvs_flash.h"
#include "openthread/cli.h"
#include <openthread/coap.h>
#include <openthread/thread.h>
#include <openthread/platform/radio.h>
#include <openthread/instance.h>
#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/tasklet.h>
#include <openthread/logging.h>

#define UART_RX_PIN 10 
#define BUF_SIZE 2048

// I2C configuration  found in bme280.h


#define DSMR_TAG "DSMR"
#define UART_TAG "UART"
#define COAP_TAG "CoAP"
#define COAP_URI_PATH "dsmr"
#define COAP_SERVER "fd3f:4f29:5339:1ff9:ffc:7570:df42:e75f"
#define SEND_DELAY_MS 10000

#define UART_PORT_NUM      UART_NUM_1

static otInstance *instance;
static otCoapResource coapResource;
// Mutex for protecting access to sensor data
SemaphoreHandle_t data_mutex;

// Global variables for p1 data

//function to init uart1
void init_uart() { 
    uart_config_t uart_config = { 
        .baud_rate = 115200, 
        .data_bits = UART_DATA_8_BITS, 
        .parity = UART_PARITY_DISABLE, 
        .stop_bits = UART_STOP_BITS_1, 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        //.source_clk = UART_SCLK_APB, 
        }; 
    int intr_alloc_flags = 0;
    #if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config)); 
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); 
    // Invert RX signal 
    // ESP_ERROR_CHECK(uart_set_line_inverse(UART_PORT_NUM, UART_SIGNAL_RXD_INV)); 
    ESP_LOGI(UART_TAG, "UART setup completed.");
        }

void coap_init(otInstance *instance);

#define OT_COAP_PORT 5683

static esp_netif_t *init_openthread_netif(const esp_openthread_platform_config_t *config)
{
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_OPENTHREAD();
    esp_netif_t *netif = esp_netif_new(&cfg);
    assert(netif != NULL);
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_openthread_netif_glue_init(config)));

    return netif;
}

//CoAP response function
void handle_coap_response(void *context, otMessage *message, const otMessageInfo *messageInfo, otError result) {
    if (result == OT_ERROR_NONE) {
        ESP_LOGI(COAP_TAG, "CoAP response received");
    } else {
        ESP_LOGE(COAP_TAG, "Failed to receive CoAP response: %d", result);
    }
}

//send CoAP message
void send_coap_message(void) {
    otError error;
    otMessage *message;
    otMessageInfo messageInfo;
    otMessageSettings messageSettings = { true, OT_COAP_PORT };
    
    otInstance *instance = esp_openthread_get_instance();
    if (instance == NULL) {
        ESP_LOGE(COAP_TAG, "OpenThread instance is NULL. Cannot send CoAP message.");
    return;
}

    // Create a new CoAP message
    message = otCoapNewMessage(instance, NULL);
    if (message == NULL) {
        ESP_LOGE(COAP_TAG, "Failed to allocate CoAP message");
        return;
    }

    // Set CoAP message type and code (POST request)
    otCoapMessageInit(message, OT_COAP_TYPE_CONFIRMABLE, OT_COAP_CODE_PUT);
    otCoapMessageAppendUriPathOptions(message, COAP_URI_PATH);
    
    //set the message type to JSON
    otCoapMessageAppendContentFormatOption(message, OT_COAP_OPTION_CONTENT_FORMAT_JSON);
    // Build JSON payload with sensor data
    char payload[128];
    snprintf(payload, sizeof(payload), "{\"test\":123}");
    
    //set a payload marker         
    otCoapMessageSetPayloadMarker(message);
    
    // Add the payload to the CoAP message
    otMessageAppend(message, payload, strlen(payload));

    // Set the destination address (replace with actual CoAP server address)
    memset(&messageInfo, 0, sizeof(messageInfo));
    otIp6AddressFromString(COAP_SERVER, &messageInfo.mPeerAddr); // Multicast or specific server address
    messageInfo.mPeerPort = OT_COAP_PORT;

    // Send the CoAP message
    error = otCoapSendRequest(instance, message, &messageInfo, handle_coap_response, NULL);
    if (error != OT_ERROR_NONE) {
        ESP_LOGE(COAP_TAG, "Failed to send CoAP message: %d", error);
        otMessageFree(message);
    } else {
        ESP_LOGI(COAP_TAG, "CoAP message sent successfully");
    }
}

void coap_init(otInstance *instance) {
    otCoapStart(instance, OT_DEFAULT_COAP_PORT);
    ESP_LOGI(COAP_TAG, "CoAP started on port %d", OT_DEFAULT_COAP_PORT);
}

//CRC check code
uint16_t CRC16(uint16_t crc, const uint8_t *buf, size_t len)
{
    for (size_t pos = 0; pos < len; pos++)
    {
        crc ^= (unsigned int)buf[pos]; // * XOR byte into least sig. byte of crc
                                       // * Loop over each bit
        for (int i = 8; i != 0; i--)
        {
            // * If the LSB is set
            if ((crc & 0x0001) != 0)
            {
                // * Shift right and XOR 0xA001
                crc >>= 1;
                crc ^= 0xA001;
            }
            // * Else LSB is not set
            else
                // * Just shift right
                crc >>= 1;
        }
    }
    return crc;
}

// DSMR reader task
void dsmr_reader_task(void *pvParameters)
{
    
    uint8_t* buffer = (uint8_t*) malloc(BUF_SIZE+1);
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    if (buffer == NULL || data == NULL) { 
        ESP_LOGE(DSMR_TAG, "Failed to allocate memory for buffer"); 
        if (buffer) free(buffer);
        if (data) free(data);
        vTaskDelete(NULL);
        return; 
    }
    int buffer_index = 0;
    int telegram_length = 0;

    while (1) {
        // Read data from UART
        int length = uart_read_bytes(UART_PORT_NUM, buffer + buffer_index, BUF_SIZE - buffer_index, pdMS_TO_TICKS(1000));
        if (length > 0) {
            buffer_index += length;
            // Check if we have received the full telegram
            for (int i = 0; i < buffer_index; i++) {
                if (buffer[i] == '/') {
                    telegram_length = 0;  // Reset telegram length at the start of a new telegram
                }
                telegram_length++;

                // Check for the end of the telegram and the 4 characters after "!"
                if (buffer[i] == '!' && (i + 4) < buffer_index) {
                    // Ensure there's enough room for the 4 characters after "!"
                    if (telegram_length + 4 <= BUF_SIZE) {
                        memcpy(data, buffer + (i + 1 - telegram_length), telegram_length + 4);
                        data[telegram_length + 4] = '\0';

                        // Log the telegram
                        //ESP_LOGI(DSMR_TAG, "Received telegram:\n%s", data);

                        // Log the data going into the CRC function
                        //ESP_LOGI(DSMR_TAG, "Data for CRC:\n%.*s", telegram_length, data);

                        // Extract CRC string
                        char crc_str[5];
                        memcpy(crc_str, data + telegram_length, 4);
                        crc_str[4] = '\0';

                        // Convert CRC string to integer
                        unsigned int received_crc;
                        sscanf(crc_str, "%4X", &received_crc);
                        ESP_LOGI(DSMR_TAG, "Received CRC: 0x%04X", received_crc);

                        // Calculate CRC for the telegram data (excluding CRC part)
                        unsigned int calculated_crc = CRC16(0x0000, data, telegram_length);
                        ESP_LOGI(DSMR_TAG, "Calculated CRC: 0x%04X", calculated_crc);

                        // Verify CRC
                        if (calculated_crc == received_crc) {
                            ESP_LOGI(DSMR_TAG, "CRC check passed!");
                        } else {
                            ESP_LOGE(DSMR_TAG, "CRC check failed!");
                            ESP_LOGI(DSMR_TAG, "Data for CRC (hex):"); 
                            for (int j = 0; j < telegram_length; j++) { 
                                printf("%02X ", data[j]); 
                                } 
                            printf("\n");
                        }

                        // Adjust buffer for the next telegram 
                        memmove(buffer, buffer + (i + 5), buffer_index - (i + 5));
                        buffer_index -= (i + 5);
                        break;
                    }
                    
                    
                }
            }
        }

        // Clear the buffer if it exceeds the capacity
        if (buffer_index >= BUF_SIZE) {
            memset(buffer, 0, BUF_SIZE);
            buffer_index = 0;
        }
    }

    free(buffer);
    free(data);
    vTaskDelete(NULL);
}

// Send data task
void send_data_task(void *pvParameters) {
    
    while (1) {
        
        // Lock the mutex before accessing the shared data
        if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
            // Read global variables

            xSemaphoreGive(data_mutex);  // Release the mutex
        } else { 
            ESP_LOGE("Mutex", "Failed to take mutex"); 
        } 

        // Code to send data to the server
        
        send_coap_message();
        vTaskDelay(pdMS_TO_TICKS(SEND_DELAY_MS));  // Delay for 30 seconds
    }
}

// Openthread task
static void ot_task_worker(void *aContext)
{
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };

    // Initialize the OpenThread stack
    ESP_ERROR_CHECK(esp_openthread_init(&config));

#if CONFIG_OPENTHREAD_LOG_LEVEL_DYNAMIC
    // The OpenThread log level directly matches ESP log level
    (void)otLoggingSetLevel(CONFIG_LOG_DEFAULT_LEVEL);
#endif
    // Initialize the OpenThread cli
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_init();
#endif

    esp_netif_t *openthread_netif;
    // Initialize the esp_netif bindings
    openthread_netif = init_openthread_netif(&config);
    esp_netif_set_default_netif(openthread_netif);

#if CONFIG_OPENTHREAD_CLI_ESP_EXTENSION
    esp_cli_custom_command_init();
#endif // CONFIG_OPENTHREAD_CLI_ESP_EXTENSION

    //initialise the coap server
    coap_init(esp_openthread_get_instance());

    // Run the main loop
#if CONFIG_OPENTHREAD_CLI
    esp_openthread_cli_create_task();
#endif

    otOperationalDatasetTlvs dataset;
    otError error = otDatasetGetActiveTlvs(esp_openthread_get_instance(), &dataset);
    ESP_ERROR_CHECK(esp_openthread_auto_start((error == OT_ERROR_NONE) ? &dataset : NULL));

    esp_openthread_launch_mainloop();

    // Clean up
    esp_netif_destroy(openthread_netif);
    esp_openthread_netif_glue_deinit();

    esp_vfs_eventfd_unregister();
    vTaskDelete(NULL);
}

void app_main(void)
{

// Used eventfds:
    // * netif
    // * ot task queue
    // * radio driver
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));
       
    // Create mutex
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE("APP", "Failed to create mutex");
        return;
    }

    ESP_LOGI(UART_TAG, "UART setup started.");
    //init drivers
    init_uart();

    xTaskCreate(ot_task_worker, "ot_cli_main", 10240, xTaskGetCurrentTaskHandle(), 5, NULL);
    xTaskCreate(dsmr_reader_task, "dsmr_reader_task",  8192, NULL, 6, NULL);
    //xTaskCreate(&send_data_task, "send_data_task", 2048, NULL, 5, NULL);
}
