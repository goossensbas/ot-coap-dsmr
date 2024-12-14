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
unsigned int CRC16(unsigned int crc, const uint8_t *buf, size_t len)
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

//temporary log function
void log_data(const uint8_t *buffer, size_t len)
{
    printf("Data: ");
    for (size_t i = 0; i < len; i++)
    {
        printf("%c", buffer[i]);
    }
    printf("\n");
}

// DSMR reader task
void dsmr_reader_task(void *pvParameters)
{
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE+1);
    if (data == NULL) { 
        ESP_LOGE(DSMR_TAG, "Failed to allocate memory for buffer"); 
        return; 
    }
    uint8_t* buffer = (uint8_t*) malloc(BUF_SIZE + 1); 
    if (buffer == NULL) { 
      ESP_LOGE(DSMR_TAG, "Failed to allocate memory for buffer"); 
      free(data); 
      return; 
    } 
    
    int buffer_index = 0; 
    int write_to_buffer = 0;

    while (1){        
        int bytes_read = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE, pdMS_TO_TICKS(10)); 
        if (bytes_read > 0) {            
            // Process each byte 
            for (int i = 0; i < bytes_read; i++) { 
                if (data[i] == '/') { 
                    // Start writing to buffer if the backslash is found 
                    write_to_buffer = 1; 
                    buffer_index = 0; 
                }
                if (write_to_buffer) { 
                    buffer[buffer_index++] = data[i];
                    
                    // Check for the end of the telegram (exclamation mark "!")
                    if (data[i] == '!'){
                            // Ensure there's enough room for the CRC (4 characters)
                        if (buffer_index + 4 <= BUF_SIZE) {
                            // Read the CRC characters after the "!"
                            for (int j = 0; j < 4; j++) {
                                int crc_bytes_read = uart_read_bytes(UART_PORT_NUM, &buffer[buffer_index], 1, pdMS_TO_TICKS(10));
                                if (crc_bytes_read > 0) {
                                    buffer_index++;
                                }
                            }

                            buffer[buffer_index] = 0; // Null-terminate the string for safe handling

                            //log the entire buffer content
                            log_data(buffer,buffer_index);

                            // Extract the CRC string 
                            char crc_str[5]; 
                            memcpy(crc_str, &buffer[buffer_index - 4], 4); 
                            crc_str[4] = 0; // Null-terminate the CRC string

                            // Convert the CRC string to an integer 
                            unsigned int received_crc; 
                            sscanf(crc_str, "%4X", &received_crc); 
                            ESP_LOGI(DSMR_TAG, "Received CRC: 0x%04X", received_crc);

                            // Log the data going into CRC calculation
                            //log_data(buffer, buffer_index);
                            
                            // Calculate CRC16 for the data before "!" 
                            unsigned int calculated_crc = CRC16(0xFFFF, buffer, buffer_index -5); 
                            ESP_LOGI(DSMR_TAG, "Calculated CRC: 0x%04X", calculated_crc);

                            // Print the telegram including the CRC part
                            ESP_LOGI(DSMR_TAG, "Received telegram: %.*s%4s", buffer_index, buffer, crc_str); 
                            
                            // Compare the calculated CRC with the received CRC 
                            if (calculated_crc == received_crc) { 
                                ESP_LOGI(DSMR_TAG, "CRC check passed!"); 
                            } 
                            else { 
                                ESP_LOGE(DSMR_TAG, "CRC check failed!"); 
                            } 
                            // Clear the buffer by resetting the index 
                            memset(buffer, 0, BUF_SIZE + 1); 
                            buffer_index = 0; 
                            write_to_buffer = 0; // Stop writing to buffer
                        }
                    }
                }
            }    
        }
    }
    free(data);
    free(buffer);
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
