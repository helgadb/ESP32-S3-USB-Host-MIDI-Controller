#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"

#include "midi_class_driver_txrx.c"

static const char *TAG = "DAEMON";

#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3

// Configuração do botão
#define BUTTON_GPIO             6
#define BUTTON_ACTIVE_LEVEL     0   // 0 para botão ativo em LOW (pull-up)

// Declaração da função de status (já está no midi_class_driver_txrx.c)
extern void midi_driver_print_status(void);

static void host_lib_daemon_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    //Signal to the class driver task that the host library is installed
    xSemaphoreGive(signaling_sem);
    vTaskDelay(10); //Short delay to let client task spin up

    while (1) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGI(TAG, "no clients available");
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "no devices connected");
        }
    }
}

static void button_check_task(void *arg)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "🎛️ SINGLE COMMAND MODE");
    ESP_LOGI(TAG, "Press button to send: 0B B0 00 00");

    bool last_button_state = true;

    while (1) {
        bool current_state = gpio_get_level(BUTTON_GPIO);
        
        if (last_button_state && !current_state) {
            ESP_LOGI(TAG, "🎹 SENDING: 0B B0 00 00");
            
            if (midi_driver_ready_for_tx()) {
                // Enviar APENAS este comando específico
                uint8_t midi_message[MIDI_MESSAGE_LENGTH] = {0x0B, 0xB0, 0x00, 0x00};
                midi_send_data(midi_message, sizeof(midi_message));
            } else {
                ESP_LOGI(TAG, "❌ Driver not ready");
            }
        }
        
        last_button_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    SemaphoreHandle_t signaling_sem = xSemaphoreCreateBinary();

    TaskHandle_t daemon_task_hdl;
    TaskHandle_t class_driver_task_hdl;
    TaskHandle_t button_task_hdl;
    
    //Create daemon task
    xTaskCreatePinnedToCore(host_lib_daemon_task,
                            "daemon",
                            4096,
                            (void *)signaling_sem,
                            DAEMON_TASK_PRIORITY,
                            &daemon_task_hdl,
                            0);
    //Create the class driver task
    xTaskCreatePinnedToCore(class_driver_task,
                            "class",
                            4096,
                            (void *)signaling_sem,
                            CLASS_TASK_PRIORITY,
                            &class_driver_task_hdl,
                            0);

    //Create button task
    xTaskCreatePinnedToCore(button_check_task,
                            "button",
                            4096,
                            NULL,
                            CLASS_TASK_PRIORITY,
                            &button_task_hdl,
                            1);

    vTaskDelay(10);     //Add a short delay to let the tasks run

    ESP_LOGI(TAG, "System started. Connect a MIDI USB device and press the button on GPIO %d", BUTTON_GPIO);
    ESP_LOGI(TAG, "The button will show MIDI driver status and send CC0 if ready.");
}