// Complete drop-in app_main.cpp for VolumeControl ESP32-C6.
// Based on https://github.com/justinlew-is/VolumeControl/blob/main/main/app_main.cpp
// Modification: command startup mute in the relay driver BEFORE Matter starts,
// then start the scaled-maximum potentiometer task after Matter defaults.
// No intermediate mute is inserted between ordinary volume changes.
// Keep Matter commissioning,
// callbacks, OTA configuration, and console behavior from the original.

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <app_priv.h>
#include <app_reset.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

static const char *TAG = "app_main";
uint16_t light_endpoint_id = 0;

// Adafruit ESP32-C6 Feather: BOOT is active-low on GPIO9.
// Leave the dedicated RESET button alone; it must remain a normal reboot.
static constexpr gpio_num_t kFactoryResetButton = GPIO_NUM_9;
static constexpr uint32_t kButtonPollMs = 20;
static constexpr int kDebounceSamples = 4; // 80 ms stable press/release.

static void boot_factory_reset_task(void *)
{
    gpio_config_t button_config = {};
    button_config.pin_bit_mask = (1ULL << kFactoryResetButton);
    button_config.mode = GPIO_MODE_INPUT;
    button_config.pull_up_en = GPIO_PULLUP_ENABLE;
    button_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    button_config.intr_type = GPIO_INTR_DISABLE;
    esp_err_t err = gpio_config(&button_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot configure BOOT button: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    // Arm only after the button has been released. This avoids interpreting
    // a button held during startup as a request to delete credentials.
    int released_samples = 0;
    while (released_samples < kDebounceSamples) {
        released_samples = (gpio_get_level(kFactoryResetButton) == 1)
            ? released_samples + 1 : 0;
        vTaskDelay(pdMS_TO_TICKS(kButtonPollMs));
    }
    ESP_LOGI(TAG, "BOOT button ready: press and release to factory-reset Matter");

    bool pressed = false;
    int low_samples = 0;
    int high_samples = 0;
    while (true) {
        if (gpio_get_level(kFactoryResetButton) == 0) {
            low_samples = (low_samples < kDebounceSamples)
                ? low_samples + 1 : low_samples;
            high_samples = 0;
            if (low_samples >= kDebounceSamples) pressed = true;
        } else {
            low_samples = 0;
            high_samples = (high_samples < kDebounceSamples)
                ? high_samples + 1 : high_samples;
            if (pressed && high_samples >= kDebounceSamples) {
                ESP_LOGW(TAG, "BOOT press detected; scheduling Matter factory reset");
                // ESP-Matter clears SDK-owned commissioning state and asks
                // the CHIP stack to factory reset and reboot the board.
                err = esp_matter::factory_reset();
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Factory-reset request returned: %s", esp_err_to_name(err));
                }
                // A successful request reboots asynchronously; do not keep
                // re-triggering the reset while it is pending.
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(kButtonPollMs));
    }
    vTaskDelete(nullptr);
}

static void start_boot_factory_reset_button()
{
    BaseType_t result = xTaskCreate(boot_factory_reset_task, "boot_reset", 3072,
                                    nullptr, 4, nullptr);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Cannot create BOOT factory-reset task");
    }
}

// New driver starts this task after Matter is up and the output is muted.
void app_driver_start_potentiometer();

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;
#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }
    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;
    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}
// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}
// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;
    if (type == PRE_UPDATE) {
        /* Driver update */
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize relay driver and issue HARD MUTE before Matter starts.
       In RELAY_TEST_MODE this is simulated; no I2C hardware is touched. */
    app_driver_handle_t light_handle = app_driver_light_init();
    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;

    // node handle can be used to add/modify other endpoints.
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));
    dimmable_light::config_t light_config;
    light_config.on_off.on_off = DEFAULT_POWER;
    light_config.on_off.lighting.start_up_on_off = nullptr;
    light_config.level_control.current_level = DEFAULT_BRIGHTNESS;
    // Matter OnLevel is nullable (0xFF is null for nullable uint8 in the
    // legacy ESP-Matter v1.3 configuration). Leaving it null asks Matter to
    // restore the prior CurrentLevel on On, rather than forcing level 1.
    // The driver also retains the requested level independently during Off.
    light_config.level_control.on_level = 0xFF;
    light_config.level_control.lighting.start_up_current_level = DEFAULT_BRIGHTNESS;
    // endpoint handles can be used to add/modify clusters.
    endpoint_t *endpoint = dimmable_light::create(node, &light_config, ENDPOINT_FLAG_NONE, light_handle);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create extended color light endpoint"));

    light_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Light created with endpoint_id %d", light_endpoint_id);
    /* Mark deferred persistence for some attributes that might be changed rapidly */
    cluster_t *level_control_cluster = cluster::get(endpoint, LevelControl::Id);
    attribute_t *current_level_attribute = attribute::get(level_control_cluster, LevelControl::Attributes::CurrentLevel::Id);
    attribute::set_deferred_persistence(current_level_attribute);
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif
    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

    /* Starting driver with default values */
    err = app_driver_light_set_defaults(light_endpoint_id);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to apply safe startup volume: %s", esp_err_to_name(err)));
    // Start ADC-based physical maximum only AFTER Matter initialization and mute.
    app_driver_start_potentiometer();
    // BOOT becomes an explicit one-press factory reset AFTER the Matter stack is ready.
    start_boot_factory_reset_button();
#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif // CONFIG_ENABLE_ENCRYPTED_OTA
#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif
}
