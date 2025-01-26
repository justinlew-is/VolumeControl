#include <esp_log.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"

#include <esp_matter.h>
#include "bsp/esp-bsp.h"

#include <app_priv.h>

using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t light_endpoint_id;

static int potValue = -1;

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_POWER_PIN GPIO_NUM_20
#define I2C_MASTER_SDA_IO 19
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_FREQ_HZ 100000
#define CHANNEL_0 0x01 // To select channel 0 on the multiplexer
#define CHANNEL_1 0x02 // To select channel 1 on the multiplexer
#define RELAY_ADDR_0 0x20
#define RELAY_ADDR_1 0x21
#define MUTE 0
#define MAX_VOLUME 100 // 63 is absolute max

esp_err_t enable_i2c_power() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = ESP_OK;
    err |= gpio_config(&io_conf);
    err |= gpio_set_level(I2C_POWER_PIN, 1);
    return err;
}

esp_err_t i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    err |= i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }
    err |= enable_i2c_power();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2C power: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t attenuator_init(uint8_t address) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 0x1);
    i2c_master_write_byte(cmd, 0x00, 0x1); // Send 0
    i2c_master_write_byte(cmd, 0x00, 0x1); // Send 0
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                                         1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int nibbleShift(int num) {
    int var = 0;
    int s = 8;
    for (int i = 0; i < (s / 2); i++) {
        int p = s - i - 1;
        int x = (num >> p) & 1;
        int y = (num >> i) & 1;
        var |= x << i;
        var |= y << p;
    }
    return var;
}

static esp_err_t attenuator_set_value(uint8_t address, uint8_t attenuation_value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 0x1);
    i2c_master_write_byte(cmd, 0x09, 0x1); // Select the GPIO register
    i2c_master_write_byte(cmd, attenuation_value, 0x1); // Set registers
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t setVolume(int newLevel) {
    if (newLevel < 0) {
        newLevel = 0;
    } else if (newLevel > 127) {
        newLevel = 127;
    }
    int attenuation_value = nibbleShift(newLevel);
    attenuation_value |= 1 << 0; // Set bit 0 to 1
    esp_err_t err = ESP_OK;
    err |= attenuator_set_value(RELAY_ADDR_0 ,attenuation_value);
    err |= attenuator_set_value(RELAY_ADDR_1 ,attenuation_value);
    return err;
}

static esp_err_t updateLevelAttribute(uint16_t endpoint_id, uint8_t new_level)
{
    int clampedPot = (potValue <= 0) ? 1 : potValue;
    if (clampedPot > 100) {
        clampedPot = 100;
    }
    uint8_t matterLevel = (uint8_t)((clampedPot * 254) / 100);
    if (matterLevel < 1) matterLevel = 1;
    if (matterLevel > 254) matterLevel = 254;
    esp_matter_attr_val_t val = {
        .type = ESP_MATTER_VAL_TYPE_UINT8,
        .val = {
            .u8 = matterLevel
        }
    };

    esp_err_t err = esp_matter::attribute::update(endpoint_id,
                                                 LevelControl::Id,
                                                 LevelControl::Attributes::CurrentLevel::Id,
                                                 &val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update attribute to %u (err=0x%x)", new_level, err);
    }
    return err;
}

static esp_err_t app_driver_light_set_power(led_indicator_handle_t handle, esp_matter_attr_val_t *val) {
    esp_err_t err = ESP_OK;
    if (!val->val.b) {
        ESP_LOGI(TAG, "Muting device");
        err = setVolume(MUTE);
    }
    return err;
}

static esp_err_t app_driver_light_set_brightness(led_indicator_handle_t handle, esp_matter_attr_val_t *val) {
    int newLevelFromController = val->val.u8; 
    if (newLevelFromController > MAX_VOLUME) {
        newLevelFromController = MAX_VOLUME;
    }
    ESP_LOGI(TAG, "Setting volume (from controller): %d", newLevelFromController);
    setVolume(newLevelFromController);
    return ESP_OK;
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val) {
    esp_err_t err = ESP_OK;
    if (endpoint_id == light_endpoint_id) {
        led_indicator_handle_t handle = (led_indicator_handle_t)driver_handle;
        if (cluster_id == OnOff::Id) {
            if (attribute_id == OnOff::Attributes::OnOff::Id) {
                err = app_driver_light_set_power(handle, val);
            }
        } else if (cluster_id == LevelControl::Id) {
            if (attribute_id == LevelControl::Attributes::CurrentLevel::Id) {
                err = app_driver_light_set_brightness(handle, val);
            }
        }
    }
    return err;
}

esp_err_t app_driver_light_set_defaults(uint16_t endpoint_id) {
    esp_err_t err = ESP_OK;
    void *priv_data = endpoint::get_priv_data(endpoint_id);
    led_indicator_handle_t handle = (led_indicator_handle_t)priv_data;
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);

    /* Setting brightness */
    attribute_t *attribute = attribute::get(endpoint_id, LevelControl::Id, LevelControl::Attributes::CurrentLevel::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_light_set_brightness(handle, &val);

    /* Setting power */
    attribute = attribute::get(endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id);
    attribute::get_val(attribute, &val);
    err |= app_driver_light_set_power(handle, &val);

    return err;
}

static void PotentiometerTask(void *pvParam)
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_1, &channel_config));

    while (true) {
        int adc_reading = 0;
        if (adc_oneshot_read(adc_handle, ADC_CHANNEL_1, &adc_reading) == ESP_OK) {
            float rawF = adc_reading * (100.0f / 3328.0f);
            int newVal = (int)roundf(rawF);
            if (newVal < 0) newVal = 0;
            if (newVal > 100) newVal = 100;
            newVal = ((newVal + 5) / 10) * 10;
            if (potValue == -1) {
                potValue = newVal;
            }

            if (newVal != potValue) {
                potValue = newVal;
                setVolume(potValue);
                ESP_LOGI(TAG, "Pot changed -> set hardware volume to %d", potValue);
                updateLevelAttribute(light_endpoint_id, (uint8_t)potValue);
            }
        }

        // Poll every 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    adc_oneshot_del_unit(adc_handle);
    vTaskDelete(NULL);
}

app_driver_handle_t app_driver_light_init()
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(attenuator_init(RELAY_ADDR_0));
    ESP_ERROR_CHECK(attenuator_init(RELAY_ADDR_1));
    xTaskCreate(PotentiometerTask, "PotentiometerTask", 4096, NULL, 5, NULL);
    return NULL;
}
