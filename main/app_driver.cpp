// VolumeControl ESP32-C6 app_driver.cpp: scaled physical-pot maximum.
// Based on https://github.com/justinlew-is/VolumeControl main/app_driver.cpp
// PRESERVED: nibbleShift(), I2C addresses, 5V relay hardware, 10% pot
// quantization, and restoration of the last requested level on Matter On.
// 0% or Matter Off: de-energize K16 (GP0 low, outputs routed to AGND).
// Nonzero -> nonzero: change the attenuation relays DIRECTLY; no transient mute.
// On startup: explicitly drive both MCP23008 expanders to 0x00 before Matter.
// Off -> On: restore the previous remote request, limited by the CURRENT pot.
// A reboot is different: output starts muted until an explicit control action.
// RELAY_TEST_MODE=1 allows full Matter + ADC testing without MCP23008 boards.
// Matter CurrentLevel = REQUEST, not physical relay output. Pot sets the ceiling.

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "driver/i2c.h"           // ESP-IDF v5.2.3 legacy I2C master API.
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_matter.h"
#include "bsp/esp-bsp.h"
#include "app_priv.h"

using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t light_endpoint_id;

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_POWER_PIN GPIO_NUM_20
#define I2C_MASTER_SDA_IO 19
#define I2C_MASTER_SCL_IO 18
#define I2C_MASTER_FREQ_HZ 100000
#define RELAY_ADDR_0 0x20
#define RELAY_ADDR_1 0x21
#define MUTE 0
#define MAX_VOLUME 100 // Preserves previous maximum; don't raise without listening tests.
#define RELAY_TEST_MODE 0 // 1 = simulate relay commands (no I2C); 0 = use both powered MCP23008 boards.

// C203 is not a complete manufacturer part number; the taper below is a
// calibration estimate, not a verified datasheet characteristic.
// If the knob is LINEAR as wired, set kPotMidpointFraction to 0.50f.
// To calibrate, measure ADC at minimum, mechanical 50%, and maximum:
// kPotMidpointFraction = (ADC_mid - kPotAdcMin) / (kPotAdcMax - kPotAdcMin).
static constexpr int kPotAdcMin = 0;
// Measure POT_RAW with the knob at its mechanical maximum; replace 3328 with
// that measured value if the full-scale reading is below the expected endpoint.
// Do not auto-learn the maximum: booting at mid-knob could incorrectly raise the
// ceiling to 100%, which is undesirable for an audio volume safety limiter.
static constexpr int kPotAdcMax = 3328;     // ORIGINAL estimate; needs hardware calibration.
static constexpr float kPotMidpointFraction = 0.85f; // Estimate for reverse-audio taper.
static constexpr bool kPotReverseDirection = false; // Original wiring: ADC rises with volume.
static constexpr float kPotHysteresis = 0.015f;     // 1.5% of knob rotation.
// ADC noise / endpoint mismatch can make a 100% bucket hard to reach with
// a steep reverse-audio taper. Accept the final 1% of calibrated ADC range
// as the mechanical end stop; use a slightly lower threshold to release it.
// These thresholds do not replace the need for correct kPotAdcMax calibration.
static constexpr float kPotFullScaleEnter = 0.990f;
static constexpr float kPotFullScaleExit = 0.987f;
static constexpr bool kPrintPotCalibration = true;  // Raw ADC sample every 5 seconds.

static SemaphoreHandle_t relayMutex = nullptr;
static SemaphoreHandle_t controlMutex = nullptr;
// All variables below are guarded by controlMutex.
static uint8_t requestedMatterLevel = 1; // Matter 1..254 (percent of available pot-limited range)
static int potLimit = 0;                  // 0..MAX_VOLUME, in deliberate 10-point increments
static int lastPotBucket = -1;
static bool potReady = false;             // Do not apply uncontrolled gain before first ADC reading
static bool matterOn = false;
static bool armed = false;                // Cold-boot safety latch; Matter On explicitly arms output
static int lastAppliedRelayLevel = -1;    // Last FULLY successful 2-board I2C write


static int clampInt(int v, int lo, int hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

// Invert a smooth exponential approximation to the pot taper, so equally
// spaced knob rotations approximate equally spaced 10%-volume commands.
// A measured multi-point calibration would be more accurate than one midpoint.
static float adcToNormalizedFraction(int reading)
{
    float x = (float)(reading - kPotAdcMin) / (float)(kPotAdcMax - kPotAdcMin);
    x = fminf(1.0f, fmaxf(0.0f, x));
    return kPotReverseDirection ? (1.0f - x) : x;
}

static float adcToKnobFraction(int reading)
{
    const float x = adcToNormalizedFraction(reading);
    const float midpoint = kPotMidpointFraction;
    if (midpoint <= 0.01f || midpoint >= 0.99f || fabsf(midpoint - 0.5f) < 0.01f) {
        return x;
    }
    const float k = 2.0f * logf(midpoint / (1.0f - midpoint));
    // exp(-k) is finite for midpoint in [0.01,0.99].
    float rotation = -log1pf(-x * (1.0f - expf(-k))) / k;
    return fminf(1.0f, fmaxf(0.0f, rotation));
}

// Return 0..10 so the physical control still has ELEVEN deliberate 10% stops.
static int bucketFromKnob(float rotation)
{
    return clampInt((int)lroundf(rotation * 10.0f), 0, 10);
}

// Transition only if we've passed the nominal boundary with some hysteresis.
// A stationary knob near a 5%-boundary no longer chatters between two relays.
static bool bucketMovedFarEnough(float rotation, int oldBucket, int newBucket)
{
    if (newBucket == oldBucket) return false;
    float boundary = (newBucket > oldBucket)
        ? (oldBucket + 0.5f) / 10.0f
        : (oldBucket - 0.5f) / 10.0f;
    return (newBucket > oldBucket)
        ? (rotation >= boundary + kPotHysteresis)
        : (rotation <= boundary - kPotHysteresis);
}

esp_err_t enable_i2c_power()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;
    return gpio_set_level(I2C_POWER_PIN, 1);
}

esp_err_t i2c_master_init()
{
    i2c_config_t conf = {};
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
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = enable_i2c_power();
    if (err != ESP_OK) ESP_LOGE(TAG, "Failed to enable I2C power: %s", esp_err_to_name(err));
    return err;
}

// A single MCP23008 register write. Used for both safe startup and direct
// relay updates; it does not change any other relay outputs between levels.
static esp_err_t mcp23008_write_register(uint8_t address, uint8_t reg, uint8_t value)
{
#if RELAY_TEST_MODE
    (void)address;
    (void)reg;
    (void)value;
    return ESP_OK;
#else
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return err;
#endif
}

// Put the output latches LOW *before* changing IODIR from inputs to outputs.
// Important after a Feather-only reboot: the expander may have stayed powered
// and retained a previously energized K16. Writing OLAT=0 releases K16 first.
static esp_err_t attenuator_init(uint8_t address)
{
    esp_err_t err = mcp23008_write_register(address, 0x0A, 0x00); // OLAT: all low
    if (err != ESP_OK) return err;
    return mcp23008_write_register(address, 0x00, 0x00); // IODIR: all outputs
}

// EXACT original bit mapping: bit-reverse the 8-bit value.
int nibbleShift(int num)
{
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

static esp_err_t attenuator_set_value(uint8_t address, uint8_t value)
{
    return mcp23008_write_register(address, 0x09, value); // MCP23008 GPIO
}

esp_err_t setVolume(int newLevel)
{
    newLevel = clampInt(newLevel, MUTE, MAX_VOLUME);
    int attenuation_value = nibbleShift(newLevel);
    // K16 (GP0) is the output mute relay on both stereo channels.
    // GP0 HIGH energizes K16 and selects the attenuator ladder (NO contact).
    // GP0 LOW releases K16 to its NC contact, tying both outputs to AGND.
    // Do NOT force GP0 HIGH at volume 0: that would defeat hard mute.
    // Between NONZERO levels there is no mute step or deliberate delay.
    if (newLevel > MUTE) {
        attenuation_value |= (1 << 0);
    } else {
        attenuation_value &= ~(1 << 0);
    }

#if RELAY_TEST_MODE
    ESP_LOGI(TAG, "[SIMULATED RELAYS] target=%d%% mappedGPIO=0x%02X (I2C disabled)",
             newLevel, (unsigned)attenuation_value);
    return ESP_OK;
#else
    if (relayMutex) xSemaphoreTake(relayMutex, portMAX_DELAY);
    esp_err_t left = attenuator_set_value(RELAY_ADDR_0, (uint8_t)attenuation_value);
    esp_err_t right = attenuator_set_value(RELAY_ADDR_1, (uint8_t)attenuation_value);
    if (relayMutex) xSemaphoreGive(relayMutex);

    if (left != ESP_OK) ESP_LOGE(TAG, "I2C relay 0x%02x failed: %s", RELAY_ADDR_0, esp_err_to_name(left));
    if (right != ESP_OK) ESP_LOGE(TAG, "I2C relay 0x%02x failed: %s", RELAY_ADDR_1, esp_err_to_name(right));
    return (left != ESP_OK) ? left : right; // Do not obscure which controller failed.
#endif
}

// Physical pot is a maximum (NOT a competing remote volume control).
// Standard Matter CurrentLevel continues to be the remote REQUEST: 1..254.
// Example: CurrentLevel=203 (~80%), potLimit=50 => computed step ~40.
static int scaledRequestedTargetLocked()
{
    return clampInt(((int)requestedMatterLevel * potLimit + 127) / 254,
                    MUTE, MAX_VOLUME);
}

static int scaledRelayTargetLocked()
{
    if (!matterOn || !armed || !potReady) return MUTE;
    return scaledRequestedTargetLocked();
}

// Caller must hold controlMutex; setVolume() itself holds relayMutex.
// Logging is driven by INPUT changes, not only changes to the rounded relay
// target: the Home slider must still be visible when potLimit is 0%.
static esp_err_t applyVolumeLocked(const char *reason = nullptr, int rawAdc = -1)
{
    const int target = scaledRelayTargetLocked();
    const int calculated = scaledRequestedTargetLocked();
    esp_err_t err = ESP_OK;
    if (target != lastAppliedRelayLevel) {
        err = setVolume(target);
        if (err == ESP_OK) {
            lastAppliedRelayLevel = target;
        }
    }

    if (reason != nullptr) {
        const int matterPercent = ((int)requestedMatterLevel * 100 + 127) / 254;
        if (rawAdc >= 0) {
            ESP_LOGI(TAG,
                     "%s: Home=%d%% (Matter level=%u/254), pot max=%d%% (ADC raw=%d), "
                     "scaled=%d%%, relay output=%d%%, power=%s, armed=%s%s",
                     reason, matterPercent, (unsigned)requestedMatterLevel, potLimit,
                     rawAdc, calculated, target, matterOn ? "ON" : "OFF", armed ? "YES" : "NO",
                     err == ESP_OK ? "" : " [RELAY WRITE FAILED]");
        } else {
            ESP_LOGI(TAG,
                     "%s: Home=%d%% (Matter level=%u/254), pot max=%d%%, "
                     "scaled=%d%%, relay output=%d%%, power=%s, armed=%s%s",
                     reason, matterPercent, (unsigned)requestedMatterLevel, potLimit,
                     calculated, target, matterOn ? "ON" : "OFF", armed ? "YES" : "NO",
                     err == ESP_OK ? "" : " [RELAY WRITE FAILED]");
        }
    }
    // On partial hardware failure, leave lastAppliedRelayLevel unchanged to
    // retry on the next ADC poll. In test mode, this always succeeds.
    return err;
}

static esp_err_t app_driver_light_set_power(led_indicator_handle_t, esp_matter_attr_val_t *val)
{
    xSemaphoreTake(controlMutex, portMAX_DELAY);
    bool oldOn = matterOn;
    bool oldArmed = armed;
    if (!val->val.b) {
        // Mute physically, but retain requestedMatterLevel so On can restore it.
        matterOn = false;
        armed = false;
    } else {
        // User-requested behavior: On restores the last selected Matter level.
        // The *current* potentiometer limit still caps the physical relay step.
        // A stored request of zero remains muted after On.
        matterOn = true;
        armed = true;
    }
    const bool stateChanged = (matterOn != oldOn || armed != oldArmed);
    esp_err_t err = applyVolumeLocked(stateChanged ? "Matter On/Off" : nullptr);
    if (err != ESP_OK) { // Matter PRE_UPDATE can reject failed hardware writes.
        matterOn = oldOn;
        armed = oldArmed;
    }
    xSemaphoreGive(controlMutex);
    return err;
}

static esp_err_t app_driver_light_set_brightness(led_indicator_handle_t, esp_matter_attr_val_t *val)
{
    // An explicit 0 level must be retained as an absolute mute request.
    uint8_t newRequested = (uint8_t)clampInt(val->val.u8, 0, 254);
    xSemaphoreTake(controlMutex, portMAX_DELAY);
    uint8_t oldRequested = requestedMatterLevel;
    bool oldArmed = armed;
    // The Level Control cluster can emit a temporary minimum during a Matter
    // On/Off transition. Do not let that erase the last volume while Off.
    // A genuine non-minimum level selected while Off can still become the next
    // requested volume. Zero while On remains an explicit mute request.
    if (matterOn || newRequested > 1) {
        requestedMatterLevel = newRequested;
    }
    if (matterOn && requestedMatterLevel != oldRequested) armed = true;
    const bool stateChanged = (requestedMatterLevel != oldRequested || armed != oldArmed);
    esp_err_t err = applyVolumeLocked(stateChanged ? "Matter brightness" : nullptr);
    if (err != ESP_OK) {
        requestedMatterLevel = oldRequested;
        armed = oldArmed;
    }
    xSemaphoreGive(controlMutex);
    return err;
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id,
                                      uint32_t cluster_id, uint32_t attribute_id,
                                      esp_matter_attr_val_t *val)
{
    if (endpoint_id != light_endpoint_id) return ESP_OK;
    led_indicator_handle_t handle = (led_indicator_handle_t)driver_handle;
    if (cluster_id == OnOff::Id && attribute_id == OnOff::Attributes::OnOff::Id)
        return app_driver_light_set_power(handle, val);
    if (cluster_id == LevelControl::Id && attribute_id == LevelControl::Attributes::CurrentLevel::Id)
        return app_driver_light_set_brightness(handle, val);
    return ESP_OK;
}

esp_err_t app_driver_light_set_defaults(uint16_t endpoint_id)
{
    // Read persisted Matter attributes. Relay output remains muted on reboot
    // until a new Matter On/level command or physical knob movement arrives.
    esp_matter_attr_val_t level = esp_matter_invalid(NULL);
    esp_matter_attr_val_t power = esp_matter_invalid(NULL);
    endpoint_t *endpoint_handle = endpoint::get(node::get(), endpoint_id);
    if (!endpoint_handle) {
        return ESP_ERR_NOT_FOUND;
    }

    cluster_t *level_cluster = cluster::get(endpoint_handle, LevelControl::Id);
    cluster_t *power_cluster = cluster::get(endpoint_handle, OnOff::Id);

    if (!level_cluster || !power_cluster) {
        return ESP_ERR_NOT_FOUND;
    }

    attribute_t *levelAttr = attribute::get(
        level_cluster, LevelControl::Attributes::CurrentLevel::Id);

    attribute_t *powerAttr = attribute::get(
        power_cluster, OnOff::Attributes::OnOff::Id);

    if (!levelAttr || !powerAttr) {
        return ESP_ERR_NOT_FOUND;
    }
    attribute::get_val(levelAttr, &level);
    attribute::get_val(powerAttr, &power);
    xSemaphoreTake(controlMutex, portMAX_DELAY);
    requestedMatterLevel = (uint8_t)clampInt(level.val.u8, 0, 254);
    matterOn = power.val.b;
    armed = false;
    lastAppliedRelayLevel = -1;
    esp_err_t err = applyVolumeLocked("Boot safe mute"); // Boot stays muted; ordinary Off->On restores.
    xSemaphoreGive(controlMutex);
    return err;
}

static void PotentiometerTask(void *)
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_1, &channel_config));

    bool fullScaleLatched = false; // ADC end-stop hysteresis, local to this task.
    uint32_t pollCount = 0;
    const int fullScaleEnterRaw = kPotAdcMin +
        (int)ceilf(kPotFullScaleEnter * (kPotAdcMax - kPotAdcMin));
    ESP_LOGI(TAG, "Pot calibration: ADC min=%d, max=%d, 100%% end-stop starts at ~%d raw",
             kPotAdcMin, kPotAdcMax, fullScaleEnterRaw);

    while (true) {
        int reading = 0;
        if (adc_oneshot_read(adc_handle, ADC_CHANNEL_1, &reading) == ESP_OK) {
            const float normalized = adcToNormalizedFraction(reading);
            if (normalized >= kPotFullScaleEnter) {
                fullScaleLatched = true;
            } else if (normalized <= kPotFullScaleExit) {
                fullScaleLatched = false;
            }
            const float rotation = fullScaleLatched ? 1.0f : adcToKnobFraction(reading);
            const int bucket = bucketFromKnob(rotation);

            xSemaphoreTake(controlMutex, portMAX_DELAY);
            const bool firstSample = (lastPotBucket < 0);
            bool bucketChanged = false;
            if (firstSample) {
                // At boot learn the ceiling. Startup stays muted until a later control action.
                potReady = true;
                lastPotBucket = bucket;
                potLimit = bucket * (MAX_VOLUME / 10);
                bucketChanged = true;
            } else if (bucketMovedFarEnough(rotation, lastPotBucket, bucket)) {
                lastPotBucket = bucket;
                potLimit = bucket * (MAX_VOLUME / 10);
                if (matterOn) armed = true; // Physical movement can explicitly unmute.
                bucketChanged = true;
            }

            // No I2C traffic occurs in RELAY_TEST_MODE. A periodic poll still
            // retries failed physical writes when the relay boards are enabled.
            esp_err_t err = applyVolumeLocked(
                bucketChanged ? (firstSample ? "Pot initialized" : "Pot moved") : nullptr,
                bucketChanged ? reading : -1);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Volume update failed: %s", esp_err_to_name(err));
            }
            if (kPrintPotCalibration && (++pollCount % 5 == 0)) {
                ESP_LOGI(TAG,
                         "POT_RAW=%d normalized=%.4f correctedRotation=%.3f "
                         "bucket=%d%% endstop=%s (max calibration=%d)",
                         reading, normalized, rotation, potLimit,
                         fullScaleLatched ? "YES" : "NO", kPotAdcMax);
            }
            xSemaphoreGive(controlMutex);
        } else {
            ESP_LOGW(TAG, "Pot ADC read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Call from app_main only after esp_matter::start() + boot defaults.
void app_driver_start_potentiometer()
{
    BaseType_t ok = xTaskCreate(PotentiometerTask, "PotentiometerTask", 4096, NULL, 5, NULL);
    if (ok != pdPASS) ESP_LOGE(TAG, "Failed to create PotentiometerTask");
}

app_driver_handle_t app_driver_light_init()
{
    relayMutex = xSemaphoreCreateMutex();
    controlMutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK((relayMutex && controlMutex) ? ESP_OK : ESP_ERR_NO_MEM);
    if (RELAY_TEST_MODE) {
        ESP_LOGW(TAG, "RELAY TEST MODE: no I2C; 0%% mute and relay steps are SIMULATED");
    } else {
        ESP_ERROR_CHECK(i2c_master_init());
        // Force LOW output latches on both boards immediately, before Matter.
        ESP_ERROR_CHECK(attenuator_init(RELAY_ADDR_0));
        ESP_ERROR_CHECK(attenuator_init(RELAY_ADDR_1));
    }
    // First command in all modes, before the Matter stack is started.
    ESP_ERROR_CHECK(setVolume(MUTE));
    if (!RELAY_TEST_MODE) {
        ESP_LOGI(TAG, "Startup HARD MUTE commanded on both expanders (GPIO=0x00)");
    }
    // Actual hardware mode still fails fast if an expander is unavailable.
    return NULL;
}
