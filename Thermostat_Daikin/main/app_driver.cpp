/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <app_priv.h>
#include <common_macros.h>
#include <app/reporting/reporting.h> 
#include <platform/CHIPDeviceLayer.h>

#include "iot_button.h"
#include "button_gpio.h"
#include <app/server/Server.h>
#include <app/server/CommissioningWindowManager.h>

#include "s21_driver.h"
#include "antenna.h"

using namespace chip::app::Clusters;
using namespace chip::app::Clusters::Thermostat;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t ac_endpoint_id;

// FanControl WindSetting/WindSupport bit 1 = NaturalWind, used as the Daikin Powerful flag.
#define WIND_NATURAL_BIT 0x02
static DaikinS21 s21;

static uint8_t s21_fan_to_matter(uint8_t s21_fan)
{
    switch (s21_fan) {
        case FAIKIN_FAN_AUTO:  return 5; // Auto
        case FAIKIN_FAN_QUIET:
        case FAIKIN_FAN_1:
        case FAIKIN_FAN_2:     return 1; // Low
        case FAIKIN_FAN_3:     return 2; // Medium
        case FAIKIN_FAN_4:
        case FAIKIN_FAN_5:     return 3; // High
        default:               return 5;
    }
}

static uint8_t matter_fan_to_s21(uint8_t matter_fan)
{
    switch (matter_fan) {
        case 1: return FAIKIN_FAN_1; // Low
        case 2: return FAIKIN_FAN_3; // Medium
        case 3: return FAIKIN_FAN_5; // High
        default: return FAIKIN_FAN_AUTO; // Off / On / Auto / Smart
    }
}

static uint8_t s21_fan_to_percent(uint8_t s21_fan)
{
    switch (s21_fan) {
        case FAIKIN_FAN_QUIET:
        case FAIKIN_FAN_1: return 20;
        case FAIKIN_FAN_2: return 40;
        case FAIKIN_FAN_3: return 60;
        case FAIKIN_FAN_4: return 80;
        case FAIKIN_FAN_5: return 100;
        default:           return 0;
    }
}

static uint8_t percent_to_s21_fan(uint8_t percent)
{
    if (percent == 0)  return FAIKIN_FAN_AUTO;
    if (percent <= 20) return FAIKIN_FAN_1;
    if (percent <= 40) return FAIKIN_FAN_2;
    if (percent <= 60) return FAIKIN_FAN_3;
    if (percent <= 80) return FAIKIN_FAN_4;
    return FAIKIN_FAN_5;
}

// Global Temperature Storage
int16_t g_current_temp_int = 2100; 

#define S21_TX_PIN 16
#define S21_RX_PIN 17
#define BUTTON_GPIO_PIN 23

// 18 C is the S21 encoding base (AC_MIN_TEMP_VALUE '@'); bytes below it are untested on these units.
#define S21_TARGET_MIN_C 18.0f
#define S21_TARGET_MAX_C 32.0f

#define FLOAT_TO_MATTER(x) ((int16_t)((x) * 100.0f))
#define MATTER_TO_FLOAT(x) ((float)(x) / 100.0f)

static void s21_poll_task(void *pvParameters)
{
    ESP_LOGI(TAG, "S21 Poll Task Started");
    while (1) {
        s21.Poll();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

struct AppEventData {
    ac_state_t state;
};

static void AppDriverUpdateTask(intptr_t context)
{
    AppEventData *data = (AppEventData *)context;
    if (!data) return;

    // Local temperature (Thermostat cluster, served via LocalTempAccessor).
    int16_t new_temp = FLOAT_TO_MATTER(data->state.current_temp);
    if (g_current_temp_int != new_temp) {
        g_current_temp_int = new_temp;
        MatterReportingAttributeChangeCallback(
            ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::LocalTemperature::Id);
    }

    // Mirror the same reading on the TemperatureMeasurement cluster (used by the AC tile).
    esp_matter_attr_val_t val = esp_matter_nullable_int16(new_temp);
    esp_matter::attribute::report(ac_endpoint_id, TemperatureMeasurement::Id, TemperatureMeasurement::Attributes::MeasuredValue::Id, &val);

    // Target temperature.
    val = esp_matter_int16(FLOAT_TO_MATTER(data->state.target_temp));
    if (data->state.mode == FAIKIN_MODE_HEAT) {
        esp_matter::attribute::report(ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::OccupiedHeatingSetpoint::Id, &val);
    } else {
        esp_matter::attribute::report(ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::OccupiedCoolingSetpoint::Id, &val);
    }

    // OnOff cluster — explicit power state for the Room AC tile.
    val = esp_matter_bool(data->state.power);
    esp_matter::attribute::report(ac_endpoint_id, OnOff::Id, OnOff::Attributes::OnOff::Id, &val);

    // Thermostat SystemMode — only meaningful while powered; report current mode otherwise.
    uint8_t matter_mode = 0; // Off
    if (data->state.power) {
        switch (data->state.mode) {
            case FAIKIN_MODE_AUTO: matter_mode = 1; break;
            case FAIKIN_MODE_COOL: matter_mode = 3; break;
            case FAIKIN_MODE_HEAT: matter_mode = 4; break;
            default: matter_mode = 1; break;
        }
    }
    val = esp_matter_enum8(matter_mode);
    esp_matter::attribute::report(ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::SystemMode::Id, &val);

    // Running state (0=Idle, bit0=Heat, bit1=Cool).
    uint16_t running_state = 0;
    if (data->state.power) {
        if (data->state.mode == FAIKIN_MODE_HEAT && data->state.current_temp < data->state.target_temp) {
            running_state = 1;
        } else if (data->state.mode == FAIKIN_MODE_COOL && data->state.current_temp > data->state.target_temp) {
            running_state = 2;
        }
    }
    val = esp_matter_bitmap16(running_state);
    esp_matter::attribute::report(ac_endpoint_id, Thermostat::Id, Thermostat::Attributes::ThermostatRunningState::Id, &val);

    // Fan Control — mirror Daikin fan speed into Matter.
    val = esp_matter_enum8(s21_fan_to_matter(data->state.fan_speed));
    esp_matter::attribute::report(ac_endpoint_id, FanControl::Id, FanControl::Attributes::FanMode::Id, &val);

    uint8_t pct = s21_fan_to_percent(data->state.fan_speed);
    val = esp_matter_nullable_uint8(pct);
    esp_matter::attribute::report(ac_endpoint_id, FanControl::Id, FanControl::Attributes::PercentSetting::Id, &val);
    val = esp_matter_uint8(pct);
    esp_matter::attribute::report(ac_endpoint_id, FanControl::Id, FanControl::Attributes::PercentCurrent::Id, &val);

    val = esp_matter_bitmap8(data->state.powerful ? WIND_NATURAL_BIT : 0);
    esp_matter::attribute::report(ac_endpoint_id, FanControl::Id, FanControl::Attributes::WindSetting::Id, &val);

    free(data);
}

static void s21_state_change_callback(const ac_state_t *state)
{
    if (ac_endpoint_id == 0) return;
    AppEventData *data = (AppEventData *)malloc(sizeof(AppEventData));
    if (data) {
        data->state = *state;
        chip::DeviceLayer::PlatformMgr().ScheduleWork(AppDriverUpdateTask, (intptr_t)data);
    }
}

// OnOff and SystemMode describe one state: SystemMode is Off exactly when the unit is off, and turning
// the unit on resumes the last mode. A write to one attribute only stores that attribute, so after
// every power or mode write, report both from the commanded state (runs after the write completes).
static void report_commanded_state()
{
    ac_state_t state = s21.GetState();
    s21_state_change_callback(&state);
}

static esp_err_t app_driver_thermostat_set_value(esp_matter_attr_val_t *val, uint32_t attribute_id)
{
    ac_state_t current = s21.GetState();

    if (attribute_id == Thermostat::Attributes::SystemMode::Id) {
        uint8_t mode = val->val.u8;
        if (mode == 0) {
            s21.SetPower(false);
        } else {
            if (!current.power) {
                s21.SetPower(true);
            }
            if (mode == 1) s21.SetMode(FAIKIN_MODE_AUTO);
            else if (mode == 3) s21.SetMode(FAIKIN_MODE_COOL);
            else if (mode == 4) s21.SetMode(FAIKIN_MODE_HEAT);
        }
        report_commanded_state();
    }
    else if (attribute_id == Thermostat::Attributes::OccupiedCoolingSetpoint::Id ||
             attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id) {
        // The unit has one target temperature; only the setpoint for the active mode may drive it,
        // otherwise a controller writing both (e.g. heat 13 C + cool 18 C while cooling) sets the wrong one.
        bool is_heating_sp = attribute_id == Thermostat::Attributes::OccupiedHeatingSetpoint::Id;
        bool mode_is_heat = current.mode == FAIKIN_MODE_HEAT;
        bool applies = current.mode == FAIKIN_MODE_AUTO || is_heating_sp == mode_is_heat;
        if (!current.power) {
            ESP_LOGI(TAG, "Ignored SetTemp because device is OFF");
        } else if (!applies) {
            ESP_LOGI(TAG, "Ignored %s setpoint in mode %u", is_heating_sp ? "heating" : "cooling", current.mode);
        } else {
            float t = MATTER_TO_FLOAT(val->val.i16);
            if (t < S21_TARGET_MIN_C) t = S21_TARGET_MIN_C;
            if (t > S21_TARGET_MAX_C) t = S21_TARGET_MAX_C;
            s21.SetTemp(t);
        }
    }
    return ESP_OK;
}

static esp_err_t app_driver_on_off_set_value(esp_matter_attr_val_t *val, uint32_t attribute_id)
{
    if (attribute_id == OnOff::Attributes::OnOff::Id) {
        ESP_LOGI(TAG, "Setting Power to: %d", val->val.b);
        s21.SetPower(val->val.b);
        report_commanded_state();
    }
    return ESP_OK;
}

static esp_err_t app_driver_fan_set_value(esp_matter_attr_val_t *val, uint32_t attribute_id)
{
    if (attribute_id == FanControl::Attributes::FanMode::Id) {
        uint8_t s21_fan = matter_fan_to_s21(val->val.u8);
        ESP_LOGI(TAG, "FanMode write %u → s21 %u", val->val.u8, s21_fan);
        s21.SetFan(s21_fan);
    }
    else if (attribute_id == FanControl::Attributes::PercentSetting::Id) {
        // Nullable uint8: when null, controller wants Auto.
        uint8_t s21_fan = (val->type == ESP_MATTER_VAL_TYPE_NULLABLE_UINT8 && val->val.u8 == 0xFF)
                          ? FAIKIN_FAN_AUTO
                          : percent_to_s21_fan(val->val.u8);
        ESP_LOGI(TAG, "PercentSetting write %u → s21 %u", val->val.u8, s21_fan);
        s21.SetFan(s21_fan);
    }
    else if (attribute_id == FanControl::Attributes::WindSetting::Id) {
        bool powerful = (val->val.u8 & WIND_NATURAL_BIT) != 0;
        ESP_LOGI(TAG, "WindSetting write 0x%02x → powerful %d", val->val.u8, powerful);
        s21.SetPowerful(powerful);
    }
    return ESP_OK;
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    if (endpoint_id != ac_endpoint_id) {
        return ESP_OK;
    }
    if (cluster_id == Thermostat::Id) {
        return app_driver_thermostat_set_value(val, attribute_id);
    }
    if (cluster_id == OnOff::Id) {
        return app_driver_on_off_set_value(val, attribute_id);
    }
    if (cluster_id == FanControl::Id) {
        return app_driver_fan_set_value(val, attribute_id);
    }
    return ESP_OK;
}

esp_err_t app_driver_thermostat_set_defaults(uint16_t endpoint_id) { return ESP_OK; }

app_driver_handle_t app_driver_thermostat_init()
{
    s21.Init(S21_TX_PIN, S21_RX_PIN);
    s21.SetStateCallback(s21_state_change_callback);
    xTaskCreate(s21_poll_task, "s21_poll", 4096, NULL, 5, NULL);
    return (app_driver_handle_t)1;
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button Pressed: Opening Commissioning Window");
    chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (!commissionMgr.IsCommissioningWindowOpen()) {
        commissionMgr.OpenBasicCommissioningWindow(chip::System::Clock::Seconds16(300),
                                                   chip::CommissioningWindowAdvertisement::kDnssdOnly);
    }
}

static void app_driver_button_antenna_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button: 5 clicks, switching antenna");
    antenna_toggle();
}

app_driver_handle_t app_driver_button_init()
{
    button_config_t btn_cfg = {0};
    button_gpio_config_t btn_gpio_cfg = { .gpio_num = BUTTON_GPIO_PIN, .active_level = 0 };
    button_handle_t btn_handle = NULL;
    esp_err_t err = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &btn_handle);
    if (err == ESP_OK && btn_handle) {
        iot_button_register_cb(btn_handle, BUTTON_PRESS_DOWN, NULL, app_driver_button_toggle_cb, NULL);
        button_event_args_t antenna_args = {};
        antenna_args.multiple_clicks.clicks = 5;
        iot_button_register_cb(btn_handle, BUTTON_MULTIPLE_CLICK, &antenna_args, app_driver_button_antenna_cb, NULL);
        return (app_driver_handle_t)btn_handle;
    }
    return NULL;
}