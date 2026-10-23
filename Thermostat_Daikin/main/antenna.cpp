#include "antenna.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <nvs.h>
#include <sdkconfig.h>

static const char *TAG = "antenna";

#define ANT_ENABLE_GPIO GPIO_NUM_3
#define ANT_SELECT_GPIO GPIO_NUM_14
#define ANT_NVS_NAMESPACE "daikin_ac"
#define ANT_NVS_KEY "ant_ext"

static bool s_external = false;

static void apply(bool external)
{
    gpio_set_level(ANT_SELECT_GPIO, external ? 1 : 0);
    s_external = external;
}

void antenna_init(void)
{
    gpio_reset_pin(ANT_ENABLE_GPIO);
    gpio_set_direction(ANT_ENABLE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ANT_ENABLE_GPIO, 0);
    gpio_reset_pin(ANT_SELECT_GPIO);
    gpio_set_direction(ANT_SELECT_GPIO, GPIO_MODE_OUTPUT);

#ifdef CONFIG_DAIKIN_ANTENNA_EXTERNAL_DEFAULT
    bool external = true;
#else
    bool external = false;
#endif

    nvs_handle_t handle;
    if (nvs_open(ANT_NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        uint8_t stored = 0;
        if (nvs_get_u8(handle, ANT_NVS_KEY, &stored) == ESP_OK) {
            external = stored != 0;
        }
        nvs_close(handle);
    }

    apply(external);
    ESP_LOGI(TAG, "RF switch enabled (GPIO3=0); antenna: %s (GPIO14=%d)", external ? "external u.FL" : "onboard ceramic",
             external ? 1 : 0);
}

bool antenna_is_external(void)
{
    return s_external;
}

esp_err_t antenna_set_external(bool external)
{
    apply(external);

    nvs_handle_t handle;
    esp_err_t err = nvs_open(ANT_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, ANT_NVS_KEY, external ? 1 : 0);
        if (err == ESP_OK) {
            err = nvs_commit(handle);
        }
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Antenna set to %s%s", external ? "external u.FL" : "onboard ceramic",
             err == ESP_OK ? "" : " (NOT saved: NVS write failed)");
    return err;
}

void antenna_toggle(void)
{
    antenna_set_external(!s_external);
}
