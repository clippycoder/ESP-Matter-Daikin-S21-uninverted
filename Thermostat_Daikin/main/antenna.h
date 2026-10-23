#pragma once

#include <stdbool.h>
#include <esp_err.h>

/**
 * Antenna selection for the Seeed XIAO ESP32-C6 (onboard ceramic antenna or external u.FL).
 *
 * GPIO3 enables the RF switch (LOW = enabled); GPIO14 selects the antenna (LOW = onboard,
 * HIGH = u.FL). The choice is stored in NVS and applied exactly as stored: there is no
 * automatic fallback, so selecting u.FL with no antenna attached will degrade or drop the
 * Thread link until the choice is changed back.
 */

/** Needs NVS initialised. Call before the radio starts (before esp_matter::start()). */
void antenna_init(void);

bool antenna_is_external(void);

/** Applies the choice immediately and stores it in NVS. */
esp_err_t antenna_set_external(bool external);

/** Switches to the other antenna and stores the choice. */
void antenna_toggle(void);
