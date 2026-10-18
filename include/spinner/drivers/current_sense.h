/**
 * @file
 *
 * Current sense adapter API.
 *
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SPINNER_DRIVERS_CURRENT_SENSE_H_
#define _SPINNER_DRIVERS_CURRENT_SENSE_H_

#include <zephyr/device.h>
#include <zephyr/types.h>

#include <spinner/drivers/currsmp.h>

/**
 * @defgroup spinner_drivers_current_sense Current Sense API
 * @ingroup spinner_drivers
 * @{
 */

/**
 * @brief Regulation callback, called from the current sense thread.
 *
 * @param ctx Callback context.
 * @param curr Filtered phase currents (A).
 */
typedef void (*current_sense_regulation_cb_t)(void *ctx,
					      const struct currsmp_curr *curr);

/** @cond INTERNAL_HIDDEN */

struct current_sense_driver_api {
	void (*configure)(const struct device *dev,
			  current_sense_regulation_cb_t regulation_cb, void *ctx);
	void (*get_currents)(const struct device *dev, struct currsmp_curr *curr);
};

/** @endcond */

/**
 * @brief Configure the regulation callback.
 *
 * @param[in] dev Current sense device.
 * @param[in] regulation_cb Callback called on each sample with filtered currents.
 * @param[in] ctx Callback context.
 */
static inline void current_sense_configure(const struct device *dev,
					   current_sense_regulation_cb_t regulation_cb,
					   void *ctx)
{
	const struct current_sense_driver_api *api = dev->api;

	api->configure(dev, regulation_cb, ctx);
}

/**
 * @brief Get phase currents from the latest raw sample.
 *
 * @param[in] dev Current sense device.
 * @param[out] curr Pointer where phase currents (A) will be stored.
 */
static inline void current_sense_get_currents(const struct device *dev,
					      struct currsmp_curr *curr)
{
	const struct current_sense_driver_api *api = dev->api;

	api->get_currents(dev, curr);
}

/** @} */

#endif /* _SPINNER_DRIVERS_CURRENT_SENSE_H_ */
