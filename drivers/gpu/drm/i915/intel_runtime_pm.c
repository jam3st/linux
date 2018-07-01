/*
 * Copyright © 2012-2014 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Eugeni Dodonov <eugeni.dodonov@intel.com>
 *    Daniel Vetter <daniel.vetter@ffwll.ch>
 *
 */

#include <linux/pm_runtime.h>
#include <linux/vgaarb.h>

#include "i915_drv.h"
#include "intel_drv.h"


static void intel_power_well_enable(struct drm_i915_private *dev_priv,
				    struct i915_power_well *power_well)
{
	DRM_DEBUG_KMS("enabling %s\n", power_well->name);
	power_well->ops->enable(dev_priv, power_well);
	power_well->hw_enabled = true;
}

static void intel_power_well_disable(struct drm_i915_private *dev_priv,
				     struct i915_power_well *power_well)
{
	DRM_DEBUG_KMS("disabling %s\n", power_well->name);
	power_well->hw_enabled = false;
	power_well->ops->disable(dev_priv, power_well);
}

static void intel_power_well_get(struct drm_i915_private *dev_priv,
				 struct i915_power_well *power_well)
{
	if (!power_well->count++)
		intel_power_well_enable(dev_priv, power_well);
}

static void intel_power_well_put(struct drm_i915_private *dev_priv,
				 struct i915_power_well *power_well)
{
	WARN(!power_well->count, "Use count on power well %s is already zero",
	     power_well->name);
    if (!--power_well->count) {
        intel_power_well_disable(dev_priv, power_well);
    }
}

/**
 * __intel_display_power_is_enabled - unlocked check for a power domain
 * @dev_priv: i915 device instance
 * @domain: power domain to check
 *
 * This is the unlocked version of intel_display_power_is_enabled() and should
 * only be used from error capture and recovery code where deadlocks are
 * possible.
 *
 * Returns:
 * True when the power domain is enabled, false otherwise.
 */
bool __intel_display_power_is_enabled(struct drm_i915_private *dev_priv,
				      enum intel_display_power_domain domain)
{
	struct i915_power_well *power_well;
	bool is_enabled;

	if (dev_priv->runtime_pm.suspended)
		return false;

	is_enabled = true;

	for_each_power_domain_well_rev(dev_priv, power_well, BIT_ULL(domain)) {
		if (power_well->always_on)
			continue;

		if (!power_well->hw_enabled) {
			is_enabled = false;
			break;
		}
	}

	return is_enabled;
}

/**
 * intel_display_power_is_enabled - check for a power domain
 * @dev_priv: i915 device instance
 * @domain: power domain to check
 *
 * This function can be used to check the hw power domain state. It is mostly
 * used in hardware state readout functions. Everywhere else code should rely
 * upon explicit power domain reference counting to ensure that the hardware
 * block is powered up before accessing it.
 *
 * Callers must hold the relevant modesetting locks to ensure that concurrent
 * threads can't disable the power well while the caller tries to read a few
 * registers.
 *
 * Returns:
 * True when the power domain is enabled, false otherwise.
 */
bool intel_display_power_is_enabled(struct drm_i915_private *dev_priv,
				    enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains;
	bool ret;

	power_domains = &dev_priv->power_domains;

	mutex_lock(&power_domains->lock);
	ret = __intel_display_power_is_enabled(dev_priv, domain);
	mutex_unlock(&power_domains->lock);

	return ret;
}

/**
 * intel_display_set_init_power - set the initial power domain state
 * @dev_priv: i915 device instance
 * @enable: whether to enable or disable the initial power domain state
 *
 * For simplicity our driver load/unload and system suspend/resume code assumes
 * that all power domains are always enabled. This functions controls the state
 * of this little hack. While the initial power domain state is enabled runtime
 * pm is effectively disabled.
 */
void intel_display_set_init_power(struct drm_i915_private *dev_priv,
				  bool enable)
{
	if (dev_priv->power_domains.init_power_on == enable)
		return;

	if (enable)
		intel_display_power_get(dev_priv, POWER_DOMAIN_INIT);
	else
		intel_display_power_put(dev_priv, POWER_DOMAIN_INIT);

	dev_priv->power_domains.init_power_on = enable;
}



static void hsw_power_well_pre_disable(struct drm_i915_private *dev_priv,
                       u8 irq_pipe_mask)
{

}


static void hsw_wait_for_power_well_enable(struct drm_i915_private *dev_priv,
                       struct i915_power_well *power_well)
{
    enum i915_power_well_id id = power_well->id;

    /* Timeout for PW1:10 us, AUX:not specified, other PWs:20 us. */
    WARN_ON(intel_wait_for_register(dev_priv,
                    HSW_PWR_WELL_CTL_DRIVER(id),
                    HSW_PWR_WELL_CTL_STATE(id),
                    HSW_PWR_WELL_CTL_STATE(id),
                    1));
}

static void hsw_power_well_enable(struct drm_i915_private *dev_priv,
                  struct i915_power_well *power_well)
{
    enum i915_power_well_id id = power_well->id;
    enum skl_power_gate uninitialized_var(pg);
    u32 val;


    val = I915_READ(HSW_PWR_WELL_CTL_DRIVER(id));
    I915_WRITE(HSW_PWR_WELL_CTL_DRIVER(id), val | HSW_PWR_WELL_CTL_REQ(id));
    hsw_wait_for_power_well_enable(dev_priv, power_well);

}

static void hsw_power_well_disable(struct drm_i915_private *dev_priv,
                   struct i915_power_well *power_well)
{
    enum i915_power_well_id id = power_well->id;
    u32 val;

    hsw_power_well_pre_disable(dev_priv, power_well->hsw.irq_pipe_mask);

    val = I915_READ(HSW_PWR_WELL_CTL_DRIVER(id));
    I915_WRITE(HSW_PWR_WELL_CTL_DRIVER(id),
           val & ~HSW_PWR_WELL_CTL_REQ(id));
}



static void i9xx_always_on_power_well_noop(struct drm_i915_private *dev_priv,
					   struct i915_power_well *power_well)
{
}

static void
__intel_display_power_get_domain(struct drm_i915_private *dev_priv,
				 enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains = &dev_priv->power_domains;
	struct i915_power_well *power_well;

	for_each_power_domain_well(dev_priv, power_well, BIT_ULL(domain))
		intel_power_well_get(dev_priv, power_well);

	power_domains->domain_use_count[domain]++;
}

/**
 * intel_display_power_get - grab a power domain reference
 * @dev_priv: i915 device instance
 * @domain: power domain to reference
 *
 * This function grabs a power domain reference for @domain and ensures that the
 * power domain and all its parents are powered up. Therefore users should only
 * grab a reference to the innermost power domain they need.
 *
 * Any power domain reference obtained by this function must have a symmetric
 * call to intel_display_power_put() to release the reference again.
 */
void intel_display_power_get(struct drm_i915_private *dev_priv,
			     enum intel_display_power_domain domain)
{
	__intel_display_power_get_domain(dev_priv, domain);
}

/**
 * intel_display_power_get_if_enabled - grab a reference for an enabled display power domain
 * @dev_priv: i915 device instance
 * @domain: power domain to reference
 *
 * This function grabs a power domain reference for @domain and ensures that the
 * power domain and all its parents are powered up. Therefore users should only
 * grab a reference to the innermost power domain they need.
 *
 * Any power domain reference obtained by this function must have a symmetric
 * call to intel_display_power_put() to release the reference again.
 */
bool intel_display_power_get_if_enabled(struct drm_i915_private *dev_priv,
					enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains = &dev_priv->power_domains;
	bool is_enabled;


	mutex_lock(&power_domains->lock);

	if (__intel_display_power_is_enabled(dev_priv, domain)) {
		__intel_display_power_get_domain(dev_priv, domain);
		is_enabled = true;
	} else {
		is_enabled = false;
	}

	mutex_unlock(&power_domains->lock);


	return is_enabled;
}

/**
 * intel_display_power_put - release a power domain reference
 * @dev_priv: i915 device instance
 * @domain: power domain to reference
 *
 * This function drops the power domain reference obtained by
 * intel_display_power_get() and might power down the corresponding hardware
 * block right away if this is the last reference.
 */
void intel_display_power_put(struct drm_i915_private *dev_priv,
			     enum intel_display_power_domain domain)
{
	struct i915_power_domains *power_domains;
	struct i915_power_well *power_well;

	power_domains = &dev_priv->power_domains;

	mutex_lock(&power_domains->lock);


	power_domains->domain_use_count[domain]--;

	for_each_power_domain_well_rev(dev_priv, power_well, BIT_ULL(domain))
		intel_power_well_put(dev_priv, power_well);

	mutex_unlock(&power_domains->lock);

}

#define I830_PIPES_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PIPE_A) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_A_PANEL_FITTER) |	\
	BIT_ULL(POWER_DOMAIN_PIPE_B_PANEL_FITTER) |	\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_A) |	\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_B) |	\
	BIT_ULL(POWER_DOMAIN_INIT))

#define VLV_DISPLAY_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PIPE_A) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_A_PANEL_FITTER) |	\
	BIT_ULL(POWER_DOMAIN_PIPE_B_PANEL_FITTER) |	\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_A) |	\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_B) |	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |	\
	BIT_ULL(POWER_DOMAIN_PORT_DSI) |		\
	BIT_ULL(POWER_DOMAIN_PORT_CRT) |		\
	BIT_ULL(POWER_DOMAIN_VGA) |			\
	BIT_ULL(POWER_DOMAIN_AUDIO) |		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |		\
	BIT_ULL(POWER_DOMAIN_AUX_C) |		\
	BIT_ULL(POWER_DOMAIN_GMBUS) |		\
	BIT_ULL(POWER_DOMAIN_INIT))

#define VLV_DPIO_CMN_BC_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |	\
	BIT_ULL(POWER_DOMAIN_PORT_CRT) |		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |		\
	BIT_ULL(POWER_DOMAIN_AUX_C) |		\
	BIT_ULL(POWER_DOMAIN_INIT))

#define VLV_DPIO_TX_B_LANES_01_POWER_DOMAINS (	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |	\
	BIT_ULL(POWER_DOMAIN_AUX_B) |		\
	BIT_ULL(POWER_DOMAIN_INIT))

#define VLV_DPIO_TX_B_LANES_23_POWER_DOMAINS (	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |	\
	BIT_ULL(POWER_DOMAIN_AUX_B) |		\
	BIT_ULL(POWER_DOMAIN_INIT))

#define VLV_DPIO_TX_C_LANES_01_POWER_DOMAINS (	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |	\
	BIT_ULL(POWER_DOMAIN_AUX_C) |		\
	BIT_ULL(POWER_DOMAIN_INIT))

#define VLV_DPIO_TX_C_LANES_23_POWER_DOMAINS (	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |	\
	BIT_ULL(POWER_DOMAIN_AUX_C) |		\
	BIT_ULL(POWER_DOMAIN_INIT))

#define CHV_DISPLAY_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PIPE_A) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_A_PANEL_FITTER) |	\
	BIT_ULL(POWER_DOMAIN_PIPE_B_PANEL_FITTER) |	\
	BIT_ULL(POWER_DOMAIN_PIPE_C_PANEL_FITTER) |	\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_A) |	\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_B) |	\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_C) |	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_D_LANES) |	\
	BIT_ULL(POWER_DOMAIN_PORT_DSI) |		\
	BIT_ULL(POWER_DOMAIN_VGA) |			\
	BIT_ULL(POWER_DOMAIN_AUDIO) |		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |		\
	BIT_ULL(POWER_DOMAIN_AUX_C) |		\
	BIT_ULL(POWER_DOMAIN_AUX_D) |		\
	BIT_ULL(POWER_DOMAIN_GMBUS) |		\
	BIT_ULL(POWER_DOMAIN_INIT))

#define CHV_DPIO_CMN_BC_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |	\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |	\
	BIT_ULL(POWER_DOMAIN_AUX_B) |		\
	BIT_ULL(POWER_DOMAIN_AUX_C) |		\
	BIT_ULL(POWER_DOMAIN_INIT))

#define CHV_DPIO_CMN_D_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_D_LANES) |	\
	BIT_ULL(POWER_DOMAIN_AUX_D) |		\
	BIT_ULL(POWER_DOMAIN_INIT))

#define HSW_DISPLAY_POWER_DOMAINS (			\
	BIT_ULL(POWER_DOMAIN_PIPE_B) |			\
	BIT_ULL(POWER_DOMAIN_PIPE_C) |			\
	BIT_ULL(POWER_DOMAIN_PIPE_A_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_A) |		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_B) |		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_C) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_D_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_CRT) | /* DDI E */	\
	BIT_ULL(POWER_DOMAIN_VGA) |				\
	BIT_ULL(POWER_DOMAIN_AUDIO) |			\
	BIT_ULL(POWER_DOMAIN_INIT))

#define BDW_DISPLAY_POWER_DOMAINS (			\
	BIT_ULL(POWER_DOMAIN_PIPE_B) |			\
	BIT_ULL(POWER_DOMAIN_PIPE_C) |			\
	BIT_ULL(POWER_DOMAIN_PIPE_B_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_A) |		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_B) |		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_C) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_D_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_CRT) | /* DDI E */	\
	BIT_ULL(POWER_DOMAIN_VGA) |				\
	BIT_ULL(POWER_DOMAIN_AUDIO) |			\
	BIT_ULL(POWER_DOMAIN_INIT))

#define SKL_DISPLAY_POWERWELL_2_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_A) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B) |			\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_B) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C) |			\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_C) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_D_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_E_LANES) |		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |                       \
	BIT_ULL(POWER_DOMAIN_AUX_C) |			\
	BIT_ULL(POWER_DOMAIN_AUX_D) |			\
	BIT_ULL(POWER_DOMAIN_AUDIO) |			\
	BIT_ULL(POWER_DOMAIN_VGA) |				\
	BIT_ULL(POWER_DOMAIN_INIT))
#define SKL_DISPLAY_DDI_IO_A_E_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_A_IO) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_E_IO) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define SKL_DISPLAY_DDI_IO_B_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_IO) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define SKL_DISPLAY_DDI_IO_C_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_IO) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define SKL_DISPLAY_DDI_IO_D_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_D_IO) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define SKL_DISPLAY_DC_OFF_POWER_DOMAINS (		\
	SKL_DISPLAY_POWERWELL_2_POWER_DOMAINS |		\
	BIT_ULL(POWER_DOMAIN_GT_IRQ) |			\
	BIT_ULL(POWER_DOMAIN_MODESET) |			\
	BIT_ULL(POWER_DOMAIN_AUX_A) |			\
	BIT_ULL(POWER_DOMAIN_INIT))

#define BXT_DISPLAY_POWERWELL_2_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_A) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B) |			\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_B) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C) |			\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_C) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |			\
	BIT_ULL(POWER_DOMAIN_AUX_C) |			\
	BIT_ULL(POWER_DOMAIN_AUDIO) |			\
	BIT_ULL(POWER_DOMAIN_VGA) |				\
	BIT_ULL(POWER_DOMAIN_INIT))
#define BXT_DISPLAY_DC_OFF_POWER_DOMAINS (		\
	BXT_DISPLAY_POWERWELL_2_POWER_DOMAINS |		\
	BIT_ULL(POWER_DOMAIN_GT_IRQ) |			\
	BIT_ULL(POWER_DOMAIN_MODESET) |			\
	BIT_ULL(POWER_DOMAIN_AUX_A) |			\
	BIT_ULL(POWER_DOMAIN_GMBUS) |			\
	BIT_ULL(POWER_DOMAIN_INIT))
#define BXT_DPIO_CMN_A_POWER_DOMAINS (			\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_A_LANES) |		\
	BIT_ULL(POWER_DOMAIN_AUX_A) |			\
	BIT_ULL(POWER_DOMAIN_INIT))
#define BXT_DPIO_CMN_BC_POWER_DOMAINS (			\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |			\
	BIT_ULL(POWER_DOMAIN_AUX_C) |			\
	BIT_ULL(POWER_DOMAIN_INIT))

#define GLK_DISPLAY_POWERWELL_2_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_A) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B) |			\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_B) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C) |			\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_C) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |                       \
	BIT_ULL(POWER_DOMAIN_AUX_C) |			\
	BIT_ULL(POWER_DOMAIN_AUDIO) |			\
	BIT_ULL(POWER_DOMAIN_VGA) |				\
	BIT_ULL(POWER_DOMAIN_INIT))
#define GLK_DISPLAY_DDI_IO_A_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_A_IO))
#define GLK_DISPLAY_DDI_IO_B_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_IO))
#define GLK_DISPLAY_DDI_IO_C_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_IO))
#define GLK_DPIO_CMN_A_POWER_DOMAINS (			\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_A_LANES) |		\
	BIT_ULL(POWER_DOMAIN_AUX_A) |			\
	BIT_ULL(POWER_DOMAIN_INIT))
#define GLK_DPIO_CMN_B_POWER_DOMAINS (			\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |			\
	BIT_ULL(POWER_DOMAIN_INIT))
#define GLK_DPIO_CMN_C_POWER_DOMAINS (			\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |		\
	BIT_ULL(POWER_DOMAIN_AUX_C) |			\
	BIT_ULL(POWER_DOMAIN_INIT))
#define GLK_DISPLAY_AUX_A_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_AUX_A) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define GLK_DISPLAY_AUX_B_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define GLK_DISPLAY_AUX_C_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_AUX_C) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define GLK_DISPLAY_DC_OFF_POWER_DOMAINS (		\
	GLK_DISPLAY_POWERWELL_2_POWER_DOMAINS |		\
	BIT_ULL(POWER_DOMAIN_GT_IRQ) |			\
	BIT_ULL(POWER_DOMAIN_MODESET) |			\
	BIT_ULL(POWER_DOMAIN_AUX_A) |			\
	BIT_ULL(POWER_DOMAIN_GMBUS) |			\
	BIT_ULL(POWER_DOMAIN_INIT))

#define CNL_DISPLAY_POWERWELL_2_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_A) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B) |			\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_B) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C) |			\
	BIT_ULL(POWER_DOMAIN_TRANSCODER_C) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_B_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PIPE_C_PANEL_FITTER) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_D_LANES) |		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_F_LANES) |		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |                       \
	BIT_ULL(POWER_DOMAIN_AUX_C) |			\
	BIT_ULL(POWER_DOMAIN_AUX_D) |			\
	BIT_ULL(POWER_DOMAIN_AUX_F) |			\
	BIT_ULL(POWER_DOMAIN_AUDIO) |			\
	BIT_ULL(POWER_DOMAIN_VGA) |				\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_DDI_A_IO_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_A_IO) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_DDI_B_IO_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_B_IO) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_DDI_C_IO_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_C_IO) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_DDI_D_IO_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_D_IO) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_AUX_A_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_AUX_A) |			\
	BIT_ULL(POWER_DOMAIN_AUX_IO_A) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_AUX_B_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_AUX_B) |			\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_AUX_C_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_AUX_C) |			\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_AUX_D_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_AUX_D) |			\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_AUX_F_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_AUX_F) |			\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_DDI_F_IO_POWER_DOMAINS (		\
	BIT_ULL(POWER_DOMAIN_PORT_DDI_F_IO) |		\
	BIT_ULL(POWER_DOMAIN_INIT))
#define CNL_DISPLAY_DC_OFF_POWER_DOMAINS (		\
	CNL_DISPLAY_POWERWELL_2_POWER_DOMAINS |		\
	BIT_ULL(POWER_DOMAIN_GT_IRQ) |			\
	BIT_ULL(POWER_DOMAIN_MODESET) |			\
	BIT_ULL(POWER_DOMAIN_AUX_A) |			\
	BIT_ULL(POWER_DOMAIN_INIT))

#define POWER_DOMAIN_MASK (GENMASK_ULL(POWER_DOMAIN_NUM - 1, 0))

static const struct i915_power_well_ops i9xx_always_on_power_well_ops = {
    .sync_hw = NULL,
	.enable = i9xx_always_on_power_well_noop,
	.disable = i9xx_always_on_power_well_noop,
    .is_enabled = NULL,
};


static const struct i915_power_well_ops chv_dpio_cmn_power_well_ops = {
    .sync_hw = NULL,
    .enable = NULL,
    .disable = NULL,
    .is_enabled = NULL,
};


static const struct i915_power_well_ops i830_pipes_power_well_ops = {
    .sync_hw = NULL,
    .enable = NULL,
    .disable = NULL,
    .is_enabled = NULL,
};



static const struct i915_power_well_ops hsw_power_well_ops = {
    .sync_hw = NULL,
	.enable = hsw_power_well_enable,
	.disable = hsw_power_well_disable,
    .is_enabled = NULL,
};

static const struct i915_power_well_ops gen9_dc_off_power_well_ops = {
    .sync_hw = NULL,
    .enable = NULL,
    .disable = NULL,
    .is_enabled = NULL
};

static const struct i915_power_well_ops bxt_dpio_cmn_power_well_ops = {
    .sync_hw = NULL,
    .enable = NULL,
    .disable = NULL,
    .is_enabled = NULL,
};

static struct i915_power_well hsw_power_wells[] = {
	{
		.name = "always-on",
		.always_on = 1,
		.domains = POWER_DOMAIN_MASK,
		.ops = &i9xx_always_on_power_well_ops,
		.id = I915_DISP_PW_ALWAYS_ON,
	},
	{
		.name = "display",
		.domains = HSW_DISPLAY_POWER_DOMAINS,
		.ops = &hsw_power_well_ops,
		.id = HSW_DISP_PW_GLOBAL,
		{
			.hsw.has_vga = true,
		},
	},
};




static int
sanitize_disable_power_well_option(const struct drm_i915_private *dev_priv,
				   int disable_power_well)
{
	if (disable_power_well >= 0)
		return !!disable_power_well;

	return 1;
}

static uint32_t get_allowed_dc_mask(const struct drm_i915_private *dev_priv,
				    int enable_dc)
{
	uint32_t mask;
	int requested_dc;
	int max_dc;

	if (IS_GEN9_BC(dev_priv) || IS_CANNONLAKE(dev_priv)) {
		max_dc = 2;
		mask = 0;
	} else if (IS_GEN9_LP(dev_priv)) {
		max_dc = 1;
		/*
		 * DC9 has a separate HW flow from the rest of the DC states,
		 * not depending on the DMC firmware. It's needed by system
		 * suspend/resume, so allow it unconditionally.
		 */
		mask = DC_STATE_EN_DC9;
	} else {
		max_dc = 0;
		mask = 0;
	}

	if (!i915_modparams.disable_power_well)
		max_dc = 0;

	if (enable_dc >= 0 && enable_dc <= max_dc) {
		requested_dc = enable_dc;
	} else if (enable_dc == -1) {
		requested_dc = max_dc;
	} else if (enable_dc > max_dc && enable_dc <= 2) {
		DRM_DEBUG_KMS("Adjusting requested max DC state (%d->%d)\n",
			      enable_dc, max_dc);
		requested_dc = max_dc;
	} else {
		DRM_ERROR("Unexpected value for enable_dc (%d)\n", enable_dc);
		requested_dc = max_dc;
	}

	if (requested_dc > 1)
		mask |= DC_STATE_EN_UPTO_DC6;
	if (requested_dc > 0)
		mask |= DC_STATE_EN_UPTO_DC5;

	DRM_DEBUG_KMS("Allowed DC state mask %02x\n", mask);

	return mask;
}

static void assert_power_well_ids_unique(struct drm_i915_private *dev_priv)
{
	struct i915_power_domains *power_domains = &dev_priv->power_domains;
	u64 power_well_ids;
	int i;

	power_well_ids = 0;
	for (i = 0; i < power_domains->power_well_count; i++) {
		enum i915_power_well_id id = power_domains->power_wells[i].id;

		WARN_ON(id >= sizeof(power_well_ids) * 8);
		WARN_ON(power_well_ids & BIT_ULL(id));
		power_well_ids |= BIT_ULL(id);
	}
}

#define set_power_wells(power_domains, __power_wells) ({		\
	(power_domains)->power_wells = (__power_wells);			\
	(power_domains)->power_well_count = ARRAY_SIZE(__power_wells);	\
})

/**
 * intel_power_domains_init - initializes the power domain structures
 * @dev_priv: i915 device instance
 *
 * Initializes the power domain structures for @dev_priv depending upon the
 * supported platform.
 */
int intel_power_domains_init(struct drm_i915_private *dev_priv)
{
	struct i915_power_domains *power_domains = &dev_priv->power_domains;

	i915_modparams.disable_power_well =
		sanitize_disable_power_well_option(dev_priv,
						   i915_modparams.disable_power_well);
	dev_priv->csr.allowed_dc_mask =
		get_allowed_dc_mask(dev_priv, i915_modparams.enable_dc);

	BUILD_BUG_ON(POWER_DOMAIN_NUM > 64);

	mutex_init(&power_domains->lock);

    set_power_wells(power_domains, hsw_power_wells);

	assert_power_well_ids_unique(dev_priv);

	return 0;
}




