/*
 * Copyright © 2006-2017 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "intel_drv.h"


int intel_crtc_compute_min_cdclk(const struct intel_crtc_state *crtc_state)
{
    struct drm_i915_private *dev_priv =
        to_i915(crtc_state->base.crtc->dev);
    int min_cdclk;

        return 0;
}
static void hsw_get_cdclk(struct drm_i915_private *dev_priv,
			  struct intel_cdclk_state *cdclk_state)
{
	uint32_t lcpll = I915_READ(LCPLL_CTL);
	uint32_t freq = lcpll & LCPLL_CLK_FREQ_MASK;

	if (lcpll & LCPLL_CD_SOURCE_FCLK)
		cdclk_state->cdclk = 800000;
	else if (I915_READ(FUSE_STRAP) & HSW_CDCLK_LIMIT)
		cdclk_state->cdclk = 450000;
	else if (freq == LCPLL_CLK_FREQ_450)
		cdclk_state->cdclk = 450000;
	else if (IS_HSW_ULT(dev_priv))
		cdclk_state->cdclk = 337500;
	else
		cdclk_state->cdclk = 540000;
}


/**
 * intel_cdclk_needs_modeset - Determine if two CDCLK states require a modeset on all pipes
 * @a: first CDCLK state
 * @b: second CDCLK state
 *
 * Returns:
 * True if the CDCLK states require pipes to be off during reprogramming, false if not.
 */
bool intel_cdclk_needs_modeset(const struct intel_cdclk_state *a,
			       const struct intel_cdclk_state *b)
{
	return a->cdclk != b->cdclk ||
		a->vco != b->vco ||
		a->ref != b->ref;
}

/**
 * intel_cdclk_changed - Determine if two CDCLK states are different
 * @a: first CDCLK state
 * @b: second CDCLK state
 *
 * Returns:
 * True if the CDCLK states don't match, false if they do.
 */
bool intel_cdclk_changed(const struct intel_cdclk_state *a,
			 const struct intel_cdclk_state *b)
{
	return intel_cdclk_needs_modeset(a, b) ||
		a->voltage_level != b->voltage_level;
}

void intel_dump_cdclk_state(const struct intel_cdclk_state *cdclk_state,
			    const char *context)
{
	DRM_DEBUG_DRIVER("%s %d kHz, VCO %d kHz, ref %d kHz, bypass %d kHz, voltage level %d\n",
			 context, cdclk_state->cdclk, cdclk_state->vco,
			 cdclk_state->ref, cdclk_state->bypass,
			 cdclk_state->voltage_level);
}

/**
 * intel_set_cdclk - Push the CDCLK state to the hardware
 * @dev_priv: i915 device
 * @cdclk_state: new CDCLK state
 *
 * Program the hardware based on the passed in CDCLK state,
 * if necessary.
 */
void intel_set_cdclk(struct drm_i915_private *dev_priv,
		     const struct intel_cdclk_state *cdclk_state)
{
    return;
}

static int intel_compute_max_dotclk(struct drm_i915_private *dev_priv)
{
	int max_cdclk_freq = dev_priv->max_cdclk_freq;
    return max_cdclk_freq;
}

/**
 * intel_update_max_cdclk - Determine the maximum support CDCLK frequency
 * @dev_priv: i915 device
 *
 * Determine the maximum CDCLK frequency the platform supports, and also
 * derive the maximum dot clock frequency the maximum CDCLK frequency
 * allows.
 */
void intel_update_max_cdclk(struct drm_i915_private *dev_priv)
{
    dev_priv->max_cdclk_freq = dev_priv->cdclk.hw.cdclk;

	dev_priv->max_dotclk_freq = intel_compute_max_dotclk(dev_priv);

	DRM_DEBUG_DRIVER("Max CD clock rate: %d kHz\n",
			 dev_priv->max_cdclk_freq);

	DRM_DEBUG_DRIVER("Max dotclock rate: %d kHz\n",
			 dev_priv->max_dotclk_freq);
}

/**
 * intel_update_cdclk - Determine the current CDCLK frequency
 * @dev_priv: i915 device
 *
 * Determine the current CDCLK frequency.
 */
void intel_update_cdclk(struct drm_i915_private *dev_priv)
{
	dev_priv->display.get_cdclk(dev_priv, &dev_priv->cdclk.hw);

}

static int pch_rawclk(struct drm_i915_private *dev_priv)
{
	return (I915_READ(PCH_RAWCLK_FREQ) & RAWCLK_FREQ_MASK) * 1000;
}


/**
 * intel_update_rawclk - Determine the current RAWCLK frequency
 * @dev_priv: i915 device
 *
 * Determine the current RAWCLK frequency. RAWCLK is a fixed
 * frequency clock so this needs to done only once.
 */
void intel_update_rawclk(struct drm_i915_private *dev_priv)
{
    dev_priv->rawclk_freq = pch_rawclk(dev_priv);

	DRM_DEBUG_DRIVER("rawclk rate: %d kHz\n", dev_priv->rawclk_freq);
}

/**
 * intel_init_cdclk_hooks - Initialize CDCLK related modesetting hooks
 * @dev_priv: i915 device
 */
void intel_init_cdclk_hooks(struct drm_i915_private *dev_priv)
{

    dev_priv->display.get_cdclk = hsw_get_cdclk;

}
