  /*
 * Copyright © 2012 Intel Corporation
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
 *
 */

#include <linux/cpufreq.h>
#include <drm/drm_plane_helper.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "../../../platform/x86/intel_ips.h"
#include <linux/module.h>
#include <drm/drm_atomic_helper.h>

void intel_init_clock_gating(struct drm_i915_private *dev_priv)
{
    dev_priv->display.init_clock_gating(dev_priv);
}


/* Pineview has different values for various configs */
static const struct intel_watermark_params pineview_display_wm = {
	.fifo_size = PINEVIEW_DISPLAY_FIFO,
	.max_wm = PINEVIEW_MAX_WM,
	.default_wm = PINEVIEW_DFT_WM,
	.guard_size = PINEVIEW_GUARD_WM,
	.cacheline_size = PINEVIEW_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params pineview_display_hplloff_wm = {
	.fifo_size = PINEVIEW_DISPLAY_FIFO,
	.max_wm = PINEVIEW_MAX_WM,
	.default_wm = PINEVIEW_DFT_HPLLOFF_WM,
	.guard_size = PINEVIEW_GUARD_WM,
	.cacheline_size = PINEVIEW_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params pineview_cursor_wm = {
	.fifo_size = PINEVIEW_CURSOR_FIFO,
	.max_wm = PINEVIEW_CURSOR_MAX_WM,
	.default_wm = PINEVIEW_CURSOR_DFT_WM,
	.guard_size = PINEVIEW_CURSOR_GUARD_WM,
	.cacheline_size = PINEVIEW_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params pineview_cursor_hplloff_wm = {
	.fifo_size = PINEVIEW_CURSOR_FIFO,
	.max_wm = PINEVIEW_CURSOR_MAX_WM,
	.default_wm = PINEVIEW_CURSOR_DFT_WM,
	.guard_size = PINEVIEW_CURSOR_GUARD_WM,
	.cacheline_size = PINEVIEW_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params i965_cursor_wm_info = {
	.fifo_size = I965_CURSOR_FIFO,
	.max_wm = I965_CURSOR_MAX_WM,
	.default_wm = I965_CURSOR_DFT_WM,
	.guard_size = 2,
	.cacheline_size = I915_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params i945_wm_info = {
	.fifo_size = I945_FIFO_SIZE,
	.max_wm = I915_MAX_WM,
	.default_wm = 1,
	.guard_size = 2,
	.cacheline_size = I915_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params i915_wm_info = {
	.fifo_size = I915_FIFO_SIZE,
	.max_wm = I915_MAX_WM,
	.default_wm = 1,
	.guard_size = 2,
	.cacheline_size = I915_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params i830_a_wm_info = {
	.fifo_size = I855GM_FIFO_SIZE,
	.max_wm = I915_MAX_WM,
	.default_wm = 1,
	.guard_size = 2,
	.cacheline_size = I830_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params i830_bc_wm_info = {
	.fifo_size = I855GM_FIFO_SIZE,
	.max_wm = I915_MAX_WM/2,
	.default_wm = 1,
	.guard_size = 2,
	.cacheline_size = I830_FIFO_LINE_SIZE,
};
static const struct intel_watermark_params i845_wm_info = {
	.fifo_size = I830_FIFO_SIZE,
	.max_wm = I915_MAX_WM,
	.default_wm = 1,
	.guard_size = 2,
	.cacheline_size = I830_FIFO_LINE_SIZE,
};

/**
 * intel_wm_method1 - Method 1 / "small buffer" watermark formula
 * @pixel_rate: Pipe pixel rate in kHz
 * @cpp: Plane bytes per pixel
 * @latency: Memory wakeup latency in 0.1us units
 *
 * Compute the watermark using the method 1 or "small buffer"
 * formula. The caller may additonally add extra cachelines
 * to account for TLB misses and clock crossings.
 *
 * This method is concerned with the short term drain rate
 * of the FIFO, ie. it does not account for blanking periods
 * which would effectively reduce the average drain rate across
 * a longer period. The name "small" refers to the fact the
 * FIFO is relatively small compared to the amount of data
 * fetched.
 *
 * The FIFO level vs. time graph might look something like:
 *
 *   |\   |\
 *   | \  | \
 * __---__---__ (- plane active, _ blanking)
 * -> time
 *
 * or perhaps like this:
 *
 *   |\|\  |\|\
 * __----__----__ (- plane active, _ blanking)
 * -> time
 *
 * Returns:
 * The watermark in bytes
 */
static unsigned int intel_wm_method1(unsigned int pixel_rate,
				     unsigned int cpp,
				     unsigned int latency)
{
	uint64_t ret;

	ret = (uint64_t) pixel_rate * cpp * latency;
	ret = DIV_ROUND_UP_ULL(ret, 10000);

	return ret;
}

void intel_update_watermarks(struct intel_crtc *crtc)
{
    struct drm_i915_private *dev_priv = to_i915(crtc->base.dev);

    if (dev_priv->display.update_wm)
        dev_priv->display.update_wm(crtc);
}

/**
 * intel_wm_method2 - Method 2 / "large buffer" watermark formula
 * @pixel_rate: Pipe pixel rate in kHz
 * @htotal: Pipe horizontal total
 * @width: Plane width in pixels
 * @cpp: Plane bytes per pixel
 * @latency: Memory wakeup latency in 0.1us units
 *
 * Compute the watermark using the method 2 or "large buffer"
 * formula. The caller may additonally add extra cachelines
 * to account for TLB misses and clock crossings.
 *
 * This method is concerned with the long term drain rate
 * of the FIFO, ie. it does account for blanking periods
 * which effectively reduce the average drain rate across
 * a longer period. The name "large" refers to the fact the
 * FIFO is relatively large compared to the amount of data
 * fetched.
 *
 * The FIFO level vs. time graph might look something like:
 *
 *    |\___       |\___
 *    |    \___   |    \___
 *    |        \  |        \
 * __ --__--__--__--__--__--__ (- plane active, _ blanking)
 * -> time
 *
 * Returns:
 * The watermark in bytes
 */
static unsigned int intel_wm_method2(unsigned int pixel_rate,
				     unsigned int htotal,
				     unsigned int width,
				     unsigned int cpp,
				     unsigned int latency)
{
	unsigned int ret;

	/*
	 * FIXME remove once all users are computing
	 * watermarks in the correct place.
	 */
	if (WARN_ON_ONCE(htotal == 0))
		htotal = 1;

	ret = (latency * pixel_rate) / (htotal * 10000);
	ret = (ret + 1) * width * cpp;

	return ret;
}


static bool intel_wm_plane_visible(const struct intel_crtc_state *crtc_state,
				   const struct intel_plane_state *plane_state)
{
	struct intel_plane *plane = to_intel_plane(plane_state->base.plane);

	/* FIXME check the 'enable' instead */
	if (!crtc_state->base.active)
		return false;

	/*
	 * Treat cursor with fb as always visible since cursor updates
	 * can happen faster than the vrefresh rate, and the current
	 * watermark code doesn't handle that correctly. Cursor updates
	 * which set/clear the fb or change the cursor size are going
	 * to get throttled by intel_legacy_cursor_update() to work
	 * around this problem with the watermark code.
	 */
	if (plane->id == PLANE_CURSOR)
		return plane_state->base.fb != NULL;
	else
		return plane_state->base.visible;
}



/* latency must be in 0.1us units. */
static unsigned int ilk_wm_method1(unsigned int pixel_rate,
				   unsigned int cpp,
				   unsigned int latency)
{
	unsigned int ret;

	ret = intel_wm_method1(pixel_rate, cpp, latency);
	ret = DIV_ROUND_UP(ret, 64) + 2;

	return ret;
}

/* latency must be in 0.1us units. */
static unsigned int ilk_wm_method2(unsigned int pixel_rate,
				   unsigned int htotal,
				   unsigned int width,
				   unsigned int cpp,
				   unsigned int latency)
{
	unsigned int ret;

	ret = intel_wm_method2(pixel_rate, htotal,
			       width, cpp, latency);
	ret = DIV_ROUND_UP(ret, 64) + 2;

	return ret;
}

static uint32_t ilk_wm_fbc(uint32_t pri_val, uint32_t horiz_pixels,
			   uint8_t cpp)
{
	/*
	 * Neither of these should be possible since this function shouldn't be
	 * called if the CRTC is off or the plane is invisible.  But let's be
	 * extra paranoid to avoid a potential divide-by-zero if we screw up
	 * elsewhere in the driver.
	 */
	if (WARN_ON(!cpp))
		return 0;
	if (WARN_ON(!horiz_pixels))
		return 0;

	return DIV_ROUND_UP(pri_val * 64, horiz_pixels * cpp) + 2;
}

struct ilk_wm_maximums {
	uint16_t pri;
	uint16_t spr;
	uint16_t cur;
	uint16_t fbc;
};

/*
 * For both WM_PIPE and WM_LP.
 * mem_value must be in 0.1us units.
 */
static uint32_t ilk_compute_pri_wm(const struct intel_crtc_state *cstate,
				   const struct intel_plane_state *pstate,
				   uint32_t mem_value,
				   bool is_lp)
{
	uint32_t method1, method2;
	int cpp;

	if (!intel_wm_plane_visible(cstate, pstate))
		return 0;

	cpp = pstate->base.fb->format->cpp[0];

	method1 = ilk_wm_method1(cstate->pixel_rate, cpp, mem_value);

	if (!is_lp)
		return method1;

	method2 = ilk_wm_method2(cstate->pixel_rate,
				 cstate->base.adjusted_mode.crtc_htotal,
				 drm_rect_width(&pstate->base.dst),
				 cpp, mem_value);

	return min(method1, method2);
}

/* Only for WM_LP. */
static uint32_t ilk_compute_fbc_wm(const struct intel_crtc_state *cstate,
				   const struct intel_plane_state *pstate,
				   uint32_t pri_val)
{
	int cpp;

	if (!intel_wm_plane_visible(cstate, pstate))
		return 0;

	cpp = pstate->base.fb->format->cpp[0];

	return ilk_wm_fbc(pri_val, drm_rect_width(&pstate->base.dst), cpp);
}

static unsigned int
ilk_display_fifo_size(const struct drm_i915_private *dev_priv)
{
	if (INTEL_GEN(dev_priv) >= 8)
		return 3072;
	else if (INTEL_GEN(dev_priv) >= 7)
		return 768;
	else
		return 512;
}

static unsigned int
ilk_plane_wm_reg_max(const struct drm_i915_private *dev_priv,
		     int level, bool is_sprite)
{

		/* IVB/HSW primary/sprite plane watermarks */
		return level == 0 ? 127 : 1023;

}

static unsigned int
ilk_cursor_wm_reg_max(const struct drm_i915_private *dev_priv, int level)
{

		return level == 0 ? 63 : 255;
}

static unsigned int ilk_fbc_wm_reg_max(const struct drm_i915_private *dev_priv)
{

		return 15;
}

/* Calculate the maximum primary/sprite plane watermark */
static unsigned int ilk_plane_wm_max(const struct drm_device *dev,
				     int level,
				     const struct intel_wm_config *config,
				     enum intel_ddb_partitioning ddb_partitioning,
				     bool is_sprite)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	unsigned int fifo_size = ilk_display_fifo_size(dev_priv);

	/* if sprites aren't enabled, sprites get nothing */
	if (is_sprite && !config->sprites_enabled)
		return 0;

	/* HSW allows LP1+ watermarks even with multiple pipes */
	if (level == 0 || config->num_pipes_active > 1) {
		fifo_size /= INTEL_INFO(dev_priv)->num_pipes;

		/*
		 * For some reason the non self refresh
		 * FIFO size is only half of the self
		 * refresh FIFO size on ILK/SNB.
		 */
		if (INTEL_GEN(dev_priv) <= 6)
			fifo_size /= 2;
	}

	if (config->sprites_enabled) {
		/* level 0 is always calculated with 1:1 split */
		if (level > 0 && ddb_partitioning == INTEL_DDB_PART_5_6) {
			if (is_sprite)
				fifo_size *= 5;
			fifo_size /= 6;
		} else {
			fifo_size /= 2;
		}
	}

	/* clamp to max that the registers can hold */
	return min(fifo_size, ilk_plane_wm_reg_max(dev_priv, level, is_sprite));
}

/* Calculate the maximum cursor plane watermark */
static unsigned int ilk_cursor_wm_max(const struct drm_device *dev,
				      int level,
				      const struct intel_wm_config *config)
{
	/* HSW LP1+ watermarks w/ multiple pipes */
	if (level > 0 && config->num_pipes_active > 1)
		return 64;

	/* otherwise just report max that registers can hold */
	return ilk_cursor_wm_reg_max(to_i915(dev), level);
}

static void ilk_compute_wm_maximums(const struct drm_device *dev,
				    int level,
				    const struct intel_wm_config *config,
				    enum intel_ddb_partitioning ddb_partitioning,
				    struct ilk_wm_maximums *max)
{
	max->pri = ilk_plane_wm_max(dev, level, config, ddb_partitioning, false);
	max->spr = ilk_plane_wm_max(dev, level, config, ddb_partitioning, true);
	max->cur = ilk_cursor_wm_max(dev, level, config);
	max->fbc = ilk_fbc_wm_reg_max(to_i915(dev));
}

static void ilk_compute_wm_reg_maximums(const struct drm_i915_private *dev_priv,
					int level,
					struct ilk_wm_maximums *max)
{
	max->pri = ilk_plane_wm_reg_max(dev_priv, level, false);
	max->spr = ilk_plane_wm_reg_max(dev_priv, level, true);
	max->cur = ilk_cursor_wm_reg_max(dev_priv, level);
	max->fbc = ilk_fbc_wm_reg_max(dev_priv);
}

static bool ilk_validate_wm_level(int level,
				  const struct ilk_wm_maximums *max,
				  struct intel_wm_level *result)
{
	bool ret;

	/* already determined to be invalid? */
	if (!result->enable)
		return false;

	result->enable = result->pri_val <= max->pri &&
			 result->spr_val <= max->spr &&
			 result->cur_val <= max->cur;

	ret = result->enable;

	/*
	 * HACK until we can pre-compute everything,
	 * and thus fail gracefully if LP0 watermarks
	 * are exceeded...
	 */
	if (level == 0 && !result->enable) {
		if (result->pri_val > max->pri)
			DRM_DEBUG_KMS("Primary WM%d too large %u (max %u)\n",
				      level, result->pri_val, max->pri);
		if (result->spr_val > max->spr)
			DRM_DEBUG_KMS("Sprite WM%d too large %u (max %u)\n",
				      level, result->spr_val, max->spr);
		if (result->cur_val > max->cur)
			DRM_DEBUG_KMS("Cursor WM%d too large %u (max %u)\n",
				      level, result->cur_val, max->cur);

		result->pri_val = min_t(uint32_t, result->pri_val, max->pri);
		result->spr_val = min_t(uint32_t, result->spr_val, max->spr);
		result->cur_val = min_t(uint32_t, result->cur_val, max->cur);
		result->enable = true;
	}

	return ret;
}

static void ilk_compute_wm_level(const struct drm_i915_private *dev_priv,
				 const struct intel_crtc *intel_crtc,
				 int level,
				 struct intel_crtc_state *cstate,
				 const struct intel_plane_state *pristate,
				 const struct intel_plane_state *sprstate,
				 const struct intel_plane_state *curstate,
				 struct intel_wm_level *result)
{
	uint16_t pri_latency = dev_priv->wm.pri_latency[level];
	uint16_t spr_latency = dev_priv->wm.spr_latency[level];
	uint16_t cur_latency = dev_priv->wm.cur_latency[level];

	/* WM1+ latency values stored in 0.5us units */
	if (level > 0) {
		pri_latency *= 5;
		spr_latency *= 5;
		cur_latency *= 5;
	}

	if (pristate) {
		result->pri_val = ilk_compute_pri_wm(cstate, pristate,
						     pri_latency, level);
		result->fbc_val = ilk_compute_fbc_wm(cstate, pristate, result->pri_val);
	}


	result->enable = true;
}

static uint32_t
hsw_compute_linetime_wm(const struct intel_crtc_state *cstate)
{
	const struct intel_atomic_state *intel_state =
		to_intel_atomic_state(cstate->base.state);
	const struct drm_display_mode *adjusted_mode =
		&cstate->base.adjusted_mode;
	u32 linetime, ips_linetime;

	if (!cstate->base.active)
		return 0;
	if (WARN_ON(adjusted_mode->crtc_clock == 0))
		return 0;
	if (WARN_ON(intel_state->cdclk.logical.cdclk == 0))
		return 0;

	/* The WM are computed with base on how long it takes to fill a single
	 * row at the given clock rate, multiplied by 8.
	 * */
	linetime = DIV_ROUND_CLOSEST(adjusted_mode->crtc_htotal * 1000 * 8,
				     adjusted_mode->crtc_clock);
	ips_linetime = DIV_ROUND_CLOSEST(adjusted_mode->crtc_htotal * 1000 * 8,
					 intel_state->cdclk.logical.cdclk);

	return PIPE_WM_LINETIME_IPS_LINETIME(ips_linetime) |
	       PIPE_WM_LINETIME_TIME(linetime);
}

static void intel_read_wm_latency(struct drm_i915_private *dev_priv,
				  uint16_t wm[8])
{

		uint64_t sskpd = I915_READ64(MCH_SSKPD);

		wm[0] = (sskpd >> 56) & 0xFF;
		if (wm[0] == 0)
			wm[0] = sskpd & 0xF;
		wm[1] = (sskpd >> 4) & 0xFF;
		wm[2] = (sskpd >> 12) & 0xFF;
		wm[3] = (sskpd >> 20) & 0x1FF;
		wm[4] = (sskpd >> 32) & 0x1FF;
}

static void intel_fixup_spr_wm_latency(struct drm_i915_private *dev_priv,
				       uint16_t wm[5])
{
	/* ILK sprite LP0 latency is 1300 ns */
	if (IS_GEN5(dev_priv))
		wm[0] = 13;
}

static void intel_fixup_cur_wm_latency(struct drm_i915_private *dev_priv,
				       uint16_t wm[5])
{
	/* ILK cursor LP0 latency is 1300 ns */
	if (IS_GEN5(dev_priv))
		wm[0] = 13;
}

int ilk_wm_max_level(const struct drm_i915_private *dev_priv)
{
	/* how many WM levels are we expecting */
	if (INTEL_GEN(dev_priv) >= 9)
		return 7;
	else if (IS_HASWELL(dev_priv) || IS_BROADWELL(dev_priv))
		return 4;
	else if (INTEL_GEN(dev_priv) >= 6)
		return 3;
	else
		return 2;
}

static void intel_print_wm_latency(struct drm_i915_private *dev_priv,
				   const char *name,
				   const uint16_t wm[8])
{
	int level, max_level = ilk_wm_max_level(dev_priv);

	for (level = 0; level <= max_level; level++) {
		unsigned int latency = wm[level];

		if (latency == 0) {
			DRM_ERROR("%s WM%d latency not provided\n",
				  name, level);
			continue;
		}

		/*
		 * - latencies are in us on gen9.
		 * - before then, WM1+ latency values are in 0.5us units
		 */
		if (INTEL_GEN(dev_priv) >= 9)
			latency *= 10;
		else if (level > 0)
			latency *= 5;

		DRM_DEBUG_KMS("%s WM%d latency %u (%u.%u usec)\n",
			      name, level, wm[level],
			      latency / 10, latency % 10);
	}
}



static void ilk_setup_wm_latency(struct drm_i915_private *dev_priv)
{
	intel_read_wm_latency(dev_priv, dev_priv->wm.pri_latency);

	memcpy(dev_priv->wm.spr_latency, dev_priv->wm.pri_latency,
	       sizeof(dev_priv->wm.pri_latency));
	memcpy(dev_priv->wm.cur_latency, dev_priv->wm.pri_latency,
	       sizeof(dev_priv->wm.pri_latency));

	intel_fixup_spr_wm_latency(dev_priv, dev_priv->wm.spr_latency);
	intel_fixup_cur_wm_latency(dev_priv, dev_priv->wm.cur_latency);

	intel_print_wm_latency(dev_priv, "Primary", dev_priv->wm.pri_latency);
	intel_print_wm_latency(dev_priv, "Sprite", dev_priv->wm.spr_latency);
	intel_print_wm_latency(dev_priv, "Cursor", dev_priv->wm.cur_latency);


}



static bool ilk_validate_pipe_wm(struct drm_device *dev,
				 struct intel_pipe_wm *pipe_wm)
{
	/* LP0 watermark maximums depend on this pipe alone */
	const struct intel_wm_config config = {
		.num_pipes_active = 1,
		.sprites_enabled = pipe_wm->sprites_enabled,
		.sprites_scaled = pipe_wm->sprites_scaled,
	};
	struct ilk_wm_maximums max;

	/* LP0 watermarks always use 1/2 DDB partitioning */
	ilk_compute_wm_maximums(dev, 0, &config, INTEL_DDB_PART_1_2, &max);

	/* At least LP0 must be valid */
	if (!ilk_validate_wm_level(0, &max, &pipe_wm->wm[0])) {
		DRM_DEBUG_KMS("LP0 watermark invalid\n");
		return false;
	}

	return true;
}

/* Compute new watermarks for the pipe */
static int ilk_compute_pipe_wm(struct intel_crtc_state *cstate)
{
	struct drm_atomic_state *state = cstate->base.state;
	struct intel_crtc *intel_crtc = to_intel_crtc(cstate->base.crtc);
	struct intel_pipe_wm *pipe_wm;
	struct drm_device *dev = state->dev;
	const struct drm_i915_private *dev_priv = to_i915(dev);
	struct drm_plane *plane;
	const struct drm_plane_state *plane_state;
	const struct intel_plane_state *pristate = NULL;
	const struct intel_plane_state *sprstate = NULL;
	const struct intel_plane_state *curstate = NULL;
	int level, max_level = ilk_wm_max_level(dev_priv), usable_level;
	struct ilk_wm_maximums max;

	pipe_wm = &cstate->wm.ilk.optimal;

	drm_atomic_crtc_state_for_each_plane_state(plane, plane_state, &cstate->base) {
		const struct intel_plane_state *ps = to_intel_plane_state(plane_state);

		if (plane->type == DRM_PLANE_TYPE_PRIMARY)
			pristate = ps;
		else if (plane->type == DRM_PLANE_TYPE_OVERLAY)
			sprstate = ps;
		else if (plane->type == DRM_PLANE_TYPE_CURSOR)
			curstate = ps;
	}

	pipe_wm->pipe_enabled = cstate->base.active;
	if (sprstate) {
		pipe_wm->sprites_enabled = sprstate->base.visible;
		pipe_wm->sprites_scaled = sprstate->base.visible &&
			(drm_rect_width(&sprstate->base.dst) != drm_rect_width(&sprstate->base.src) >> 16 ||
			 drm_rect_height(&sprstate->base.dst) != drm_rect_height(&sprstate->base.src) >> 16);
	}

	usable_level = max_level;

	/* ILK/SNB: LP2+ watermarks only w/o sprites */
	if (INTEL_GEN(dev_priv) <= 6 && pipe_wm->sprites_enabled)
		usable_level = 1;

	/* ILK/SNB/IVB: LP1+ watermarks only w/o scaling */
	if (pipe_wm->sprites_scaled)
		usable_level = 0;

	memset(&pipe_wm->wm, 0, sizeof(pipe_wm->wm));
	ilk_compute_wm_level(dev_priv, intel_crtc, 0, cstate,
			     pristate, sprstate, curstate, &pipe_wm->wm[0]);

	if (IS_HASWELL(dev_priv) || IS_BROADWELL(dev_priv))
		pipe_wm->linetime = hsw_compute_linetime_wm(cstate);

	if (!ilk_validate_pipe_wm(dev, pipe_wm))
		return -EINVAL;

	ilk_compute_wm_reg_maximums(dev_priv, 1, &max);

	for (level = 1; level <= usable_level; level++) {
		struct intel_wm_level *wm = &pipe_wm->wm[level];

		ilk_compute_wm_level(dev_priv, intel_crtc, level, cstate,
				     pristate, sprstate, curstate, wm);

		/*
		 * Disable any watermark level that exceeds the
		 * register maximums since such watermarks are
		 * always invalid.
		 */
		if (!ilk_validate_wm_level(level, &max, wm)) {
			memset(wm, 0, sizeof(*wm));
			break;
		}
	}

	return 0;
}

/*
 * Build a set of 'intermediate' watermark values that satisfy both the old
 * state and the new state.  These can be programmed to the hardware
 * immediately.
 */
static int ilk_compute_intermediate_wm(struct drm_device *dev,
				       struct intel_crtc *intel_crtc,
				       struct intel_crtc_state *newstate)
{
	struct intel_pipe_wm *a = &newstate->wm.ilk.intermediate;
	struct intel_atomic_state *intel_state =
		to_intel_atomic_state(newstate->base.state);
	const struct intel_crtc_state *oldstate =
		intel_atomic_get_old_crtc_state(intel_state, intel_crtc);
	const struct intel_pipe_wm *b = &oldstate->wm.ilk.optimal;
	int level, max_level = ilk_wm_max_level(to_i915(dev));

	/*
	 * Start with the final, target watermarks, then combine with the
	 * currently active watermarks to get values that are safe both before
	 * and after the vblank.
	 */
	*a = newstate->wm.ilk.optimal;
	if (!newstate->base.active || drm_atomic_crtc_needs_modeset(&newstate->base))
		return 0;

	a->pipe_enabled |= b->pipe_enabled;
	a->sprites_enabled |= b->sprites_enabled;
	a->sprites_scaled |= b->sprites_scaled;

	for (level = 0; level <= max_level; level++) {
		struct intel_wm_level *a_wm = &a->wm[level];
		const struct intel_wm_level *b_wm = &b->wm[level];

		a_wm->enable &= b_wm->enable;
		a_wm->pri_val = max(a_wm->pri_val, b_wm->pri_val);
		a_wm->spr_val = max(a_wm->spr_val, b_wm->spr_val);
		a_wm->cur_val = max(a_wm->cur_val, b_wm->cur_val);
		a_wm->fbc_val = max(a_wm->fbc_val, b_wm->fbc_val);
	}

	/*
	 * We need to make sure that these merged watermark values are
	 * actually a valid configuration themselves.  If they're not,
	 * there's no safe way to transition from the old state to
	 * the new state, so we need to fail the atomic transaction.
	 */
	if (!ilk_validate_pipe_wm(dev, a))
		return -EINVAL;

	/*
	 * If our intermediate WM are identical to the final WM, then we can
	 * omit the post-vblank programming; only update if it's different.
	 */
	if (memcmp(a, &newstate->wm.ilk.optimal, sizeof(*a)) != 0)
		newstate->wm.need_postvbl_update = true;

	return 0;
}



/* dirty bits used to track which watermarks need changes */
#define WM_DIRTY_PIPE(pipe) (1 << (pipe))
#define WM_DIRTY_LINETIME(pipe) (1 << (8 + (pipe)))
#define WM_DIRTY_LP(wm_lp) (1 << (15 + (wm_lp)))
#define WM_DIRTY_LP_ALL (WM_DIRTY_LP(1) | WM_DIRTY_LP(2) | WM_DIRTY_LP(3))
#define WM_DIRTY_FBC (1 << 24)
#define WM_DIRTY_DDB (1 << 25)

static bool _ilk_disable_lp_wm(struct drm_i915_private *dev_priv,
			       unsigned int dirty)
{
	struct ilk_wm_values *previous = &dev_priv->wm.hw;
	bool changed = false;

	if (dirty & WM_DIRTY_LP(3) && previous->wm_lp[2] & WM1_LP_SR_EN) {
		previous->wm_lp[2] &= ~WM1_LP_SR_EN;
		I915_WRITE(WM3_LP_ILK, previous->wm_lp[2]);
		changed = true;
	}
	if (dirty & WM_DIRTY_LP(2) && previous->wm_lp[1] & WM1_LP_SR_EN) {
		previous->wm_lp[1] &= ~WM1_LP_SR_EN;
		I915_WRITE(WM2_LP_ILK, previous->wm_lp[1]);
		changed = true;
	}
	if (dirty & WM_DIRTY_LP(1) && previous->wm_lp[0] & WM1_LP_SR_EN) {
		previous->wm_lp[0] &= ~WM1_LP_SR_EN;
		I915_WRITE(WM1_LP_ILK, previous->wm_lp[0]);
		changed = true;
	}

	/*
	 * Don't touch WM1S_LP_EN here.
	 * Doing so could cause underruns.
	 */

	return changed;
}


bool ilk_disable_lp_wm(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);

	return _ilk_disable_lp_wm(dev_priv, WM_DIRTY_LP_ALL);
}

static void ilk_pipe_wm_get_hw_state(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct ilk_wm_values *hw = &dev_priv->wm.hw;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_crtc_state *cstate = to_intel_crtc_state(crtc->state);
	struct intel_pipe_wm *active = &cstate->wm.ilk.optimal;
	enum pipe pipe = intel_crtc->pipe;
	static const i915_reg_t wm0_pipe_reg[] = {
		[PIPE_A] = WM0_PIPEA_ILK,
		[PIPE_B] = WM0_PIPEB_ILK,
		[PIPE_C] = WM0_PIPEC_IVB,
	};

	hw->wm_pipe[pipe] = I915_READ(wm0_pipe_reg[pipe]);
	if (IS_HASWELL(dev_priv) || IS_BROADWELL(dev_priv))
		hw->wm_linetime[pipe] = I915_READ(PIPE_WM_LINETIME(pipe));

	memset(active, 0, sizeof(*active));

	active->pipe_enabled = intel_crtc->active;

	if (active->pipe_enabled) {
		u32 tmp = hw->wm_pipe[pipe];

		/*
		 * For active pipes LP0 watermark is marked as
		 * enabled, and LP1+ watermaks as disabled since
		 * we can't really reverse compute them in case
		 * multiple pipes are active.
		 */
		active->wm[0].enable = true;
		active->wm[0].pri_val = (tmp & WM0_PIPE_PLANE_MASK) >> WM0_PIPE_PLANE_SHIFT;
		active->wm[0].spr_val = (tmp & WM0_PIPE_SPRITE_MASK) >> WM0_PIPE_SPRITE_SHIFT;
		active->wm[0].cur_val = tmp & WM0_PIPE_CURSOR_MASK;
		active->linetime = hw->wm_linetime[pipe];
	} else {
		int level, max_level = ilk_wm_max_level(dev_priv);

		/*
		 * For inactive pipes, all watermark levels
		 * should be marked as enabled but zeroed,
		 * which is what we'd compute them to.
		 */
		for (level = 0; level <= max_level; level++)
			active->wm[level].enable = true;
	}

	intel_crtc->wm.active.ilk = *active;
}




/*
 * FIXME should probably kill this and improve
 * the real watermark readout/sanitation instead
 */
static void ilk_init_lp_watermarks(struct drm_i915_private *dev_priv)
{
	I915_WRITE(WM3_LP_ILK, I915_READ(WM3_LP_ILK) & ~WM1_LP_SR_EN);
	I915_WRITE(WM2_LP_ILK, I915_READ(WM2_LP_ILK) & ~WM1_LP_SR_EN);
	I915_WRITE(WM1_LP_ILK, I915_READ(WM1_LP_ILK) & ~WM1_LP_SR_EN);

	/*
	 * Don't touch WM1S_LP_EN here.
	 * Doing so could cause underruns.
	 */
}

void ilk_wm_get_hw_state(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct ilk_wm_values *hw = &dev_priv->wm.hw;
	struct drm_crtc *crtc;

	ilk_init_lp_watermarks(dev_priv);

	for_each_crtc(dev, crtc)
		ilk_pipe_wm_get_hw_state(crtc);

	hw->wm_lp[0] = I915_READ(WM1_LP_ILK);
	hw->wm_lp[1] = I915_READ(WM2_LP_ILK);
	hw->wm_lp[2] = I915_READ(WM3_LP_ILK);

	hw->wm_lp_spr[0] = I915_READ(WM1S_LP_ILK);
	if (INTEL_GEN(dev_priv) >= 7) {
		hw->wm_lp_spr[1] = I915_READ(WM2S_LP_IVB);
		hw->wm_lp_spr[2] = I915_READ(WM3S_LP_IVB);
	}

	if (IS_HASWELL(dev_priv) || IS_BROADWELL(dev_priv))
		hw->partitioning = (I915_READ(WM_MISC) & WM_MISC_DATA_PARTITION_5_6) ?
			INTEL_DDB_PART_5_6 : INTEL_DDB_PART_1_2;
	else if (IS_IVYBRIDGE(dev_priv))
		hw->partitioning = (I915_READ(DISP_ARB_CTL2) & DISP_DATA_PARTITION_5_6) ?
			INTEL_DDB_PART_5_6 : INTEL_DDB_PART_1_2;

	hw->enable_fbc_wm =
		!(I915_READ(DISP_ARB_CTL) & DISP_FBC_WM_DIS);
}


void intel_init_ipc(struct drm_i915_private *dev_priv)
{
	dev_priv->ipc_enabled = false;
    return;

}



static void gen6_disable_rc6(struct drm_i915_private *dev_priv)
{
	I915_WRITE(GEN6_RC_CONTROL, 0);
}

static void gen6_disable_rps(struct drm_i915_private *dev_priv)
{
	I915_WRITE(GEN6_RPNSWREQ, 1 << 31);
	I915_WRITE(GEN6_RP_CONTROL, 0);
}

void intel_sanitize_gt_powersave(struct drm_i915_private *dev_priv)
{
	dev_priv->gt_pm.rps.enabled = true; /* force RPS disabling */
	dev_priv->gt_pm.rc6.enabled = true; /* force RC6 disabling */
	intel_disable_gt_powersave(dev_priv);
printk("intel_sanitize_gt_powersave before");
    gen6_reset_rps_interrupts(dev_priv);
printk("intel_sanitize_gt_powersave after");
}

static inline void intel_disable_llc_pstate(struct drm_i915_private *i915)
{
	lockdep_assert_held(&i915->pcu_lock);

	if (!i915->gt_pm.llc_pstate.enabled)
		return;

	/* Currently there is no HW configuration to be done to disable. */

	i915->gt_pm.llc_pstate.enabled = false;
}

static void intel_disable_rc6(struct drm_i915_private *dev_priv)
{
	lockdep_assert_held(&dev_priv->pcu_lock);

	if (!dev_priv->gt_pm.rc6.enabled)
		return;

    gen6_disable_rc6(dev_priv);

	dev_priv->gt_pm.rc6.enabled = false;
}

static void intel_disable_rps(struct drm_i915_private *dev_priv)
{
	lockdep_assert_held(&dev_priv->pcu_lock);

	if (!dev_priv->gt_pm.rps.enabled)
		return;

    gen6_disable_rps(dev_priv);

	dev_priv->gt_pm.rps.enabled = false;
}

void intel_disable_gt_powersave(struct drm_i915_private *dev_priv)
{
	mutex_lock(&dev_priv->pcu_lock);

	intel_disable_rc6(dev_priv);
	intel_disable_rps(dev_priv);
	if (HAS_LLC(dev_priv))
		intel_disable_llc_pstate(dev_priv);

	mutex_unlock(&dev_priv->pcu_lock);
}


static void lpt_init_clock_gating(struct drm_i915_private *dev_priv)
{
    /*
     * TODO: this bit should only be enabled when really needed, then
     * disabled when not needed anymore in order to save power.
     */
    if (HAS_PCH_LPT_LP(dev_priv))
        I915_WRITE(SOUTH_DSPCLK_GATE_D,
               I915_READ(SOUTH_DSPCLK_GATE_D) |
               PCH_LP_PARTITION_LEVEL_DISABLE);

    /* WADPOClockGatingDisable:hsw */
    I915_WRITE(TRANS_CHICKEN1(PIPE_A),
           I915_READ(TRANS_CHICKEN1(PIPE_A)) |
           TRANS_CHICKEN1_DP0UNIT_GC_DISABLE);
}


static void hsw_init_clock_gating(struct drm_i915_private *dev_priv)
{
    /* L3 caching of data atomics doesn't work -- disable it. */
    I915_WRITE(HSW_SCRATCH1, HSW_SCRATCH1_L3_DATA_ATOMICS_DISABLE);
    I915_WRITE(HSW_ROW_CHICKEN3,
           _MASKED_BIT_ENABLE(HSW_ROW_CHICKEN3_L3_GLOBAL_ATOMICS_DISABLE));

    /* This is required by WaCatErrorRejectionIssue:hsw */
    I915_WRITE(GEN7_SQ_CHICKEN_MBCUNIT_CONFIG,
            I915_READ(GEN7_SQ_CHICKEN_MBCUNIT_CONFIG) |
            GEN7_SQ_CHICKEN_MBCUNIT_SQINTMOB);

    /* WaVSRefCountFullforceMissDisable:hsw */
    I915_WRITE(GEN7_FF_THREAD_MODE,
           I915_READ(GEN7_FF_THREAD_MODE) & ~GEN7_FF_VS_REF_CNT_FFME);

    /* WaDisable_RenderCache_OperationalFlush:hsw */
    I915_WRITE(CACHE_MODE_0_GEN7, _MASKED_BIT_DISABLE(RC_OP_FLUSH_ENABLE));

    /* enable HiZ Raw Stall Optimization */
    I915_WRITE(CACHE_MODE_0_GEN7,
           _MASKED_BIT_DISABLE(HIZ_RAW_STALL_OPT_DISABLE));

    /* WaDisable4x2SubspanOptimization:hsw */
    I915_WRITE(CACHE_MODE_1,
           _MASKED_BIT_ENABLE(PIXEL_SUBSPAN_COLLECT_OPT_DISABLE));

    /*
     * BSpec recommends 8x4 when MSAA is used,
     * however in practice 16x4 seems fastest.
     *
     * Note that PS/WM thread counts depend on the WIZ hashing
     * disable bit, which we don't touch here, but it's good
     * to keep in mind (see 3DSTATE_PS and 3DSTATE_WM).
     */
    I915_WRITE(GEN7_GT_MODE,
           _MASKED_FIELD(GEN6_WIZ_HASHING_MASK, GEN6_WIZ_HASHING_16x4));

    /* WaSampleCChickenBitEnable:hsw */
    I915_WRITE(HALF_SLICE_CHICKEN3,
           _MASKED_BIT_ENABLE(HSW_SAMPLE_C_PERFORMANCE));

    /* WaSwitchSolVfFArbitrationPriority:hsw */
    I915_WRITE(GAM_ECOCHK, I915_READ(GAM_ECOCHK) | HSW_ECOCHK_ARB_PRIO_SOL);

    lpt_init_clock_gating(dev_priv);
}

/**
 * intel_init_clock_gating_hooks - setup the clock gating hooks
 * @dev_priv: device private
 *
 * Setup the hooks that configure which clocks of a given platform can be
 * gated and also apply various GT and display specific workarounds for these
 * platforms. Note that some GT specific workarounds are applied separately
 * when GPU contexts or batchbuffers start their execution.
 */
void intel_init_clock_gating_hooks(struct drm_i915_private *dev_priv)
{
    dev_priv->display.init_clock_gating = hsw_init_clock_gating;
}

/* Set up chip specific power management-related functions */
void intel_init_pm(struct drm_i915_private *dev_priv)
{
	intel_fbc_init(dev_priv);

    ilk_setup_wm_latency(dev_priv);

        dev_priv->display.compute_pipe_wm = ilk_compute_pipe_wm;
        dev_priv->display.compute_intermediate_wm =
            ilk_compute_intermediate_wm;


}

void intel_pm_setup(struct drm_i915_private *dev_priv)
{
	mutex_init(&dev_priv->pcu_lock);

	atomic_set(&dev_priv->gt_pm.rps.num_waiters, 0);

	dev_priv->runtime_pm.suspended = false;
	atomic_set(&dev_priv->runtime_pm.wakeref_count, 0);
}
