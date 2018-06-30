/*
 * Copyright © 2016 Intel Corporation
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
 */

#include <drm/drm_print.h>

#include "intel_device_info.h"
#include "i915_drv.h"

static u16 compute_eu_total(const struct sseu_dev_info *sseu)
{
	u16 i, total = 0;

	for (i = 0; i < ARRAY_SIZE(sseu->eu_mask); i++)
		total += hweight8(sseu->eu_mask[i]);

	return total;
}


static void haswell_sseu_info_init(struct drm_i915_private *dev_priv)
{
	struct intel_device_info *info = mkwrite_device_info(dev_priv);
	struct sseu_dev_info *sseu = &info->sseu;
	u32 fuse1;
	int s, ss;

	/*
	 * There isn't a register to tell us how many slices/subslices. We
	 * work off the PCI-ids here.
	 */
	switch (info->gt) {
	default:
		MISSING_CASE(info->gt);
		/* fall through */
	case 1:
		sseu->slice_mask = BIT(0);
		sseu->subslice_mask[0] = BIT(0);
		break;
	case 2:
		sseu->slice_mask = BIT(0);
		sseu->subslice_mask[0] = BIT(0) | BIT(1);
		break;
	case 3:
		sseu->slice_mask = BIT(0) | BIT(1);
		sseu->subslice_mask[0] = BIT(0) | BIT(1);
		sseu->subslice_mask[1] = BIT(0) | BIT(1);
		break;
	}

	sseu->max_slices = hweight8(sseu->slice_mask);
	sseu->max_subslices = hweight8(sseu->subslice_mask[0]);

	fuse1 = I915_READ(HSW_PAVP_FUSE1);
	switch ((fuse1 & HSW_F1_EU_DIS_MASK) >> HSW_F1_EU_DIS_SHIFT) {
	default:
		MISSING_CASE((fuse1 & HSW_F1_EU_DIS_MASK) >>
			     HSW_F1_EU_DIS_SHIFT);
		/* fall through */
	case HSW_F1_EU_DIS_10EUS:
		sseu->eu_per_subslice = 10;
		break;
	case HSW_F1_EU_DIS_8EUS:
		sseu->eu_per_subslice = 8;
		break;
	case HSW_F1_EU_DIS_6EUS:
		sseu->eu_per_subslice = 6;
		break;
	}
	sseu->max_eus_per_subslice = sseu->eu_per_subslice;

	for (s = 0; s < sseu->max_slices; s++) {
		for (ss = 0; ss < sseu->max_subslices; ss++) {
			sseu_set_eus(sseu, s, ss,
				     (1UL << sseu->eu_per_subslice) - 1);
		}
	}

	sseu->eu_total = compute_eu_total(sseu);

	/* No powergating for you. */
	sseu->has_slice_pg = 0;
	sseu->has_subslice_pg = 0;
	sseu->has_eu_pg = 0;
}

static u32 read_timestamp_frequency(struct drm_i915_private *dev_priv)
{
	u32 f12_5_mhz = 12500;
   /* PRMs say:
		 *
		 *     "The PCU TSC counts 10ns increments; this timestamp
		 *      reflects bits 38:3 of the TSC (i.e. 80ns granularity,
		 *      rolling over every 1.5 hours).
		 */
		return f12_5_mhz;
}

/**
 * intel_device_info_runtime_init - initialize runtime info
 * @info: intel device info struct
 *
 * Determine various intel_device_info fields at runtime.
 *
 * Use it when either:
 *   - it's judged too laborious to fill n static structures with the limit
 *     when a simple if statement does the job,
 *   - run-time checks (eg read fuse/strap registers) are needed.
 *
 * This function needs to be called:
 *   - after the MMIO has been setup as we are reading registers,
 *   - after the PCH has been detected,
 *   - before the first usage of the fields it can tweak.
 */
void intel_device_info_runtime_init(struct intel_device_info *info)
{
	struct drm_i915_private *dev_priv =
		container_of(info, struct drm_i915_private, info);
	enum pipe pipe;



	BUILD_BUG_ON(I915_NUM_ENGINES >
		     sizeof(intel_ring_mask_t) * BITS_PER_BYTE);


    for_each_pipe(dev_priv, pipe)
        info->num_sprites[pipe] = 1;



    haswell_sseu_info_init(dev_priv);

	/* Initialize command stream timestamp frequency */
	info->cs_timestamp_frequency_khz = read_timestamp_frequency(dev_priv);
}

void intel_driver_caps_print(const struct intel_driver_caps *caps,
			     struct drm_printer *p)
{
	drm_printf(p, "scheduler: %x\n", caps->scheduler);
}
