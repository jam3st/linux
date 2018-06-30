/*
 * Copyright © 2008-2015 Intel Corporation
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
 */

#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include "i915_drv.h"

/**
 * DOC: fence register handling
 *
 * Important to avoid confusions: "fences" in the i915 driver are not execution
 * fences used to track command completion but hardware detiler objects which
 * wrap a given range of the global GTT. Each platform has only a fairly limited
 * set of these objects.
 *
 * Fences are used to detile GTT memory mappings. They're also connected to the
 * hardware frontbuffer render tracking and hence interact with frontbuffer
 * compression. Furthermore on older platforms fences are required for tiled
 * objects used by the display engine. They can also be used by the render
 * engine - they're required for blitter commands and are optional for render
 * commands. But on gen4+ both display (with the exception of fbc) and rendering
 * have their own tiling state bits and don't need fences.
 *
 * Also note that fences only support X and Y tiling and hence can't be used for
 * the fancier new tiling formats like W, Ys and Yf.
 *
 * Finally note that because fences are such a restricted resource they're
 * dynamically associated with objects. Furthermore fence state is committed to
 * the hardware lazily to avoid unnecessary stalls on gen2/3. Therefore code must
 * explicitly call i915_gem_object_get_fence() to synchronize fencing status
 * for cpu access. Also note that some code wants an unfenced view, for those
 * cases the fence can be removed forcefully with i915_gem_object_put_fence().
 *
 * Internally these functions will synchronize with userspace access by removing
 * CPU ptes into GTT mmaps (not the GTT ptes themselves) as needed.
 */

#define pipelined 0

static void i965_write_fence_reg(struct drm_i915_fence_reg *fence,
				 struct i915_vma *vma)
{
	i915_reg_t fence_reg_lo, fence_reg_hi;
	int fence_pitch_shift;
	u64 val;

	if (INTEL_GEN(fence->i915) >= 6) {
		fence_reg_lo = FENCE_REG_GEN6_LO(fence->id);
		fence_reg_hi = FENCE_REG_GEN6_HI(fence->id);
		fence_pitch_shift = GEN6_FENCE_PITCH_SHIFT;

	} else {
		fence_reg_lo = FENCE_REG_965_LO(fence->id);
		fence_reg_hi = FENCE_REG_965_HI(fence->id);
		fence_pitch_shift = I965_FENCE_PITCH_SHIFT;
	}

	val = 0;
	if (vma) {
		unsigned int stride = i915_gem_object_get_stride(vma->obj);

		GEM_BUG_ON(!i915_vma_is_map_and_fenceable(vma));
		GEM_BUG_ON(!IS_ALIGNED(vma->node.start, I965_FENCE_PAGE));
		GEM_BUG_ON(!IS_ALIGNED(vma->fence_size, I965_FENCE_PAGE));
		GEM_BUG_ON(!IS_ALIGNED(stride, 128));

		val = (vma->node.start + vma->fence_size - I965_FENCE_PAGE) << 32;
		val |= vma->node.start;
		val |= (u64)((stride / 128) - 1) << fence_pitch_shift;
		if (i915_gem_object_get_tiling(vma->obj) == I915_TILING_Y)
			val |= BIT(I965_FENCE_TILING_Y_SHIFT);
		val |= I965_FENCE_REG_VALID;
	}

	if (!pipelined) {
		struct drm_i915_private *dev_priv = fence->i915;

		/* To w/a incoherency with non-atomic 64-bit register updates,
		 * we split the 64-bit update into two 32-bit writes. In order
		 * for a partial fence not to be evaluated between writes, we
		 * precede the update with write to turn off the fence register,
		 * and only enable the fence as the last step.
		 *
		 * For extra levels of paranoia, we make sure each step lands
		 * before applying the next step.
		 */
		I915_WRITE(fence_reg_lo, 0);
		POSTING_READ(fence_reg_lo);

		I915_WRITE(fence_reg_hi, upper_32_bits(val));
		I915_WRITE(fence_reg_lo, lower_32_bits(val));
		POSTING_READ(fence_reg_lo);
	}
}



static void fence_write(struct drm_i915_fence_reg *fence,
			struct i915_vma *vma)
{
	/* Previous access through the fence register is marshalled by
	 * the mb() inside the fault handlers (i915_gem_release_mmaps)
	 * and explicitly managed for internal users.
	 */
    i965_write_fence_reg(fence, vma);

	/* Access through the fenced region afterwards is
	 * ordered by the posting reads whilst writing the registers.
	 */

	fence->dirty = false;
}

/**
 * i915_vma_pin_fence - set up fencing for a vma
 * @vma: vma to map through a fence reg
 *
 * When mapping objects through the GTT, userspace wants to be able to write
 * to them without having to worry about swizzling if the object is tiled.
 * This function walks the fence regs looking for a free one for @obj,
 * stealing one if it can't find any.
 *
 * It then sets up the reg based on the object's properties: address, pitch
 * and tiling format.
 *
 * For an untiled surface, this removes any existing fence.
 *
 * Returns:
 *
 * 0 on success, negative error code on failure.
 */
int
i915_vma_pin_fence(struct i915_vma *vma)
{

    return 0;

}

/**
 * i915_gem_restore_fences - restore fence state
 * @dev_priv: i915 device private
 *
 * Restore the hw fence state to match the software tracking again, to be called
 * after a gpu reset and on resume. Note that on runtime suspend we only cancel
 * the fences, to be reacquired by the user later.
 */
void i915_gem_restore_fences(struct drm_i915_private *dev_priv)
{
	int i;

	for (i = 0; i < dev_priv->num_fence_regs; i++) {
		struct drm_i915_fence_reg *reg = &dev_priv->fence_regs[i];
		struct i915_vma *vma = reg->vma;

		GEM_BUG_ON(vma && vma->fence != reg);

		/*
		 * Commit delayed tiling changes if we have an object still
		 * attached to the fence, otherwise just clear the fence.
		 */
		if (vma && !i915_gem_object_is_tiled(vma->obj)) {
			GEM_BUG_ON(!reg->dirty);
			GEM_BUG_ON(i915_vma_has_userfault(vma));

			list_move(&reg->link, &dev_priv->mm.fence_list);
			vma->fence = NULL;
			vma = NULL;
		}

		fence_write(reg, vma);
		reg->vma = vma;
	}
}

/**
 * DOC: tiling swizzling details
 *
 * The idea behind tiling is to increase cache hit rates by rearranging
 * pixel data so that a group of pixel accesses are in the same cacheline.
 * Performance improvement from doing this on the back/depth buffer are on
 * the order of 30%.
 *
 * Intel architectures make this somewhat more complicated, though, by
 * adjustments made to addressing of data when the memory is in interleaved
 * mode (matched pairs of DIMMS) to improve memory bandwidth.
 * For interleaved memory, the CPU sends every sequential 64 bytes
 * to an alternate memory channel so it can get the bandwidth from both.
 *
 * The GPU also rearranges its accesses for increased bandwidth to interleaved
 * memory, and it matches what the CPU does for non-tiled.  However, when tiled
 * it does it a little differently, since one walks addresses not just in the
 * X direction but also Y.  So, along with alternating channels when bit
 * 6 of the address flips, it also alternates when other bits flip --  Bits 9
 * (every 512 bytes, an X tile scanline) and 10 (every two X tile scanlines)
 * are common to both the 915 and 965-class hardware.
 *
 * The CPU also sometimes XORs in higher bits as well, to improve
 * bandwidth doing strided access like we do so frequently in graphics.  This
 * is called "Channel XOR Randomization" in the MCH documentation.  The result
 * is that the CPU is XORing in either bit 11 or bit 17 to bit 6 of its address
 * decode.
 *
 * All of this bit 6 XORing has an effect on our memory management,
 * as we need to make sure that the 3d driver can correctly address object
 * contents.
 *
 * If we don't have interleaved memory, all tiling is safe and no swizzling is
 * required.
 *
 * When bit 17 is XORed in, we simply refuse to tile at all.  Bit
 * 17 is not just a page offset, so as we page an object out and back in,
 * individual pages in it will have different bit 17 addresses, resulting in
 * each 64 bytes being swapped with its neighbor!
 *
 * Otherwise, if interleaved, we have to tell the 3d driver what the address
 * swizzling it needs to do is, since it's writing with the CPU to the pages
 * (bit 6 and potentially bit 11 XORed in), and the GPU is reading from the
 * pages (bit 6, 9, and 10 XORed in), resulting in a cumulative bit swizzling
 * required by the CPU of XORing in bit 6, 9, 10, and potentially 11, in order
 * to match what the GPU expects.
 */

/**
 * i915_gem_detect_bit_6_swizzle - detect bit 6 swizzling pattern
 * @dev_priv: i915 device private
 *
 * Detects bit 6 swizzling of address lookup between IGD access and CPU
 * access through main memory.
 */
void
i915_gem_detect_bit_6_swizzle(struct drm_i915_private *dev_priv)
{
	uint32_t swizzle_x = I915_BIT_6_SWIZZLE_UNKNOWN;
	uint32_t swizzle_y = I915_BIT_6_SWIZZLE_UNKNOWN;

    if (dev_priv->preserve_bios_swizzle) {
        if (I915_READ(DISP_ARB_CTL) &
            DISP_TILE_SURFACE_SWIZZLING) {
            swizzle_x = I915_BIT_6_SWIZZLE_9_10;
            swizzle_y = I915_BIT_6_SWIZZLE_9;
        } else {
            swizzle_x = I915_BIT_6_SWIZZLE_NONE;
            swizzle_y = I915_BIT_6_SWIZZLE_NONE;
        }
    } else {
        uint32_t dimm_c0, dimm_c1;
        dimm_c0 = I915_READ(MAD_DIMM_C0);
        dimm_c1 = I915_READ(MAD_DIMM_C1);
        dimm_c0 &= MAD_DIMM_A_SIZE_MASK | MAD_DIMM_B_SIZE_MASK;
        dimm_c1 &= MAD_DIMM_A_SIZE_MASK | MAD_DIMM_B_SIZE_MASK;
        /* Enable swizzling when the channels are populated
         * with identically sized dimms. We don't need to check
         * the 3rd channel because no cpu with gpu attached
         * ships in that configuration. Also, swizzling only
         * makes sense for 2 channels anyway. */
        if (dimm_c0 == dimm_c1) {
            swizzle_x = I915_BIT_6_SWIZZLE_9_10;
            swizzle_y = I915_BIT_6_SWIZZLE_9;
        } else {
            swizzle_x = I915_BIT_6_SWIZZLE_NONE;
            swizzle_y = I915_BIT_6_SWIZZLE_NONE;
        }
    }


	if (swizzle_x == I915_BIT_6_SWIZZLE_UNKNOWN ||
	    swizzle_y == I915_BIT_6_SWIZZLE_UNKNOWN) {
		/* Userspace likes to explode if it sees unknown swizzling,
		 * so lie. We will finish the lie when reporting through
		 * the get-tiling-ioctl by reporting the physical swizzle
		 * mode as unknown instead.
		 *
		 * As we don't strictly know what the swizzling is, it may be
		 * bit17 dependent, and so we need to also prevent the pages
		 * from being moved.
		 */
		dev_priv->quirks |= QUIRK_PIN_SWIZZLED_PAGES;
		swizzle_x = I915_BIT_6_SWIZZLE_NONE;
		swizzle_y = I915_BIT_6_SWIZZLE_NONE;
	}

	dev_priv->mm.bit_6_swizzle_x = swizzle_x;
	dev_priv->mm.bit_6_swizzle_y = swizzle_y;
}

