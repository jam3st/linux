/*
 * Copyright © 2008 Intel Corporation
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
 *    Eric Anholt <eric@anholt.net>
 *
 */

#include <linux/string.h>
#include <linux/bitops.h>
#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include "i915_drv.h"

/**
 * DOC: buffer object tiling
 *
 * i915_gem_set_tiling_ioctl() and i915_gem_get_tiling_ioctl() is the userspace
 * interface to declare fence register requirements.
 *
 * In principle GEM doesn't care at all about the internal data layout of an
 * object, and hence it also doesn't care about tiling or swizzling. There's two
 * exceptions:
 *
 * - For X and Y tiling the hardware provides detilers for CPU access, so called
 *   fences. Since there's only a limited amount of them the kernel must manage
 *   these, and therefore userspace must tell the kernel the object tiling if it
 *   wants to use fences for detiling.
 * - On gen3 and gen4 platforms have a swizzling pattern for tiled objects which
 *   depends upon the physical page frame number. When swapping such objects the
 *   page frame number might change and the kernel must be able to fix this up
 *   and hence now the tiling. Note that on a subset of platforms with
 *   asymmetric memory channel population the swizzling pattern changes in an
 *   unknown way, and for those the kernel simply forbids swapping completely.
 *
 * Since neither of this applies for new tiling layouts on modern platforms like
 * W, Ys and Yf tiling GEM only allows object tiling to be set to X or Y tiled.
 * Anything else can be handled in userspace entirely without the kernel's
 * invovlement.
 */

/**
 * i915_gem_fence_size - required global GTT size for a fence
 * @i915: i915 device
 * @size: object size
 * @tiling: tiling mode
 * @stride: tiling stride
 *
 * Return the required global GTT size for a fence (view of a tiled object),
 * taking into account potential fence register mapping.
 */
u32 i915_gem_fence_size(struct drm_i915_private *i915,
			u32 size, unsigned int tiling, unsigned int stride)
{

	GEM_BUG_ON(!size);

    return size;
}

/**
 * i915_gem_fence_alignment - required global GTT alignment for a fence
 * @i915: i915 device
 * @size: object size
 * @tiling: tiling mode
 * @stride: tiling stride
 *
 * Return the required global GTT alignment for a fence (a view of a tiled
 * object), taking into account potential fence register mapping.
 */
u32 i915_gem_fence_alignment(struct drm_i915_private *i915, u32 size,
			     unsigned int tiling, unsigned int stride)
{
	GEM_BUG_ON(!size);

    return I915_GTT_MIN_ALIGNMENT;

}
