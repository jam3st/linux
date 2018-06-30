# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

# Add a set of useful warning flags and enable -Werror for CI to prevent
# trivial mistakes from creeping in. We have to do this piecemeal as we reject
# any patch that isn't warning clean, so turning on -Wall -Wextra (or W=1) we
# need to filter out dubious warnings.  Still it is our interest
# to keep running locally with W=1 C=1 until we are completely clean.
#
# Note the danger in using -Wall -Wextra is that when CI updates gcc we
# will most likely get a sudden build breakage... Hopefully we will fix
# new warnings before CI updates!
subdir-ccflags-y := -Wall -Wextra
subdir-ccflags-y += $(call cc-disable-warning, unused-parameter)
subdir-ccflags-y += $(call cc-disable-warning, type-limits)
subdir-ccflags-y += $(call cc-disable-warning, missing-field-initializers)
subdir-ccflags-y += $(call cc-disable-warning, implicit-fallthrough)
subdir-ccflags-y += $(call cc-disable-warning, unused-but-set-variable)
subdir-ccflags-$(CONFIG_DRM_I915_WERROR) += -Werror

# Fine grained warnings disable
CFLAGS_i915_pci.o = $(call cc-disable-warning, override-init)
CFLAGS_intel_fbdev.o = $(call cc-disable-warning, override-init)

subdir-ccflags-y += \
	$(call as-instr,movntdqa (%eax)$(comma)%xmm0,-DCONFIG_AS_MOVNTDQA)

# Please keep these build lists sorted!

# core driver code
i915-y := i915_drv.o \
	  i915_irq.o \
	  i915_memcpy.o \
	  i915_params.o \
	  i915_pci.o \
	  i915_syncmap.o \
	  i915_sw_fence.o \
	  intel_csr.o \
	  intel_device_info.o \
	  intel_pm.o \
	  intel_runtime_pm.o

# GEM code
i915-y += \
	  i915_gem_clflush.o \
	  i915_gem_evict.o \
	  i915_gem_fence_reg.o \
	  i915_gem_gtt.o \
	  i915_gem_internal.o \
	  i915_gem.o \
	  i915_gem_object.o \
	  i915_gem_shrinker.o \
	  i915_gem_stolen.o \
	  i915_gem_tiling.o \
	  i915_gem_timeline.o \
	  i915_request.o \
	  i915_trace_points.o \
	  i915_vma.o \
	  intel_engine_cs.o \
	  intel_hangcheck.o \
	  intel_lrc.o \
	  intel_mocs.o \
	  intel_ringbuffer.o \
	  intel_uncore.o

# modesetting core code
i915-y +=  \
	  intel_atomic.o \
	  intel_atomic_plane.o \
	  intel_bios.o \
	  intel_cdclk.o \
	  intel_color.o \
	  intel_display.o \
	  intel_dpio_phy.o \
	  intel_dpll_mgr.o \
	  intel_fbc.o \
	  intel_frontbuffer.o \
	  intel_hdcp.o \
	  intel_hotplug.o \
	  intel_modes.o \
	  intel_overlay.o \
	  intel_sideband.o \
i915-$(CONFIG_DRM_FBDEV_EMULATION)	+= intel_fbdev.o

# modesetting output/encoder code
i915-y += \
	  intel_crt.o \
	  intel_ddi.o \
	  intel_hdmi.o \
	  intel_i2c.o \
	  intel_panel.o \


obj-$(CONFIG_DRM_I915) += i915.o
