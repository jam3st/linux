# SPDX-License-Identifier: GPL-2.0

# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

drm-y       :=	\
		drm_gem.o drm_irq.o \
		drm_drv.o \
		drm_mm.o \
		drm_crtc.o drm_fourcc.o drm_modes.o drm_edid.o \
		drm_trace_points.o drm_prime.o \
		drm_rect.o \
		drm_modeset_lock.o drm_atomic.o drm_bridge.o \
		drm_framebuffer.o drm_connector.o drm_blend.o \
		drm_encoder.o drm_mode_object.o drm_property.o \
		drm_plane.o drm_color_mgmt.o drm_print.o \
		drm_mode_config.o drm_vblank.o \
		drm_syncobj.o

drm-$(CONFIG_DRM_VM) += drm_vm.o
drm-$(CONFIG_COMPAT) += drm_ioc32.o
drm-$(CONFIG_DRM_GEM_CMA_HELPER) += drm_gem_cma_helper.o
drm-$(CONFIG_OF) += drm_of.o
drm-$(CONFIG_DRM_LOAD_EDID_FIRMWARE) += drm_edid_load.o

drm_kms_helper-y :=  drm_probe_helper.o \
		drm_atomic_helper.o \
		drm_kms_helper_common.o drm_dp_dual_mode_helper.o \
		drm_modeset_helper.o

drm_kms_helper-$(CONFIG_DRM_FBDEV_EMULATION) += drm_fb_helper.o
drm_kms_helper-$(CONFIG_DRM_KMS_CMA_HELPER) += drm_fb_cma_helper.o
drm_kms_helper-$(CONFIG_DRM_DP_AUX_CHARDEV) += drm_dp_aux_dev.o

obj-$(CONFIG_DRM_KMS_HELPER) += drm_kms_helper.o
obj-$(CONFIG_DRM_DEBUG_MM_SELFTEST) += selftests/

obj-$(CONFIG_DRM)	+= drm.o
obj-$(CONFIG_DRM_I915)	+= i915/
