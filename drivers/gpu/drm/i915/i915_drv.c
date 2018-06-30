/* i915_drv.c -- i830,i845,i855,i865,i915 driver -*- linux-c -*-
 */
/*
 *
 * Copyright 2003 Tungsten Graphics, Inc., Cedar Park, Texas.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <linux/device.h>
#include <linux/oom.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/pnp.h>
#include <linux/slab.h>
#include <linux/vgaarb.h>
#include <linux/vt.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/i915_drm.h>

#include "i915_drv.h"
#include "i915_trace.h"
#include "i915_pmu.h"
#include "intel_drv.h"

static struct drm_driver driver;


#define FDO_BUG_URL "https://bugs.freedesktop.org/enter_bug.cgi?product=DRI"
#define FDO_BUG_MSG "Please file a bug at " FDO_BUG_URL " against DRM/Intel " \
		    "providing the dmesg log by booting with drm.debug=0xf"

void
__i915_printk(struct drm_i915_private *dev_priv, const char *level,
	      const char *fmt, ...)
{
	static bool shown_bug_once;
	struct device *kdev = dev_priv->drm.dev;
	bool is_error = level[1] <= KERN_ERR[1];
	bool is_debug = level[1] == KERN_DEBUG[1];
	struct va_format vaf;
	va_list args;

	if (is_debug && !(drm_debug & DRM_UT_DRIVER))
		return;

	va_start(args, fmt);

	vaf.fmt = fmt;
	vaf.va = &args;

	dev_printk(level, kdev, "[" DRM_NAME ":%ps] %pV",
		   __builtin_return_address(0), &vaf);

	if (is_error && !shown_bug_once) {
		dev_notice(kdev, "%s", FDO_BUG_MSG);
		shown_bug_once = true;
	}

	va_end(args);
}

static bool i915_error_injected(struct drm_i915_private *dev_priv)
{
#if IS_ENABLED(CONFIG_DRM_I915_DEBUG)
	return i915_modparams.inject_load_failure &&
	       i915_load_fail_count == i915_modparams.inject_load_failure;
#else
	return false;
#endif
}

#define i915_load_error(dev_priv, fmt, ...)				     \
	__i915_printk(dev_priv,						     \
		      i915_error_injected(dev_priv) ? KERN_DEBUG : KERN_ERR, \
		      fmt, ##__VA_ARGS__)

/* Map PCH device id to PCH type, or PCH_NONE if unknown. */
static enum intel_pch
intel_pch_type(const struct drm_i915_private *dev_priv, unsigned short id)
{
	switch (id) {
	case INTEL_PCH_IBX_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found Ibex Peak PCH\n");
		WARN_ON(!IS_GEN5(dev_priv));
		return PCH_IBX;
	case INTEL_PCH_CPT_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found CougarPoint PCH\n");
		WARN_ON(!IS_GEN6(dev_priv) && !IS_IVYBRIDGE(dev_priv));
		return PCH_CPT;
	case INTEL_PCH_PPT_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found PantherPoint PCH\n");
		WARN_ON(!IS_GEN6(dev_priv) && !IS_IVYBRIDGE(dev_priv));
		/* PantherPoint is CPT compatible */
		return PCH_CPT;
	case INTEL_PCH_LPT_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found LynxPoint PCH\n");
		WARN_ON(!IS_HASWELL(dev_priv) && !IS_BROADWELL(dev_priv));
		WARN_ON(IS_HSW_ULT(dev_priv) || IS_BDW_ULT(dev_priv));
		return PCH_LPT;
	case INTEL_PCH_LPT_LP_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found LynxPoint LP PCH\n");
		WARN_ON(!IS_HASWELL(dev_priv) && !IS_BROADWELL(dev_priv));
		WARN_ON(!IS_HSW_ULT(dev_priv) && !IS_BDW_ULT(dev_priv));
		return PCH_LPT;
	case INTEL_PCH_WPT_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found WildcatPoint PCH\n");
		WARN_ON(!IS_HASWELL(dev_priv) && !IS_BROADWELL(dev_priv));
		WARN_ON(IS_HSW_ULT(dev_priv) || IS_BDW_ULT(dev_priv));
		/* WildcatPoint is LPT compatible */
		return PCH_LPT;
	case INTEL_PCH_WPT_LP_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found WildcatPoint LP PCH\n");
		WARN_ON(!IS_HASWELL(dev_priv) && !IS_BROADWELL(dev_priv));
		WARN_ON(!IS_HSW_ULT(dev_priv) && !IS_BDW_ULT(dev_priv));
		/* WildcatPoint is LPT compatible */
		return PCH_LPT;
	case INTEL_PCH_SPT_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found SunrisePoint PCH\n");
		WARN_ON(!IS_SKYLAKE(dev_priv) && !IS_KABYLAKE(dev_priv));
		return PCH_SPT;
	case INTEL_PCH_SPT_LP_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found SunrisePoint LP PCH\n");
		WARN_ON(!IS_SKYLAKE(dev_priv) && !IS_KABYLAKE(dev_priv));
		return PCH_SPT;
	case INTEL_PCH_KBP_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found Kaby Lake PCH (KBP)\n");
		WARN_ON(!IS_SKYLAKE(dev_priv) && !IS_KABYLAKE(dev_priv) &&
			!IS_COFFEELAKE(dev_priv));
		return PCH_KBP;
	case INTEL_PCH_CNP_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found Cannon Lake PCH (CNP)\n");
		WARN_ON(!IS_CANNONLAKE(dev_priv) && !IS_COFFEELAKE(dev_priv));
		return PCH_CNP;
	case INTEL_PCH_CNP_LP_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found Cannon Lake LP PCH (CNP-LP)\n");
		WARN_ON(!IS_CANNONLAKE(dev_priv) && !IS_COFFEELAKE(dev_priv));
		return PCH_CNP;
	case INTEL_PCH_ICP_DEVICE_ID_TYPE:
		DRM_DEBUG_KMS("Found Ice Lake PCH\n");
		WARN_ON(!IS_ICELAKE(dev_priv));
		return PCH_ICP;
	default:
		return PCH_NONE;
	}
}

static bool intel_is_virt_pch(unsigned short id,
			      unsigned short svendor, unsigned short sdevice)
{
	return (id == INTEL_PCH_P2X_DEVICE_ID_TYPE ||
		id == INTEL_PCH_P3X_DEVICE_ID_TYPE ||
		(id == INTEL_PCH_QEMU_DEVICE_ID_TYPE &&
		 svendor == PCI_SUBVENDOR_ID_REDHAT_QUMRANET &&
		 sdevice == PCI_SUBDEVICE_ID_QEMU));
}

static unsigned short
intel_virt_detect_pch(const struct drm_i915_private *dev_priv)
{
	unsigned short id = 0;

	/*
	 * In a virtualized passthrough environment we can be in a
	 * setup where the ISA bridge is not able to be passed through.
	 * In this case, a south bridge can be emulated and we have to
	 * make an educated guess as to which PCH is really there.
	 */

	if (IS_GEN5(dev_priv))
		id = INTEL_PCH_IBX_DEVICE_ID_TYPE;
	else if (IS_GEN6(dev_priv) || IS_IVYBRIDGE(dev_priv))
		id = INTEL_PCH_CPT_DEVICE_ID_TYPE;
	else if (IS_HSW_ULT(dev_priv) || IS_BDW_ULT(dev_priv))
		id = INTEL_PCH_LPT_LP_DEVICE_ID_TYPE;
	else if (IS_HASWELL(dev_priv) || IS_BROADWELL(dev_priv))
		id = INTEL_PCH_LPT_DEVICE_ID_TYPE;
	else if (IS_SKYLAKE(dev_priv) || IS_KABYLAKE(dev_priv))
		id = INTEL_PCH_SPT_DEVICE_ID_TYPE;
	else if (IS_COFFEELAKE(dev_priv) || IS_CANNONLAKE(dev_priv))
		id = INTEL_PCH_CNP_DEVICE_ID_TYPE;

	if (id)
		DRM_DEBUG_KMS("Assuming PCH ID %04x\n", id);
	else
		DRM_DEBUG_KMS("Assuming no PCH\n");

	return id;
}

static void intel_detect_pch(struct drm_i915_private *dev_priv)
{
	struct pci_dev *pch = NULL;

	/* In all current cases, num_pipes is equivalent to the PCH_NOP setting
	 * (which really amounts to a PCH but no South Display).
	 */
	if (INTEL_INFO(dev_priv)->num_pipes == 0) {
		dev_priv->pch_type = PCH_NOP;
		return;
	}

	/*
	 * The reason to probe ISA bridge instead of Dev31:Fun0 is to
	 * make graphics device passthrough work easy for VMM, that only
	 * need to expose ISA bridge to let driver know the real hardware
	 * underneath. This is a requirement from virtualization team.
	 *
	 * In some virtualized environments (e.g. XEN), there is irrelevant
	 * ISA bridge in the system. To work reliably, we should scan trhough
	 * all the ISA bridge devices and check for the first match, instead
	 * of only checking the first one.
	 */
	while ((pch = pci_get_class(PCI_CLASS_BRIDGE_ISA << 8, pch))) {
		unsigned short id;
		enum intel_pch pch_type;

		if (pch->vendor != PCI_VENDOR_ID_INTEL)
			continue;

		id = pch->device & INTEL_PCH_DEVICE_ID_MASK;

		pch_type = intel_pch_type(dev_priv, id);
		if (pch_type != PCH_NONE) {
			dev_priv->pch_type = pch_type;
			dev_priv->pch_id = id;
			break;
		} else if (intel_is_virt_pch(id, pch->subsystem_vendor,
					 pch->subsystem_device)) {
			id = intel_virt_detect_pch(dev_priv);
			if (id) {
				pch_type = intel_pch_type(dev_priv, id);
				if (WARN_ON(pch_type == PCH_NONE))
					pch_type = PCH_NOP;
			} else {
				pch_type = PCH_NOP;
			}
			dev_priv->pch_type = pch_type;
			dev_priv->pch_id = id;
			break;
		}
	}
	if (!pch)
		DRM_DEBUG_KMS("No PCH found.\n");

	pci_dev_put(pch);
}


static int i915_get_bridge_dev(struct drm_i915_private *dev_priv)
{
	int domain = pci_domain_nr(dev_priv->drm.pdev->bus);

	dev_priv->bridge_dev =
		pci_get_domain_bus_and_slot(domain, 0, PCI_DEVFN(0, 0));
	if (!dev_priv->bridge_dev) {
		DRM_ERROR("bridge device not found\n");
		return -1;
	}
	return 0;
}

/* Allocate space for the MCH regs if needed, return nonzero on error */
static int
intel_alloc_mchbar_resource(struct drm_i915_private *dev_priv)
{
	int reg = INTEL_GEN(dev_priv) >= 4 ? MCHBAR_I965 : MCHBAR_I915;
	u32 temp_lo, temp_hi = 0;
	u64 mchbar_addr;
	int ret;

	if (INTEL_GEN(dev_priv) >= 4)
		pci_read_config_dword(dev_priv->bridge_dev, reg + 4, &temp_hi);
	pci_read_config_dword(dev_priv->bridge_dev, reg, &temp_lo);
	mchbar_addr = ((u64)temp_hi << 32) | temp_lo;

	/* If ACPI doesn't have it, assume we need to allocate it ourselves */
#ifdef CONFIG_PNP
	if (mchbar_addr &&
	    pnp_range_reserved(mchbar_addr, mchbar_addr + MCHBAR_SIZE))
		return 0;
#endif

	/* Get some space for it */
	dev_priv->mch_res.name = "i915 MCHBAR";
	dev_priv->mch_res.flags = IORESOURCE_MEM;
	ret = pci_bus_alloc_resource(dev_priv->bridge_dev->bus,
				     &dev_priv->mch_res,
				     MCHBAR_SIZE, MCHBAR_SIZE,
				     PCIBIOS_MIN_MEM,
				     0, pcibios_align_resource,
				     dev_priv->bridge_dev);
	if (ret) {
		DRM_DEBUG_DRIVER("failed bus alloc: %d\n", ret);
		dev_priv->mch_res.start = 0;
		return ret;
	}

	if (INTEL_GEN(dev_priv) >= 4)
		pci_write_config_dword(dev_priv->bridge_dev, reg + 4,
				       upper_32_bits(dev_priv->mch_res.start));

	pci_write_config_dword(dev_priv->bridge_dev, reg,
			       lower_32_bits(dev_priv->mch_res.start));
	return 0;
}

/* Setup MCHBAR if possible, return true if we should disable it again */
static void
intel_setup_mchbar(struct drm_i915_private *dev_priv)
{
	int mchbar_reg = INTEL_GEN(dev_priv) >= 4 ? MCHBAR_I965 : MCHBAR_I915;
	u32 temp;
	bool enabled;

	if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		return;

	dev_priv->mchbar_need_disable = false;

	if (IS_I915G(dev_priv) || IS_I915GM(dev_priv)) {
		pci_read_config_dword(dev_priv->bridge_dev, DEVEN, &temp);
		enabled = !!(temp & DEVEN_MCHBAR_EN);
	} else {
		pci_read_config_dword(dev_priv->bridge_dev, mchbar_reg, &temp);
		enabled = temp & 1;
	}

	/* If it's already enabled, don't have to do anything */
	if (enabled)
		return;

	if (intel_alloc_mchbar_resource(dev_priv))
		return;

	dev_priv->mchbar_need_disable = true;

	/* Space is allocated or reserved, so enable it. */
	if (IS_I915G(dev_priv) || IS_I915GM(dev_priv)) {
		pci_write_config_dword(dev_priv->bridge_dev, DEVEN,
				       temp | DEVEN_MCHBAR_EN);
	} else {
		pci_read_config_dword(dev_priv->bridge_dev, mchbar_reg, &temp);
		pci_write_config_dword(dev_priv->bridge_dev, mchbar_reg, temp | 1);
	}
}

static void
intel_teardown_mchbar(struct drm_i915_private *dev_priv)
{
	int mchbar_reg = INTEL_GEN(dev_priv) >= 4 ? MCHBAR_I965 : MCHBAR_I915;

	if (dev_priv->mchbar_need_disable) {
		if (IS_I915G(dev_priv) || IS_I915GM(dev_priv)) {
			u32 deven_val;

			pci_read_config_dword(dev_priv->bridge_dev, DEVEN,
					      &deven_val);
			deven_val &= ~DEVEN_MCHBAR_EN;
			pci_write_config_dword(dev_priv->bridge_dev, DEVEN,
					       deven_val);
		} else {
			u32 mchbar_val;

			pci_read_config_dword(dev_priv->bridge_dev, mchbar_reg,
					      &mchbar_val);
			mchbar_val &= ~1;
			pci_write_config_dword(dev_priv->bridge_dev, mchbar_reg,
					       mchbar_val);
		}
	}

	if (dev_priv->mch_res.start)
		release_resource(&dev_priv->mch_res);
}

/* true = enable decode, false = disable decoder */
static unsigned int i915_vga_set_decode(void *cookie, bool state)
{
	struct drm_i915_private *dev_priv = cookie;

	intel_modeset_vga_set_state(dev_priv, state);
	if (state)
		return VGA_RSRC_LEGACY_IO | VGA_RSRC_LEGACY_MEM |
		       VGA_RSRC_NORMAL_IO | VGA_RSRC_NORMAL_MEM;
	else
		return VGA_RSRC_NORMAL_IO | VGA_RSRC_NORMAL_MEM;
}


static void i915_gem_fini(struct drm_i915_private *dev_priv)
{
	/* Flush any outstanding unpin_work. */
	i915_gem_drain_workqueue(dev_priv);

	mutex_lock(&dev_priv->drm.struct_mutex);

	i915_gem_cleanup_engines(dev_priv);
	mutex_unlock(&dev_priv->drm.struct_mutex);

	i915_gem_drain_freed_objects(dev_priv);

	WARN_ON(!list_empty(&dev_priv->contexts.list));
}

static int i915_load_modeset_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = to_i915(dev);
	struct pci_dev *pdev = dev_priv->drm.pdev;
	int ret;



	intel_bios_init(dev_priv);

	/* If we have > 1 VGA cards, then we need to arbitrate access
	 * to the common VGA resources.
	 *
	 * If we are a secondary display controller (!PCI_DISPLAY_CLASS_VGA),
	 * then we do not take part in VGA arbitration and the
	 * vga_client_register() fails with -ENODEV.
	 */
//	ret = vga_client_register(pdev, dev_priv, NULL, i915_vga_set_decode);
//	if (ret && ret != -ENODEV)
//		goto out;


	/* must happen before intel_power_domains_init_hw() on VLV/CHV */
	intel_update_rawclk(dev_priv);

	intel_csr_ucode_init(dev_priv);

	ret = intel_irq_install(dev_priv);
	if (ret)
		goto cleanup_csr;

	intel_setup_gmbus(dev_priv);

	/* Important: The output setup functions called by modeset_init need
	 * working irqs for e.g. gmbus and dp aux transfers. */
	ret = intel_modeset_init(dev);
	if (ret)
		goto cleanup_irq;


//	ret = i915_gem_init(dev_priv);
//	if (ret)
//		goto cleanup_uc;

//	intel_setup_overlay(dev_priv);

	if (INTEL_INFO(dev_priv)->num_pipes == 0)
		return 0;

	ret = intel_fbdev_init(dev);
	if (ret)
		goto cleanup_gem;

	/* Only enable hotplug handling once the fbdev is fully set up. */
	intel_hpd_init(dev_priv);

	return 0;

cleanup_gem:

cleanup_uc:
cleanup_irq:
cleanup_csr:
out:
	return ret;
}

static int i915_kick_out_firmware_fb(struct drm_i915_private *dev_priv)
{
	struct apertures_struct *ap;
	struct pci_dev *pdev = dev_priv->drm.pdev;
	struct i915_ggtt *ggtt = &dev_priv->ggtt;
	bool primary;
	int ret;

	ap = alloc_apertures(1);
	if (!ap)
		return -ENOMEM;

	ap->ranges[0].base = ggtt->gmadr.start;
	ap->ranges[0].size = ggtt->mappable_end;

	primary =
		pdev->resource[PCI_ROM_RESOURCE].flags & IORESOURCE_ROM_SHADOW;

	ret = drm_fb_helper_remove_conflicting_framebuffers(ap, "inteldrmfb", primary);

	kfree(ap);

	return ret;
}


static void intel_init_dpio(struct drm_i915_private *dev_priv)
{
	/*
	 * IOSF_PORT_DPIO is used for VLV x2 PHY (DP/HDMI B and C),
	 * CHV x1 PHY (DP/HDMI D)
	 * IOSF_PORT_DPIO_2 is used for CHV x2 PHY (DP/HDMI B and C)
	 */
	if (IS_CHERRYVIEW(dev_priv)) {
		DPIO_PHY_IOSF_PORT(DPIO_PHY0) = IOSF_PORT_DPIO_2;
		DPIO_PHY_IOSF_PORT(DPIO_PHY1) = IOSF_PORT_DPIO;
	} else if (IS_VALLEYVIEW(dev_priv)) {
		DPIO_PHY_IOSF_PORT(DPIO_PHY0) = IOSF_PORT_DPIO;
	}
}

static int i915_workqueues_init(struct drm_i915_private *dev_priv)
{
	/*
	 * The i915 workqueue is primarily used for batched retirement of
	 * requests (and thus managing bo) once the task has been completed
	 * by the GPU. i915_retire_requests() is called directly when we
	 * need high-priority retirement, such as waiting for an explicit
	 * bo.
	 *
	 * It is also used for periodic low-priority events, such as
	 * idle-timers and recording error state.
	 *
	 * All tasks on the workqueue are expected to acquire the dev mutex
	 * so there is no point in running more than one instance of the
	 * workqueue at any time.  Use an ordered one.
	 */
	dev_priv->wq = alloc_ordered_workqueue("i915", 0);
	if (dev_priv->wq == NULL)
		goto out_err;

	dev_priv->hotplug.dp_wq = alloc_ordered_workqueue("i915-dp", 0);
	if (dev_priv->hotplug.dp_wq == NULL)
		goto out_free_wq;

	return 0;

out_free_wq:
	destroy_workqueue(dev_priv->wq);
out_err:
	DRM_ERROR("Failed to allocate workqueues.\n");

	return -ENOMEM;
}

static void i915_engines_cleanup(struct drm_i915_private *i915)
{
	struct intel_engine_cs *engine;
	enum intel_engine_id id;

	for_each_engine(engine, i915, id)
		kfree(engine);
}

static void i915_workqueues_cleanup(struct drm_i915_private *dev_priv)
{
	destroy_workqueue(dev_priv->hotplug.dp_wq);
	destroy_workqueue(dev_priv->wq);
}

/*
 * We don't keep the workarounds for pre-production hardware, so we expect our
 * driver to fail on these machines in one way or another. A little warning on
 * dmesg may help both the user and the bug triagers.
 *
 * Our policy for removing pre-production workarounds is to keep the
 * current gen workarounds as a guide to the bring-up of the next gen
 * (workarounds have a habit of persisting!). Anything older than that
 * should be removed along with the complications they introduce.
 */
static void intel_detect_preproduction_hw(struct drm_i915_private *dev_priv)
{
	bool pre = false;

	pre |= IS_HSW_EARLY_SDV(dev_priv);
	pre |= IS_SKL_REVID(dev_priv, 0, SKL_REVID_F0);
	pre |= IS_BXT_REVID(dev_priv, 0, BXT_REVID_B_LAST);

	if (pre) {
		DRM_ERROR("This is a pre-production stepping. "
			  "It may not be fully functional.\n");
		add_taint(TAINT_MACHINE_CHECK, LOCKDEP_STILL_OK);
	}
}

/**
 * i915_driver_init_early - setup state not requiring device access
 * @dev_priv: device private
 * @ent: the matching pci_device_id
 *
 * Initialize everything that is a "SW-only" state, that is state not
 * requiring accessing the device or exposing the driver via kernel internal
 * or userspace interfaces. Example steps belonging here: lock initialization,
 * system memory allocation, setting up device specific attributes and
 * function hooks not requiring accessing the device.
 */
static int i915_driver_init_early(struct drm_i915_private *dev_priv,
				  const struct pci_device_id *ent)
{
	const struct intel_device_info *match_info =
		(struct intel_device_info *)ent->driver_data;
	struct intel_device_info *device_info;
	int ret = 0;

	if (i915_inject_load_failure())
		return -ENODEV;

	/* Setup the write-once "constant" device info */
	device_info = mkwrite_device_info(dev_priv);
	memcpy(device_info, match_info, sizeof(*device_info));
	device_info->device_id = dev_priv->drm.pdev->device;

	BUILD_BUG_ON(INTEL_MAX_PLATFORMS >
		     sizeof(device_info->platform_mask) * BITS_PER_BYTE);
	BUG_ON(device_info->gen > sizeof(device_info->gen_mask) * BITS_PER_BYTE);
	spin_lock_init(&dev_priv->irq_lock);
	spin_lock_init(&dev_priv->gpu_error.lock);
	mutex_init(&dev_priv->backlight_lock);
	spin_lock_init(&dev_priv->uncore.lock);

	mutex_init(&dev_priv->sb_lock);
	mutex_init(&dev_priv->modeset_restore_lock);
	mutex_init(&dev_priv->av_mutex);
	mutex_init(&dev_priv->wm.wm_mutex);
	mutex_init(&dev_priv->pps_mutex);

	i915_memcpy_init_early(dev_priv);

	ret = i915_workqueues_init(dev_priv);
	if (ret < 0)
		goto err_engines;

	/* This must be called before any calls to HAS_PCH_* */
	intel_detect_pch(dev_priv);

	intel_pm_setup(dev_priv);
	intel_init_dpio(dev_priv);
	intel_power_domains_init(dev_priv);
	intel_irq_init(dev_priv);
	intel_init_display_hooks(dev_priv);
	intel_init_clock_gating_hooks(dev_priv);
    ret = i915_gem_load_init(dev_priv);
    if (ret < 0)
        goto err_irq;

	intel_display_crc_init(dev_priv);

	intel_detect_preproduction_hw(dev_priv);

	return 0;

err_irq:
err_engines:
	return ret;
}

static int i915_mmio_setup(struct drm_i915_private *dev_priv)
{
	struct pci_dev *pdev = dev_priv->drm.pdev;
	int mmio_bar;
	int mmio_size;

	mmio_bar = IS_GEN2(dev_priv) ? 1 : 0;
	/*
	 * Before gen4, the registers and the GTT are behind different BARs.
	 * However, from gen4 onwards, the registers and the GTT are shared
	 * in the same BAR, so we want to restrict this ioremap from
	 * clobbering the GTT which we want ioremap_wc instead. Fortunately,
	 * the register BAR remains the same size for all the earlier
	 * generations up to Ironlake.
	 */
	if (INTEL_GEN(dev_priv) < 5)
		mmio_size = 512 * 1024;
	else
		mmio_size = 2 * 1024 * 1024;
	dev_priv->regs = pci_iomap(pdev, mmio_bar, mmio_size);
	if (dev_priv->regs == NULL) {
		DRM_ERROR("failed to map registers\n");

		return -EIO;
	}

	/* Try to make sure MCHBAR is enabled before poking at it */
	intel_setup_mchbar(dev_priv);

	return 0;
}

static void i915_mmio_cleanup(struct drm_i915_private *dev_priv)
{
	struct pci_dev *pdev = dev_priv->drm.pdev;

	intel_teardown_mchbar(dev_priv);
	pci_iounmap(pdev, dev_priv->regs);
}

/**
 * i915_driver_init_mmio - setup device MMIO
 * @dev_priv: device private
 *
 * Setup minimal device state necessary for MMIO accesses later in the
 * initialization sequence. The setup here should avoid any other device-wide
 * side effects or exposing the driver via kernel internal or user space
 * interfaces.
 */
static int i915_driver_init_mmio(struct drm_i915_private *dev_priv)
{
	int ret;

	if (i915_inject_load_failure())
		return -ENODEV;

	if (i915_get_bridge_dev(dev_priv))
		return -EIO;

	ret = i915_mmio_setup(dev_priv);
	if (ret < 0)
		goto err_bridge;

	intel_uncore_init(dev_priv);


//	ret = intel_engines_init_mmio(dev_priv);
//	if (ret)
//		goto err_uncore;

//	i915_gem_init_mmio(dev_priv);

	return 0;

err_uncore:
	intel_uncore_fini(dev_priv);
err_bridge:
	pci_dev_put(dev_priv->bridge_dev);

	return ret;
}

/**
 * i915_driver_cleanup_mmio - cleanup the setup done in i915_driver_init_mmio()
 * @dev_priv: device private
 */
static void i915_driver_cleanup_mmio(struct drm_i915_private *dev_priv)
{
	intel_uncore_fini(dev_priv);
	i915_mmio_cleanup(dev_priv);
	pci_dev_put(dev_priv->bridge_dev);
}

static void intel_sanitize_options(struct drm_i915_private *dev_priv)
{
	/*
	 * i915.enable_ppgtt is read-only, so do an early pass to validate the
	 * user's requested state against the hardware/driver capabilities.  We
	 * do this now so that we can print out any log messages once rather
	 * than every time we check intel_enable_ppgtt().
	 */
	i915_modparams.enable_ppgtt =
		intel_sanitize_enable_ppgtt(dev_priv,
					    i915_modparams.enable_ppgtt);
	DRM_DEBUG_DRIVER("ppgtt mode: %i\n", i915_modparams.enable_ppgtt);

}

/**
 * i915_driver_init_hw - setup state requiring device access
 * @dev_priv: device private
 *
 * Setup state that requires accessing the device, but doesn't require
 * exposing the driver via kernel internal or userspace interfaces.
 */
static int i915_driver_init_hw(struct drm_i915_private *dev_priv)
{
	struct pci_dev *pdev = dev_priv->drm.pdev;
	int ret;

	if (i915_inject_load_failure())
		return -ENODEV;

	intel_device_info_runtime_init(mkwrite_device_info(dev_priv));

	intel_sanitize_options(dev_priv);


	ret = i915_ggtt_probe_hw(dev_priv);

	/*
	 * WARNING: Apparently we must kick fbdev drivers before vgacon,
	 * otherwise the vga fbdev driver falls over.
	 */
	ret = i915_kick_out_firmware_fb(dev_priv);
	if (ret) {
		DRM_ERROR("failed to remove conflicting framebuffer drivers\n");
		goto err_ggtt;
	}


	ret = i915_ggtt_init_hw(dev_priv);
	if (ret)
		goto err_ggtt;

	ret = i915_ggtt_enable_hw(dev_priv);
	if (ret) {
		DRM_ERROR("failed to enable GGTT\n");
		goto err_ggtt;
	}

	pci_set_master(pdev);

	/* overlay on gen2 is broken and can't address above 1G */

	/* 965GM sometimes incorrectly writes to hardware status page (HWS)
	 * using 32bit addressing, overwriting memory if HWS is located
	 * above 4GB.
	 *
	 * The documentation also mentions an issue with undefined
	 * behaviour if any general state is accessed within a page above 4GB,
	 * which also needs to be handled carefully.
	 */
	if (IS_I965G(dev_priv) || IS_I965GM(dev_priv)) {
		ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));

		if (ret) {
			DRM_ERROR("failed to set DMA mask\n");

			goto err_ggtt;
		}
	}

	pm_qos_add_request(&dev_priv->pm_qos, PM_QOS_CPU_DMA_LATENCY,
			   PM_QOS_DEFAULT_VALUE);

	intel_uncore_sanitize(dev_priv);


	i915_gem_load_init_fences(dev_priv);

	/* On the 945G/GM, the chipset reports the MSI capability on the
	 * integrated graphics even though the support isn't actually there
	 * according to the published specs.  It doesn't appear to function
	 * correctly in testing on 945G.
	 * This may be a side effect of MSI having been made available for PEG
	 * and the registers being closely associated.
	 *
	 * According to chipset errata, on the 965GM, MSI interrupts may
	 * be lost or delayed, and was defeatured. MSI interrupts seem to
	 * get lost on g4x as well, and interrupt delivery seems to stay
	 * properly dead afterwards. So we'll just disable them for all
	 * pre-gen5 chipsets.
	 */
	if (INTEL_GEN(dev_priv) >= 5) {
		if (pci_enable_msi(pdev) < 0)
			DRM_DEBUG_DRIVER("can't enable MSI");
	}

printk("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx forked");
	return 0;

err_ggtt:

err_perf:
	return ret;
}


/**
 * i915_driver_register - register the driver with the rest of the system
 * @dev_priv: device private
 *
 * Perform any steps necessary to make the driver available via kernel
 * internal or userspace interfaces.
 */
static void i915_driver_register(struct drm_i915_private *dev_priv)
{
	struct drm_device *dev = &dev_priv->drm;


	/* Reveal our presence to userspace */
	if (drm_dev_register(dev, 0) == 0) {


		/* Depends on sysfs having been initialized */
	} else
		DRM_ERROR("Failed to register driver for userspace access!\n");



	/*
	 * Some ports require correctly set-up hpd registers for detection to
	 * work properly (leading to ghost connected connector status), e.g. VGA
	 * on gm45.  Hence we can only set up the initial fbdev config after hpd
	 * irqs are fully enabled. We do it last so that the async config
	 * cannot run before the connectors are registered.
	 */
	intel_fbdev_initial_config_async(dev);

	/*
	 * We need to coordinate the hotplugs with the asynchronous fbdev
	 * configuration, for which we use the fbdev->async_cookie.
	 */
	if (INTEL_INFO(dev_priv)->num_pipes)
		drm_kms_helper_poll_init(dev);
}

/**
 * i915_driver_unregister - cleanup the registration done in i915_driver_regiser()
 * @dev_priv: device private
 */
static void i915_driver_unregister(struct drm_i915_private *dev_priv)
{
	intel_fbdev_unregister(dev_priv);

	/*
	 * After flushing the fbdev (incl. a late async config which will
	 * have delayed queuing of a hotplug event), then flush the hotplug
	 * events.
	 */
	drm_kms_helper_poll_fini(&dev_priv->drm);

	intel_gpu_ips_teardown();


	drm_dev_unregister(&dev_priv->drm);

	i915_gem_shrinker_unregister(dev_priv);
}

/**
 * i915_driver_load - setup chip and create an initial config
 * @pdev: PCI device
 * @ent: matching PCI ID entry
 *
 * The driver load routine has to do several things:
 *   - drive output discovery via intel_modeset_init()
 *   - initialize the memory manager
 *   - allocate initial config memory
 *   - setup the DRM framebuffer with the allocated memory
 */
int i915_driver_load(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	const struct intel_device_info *match_info =
		(struct intel_device_info *)ent->driver_data;
	struct drm_i915_private *dev_priv;
	int ret;

	/* Enable nuclear pageflip on ILK+ */
	if (!i915_modparams.nuclear_pageflip && match_info->gen < 5)
		driver.driver_features &= ~DRIVER_ATOMIC;

	ret = -ENOMEM;
	dev_priv = kzalloc(sizeof(*dev_priv), GFP_KERNEL);
	if (dev_priv)
		ret = drm_dev_init(&dev_priv->drm, &driver, &pdev->dev);
	if (ret) {
		DRM_DEV_ERROR(&pdev->dev, "allocation failed\n");
		goto out_free;
	}

	dev_priv->drm.pdev = pdev;
	dev_priv->drm.dev_private = dev_priv;

	ret = pci_enable_device(pdev);
	if (ret)
		goto out_fini;

	pci_set_drvdata(pdev, &dev_priv->drm);
	/*
	 * Disable the system suspend direct complete optimization, which can
	 * leave the device suspended skipping the driver's suspend handlers
	 * if the device was already runtime suspended. This is needed due to
	 * the difference in our runtime and system suspend sequence and
	 * becaue the HDA driver may require us to enable the audio power
	 * domain during system suspend.
	 */
	dev_pm_set_driver_flags(&pdev->dev, DPM_FLAG_NEVER_SKIP);

	ret = i915_driver_init_early(dev_priv, ent);
	if (ret < 0)
		goto out_pci_disable;

	ret = i915_driver_init_mmio(dev_priv);
	if (ret < 0) {

        printk(" MMIO FAULED TO LOAD");
        }
	ret = i915_driver_init_hw(dev_priv);
	if (ret < 0)
		goto out_cleanup_mmio;

	/*
	 * TODO: move the vblank init and parts of modeset init steps into one
	 * of the i915_driver_init_/i915_driver_register functions according
	 * to the role/effect of the given init step.
	 */
	if (INTEL_INFO(dev_priv)->num_pipes) {
		ret = drm_vblank_init(&dev_priv->drm,
				      INTEL_INFO(dev_priv)->num_pipes);
		if (ret)
			goto out_cleanup_hw;
	}
	ret = i915_load_modeset_init(&dev_priv->drm);
	if (ret < 0)
		goto out_cleanup_hw;

	i915_driver_register(dev_priv);


	intel_init_ipc(dev_priv);


	return 0;

out_cleanup_hw:
out_cleanup_mmio:
	i915_driver_cleanup_mmio(dev_priv);
out_runtime_pm_put:
out_pci_disable:
	pci_disable_device(pdev);
out_fini:
	i915_load_error(dev_priv, "Device initialization failed (%d)\n", ret);
	drm_dev_fini(&dev_priv->drm);
out_free:
	kfree(dev_priv);
	return ret;
}

void i915_driver_unload(struct drm_device *dev)
{
}


static int vlv_resume_prepare(struct drm_i915_private *dev_priv,
			      bool rpm_resume);
static int vlv_suspend_complete(struct drm_i915_private *dev_priv);

static bool suspend_to_idle(struct drm_i915_private *dev_priv)
{
#if IS_ENABLED(CONFIG_ACPI_SLEEP)
	if (acpi_target_system_state() < ACPI_STATE_S3)
		return true;
#endif
	return false;
}


/**
 * i915_reset - reset chip after a hang
 * @i915: #drm_i915_private to reset
 * @flags: Instructions
 *
 * Reset the chip.  Useful if a hang is detected. Marks the device as wedged
 * on failure.
 *
 * Caller must hold the struct_mutex.
 *
 * Procedure is fairly simple:
 *   - reset the chip using the reset reg
 *   - re-init context state
 *   - re-init hardware status page
 *   - re-init ring buffer
 *   - re-init interrupt state
 *   - re-init display
 */
void i915_reset(struct drm_i915_private *i915, unsigned int flags)
{

}



static const struct vm_operations_struct i915_gem_vm_ops = {
    .fault = NULL,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations i915_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = drm_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.compat_ioctl = i915_compat_ioctl,
	.llseek = noop_llseek,
};


static const struct drm_ioctl_desc i915_ioctls[] = {

};

static struct drm_driver driver = {
	/* Don't use MTRRs here; the Xserver or userspace app should
	 * deal with them for Intel hardware.
	 */
	.driver_features =
	    DRIVER_HAVE_IRQ | DRIVER_IRQ_SHARED | DRIVER_GEM | DRIVER_PRIME |
	    DRIVER_RENDER | DRIVER_MODESET | DRIVER_ATOMIC | DRIVER_SYNCOBJ,
    .release = NULL,
    .open = NULL,
    .lastclose = NULL,
    .postclose = NULL,

    .gem_close_object = NULL,
    .gem_free_object_unlocked = NULL,
	.gem_vm_ops = &i915_gem_vm_ops,

    .prime_handle_to_fd = NULL,
    .prime_fd_to_handle = NULL,
    .gem_prime_export = NULL,
    .gem_prime_import = NULL,

    .dumb_create = NULL,
    .dumb_map_offset = NULL,
    .ioctls = i915_ioctls,
	.num_ioctls = ARRAY_SIZE(i915_ioctls),
    .fops = NULL,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

#if IS_ENABLED(CONFIG_DRM_I915_SELFTEST)
#include "selftests/mock_drm.c"
#endif
