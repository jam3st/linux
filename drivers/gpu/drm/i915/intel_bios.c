/*
 * Copyright © 2006 Intel Corporation
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Eric Anholt <eric@anholt.net>
 *
 */

#include <drm/drm_dp_helper.h>
#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include "i915_drv.h"

#define _INTEL_BIOS_PRIVATE
#include "intel_vbt_defs.h"

/**
 * DOC: Video BIOS Table (VBT)
 *
 * The Video BIOS Table, or VBT, provides platform and board specific
 * configuration information to the driver that is not discoverable or available
 * through other means. The configuration is mostly related to display
 * hardware. The VBT is available via the ACPI OpRegion or, on older systems, in
 * the PCI ROM.
 *
 * The VBT consists of a VBT Header (defined as &struct vbt_header), a BDB
 * Header (&struct bdb_header), and a number of BIOS Data Blocks (BDB) that
 * contain the actual configuration information. The VBT Header, and thus the
 * VBT, begins with "$VBT" signature. The VBT Header contains the offset of the
 * BDB Header. The data blocks are concatenated after the BDB Header. The data
 * blocks have a 1-byte Block ID, 2-byte Block Size, and Block Size bytes of
 * data. (Block 53, the MIPI Sequence Block is an exception.)
 *
 * The driver parses the VBT during load. The relevant information is stored in
 * driver private data for ease of use, and the actual VBT is not read after
 * that.
 */

#define	SLAVE_ADDR1	0x70
#define	SLAVE_ADDR2	0x72

/* Get BDB block size given a pointer to Block ID. */
static u32 _get_blocksize(const u8 *block_base)
{
    /* The MIPI Sequence Block v3+ has a separate size field. */
    if (*block_base == BDB_MIPI_SEQUENCE && *(block_base + 3) >= 3)
        return *((const u32 *)(block_base + 4));
    else
        return *((const u16 *)(block_base + 1));
}




/* Defaults to initialize only if there is no VBT. */
static void
init_vbt_missing_defaults(struct drm_i915_private *dev_priv)
{
    enum port port;

    for (port = PORT_A; port < I915_MAX_PORTS; port++) {
        struct ddi_vbt_port_info *info =
            &dev_priv->vbt.ddi_port_info[port];

        info->supports_dvi = (port != PORT_A && port != PORT_E);
        info->supports_hdmi = info->supports_dvi;
        info->supports_dp = false;
    }
}



/**
 * intel_bios_init - find VBT and initialize settings from the BIOS
 * @dev_priv: i915 device instance
 *
 * Parse and initialize settings from the Video BIOS Tables (VBT). If the VBT
 * was not found in ACPI OpRegion, try to find it in PCI ROM first. Also
 * initialize some defaults if the VBT is not present at all.
 */
void intel_bios_init(struct drm_i915_private *dev_priv)
{
    init_vbt_missing_defaults(dev_priv);
}



/**
 * intel_bios_is_port_present - is the specified digital port present
 * @dev_priv:	i915 device instance
 * @port:	port to check
 *
 * Return true if the device in %port is present.
 */
bool intel_bios_is_port_present(struct drm_i915_private *dev_priv, enum port port)
{
    const struct child_device_config *child;
    static const struct {
        u16 dp, hdmi;
    } port_mapping[] = {
        [PORT_B] = { DVO_PORT_DPB, DVO_PORT_HDMIB, },
        [PORT_C] = { DVO_PORT_DPC, DVO_PORT_HDMIC, },
        [PORT_D] = { DVO_PORT_DPD, DVO_PORT_HDMID, },
        [PORT_E] = { DVO_PORT_DPE, DVO_PORT_HDMIE, },
        [PORT_F] = { DVO_PORT_DPF, DVO_PORT_HDMIF, },
    };
    int i;

    /* FIXME maybe deal with port A as well? */
    if (WARN_ON(port == PORT_A) || port >= ARRAY_SIZE(port_mapping))
        return false;

    if (!dev_priv->vbt.child_dev_num)
        return false;

    for (i = 0; i < dev_priv->vbt.child_dev_num; i++) {
        child = dev_priv->vbt.child_dev + i;

        if ((child->dvo_port == port_mapping[port].dp ||
             child->dvo_port == port_mapping[port].hdmi) &&
            (child->device_type & (DEVICE_TYPE_TMDS_DVI_SIGNALING |
                       DEVICE_TYPE_DISPLAYPORT_OUTPUT)))
            return true;
    }

    return false;
}

/**
 * intel_bios_is_port_edp - is the device in given port eDP
 * @dev_priv:	i915 device instance
 * @port:	port to check
 *
 * Return true if the device in %port is eDP.
 */
bool intel_bios_is_port_edp(struct drm_i915_private *dev_priv, enum port port)
{
    const struct child_device_config *child;
    static const short port_mapping[] = {
        [PORT_B] = DVO_PORT_DPB,
        [PORT_C] = DVO_PORT_DPC,
        [PORT_D] = DVO_PORT_DPD,
        [PORT_E] = DVO_PORT_DPE,
        [PORT_F] = DVO_PORT_DPF,
    };
    int i;

    if (HAS_DDI(dev_priv))
        return dev_priv->vbt.ddi_port_info[port].supports_edp;

    if (!dev_priv->vbt.child_dev_num)
        return false;

    for (i = 0; i < dev_priv->vbt.child_dev_num; i++) {
        child = dev_priv->vbt.child_dev + i;

        if (child->dvo_port == port_mapping[port] &&
            (child->device_type & DEVICE_TYPE_eDP_BITS) ==
            (DEVICE_TYPE_eDP & DEVICE_TYPE_eDP_BITS))
            return true;
    }

    return false;
}
