
#include "i915_drv.h"

#include "intel_vbt_defs.h"



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
