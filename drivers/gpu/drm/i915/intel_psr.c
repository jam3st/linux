#include <drm/drmP.h>

#include "intel_drv.h"
#include "i915_drv.h"

void intel_psr_flush(struct drm_i915_private *dev_priv,
		     unsigned frontbuffer_bits, enum fb_op_origin origin)
{
	struct drm_crtc *crtc;
	enum pipe pipe;

	if (!CAN_PSR(dev_priv))
		return;

}
void intel_psr_disable(struct intel_dp *intel_dp,
               const struct intel_crtc_state *old_crtc_state)
{
    struct intel_digital_port *intel_dig_port = dp_to_dig_port(intel_dp);
    struct drm_device *dev = intel_dig_port->base.base.dev;
    struct drm_i915_private *dev_priv = to_i915(dev);

    if (!old_crtc_state->has_psr)
        return;

    if (WARN_ON(!CAN_PSR(dev_priv)))
        return;

}

void intel_psr_invalidate(struct drm_i915_private *dev_priv,
              unsigned frontbuffer_bits)
{
    struct drm_crtc *crtc;
    enum pipe pipe;

    if (!CAN_PSR(dev_priv))
        return;

}



/**
 * intel_psr_init - Init basic PSR work and mutex.
 * @dev_priv: i915 device private
 *
 * This function is  called only once at driver load to initialize basic
 * PSR stuff.
 */
void intel_psr_init(struct drm_i915_private *dev_priv)
{
	if (!HAS_PSR(dev_priv))
		return;

	dev_priv->psr_mmio_base = IS_HASWELL(dev_priv) ?
		HSW_EDP_PSR_BASE : BDW_EDP_PSR_BASE;

	if (!dev_priv->psr.sink_support)
		return;

}

