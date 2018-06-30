
#include <drm/drmP.h>
#include <drm/drm_hdcp.h>
#include <linux/i2c.h>
#include <linux/random.h>

#include "intel_drv.h"
#include "i915_reg.h"

void intel_hdcp_atomic_check(struct drm_connector *connector,
                             struct drm_connector_state *old_state,
                             struct drm_connector_state *new_state)
{
        uint64_t old_cp = old_state->content_protection;
        uint64_t new_cp = new_state->content_protection;
        struct drm_crtc_state *crtc_state;

        if (!new_state->crtc) {
                /*
                 * If the connector is being disabled with CP enabled, mark it
                 * desired so it's re-enabled when the connector is brought back
                 */
                if (old_cp == DRM_MODE_CONTENT_PROTECTION_ENABLED)
                        new_state->content_protection =
                                DRM_MODE_CONTENT_PROTECTION_DESIRED;
                return;
        }

        /*
         * Nothing to do if the state didn't change, or HDCP was activated since
         * the last commit
         */
        if (old_cp == new_cp ||
            (old_cp == DRM_MODE_CONTENT_PROTECTION_DESIRED &&
             new_cp == DRM_MODE_CONTENT_PROTECTION_ENABLED))
                return;

        crtc_state = drm_atomic_get_new_crtc_state(new_state->state,
                                                   new_state->crtc);
        crtc_state->mode_changed = true;
}
