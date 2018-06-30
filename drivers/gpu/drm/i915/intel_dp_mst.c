/*
 * Copyright © 2008 Intel Corporation
 *             2014 Red Hat Inc.
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

#include <drm/drmP.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>


static bool intel_dp_mst_enc_get_hw_state(struct intel_encoder *encoder,
				      enum pipe *pipe)
{
	struct intel_dp_mst_encoder *intel_mst = enc_to_mst(&encoder->base);
	*pipe = intel_mst->pipe;
	if (intel_mst->connector)
		return true;
	return false;
}
 static void intel_dp_mst_encoder_destroy(struct drm_encoder *encoder)
 {
         struct intel_dp_mst_encoder *intel_mst = enc_to_mst(encoder);

         drm_encoder_cleanup(encoder);
         kfree(intel_mst);
 }

 static const struct drm_encoder_funcs intel_dp_mst_enc_funcs = {
         .destroy = intel_dp_mst_encoder_destroy,
 };

static struct intel_dp_mst_encoder *
intel_dp_create_fake_mst_encoder(struct intel_digital_port *intel_dig_port, enum pipe pipe)
{
	struct intel_dp_mst_encoder *intel_mst;
	struct intel_encoder *intel_encoder;
	struct drm_device *dev = intel_dig_port->base.base.dev;

	intel_mst = kzalloc(sizeof(*intel_mst), GFP_KERNEL);

	if (!intel_mst)
		return NULL;

	intel_mst->pipe = pipe;
	intel_encoder = &intel_mst->base;
	intel_mst->primary = intel_dig_port;

	drm_encoder_init(dev, &intel_encoder->base, &intel_dp_mst_enc_funcs,
			 DRM_MODE_ENCODER_DPMST, "DP-MST %c", pipe_name(pipe));

	intel_encoder->type = INTEL_OUTPUT_DP_MST;
	intel_encoder->power_domain = intel_dig_port->base.power_domain;
	intel_encoder->port = intel_dig_port->base.port;
	intel_encoder->crtc_mask = 0x7;
	intel_encoder->cloneable = 0;

    intel_encoder->compute_config = NULL;
    intel_encoder->disable = NULL;
    intel_encoder->post_disable = NULL;
    intel_encoder->pre_pll_enable = NULL;
    intel_encoder->pre_enable = NULL;
    intel_encoder->enable = NULL;
	intel_encoder->get_hw_state = intel_dp_mst_enc_get_hw_state;
    intel_encoder->get_config = NULL;

	return intel_mst;

}
 static const struct drm_dp_mst_topology_cbs mst_cbs = {

 };

 static bool
intel_dp_create_fake_mst_encoders(struct intel_digital_port *intel_dig_port)
{
	struct intel_dp *intel_dp = &intel_dig_port->dp;
	struct drm_i915_private *dev_priv = to_i915(intel_dig_port->base.base.dev);
	enum pipe pipe;

	for_each_pipe(dev_priv, pipe)
		intel_dp->mst_encoders[pipe] = intel_dp_create_fake_mst_encoder(intel_dig_port, pipe);
	return true;
}

int
intel_dp_mst_encoder_init(struct intel_digital_port *intel_dig_port, int conn_base_id)
{
	struct intel_dp *intel_dp = &intel_dig_port->dp;
	struct drm_device *dev = intel_dig_port->base.base.dev;
	int ret;

	intel_dp->can_mst = true;
	intel_dp->mst_mgr.cbs = &mst_cbs;

	/* create encoders */
	intel_dp_create_fake_mst_encoders(intel_dig_port);
	ret = drm_dp_mst_topology_mgr_init(&intel_dp->mst_mgr, dev,
					   &intel_dp->aux, 16, 3, conn_base_id);
	if (ret) {
		intel_dp->can_mst = false;
		return ret;
	}
	return 0;
}

void
intel_dp_mst_encoder_cleanup(struct intel_digital_port *intel_dig_port)
{
	struct intel_dp *intel_dp = &intel_dig_port->dp;

	if (!intel_dp->can_mst)
		return;

	drm_dp_mst_topology_mgr_destroy(&intel_dp->mst_mgr);
	/* encoders will get killed by normal cleanup */
}
