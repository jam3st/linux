/*
 * Copyright 2006 Dave Airlie <airlied@linux.ie>
 * Copyright © 2006-2009 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 *	Jesse Barnes <jesse.barnes@intel.com>
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/hdmi.h>
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/drm_hdcp.h>
#include <drm/drm_scdc_helper.h>
#include "intel_drv.h"
#include <drm/i915_drm.h>
#include <drm/intel_lpe_audio.h>
#include "i915_drv.h"

static struct drm_device *intel_hdmi_to_dev(struct intel_hdmi *intel_hdmi)
{
	return hdmi_to_dig_port(intel_hdmi)->base.base.dev;
}

static void
assert_hdmi_port_disabled(struct intel_hdmi *intel_hdmi)
{
	struct drm_device *dev = intel_hdmi_to_dev(intel_hdmi);
	struct drm_i915_private *dev_priv = to_i915(dev);
	uint32_t enabled_bits;

	enabled_bits = HAS_DDI(dev_priv) ? DDI_BUF_CTL_ENABLE : SDVO_ENABLE;

	WARN(I915_READ(intel_hdmi->hdmi_reg) & enabled_bits,
	     "HDMI port enabled, expecting disabled\n");
}

struct intel_hdmi *enc_to_intel_hdmi(struct drm_encoder *encoder)
{
	struct intel_digital_port *intel_dig_port =
		container_of(encoder, struct intel_digital_port, base.base);
	return &intel_dig_port->hdmi;
}

static struct intel_hdmi *intel_attached_hdmi(struct drm_connector *connector)
{
	return enc_to_intel_hdmi(&intel_attached_encoder(connector)->base);
}

static u32 hsw_infoframe_enable(unsigned int type)
{
	switch (type) {
	case DP_SDP_VSC:
		return VIDEO_DIP_ENABLE_VSC_HSW;
	case HDMI_INFOFRAME_TYPE_AVI:
		return VIDEO_DIP_ENABLE_AVI_HSW;
	case HDMI_INFOFRAME_TYPE_SPD:
		return VIDEO_DIP_ENABLE_SPD_HSW;
	case HDMI_INFOFRAME_TYPE_VENDOR:
		return VIDEO_DIP_ENABLE_VS_HSW;
	default:
		MISSING_CASE(type);
		return 0;
	}
}

static i915_reg_t
hsw_dip_data_reg(struct drm_i915_private *dev_priv,
		 enum transcoder cpu_transcoder,
		 unsigned int type,
		 int i)
{
	switch (type) {
	case DP_SDP_VSC:
		return HSW_TVIDEO_DIP_VSC_DATA(cpu_transcoder, i);
	case HDMI_INFOFRAME_TYPE_AVI:
		return HSW_TVIDEO_DIP_AVI_DATA(cpu_transcoder, i);
	case HDMI_INFOFRAME_TYPE_SPD:
		return HSW_TVIDEO_DIP_SPD_DATA(cpu_transcoder, i);
	case HDMI_INFOFRAME_TYPE_VENDOR:
		return HSW_TVIDEO_DIP_VS_DATA(cpu_transcoder, i);
	default:
		MISSING_CASE(type);
		return INVALID_MMIO_REG;
	}
}


static void hsw_write_infoframe(struct drm_encoder *encoder,
				const struct intel_crtc_state *crtc_state,
				unsigned int type,
				const void *frame, ssize_t len)
{
	const uint32_t *data = frame;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum transcoder cpu_transcoder = crtc_state->cpu_transcoder;
	i915_reg_t ctl_reg = HSW_TVIDEO_DIP_CTL(cpu_transcoder);
	i915_reg_t data_reg;
	int data_size = type == DP_SDP_VSC ?
		VIDEO_DIP_VSC_DATA_SIZE : VIDEO_DIP_DATA_SIZE;
	int i;
	u32 val = I915_READ(ctl_reg);

	data_reg = hsw_dip_data_reg(dev_priv, cpu_transcoder, type, 0);

	val &= ~hsw_infoframe_enable(type);
	I915_WRITE(ctl_reg, val);

	mmiowb();
	for (i = 0; i < len; i += 4) {
		I915_WRITE(hsw_dip_data_reg(dev_priv, cpu_transcoder,
					    type, i >> 2), *data);
		data++;
	}
	/* Write every possible data byte to force correct ECC calculation. */
	for (; i < data_size; i += 4)
		I915_WRITE(hsw_dip_data_reg(dev_priv, cpu_transcoder,
					    type, i >> 2), 0);
	mmiowb();

	val |= hsw_infoframe_enable(type);
	I915_WRITE(ctl_reg, val);
	POSTING_READ(ctl_reg);
}

static bool hsw_infoframe_enabled(struct drm_encoder *encoder,
				  const struct intel_crtc_state *pipe_config)
{
	struct drm_i915_private *dev_priv = to_i915(encoder->dev);
	u32 val = I915_READ(HSW_TVIDEO_DIP_CTL(pipe_config->cpu_transcoder));

	return val & (VIDEO_DIP_ENABLE_VSC_HSW | VIDEO_DIP_ENABLE_AVI_HSW |
		      VIDEO_DIP_ENABLE_GCP_HSW | VIDEO_DIP_ENABLE_VS_HSW |
		      VIDEO_DIP_ENABLE_GMP_HSW | VIDEO_DIP_ENABLE_SPD_HSW);
}

/*
 * The data we write to the DIP data buffer registers is 1 byte bigger than the
 * HDMI infoframe size because of an ECC/reserved byte at position 3 (starting
 * at 0). It's also a byte used by DisplayPort so the same DIP registers can be
 * used for both technologies.
 *
 * DW0: Reserved/ECC/DP | HB2 | HB1 | HB0
 * DW1:       DB3       | DB2 | DB1 | DB0
 * DW2:       DB7       | DB6 | DB5 | DB4
 * DW3: ...
 *
 * (HB is Header Byte, DB is Data Byte)
 *
 * The hdmi pack() functions don't know about that hardware specific hole so we
 * trick them by giving an offset into the buffer and moving back the header
 * bytes by one.
 */
static void intel_write_infoframe(struct drm_encoder *encoder,
				  const struct intel_crtc_state *crtc_state,
				  union hdmi_infoframe *frame)
{
	struct intel_digital_port *intel_dig_port = enc_to_dig_port(encoder);
	uint8_t buffer[VIDEO_DIP_DATA_SIZE];
	ssize_t len;

	/* see comment above for the reason for this offset */
	len = hdmi_infoframe_pack(frame, buffer + 1, sizeof(buffer) - 1);
	if (len < 0)
		return;

	/* Insert the 'hole' (see big comment above) at position 3 */
	buffer[0] = buffer[1];
	buffer[1] = buffer[2];
	buffer[2] = buffer[3];
	buffer[3] = 0;
	len++;

	intel_dig_port->write_infoframe(encoder, crtc_state, frame->any.type, buffer, len);
}

static void intel_hdmi_set_avi_infoframe(struct drm_encoder *encoder,
					 const struct intel_crtc_state *crtc_state)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	const struct drm_display_mode *adjusted_mode =
		&crtc_state->base.adjusted_mode;
	struct drm_connector *connector = &intel_hdmi->attached_connector->base;
	bool is_hdmi2_sink = connector->display_info.hdmi.scdc.supported;
	union hdmi_infoframe frame;
	int ret;

	ret = drm_hdmi_avi_infoframe_from_display_mode(&frame.avi,
						       adjusted_mode,
						       is_hdmi2_sink);
	if (ret < 0) {
		DRM_ERROR("couldn't fill AVI infoframe\n");
		return;
	}

	if (crtc_state->ycbcr420)
		frame.avi.colorspace = HDMI_COLORSPACE_YUV420;
	else
		frame.avi.colorspace = HDMI_COLORSPACE_RGB;

	drm_hdmi_avi_infoframe_quant_range(&frame.avi, adjusted_mode,
					   crtc_state->limited_color_range ?
					   HDMI_QUANTIZATION_RANGE_LIMITED :
					   HDMI_QUANTIZATION_RANGE_FULL,
					   intel_hdmi->rgb_quant_range_selectable,
					   is_hdmi2_sink);

	/* TODO: handle pixel repetition for YCBCR420 outputs */
	intel_write_infoframe(encoder, crtc_state, &frame);
}

static void intel_hdmi_set_spd_infoframe(struct drm_encoder *encoder,
					 const struct intel_crtc_state *crtc_state)
{
	union hdmi_infoframe frame;
	int ret;

	ret = hdmi_spd_infoframe_init(&frame.spd, "Intel", "Integrated gfx");
	if (ret < 0) {
		DRM_ERROR("couldn't fill SPD infoframe\n");
		return;
	}

	frame.spd.sdi = HDMI_SPD_SDI_PC;

	intel_write_infoframe(encoder, crtc_state, &frame);
}

static void
intel_hdmi_set_hdmi_infoframe(struct drm_encoder *encoder,
			      const struct intel_crtc_state *crtc_state,
			      const struct drm_connector_state *conn_state)
{
	union hdmi_infoframe frame;
	int ret;

	ret = drm_hdmi_vendor_infoframe_from_display_mode(&frame.vendor.hdmi,
							  conn_state->connector,
							  &crtc_state->base.adjusted_mode);
	if (ret < 0)
		return;

	intel_write_infoframe(encoder, crtc_state, &frame);
}

static bool hdmi_sink_is_deep_color(const struct drm_connector_state *conn_state)
{
	struct drm_connector *connector = conn_state->connector;

	/*
	 * HDMI cloning is only supported on g4x which doesn't
	 * support deep color or GCP infoframes anyway so no
	 * need to worry about multiple HDMI sinks here.
	 */

	return connector->display_info.bpc > 8;
}

/*
 * Determine if default_phase=1 can be indicated in the GCP infoframe.
 *
 * From HDMI specification 1.4a:
 * - The first pixel of each Video Data Period shall always have a pixel packing phase of 0
 * - The first pixel following each Video Data Period shall have a pixel packing phase of 0
 * - The PP bits shall be constant for all GCPs and will be equal to the last packing phase
 * - The first pixel following every transition of HSYNC or VSYNC shall have a pixel packing
 *   phase of 0
 */
static bool gcp_default_phase_possible(int pipe_bpp,
				       const struct drm_display_mode *mode)
{
	unsigned int pixels_per_group;

	switch (pipe_bpp) {
	case 30:
		/* 4 pixels in 5 clocks */
		pixels_per_group = 4;
		break;
	case 36:
		/* 2 pixels in 3 clocks */
		pixels_per_group = 2;
		break;
	case 48:
		/* 1 pixel in 2 clocks */
		pixels_per_group = 1;
		break;
	default:
		/* phase information not relevant for 8bpc */
		return false;
	}

	return mode->crtc_hdisplay % pixels_per_group == 0 &&
		mode->crtc_htotal % pixels_per_group == 0 &&
		mode->crtc_hblank_start % pixels_per_group == 0 &&
		mode->crtc_hblank_end % pixels_per_group == 0 &&
		mode->crtc_hsync_start % pixels_per_group == 0 &&
		mode->crtc_hsync_end % pixels_per_group == 0 &&
		((mode->flags & DRM_MODE_FLAG_INTERLACE) == 0 ||
		 mode->crtc_htotal/2 % pixels_per_group == 0);
}

static bool intel_hdmi_set_gcp_infoframe(struct drm_encoder *encoder,
					 const struct intel_crtc_state *crtc_state,
					 const struct drm_connector_state *conn_state)
{
	struct drm_i915_private *dev_priv = to_i915(encoder->dev);
	struct intel_crtc *crtc = to_intel_crtc(crtc_state->base.crtc);
	i915_reg_t reg;
	u32 val = 0;

	if (HAS_DDI(dev_priv))
		reg = HSW_TVIDEO_DIP_GCP(crtc_state->cpu_transcoder);
	else if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		reg = VLV_TVIDEO_DIP_GCP(crtc->pipe);
	else if (HAS_PCH_SPLIT(dev_priv))
		reg = TVIDEO_DIP_GCP(crtc->pipe);
	else
		return false;

	/* Indicate color depth whenever the sink supports deep color */
	if (hdmi_sink_is_deep_color(conn_state))
		val |= GCP_COLOR_INDICATION;

	/* Enable default_phase whenever the display mode is suitably aligned */
	if (gcp_default_phase_possible(crtc_state->pipe_bpp,
				       &crtc_state->base.adjusted_mode))
		val |= GCP_DEFAULT_PHASE_ENABLE;

	I915_WRITE(reg, val);

	return val != 0;
}



static void hsw_set_infoframes(struct drm_encoder *encoder,
			       bool enable,
			       const struct intel_crtc_state *crtc_state,
			       const struct drm_connector_state *conn_state)
{
	struct drm_i915_private *dev_priv = to_i915(encoder->dev);
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(encoder);
	i915_reg_t reg = HSW_TVIDEO_DIP_CTL(crtc_state->cpu_transcoder);
	u32 val = I915_READ(reg);

	assert_hdmi_port_disabled(intel_hdmi);

	val &= ~(VIDEO_DIP_ENABLE_VSC_HSW | VIDEO_DIP_ENABLE_AVI_HSW |
		 VIDEO_DIP_ENABLE_GCP_HSW | VIDEO_DIP_ENABLE_VS_HSW |
		 VIDEO_DIP_ENABLE_GMP_HSW | VIDEO_DIP_ENABLE_SPD_HSW);

	if (!enable) {
		I915_WRITE(reg, val);
		POSTING_READ(reg);
		return;
	}

	if (intel_hdmi_set_gcp_infoframe(encoder, crtc_state, conn_state))
		val |= VIDEO_DIP_ENABLE_GCP_HSW;

	I915_WRITE(reg, val);
	POSTING_READ(reg);

	intel_hdmi_set_avi_infoframe(encoder, crtc_state);
	intel_hdmi_set_spd_infoframe(encoder, crtc_state);
	intel_hdmi_set_hdmi_infoframe(encoder, crtc_state, conn_state);
}

void intel_dp_dual_mode_set_tmds_output(struct intel_hdmi *hdmi, bool enable)
{
	struct drm_i915_private *dev_priv = to_i915(intel_hdmi_to_dev(hdmi));
	struct i2c_adapter *adapter =
		intel_gmbus_get_adapter(dev_priv, hdmi->ddc_bus);

	if (hdmi->dp_dual_mode.type < DRM_DP_DUAL_MODE_TYPE2_DVI)
		return;

	DRM_DEBUG_KMS("%s DP dual mode adaptor TMDS output\n",
		      enable ? "Enabling" : "Disabling");

	drm_dp_dual_mode_set_tmds_output(hdmi->dp_dual_mode.type,
					 adapter, enable);
}

static int intel_hdmi_hdcp_read(struct intel_digital_port *intel_dig_port,
				unsigned int offset, void *buffer, size_t size)
{
	struct intel_hdmi *hdmi = &intel_dig_port->hdmi;
	struct drm_i915_private *dev_priv =
		intel_dig_port->base.base.dev->dev_private;
	struct i2c_adapter *adapter = intel_gmbus_get_adapter(dev_priv,
							      hdmi->ddc_bus);
	int ret;
	u8 start = offset & 0xff;
	struct i2c_msg msgs[] = {
		{
			.addr = DRM_HDCP_DDC_ADDR,
			.flags = 0,
			.len = 1,
			.buf = &start,
		},
		{
			.addr = DRM_HDCP_DDC_ADDR,
			.flags = I2C_M_RD,
			.len = size,
			.buf = buffer
		}
	};
	ret = i2c_transfer(adapter, msgs, ARRAY_SIZE(msgs));
	if (ret == ARRAY_SIZE(msgs))
		return 0;
	return ret >= 0 ? -EIO : ret;
}

static int intel_hdmi_hdcp_write(struct intel_digital_port *intel_dig_port,
				 unsigned int offset, void *buffer, size_t size)
{
	struct intel_hdmi *hdmi = &intel_dig_port->hdmi;
	struct drm_i915_private *dev_priv =
		intel_dig_port->base.base.dev->dev_private;
	struct i2c_adapter *adapter = intel_gmbus_get_adapter(dev_priv,
							      hdmi->ddc_bus);
	int ret;
	u8 *write_buf;
	struct i2c_msg msg;

	write_buf = kzalloc(size + 1, GFP_KERNEL);
	if (!write_buf)
		return -ENOMEM;

	write_buf[0] = offset & 0xff;
	memcpy(&write_buf[1], buffer, size);

	msg.addr = DRM_HDCP_DDC_ADDR;
	msg.flags = 0,
	msg.len = size + 1,
	msg.buf = write_buf;

	ret = i2c_transfer(adapter, &msg, 1);
	if (ret == 1)
		return 0;
	return ret >= 0 ? -EIO : ret;
}

static
int intel_hdmi_hdcp_write_an_aksv(struct intel_digital_port *intel_dig_port,
				  u8 *an)
{
	struct intel_hdmi *hdmi = &intel_dig_port->hdmi;
	struct drm_i915_private *dev_priv =
		intel_dig_port->base.base.dev->dev_private;
	struct i2c_adapter *adapter = intel_gmbus_get_adapter(dev_priv,
							      hdmi->ddc_bus);
	int ret;

	ret = intel_hdmi_hdcp_write(intel_dig_port, DRM_HDCP_DDC_AN, an,
				    DRM_HDCP_AN_LEN);
	if (ret) {
		DRM_ERROR("Write An over DDC failed (%d)\n", ret);
		return ret;
	}

	ret = intel_gmbus_output_aksv(adapter);
	if (ret < 0) {
		DRM_ERROR("Failed to output aksv (%d)\n", ret);
		return ret;
	}
	return 0;
}

static int intel_hdmi_hdcp_read_bksv(struct intel_digital_port *intel_dig_port,
				     u8 *bksv)
{
	int ret;
	ret = intel_hdmi_hdcp_read(intel_dig_port, DRM_HDCP_DDC_BKSV, bksv,
				   DRM_HDCP_KSV_LEN);
	if (ret)
		DRM_ERROR("Read Bksv over DDC failed (%d)\n", ret);
	return ret;
}

static
int intel_hdmi_hdcp_read_bstatus(struct intel_digital_port *intel_dig_port,
				 u8 *bstatus)
{
	int ret;
	ret = intel_hdmi_hdcp_read(intel_dig_port, DRM_HDCP_DDC_BSTATUS,
				   bstatus, DRM_HDCP_BSTATUS_LEN);
	if (ret)
		DRM_ERROR("Read bstatus over DDC failed (%d)\n", ret);
	return ret;
}

static
int intel_hdmi_hdcp_repeater_present(struct intel_digital_port *intel_dig_port,
				     bool *repeater_present)
{
	int ret;
	u8 val;

	ret = intel_hdmi_hdcp_read(intel_dig_port, DRM_HDCP_DDC_BCAPS, &val, 1);
	if (ret) {
		DRM_ERROR("Read bcaps over DDC failed (%d)\n", ret);
		return ret;
	}
	*repeater_present = val & DRM_HDCP_DDC_BCAPS_REPEATER_PRESENT;
	return 0;
}

static
int intel_hdmi_hdcp_read_ri_prime(struct intel_digital_port *intel_dig_port,
				  u8 *ri_prime)
{
	int ret;
	ret = intel_hdmi_hdcp_read(intel_dig_port, DRM_HDCP_DDC_RI_PRIME,
				   ri_prime, DRM_HDCP_RI_LEN);
	if (ret)
		DRM_ERROR("Read Ri' over DDC failed (%d)\n", ret);
	return ret;
}

static
int intel_hdmi_hdcp_read_ksv_ready(struct intel_digital_port *intel_dig_port,
				   bool *ksv_ready)
{
	int ret;
	u8 val;

	ret = intel_hdmi_hdcp_read(intel_dig_port, DRM_HDCP_DDC_BCAPS, &val, 1);
	if (ret) {
		DRM_ERROR("Read bcaps over DDC failed (%d)\n", ret);
		return ret;
	}
	*ksv_ready = val & DRM_HDCP_DDC_BCAPS_KSV_FIFO_READY;
	return 0;
}

static
int intel_hdmi_hdcp_read_ksv_fifo(struct intel_digital_port *intel_dig_port,
				  int num_downstream, u8 *ksv_fifo)
{
	int ret;
	ret = intel_hdmi_hdcp_read(intel_dig_port, DRM_HDCP_DDC_KSV_FIFO,
				   ksv_fifo, num_downstream * DRM_HDCP_KSV_LEN);
	if (ret) {
		DRM_ERROR("Read ksv fifo over DDC failed (%d)\n", ret);
		return ret;
	}
	return 0;
}

static
int intel_hdmi_hdcp_read_v_prime_part(struct intel_digital_port *intel_dig_port,
				      int i, u32 *part)
{
	int ret;

	if (i >= DRM_HDCP_V_PRIME_NUM_PARTS)
		return -EINVAL;

	ret = intel_hdmi_hdcp_read(intel_dig_port, DRM_HDCP_DDC_V_PRIME(i),
				   part, DRM_HDCP_V_PRIME_PART_LEN);
	if (ret)
		DRM_ERROR("Read V'[%d] over DDC failed (%d)\n", i, ret);
	return ret;
}

static
int intel_hdmi_hdcp_toggle_signalling(struct intel_digital_port *intel_dig_port,
				      bool enable)
{
	int ret;

	if (!enable)
		usleep_range(6, 60); /* Bspec says >= 6us */

	ret = intel_ddi_toggle_hdcp_signalling(&intel_dig_port->base, enable);
	if (ret) {
		DRM_ERROR("%s HDCP signalling failed (%d)\n",
			  enable ? "Enable" : "Disable", ret);
		return ret;
	}
	return 0;
}

static
bool intel_hdmi_hdcp_check_link(struct intel_digital_port *intel_dig_port)
{
	struct drm_i915_private *dev_priv =
		intel_dig_port->base.base.dev->dev_private;
	enum port port = intel_dig_port->base.port;
	int ret;
	union {
		u32 reg;
		u8 shim[DRM_HDCP_RI_LEN];
	} ri;

	ret = intel_hdmi_hdcp_read_ri_prime(intel_dig_port, ri.shim);
	if (ret)
		return false;

	I915_WRITE(PORT_HDCP_RPRIME(port), ri.reg);

	/* Wait for Ri prime match */
	if (wait_for(I915_READ(PORT_HDCP_STATUS(port)) &
		     (HDCP_STATUS_RI_MATCH | HDCP_STATUS_ENC), 1)) {
		DRM_ERROR("Ri' mismatch detected, link check failed (%x)\n",
			  I915_READ(PORT_HDCP_STATUS(port)));
		return false;
	}
	return true;
}

static const struct intel_hdcp_shim intel_hdmi_hdcp_shim = {
	.write_an_aksv = intel_hdmi_hdcp_write_an_aksv,
	.read_bksv = intel_hdmi_hdcp_read_bksv,
	.read_bstatus = intel_hdmi_hdcp_read_bstatus,
	.repeater_present = intel_hdmi_hdcp_repeater_present,
	.read_ri_prime = intel_hdmi_hdcp_read_ri_prime,
	.read_ksv_ready = intel_hdmi_hdcp_read_ksv_ready,
	.read_ksv_fifo = intel_hdmi_hdcp_read_ksv_fifo,
	.read_v_prime_part = intel_hdmi_hdcp_read_v_prime_part,
	.toggle_signalling = intel_hdmi_hdcp_toggle_signalling,
	.check_link = intel_hdmi_hdcp_check_link,
};

static int intel_hdmi_source_max_tmds_clock(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = to_i915(encoder->base.dev);
	const struct ddi_vbt_port_info *info =
		&dev_priv->vbt.ddi_port_info[encoder->port];
	int max_tmds_clock;

	if (INTEL_GEN(dev_priv) >= 10 || IS_GEMINILAKE(dev_priv))
		max_tmds_clock = 594000;
	else if (INTEL_GEN(dev_priv) >= 8 || IS_HASWELL(dev_priv))
		max_tmds_clock = 300000;
	else if (INTEL_GEN(dev_priv) >= 5)
		max_tmds_clock = 225000;
	else
		max_tmds_clock = 165000;

	if (info->max_tmds_clock)
		max_tmds_clock = min(max_tmds_clock, info->max_tmds_clock);

	return max_tmds_clock;
}

static int hdmi_port_clock_limit(struct intel_hdmi *hdmi,
				 bool respect_downstream_limits,
				 bool force_dvi)
{
	struct intel_encoder *encoder = &hdmi_to_dig_port(hdmi)->base;
	int max_tmds_clock = intel_hdmi_source_max_tmds_clock(encoder);

	if (respect_downstream_limits) {
		struct intel_connector *connector = hdmi->attached_connector;
		const struct drm_display_info *info = &connector->base.display_info;

		if (hdmi->dp_dual_mode.max_tmds_clock)
			max_tmds_clock = min(max_tmds_clock,
					     hdmi->dp_dual_mode.max_tmds_clock);

		if (info->max_tmds_clock)
			max_tmds_clock = min(max_tmds_clock,
					     info->max_tmds_clock);
		else if (!hdmi->has_hdmi_sink || force_dvi)
			max_tmds_clock = min(max_tmds_clock, 165000);
	}

	return max_tmds_clock;
}

static enum drm_mode_status
hdmi_port_clock_valid(struct intel_hdmi *hdmi,
		      int clock, bool respect_downstream_limits,
		      bool force_dvi)
{
	struct drm_i915_private *dev_priv = to_i915(intel_hdmi_to_dev(hdmi));

	if (clock < 25000)
		return MODE_CLOCK_LOW;
	if (clock > hdmi_port_clock_limit(hdmi, respect_downstream_limits, force_dvi))
		return MODE_CLOCK_HIGH;

	/* BXT DPLL can't generate 223-240 MHz */
	if (IS_GEN9_LP(dev_priv) && clock > 223333 && clock < 240000)
		return MODE_CLOCK_RANGE;

	/* CHV DPLL can't generate 216-240 MHz */
	if (IS_CHERRYVIEW(dev_priv) && clock > 216000 && clock < 240000)
		return MODE_CLOCK_RANGE;

	return MODE_OK;
}

static enum drm_mode_status
intel_hdmi_mode_valid(struct drm_connector *connector,
		      struct drm_display_mode *mode)
{
	struct intel_hdmi *hdmi = intel_attached_hdmi(connector);
	struct drm_device *dev = intel_hdmi_to_dev(hdmi);
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum drm_mode_status status;
	int clock;
	int max_dotclk = to_i915(connector->dev)->max_dotclk_freq;
	bool force_dvi =
		READ_ONCE(to_intel_digital_connector_state(connector->state)->force_audio) == HDMI_AUDIO_OFF_DVI;

	clock = mode->clock;

	if ((mode->flags & DRM_MODE_FLAG_3D_MASK) == DRM_MODE_FLAG_3D_FRAME_PACKING)
		clock *= 2;

	if (clock > max_dotclk)
		return MODE_CLOCK_HIGH;

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		clock *= 2;

	if (drm_mode_is_420_only(&connector->display_info, mode))
		clock /= 2;

	/* check if we can do 8bpc */
	status = hdmi_port_clock_valid(hdmi, clock, true, force_dvi);

	/* if we can't do 8bpc we may still be able to do 12bpc */
	if (!HAS_GMCH_DISPLAY(dev_priv) && status != MODE_OK && hdmi->has_hdmi_sink && !force_dvi)
		status = hdmi_port_clock_valid(hdmi, clock * 3 / 2, true, force_dvi);

	return status;
}

static bool hdmi_12bpc_possible(const struct intel_crtc_state *crtc_state)
{
	struct drm_i915_private *dev_priv =
		to_i915(crtc_state->base.crtc->dev);
	struct drm_atomic_state *state = crtc_state->base.state;
	struct drm_connector_state *connector_state;
	struct drm_connector *connector;
	int i;

	if (HAS_GMCH_DISPLAY(dev_priv))
		return false;

	if (crtc_state->pipe_bpp <= 8*3)
		return false;

	if (!crtc_state->has_hdmi_sink)
		return false;

	/*
	 * HDMI 12bpc affects the clocks, so it's only possible
	 * when not cloning with other encoder types.
	 */
	if (crtc_state->output_types != 1 << INTEL_OUTPUT_HDMI)
		return false;

	for_each_new_connector_in_state(state, connector, connector_state, i) {
		const struct drm_display_info *info = &connector->display_info;

		if (connector_state->crtc != crtc_state->base.crtc)
			continue;

		if (crtc_state->ycbcr420) {
			const struct drm_hdmi_info *hdmi = &info->hdmi;

			if (!(hdmi->y420_dc_modes & DRM_EDID_YCBCR420_DC_36))
				return false;
		} else {
			if (!(info->edid_hdmi_dc_modes & DRM_EDID_HDMI_DC_36))
				return false;
		}
	}

	/* Display WA #1139: glk */
	if (IS_GLK_REVID(dev_priv, 0, GLK_REVID_A1) &&
	    crtc_state->base.adjusted_mode.htotal > 5460)
		return false;

	return true;
}

static bool
intel_hdmi_ycbcr420_config(struct drm_connector *connector,
			   struct intel_crtc_state *config,
			   int *clock_12bpc, int *clock_8bpc)
{

	if (!connector->ycbcr_420_allowed) {
		
        return false;
	}

	/* YCBCR420 TMDS rate requirement is half the pixel clock */
	config->port_clock /= 2;
	*clock_12bpc /= 2;
	*clock_8bpc /= 2;
	config->ycbcr420 = true;

	/* YCBCR 420 output conversion needs a scaler */
	if (skl_update_scaler_crtc(config)) {
		DRM_DEBUG_KMS("Scaler allocation for output failed\n");
		return false;
	}

	return true;
}

bool intel_hdmi_compute_config(struct intel_encoder *encoder,
			       struct intel_crtc_state *pipe_config,
			       struct drm_connector_state *conn_state)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	struct drm_i915_private *dev_priv = to_i915(encoder->base.dev);
	struct drm_display_mode *adjusted_mode = &pipe_config->base.adjusted_mode;
	struct drm_connector *connector = conn_state->connector;
	struct drm_scdc *scdc = &connector->display_info.hdmi.scdc;
	struct intel_digital_connector_state *intel_conn_state =
		to_intel_digital_connector_state(conn_state);
	int clock_8bpc = pipe_config->base.adjusted_mode.crtc_clock;
	int clock_12bpc = clock_8bpc * 3 / 2;
	int desired_bpp;
	bool force_dvi = intel_conn_state->force_audio == HDMI_AUDIO_OFF_DVI;

	pipe_config->has_hdmi_sink = !force_dvi && intel_hdmi->has_hdmi_sink;

	if (pipe_config->has_hdmi_sink)
		pipe_config->has_infoframe = true;

	if (intel_conn_state->broadcast_rgb == INTEL_BROADCAST_RGB_AUTO) {
		/* See CEA-861-E - 5.1 Default Encoding Parameters */
		pipe_config->limited_color_range =
			pipe_config->has_hdmi_sink &&
			drm_default_rgb_quant_range(adjusted_mode) ==
			HDMI_QUANTIZATION_RANGE_LIMITED;
	} else {
		pipe_config->limited_color_range =
			intel_conn_state->broadcast_rgb == INTEL_BROADCAST_RGB_LIMITED;
	}

	if (adjusted_mode->flags & DRM_MODE_FLAG_DBLCLK) {
		pipe_config->pixel_multiplier = 2;
		clock_8bpc *= 2;
		clock_12bpc *= 2;
	}

	if (drm_mode_is_420_only(&connector->display_info, adjusted_mode)) {
		if (!intel_hdmi_ycbcr420_config(connector, pipe_config,
						&clock_12bpc, &clock_8bpc)) {
			DRM_ERROR("Can't support YCBCR420 output\n");
			return false;
		}
	}

	if (HAS_PCH_SPLIT(dev_priv) && !HAS_DDI(dev_priv))
		pipe_config->has_pch_encoder = true;

	if (pipe_config->has_hdmi_sink) {
		if (intel_conn_state->force_audio == HDMI_AUDIO_AUTO)
			pipe_config->has_audio = intel_hdmi->has_audio;
		else
			pipe_config->has_audio =
				intel_conn_state->force_audio == HDMI_AUDIO_ON;
	}

	/*
	 * HDMI is either 12 or 8, so if the display lets 10bpc sneak
	 * through, clamp it down. Note that g4x/vlv don't support 12bpc hdmi
	 * outputs. We also need to check that the higher clock still fits
	 * within limits.
	 */
	if (hdmi_12bpc_possible(pipe_config) &&
	    hdmi_port_clock_valid(intel_hdmi, clock_12bpc, true, force_dvi) == MODE_OK) {
		DRM_DEBUG_KMS("picking bpc to 12 for HDMI output\n");
		desired_bpp = 12*3;

		/* Need to adjust the port link by 1.5x for 12bpc. */
		pipe_config->port_clock = clock_12bpc;
	} else {
		DRM_DEBUG_KMS("picking bpc to 8 for HDMI output\n");
		desired_bpp = 8*3;

		pipe_config->port_clock = clock_8bpc;
	}

	if (!pipe_config->bw_constrained) {
		DRM_DEBUG_KMS("forcing pipe bpp to %i for HDMI\n", desired_bpp);
		pipe_config->pipe_bpp = desired_bpp;
	}

	if (hdmi_port_clock_valid(intel_hdmi, pipe_config->port_clock,
				  false, force_dvi) != MODE_OK) {
		DRM_DEBUG_KMS("unsupported HDMI clock, rejecting mode\n");
		return false;
	}

	/* Set user selected PAR to incoming mode's member */
	adjusted_mode->picture_aspect_ratio = conn_state->picture_aspect_ratio;

	pipe_config->lane_count = 4;

	if (scdc->scrambling.supported && (INTEL_GEN(dev_priv) >= 10 ||
					   IS_GEMINILAKE(dev_priv))) {
		if (scdc->scrambling.low_rates)
			pipe_config->hdmi_scrambling = true;

		if (pipe_config->port_clock > 340000) {
			pipe_config->hdmi_scrambling = true;
			pipe_config->hdmi_high_tmds_clock_ratio = true;
		}
	}

	return true;
}

static void
intel_hdmi_unset_edid(struct drm_connector *connector)
{
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);

	intel_hdmi->has_hdmi_sink = false;
	intel_hdmi->has_audio = false;
	intel_hdmi->rgb_quant_range_selectable = false;

	intel_hdmi->dp_dual_mode.type = DRM_DP_DUAL_MODE_NONE;
	intel_hdmi->dp_dual_mode.max_tmds_clock = 0;

	kfree(to_intel_connector(connector)->detect_edid);
	to_intel_connector(connector)->detect_edid = NULL;
}

static void
intel_hdmi_dp_dual_mode_detect(struct drm_connector *connector, bool has_edid)
{
	struct drm_i915_private *dev_priv = to_i915(connector->dev);
	struct intel_hdmi *hdmi = intel_attached_hdmi(connector);
	struct i2c_adapter *adapter =
		intel_gmbus_get_adapter(dev_priv, hdmi->ddc_bus);
	enum drm_dp_dual_mode_type type = drm_dp_dual_mode_detect(adapter);

	/*
	 * Type 1 DVI adaptors are not required to implement any
	 * registers, so we can't always detect their presence.
	 * Ideally we should be able to check the state of the
	 * CONFIG1 pin, but no such luck on our hardware.
	 *
	 * The only method left to us is to check the VBT to see
	 * if the port is a dual mode capable DP port. But let's
	 * only do that when we sucesfully read the EDID, to avoid
	 * confusing log messages about DP dual mode adaptors when
	 * there's nothing connected to the port.
	 */
	if (type == DRM_DP_DUAL_MODE_UNKNOWN) {
		/* An overridden EDID imply that we want this port for testing.
		 * Make sure not to set limits for that port.
		 */

                    type = DRM_DP_DUAL_MODE_NONE;

	}

	if (type == DRM_DP_DUAL_MODE_NONE)
		return;

	hdmi->dp_dual_mode.type = type;
	hdmi->dp_dual_mode.max_tmds_clock =
		drm_dp_dual_mode_max_tmds_clock(type, adapter);

	DRM_DEBUG_KMS("DP dual mode adaptor (%s) detected (max TMDS clock: %d kHz)\n",
		      drm_dp_get_dual_mode_type_name(type),
		      hdmi->dp_dual_mode.max_tmds_clock);
}

static bool
intel_hdmi_set_edid(struct drm_connector *connector)
{
	struct drm_i915_private *dev_priv = to_i915(connector->dev);
	struct intel_hdmi *intel_hdmi = intel_attached_hdmi(connector);
	struct edid *edid;
	bool connected = false;
	struct i2c_adapter *i2c;

	intel_display_power_get(dev_priv, POWER_DOMAIN_GMBUS);

	i2c = intel_gmbus_get_adapter(dev_priv, intel_hdmi->ddc_bus);

	edid = drm_get_edid(connector, i2c);

	if (!edid && !intel_gmbus_is_forced_bit(i2c)) {
		DRM_DEBUG_KMS("HDMI GMBUS EDID read failed, retry using GPIO bit-banging\n");
		intel_gmbus_force_bit(i2c, true);
		edid = drm_get_edid(connector, i2c);
		intel_gmbus_force_bit(i2c, false);
	}

	intel_hdmi_dp_dual_mode_detect(connector, edid != NULL);

	intel_display_power_put(dev_priv, POWER_DOMAIN_GMBUS);

	to_intel_connector(connector)->detect_edid = edid;
	if (edid && edid->input & DRM_EDID_INPUT_DIGITAL) {
		intel_hdmi->rgb_quant_range_selectable =
			drm_rgb_quant_range_selectable(edid);

		intel_hdmi->has_audio = drm_detect_monitor_audio(edid);
		intel_hdmi->has_hdmi_sink = drm_detect_hdmi_monitor(edid);

		connected = true;
	}

	return connected;
}

static enum drm_connector_status
intel_hdmi_detect(struct drm_connector *connector, bool force)
{
	enum drm_connector_status status;
	struct drm_i915_private *dev_priv = to_i915(connector->dev);

	DRM_DEBUG_KMS("[CONNECTOR:%d:%s]\n",
		      connector->base.id, connector->name);

	intel_display_power_get(dev_priv, POWER_DOMAIN_GMBUS);

	intel_hdmi_unset_edid(connector);

	if (intel_hdmi_set_edid(connector))
		status = connector_status_connected;
	else
		status = connector_status_disconnected;

	intel_display_power_put(dev_priv, POWER_DOMAIN_GMBUS);

	return status;
}



static int intel_hdmi_get_modes(struct drm_connector *connector)
{
	struct edid *edid;
    int ret = 0;

	edid = to_intel_connector(connector)->detect_edid;
	if (edid == NULL)
		return 0;

        drm_mode_connector_update_edid_property(connector, edid);
    ret = drm_add_edid_modes(connector, edid);
return ret;
}



static const struct drm_connector_funcs intel_hdmi_connector_funcs = {
	.detect = intel_hdmi_detect,
    .force = NULL,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_get_property = intel_digital_connector_atomic_get_property,
	.atomic_set_property = intel_digital_connector_atomic_set_property,
	.late_register = intel_connector_register,
	.early_unregister = intel_connector_unregister,
    .destroy = NULL,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.atomic_duplicate_state = intel_digital_connector_duplicate_state,
};

static const struct drm_connector_helper_funcs intel_hdmi_connector_helper_funcs = {
	.get_modes = intel_hdmi_get_modes,
	.mode_valid = intel_hdmi_mode_valid,
	.atomic_check = intel_digital_connector_atomic_check,
};

static const struct drm_encoder_funcs intel_hdmi_enc_funcs = {
	.destroy = intel_encoder_destroy,
};

static void
intel_hdmi_add_properties(struct intel_hdmi *intel_hdmi, struct drm_connector *connector)
{
    printk("Deleted a lot here.......................");
	connector->state->picture_aspect_ratio = HDMI_PICTURE_ASPECT_NONE;
}

/*
 * intel_hdmi_handle_sink_scrambling: handle sink scrambling/clock ratio setup
 * @encoder: intel_encoder
 * @connector: drm_connector
 * @high_tmds_clock_ratio = bool to indicate if the function needs to set
 *  or reset the high tmds clock ratio for scrambling
 * @scrambling: bool to Indicate if the function needs to set or reset
 *  sink scrambling
 *
 * This function handles scrambling on HDMI 2.0 capable sinks.
 * If required clock rate is > 340 Mhz && scrambling is supported by sink
 * it enables scrambling. This should be called before enabling the HDMI
 * 2.0 port, as the sink can choose to disable the scrambling if it doesn't
 * detect a scrambled clock within 100 ms.
 */
void intel_hdmi_handle_sink_scrambling(struct intel_encoder *encoder,
				       struct drm_connector *connector,
				       bool high_tmds_clock_ratio,
				       bool scrambling)
{
	struct intel_hdmi *intel_hdmi = enc_to_intel_hdmi(&encoder->base);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct drm_scrambling *sink_scrambling =
				&connector->display_info.hdmi.scdc.scrambling;
	struct i2c_adapter *adptr = intel_gmbus_get_adapter(dev_priv,
							   intel_hdmi->ddc_bus);
	bool ret;

	if (!sink_scrambling->supported)
		return;

	DRM_DEBUG_KMS("Setting sink scrambling for enc:%s connector:%s\n",
		      encoder->base.name, connector->name);

	/* Set TMDS bit clock ratio to 1/40 or 1/10 */
	ret = drm_scdc_set_high_tmds_clock_ratio(adptr, high_tmds_clock_ratio);
	if (!ret) {
		DRM_ERROR("Set TMDS ratio failed\n");
		return;
	}

	/* Enable/disable sink scrambling */
	ret = drm_scdc_set_scrambling(adptr, scrambling);
	if (!ret) {
		DRM_ERROR("Set sink scrambling failed\n");
		return;
	}

	DRM_DEBUG_KMS("sink scrambling handled\n");
}

static u8 chv_port_to_ddc_pin(struct drm_i915_private *dev_priv, enum port port)
{
	u8 ddc_pin;

	switch (port) {
	case PORT_B:
		ddc_pin = GMBUS_PIN_DPB;
		break;
	case PORT_C:
		ddc_pin = GMBUS_PIN_DPC;
		break;
	case PORT_D:
		ddc_pin = GMBUS_PIN_DPD_CHV;
		break;
	default:
		MISSING_CASE(port);
		ddc_pin = GMBUS_PIN_DPB;
		break;
	}
	return ddc_pin;
}

static u8 bxt_port_to_ddc_pin(struct drm_i915_private *dev_priv, enum port port)
{
	u8 ddc_pin;

	switch (port) {
	case PORT_B:
		ddc_pin = GMBUS_PIN_1_BXT;
		break;
	case PORT_C:
		ddc_pin = GMBUS_PIN_2_BXT;
		break;
	default:
		MISSING_CASE(port);
		ddc_pin = GMBUS_PIN_1_BXT;
		break;
	}
	return ddc_pin;
}

static u8 cnp_port_to_ddc_pin(struct drm_i915_private *dev_priv,
			      enum port port)
{
	u8 ddc_pin;

	switch (port) {
	case PORT_B:
		ddc_pin = GMBUS_PIN_1_BXT;
		break;
	case PORT_C:
		ddc_pin = GMBUS_PIN_2_BXT;
		break;
	case PORT_D:
		ddc_pin = GMBUS_PIN_4_CNP;
		break;
	case PORT_F:
		ddc_pin = GMBUS_PIN_3_BXT;
		break;
	default:
		MISSING_CASE(port);
		ddc_pin = GMBUS_PIN_1_BXT;
		break;
	}
	return ddc_pin;
}

static u8 icl_port_to_ddc_pin(struct drm_i915_private *dev_priv, enum port port)
{
	u8 ddc_pin;

	switch (port) {
	case PORT_A:
		ddc_pin = GMBUS_PIN_1_BXT;
		break;
	case PORT_B:
		ddc_pin = GMBUS_PIN_2_BXT;
		break;
	case PORT_C:
		ddc_pin = GMBUS_PIN_9_TC1_ICP;
		break;
	case PORT_D:
		ddc_pin = GMBUS_PIN_10_TC2_ICP;
		break;
	case PORT_E:
		ddc_pin = GMBUS_PIN_11_TC3_ICP;
		break;
	case PORT_F:
		ddc_pin = GMBUS_PIN_12_TC4_ICP;
		break;
	default:
		MISSING_CASE(port);
		ddc_pin = GMBUS_PIN_2_BXT;
		break;
	}
	return ddc_pin;
}

static u8 g4x_port_to_ddc_pin(struct drm_i915_private *dev_priv,
			      enum port port)
{
	u8 ddc_pin;

	switch (port) {
	case PORT_B:
		ddc_pin = GMBUS_PIN_DPB;
		break;
	case PORT_C:
		ddc_pin = GMBUS_PIN_DPC;
		break;
	case PORT_D:
		ddc_pin = GMBUS_PIN_DPD;
		break;
	default:
		MISSING_CASE(port);
		ddc_pin = GMBUS_PIN_DPB;
		break;
	}
	return ddc_pin;
}

static u8 intel_hdmi_ddc_pin(struct drm_i915_private *dev_priv,
			     enum port port)
{
	const struct ddi_vbt_port_info *info =
		&dev_priv->vbt.ddi_port_info[port];
	u8 ddc_pin;

	if (info->alternate_ddc_pin) {
		DRM_DEBUG_KMS("Using DDC pin 0x%x for port %c (VBT)\n",
			      info->alternate_ddc_pin, port_name(port));
		return info->alternate_ddc_pin;
	}

	if (IS_CHERRYVIEW(dev_priv))
		ddc_pin = chv_port_to_ddc_pin(dev_priv, port);
	else if (IS_GEN9_LP(dev_priv))
		ddc_pin = bxt_port_to_ddc_pin(dev_priv, port);
	else if (HAS_PCH_CNP(dev_priv))
		ddc_pin = cnp_port_to_ddc_pin(dev_priv, port);
	else if (IS_ICELAKE(dev_priv))
		ddc_pin = icl_port_to_ddc_pin(dev_priv, port);
	else
		ddc_pin = g4x_port_to_ddc_pin(dev_priv, port);

	DRM_DEBUG_KMS("Using DDC pin 0x%x for port %c (platform default)\n",
		      ddc_pin, port_name(port));

	return ddc_pin;
}

void intel_infoframe_init(struct intel_digital_port *intel_dig_port)
{
	struct drm_i915_private *dev_priv =
		to_i915(intel_dig_port->base.base.dev);

    intel_dig_port->write_infoframe = hsw_write_infoframe;
    intel_dig_port->set_infoframes = hsw_set_infoframes;
    intel_dig_port->infoframe_enabled = hsw_infoframe_enabled;
}

void intel_hdmi_init_connector(struct intel_digital_port *intel_dig_port,
			       struct intel_connector *intel_connector)
{
	struct drm_connector *connector = &intel_connector->base;
	struct intel_hdmi *intel_hdmi = &intel_dig_port->hdmi;
	struct intel_encoder *intel_encoder = &intel_dig_port->base;
	struct drm_device *dev = intel_encoder->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum port port = intel_encoder->port;

	DRM_DEBUG_KMS("Adding HDMI connector on port %c\n",
		      port_name(port));

	if (WARN(intel_dig_port->max_lanes < 4,
		 "Not enough lanes (%d) for HDMI on port %c\n",
		 intel_dig_port->max_lanes, port_name(port)))
		return;

	drm_connector_init(dev, connector, &intel_hdmi_connector_funcs,
			   DRM_MODE_CONNECTOR_HDMIA);
	drm_connector_helper_add(connector, &intel_hdmi_connector_helper_funcs);

	connector->interlace_allowed = 1;
	connector->doublescan_allowed = 0;
	connector->stereo_allowed = 1;

	if (INTEL_GEN(dev_priv) >= 10 || IS_GEMINILAKE(dev_priv))
		connector->ycbcr_420_allowed = true;

	intel_hdmi->ddc_bus = intel_hdmi_ddc_pin(dev_priv, port);

	if (WARN_ON(port == PORT_A))
		return;
	intel_encoder->hpd_pin = intel_hpd_pin_default(dev_priv, port);

	if (HAS_DDI(dev_priv))
		intel_connector->get_hw_state = intel_ddi_connector_get_hw_state;
	else
		intel_connector->get_hw_state = intel_connector_get_hw_state;

	intel_hdmi_add_properties(intel_hdmi, connector);


	intel_connector_attach_encoder(intel_connector, intel_encoder);
	intel_hdmi->attached_connector = intel_connector;

	/* For G4X desktop chip, PEG_BAND_GAP_DATA 3:0 must first be written
	 * 0xd.  Failure to do so will result in spurious interrupts being
	 * generated on the port when a cable is not attached.
	 */
	if (IS_G4X(dev_priv) && !IS_GM45(dev_priv)) {
		u32 temp = I915_READ(PEG_BAND_GAP_DATA);
		I915_WRITE(PEG_BAND_GAP_DATA, (temp & ~0xf) | 0xd);
	}
}

