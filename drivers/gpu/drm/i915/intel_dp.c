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
 *    Keith Packard <keithp@keithp.com>
 *
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/types.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <asm/byteorder.h>
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_hdcp.h>
#include "intel_drv.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"

#define DP_LINK_CHECK_TIMEOUT	(10 * 1000)
#define DP_DPRX_ESI_LEN 14

/* Compliance test status bits  */
#define INTEL_DP_RESOLUTION_SHIFT_MASK	0
#define INTEL_DP_RESOLUTION_PREFERRED	(1 << INTEL_DP_RESOLUTION_SHIFT_MASK)
#define INTEL_DP_RESOLUTION_STANDARD	(2 << INTEL_DP_RESOLUTION_SHIFT_MASK)
#define INTEL_DP_RESOLUTION_FAILSAFE	(3 << INTEL_DP_RESOLUTION_SHIFT_MASK)

struct dp_link_dpll {
	int clock;
	struct dpll dpll;
};

static const struct dp_link_dpll gen4_dpll[] = {
	{ 162000,
		{ .p1 = 2, .p2 = 10, .n = 2, .m1 = 23, .m2 = 8 } },
	{ 270000,
		{ .p1 = 1, .p2 = 10, .n = 1, .m1 = 14, .m2 = 2 } }
};

static const struct dp_link_dpll pch_dpll[] = {
	{ 162000,
		{ .p1 = 2, .p2 = 10, .n = 1, .m1 = 12, .m2 = 9 } },
	{ 270000,
		{ .p1 = 1, .p2 = 10, .n = 2, .m1 = 14, .m2 = 8 } }
};

static const struct dp_link_dpll vlv_dpll[] = {
	{ 162000,
		{ .p1 = 3, .p2 = 2, .n = 5, .m1 = 3, .m2 = 81 } },
	{ 270000,
		{ .p1 = 2, .p2 = 2, .n = 1, .m1 = 2, .m2 = 27 } }
};

/*
 * CHV supports eDP 1.4 that have  more link rates.
 * Below only provides the fixed rate but exclude variable rate.
 */
static const struct dp_link_dpll chv_dpll[] = {
	/*
	 * CHV requires to program fractional division for m2.
	 * m2 is stored in fixed point format using formula below
	 * (m2_int << 22) | m2_fraction
	 */
	{ 162000,	/* m2_int = 32, m2_fraction = 1677722 */
		{ .p1 = 4, .p2 = 2, .n = 1, .m1 = 2, .m2 = 0x819999a } },
	{ 270000,	/* m2_int = 27, m2_fraction = 0 */
		{ .p1 = 4, .p2 = 1, .n = 1, .m1 = 2, .m2 = 0x6c00000 } },
	{ 540000,	/* m2_int = 27, m2_fraction = 0 */
		{ .p1 = 2, .p2 = 1, .n = 1, .m1 = 2, .m2 = 0x6c00000 } }
};

/**
 * intel_dp_is_edp - is the given port attached to an eDP panel (either CPU or PCH)
 * @intel_dp: DP struct
 *
 * If a CPU or PCH DP output is attached to an eDP panel, this function
 * will return true, and false otherwise.
 */
bool intel_dp_is_edp(struct intel_dp *intel_dp)
{
	struct intel_digital_port *intel_dig_port = dp_to_dig_port(intel_dp);

	return intel_dig_port->base.type == INTEL_OUTPUT_EDP;
}

static struct drm_device *intel_dp_to_dev(struct intel_dp *intel_dp)
{
	struct intel_digital_port *intel_dig_port = dp_to_dig_port(intel_dp);

	return intel_dig_port->base.base.dev;
}

static struct intel_dp *intel_attached_dp(struct drm_connector *connector)
{
	return enc_to_intel_dp(&intel_attached_encoder(connector)->base);
}

static void intel_dp_link_down(struct intel_encoder *encoder,
			       const struct intel_crtc_state *old_crtc_state);
static bool edp_panel_vdd_on(struct intel_dp *intel_dp);
static void edp_panel_vdd_off(struct intel_dp *intel_dp, bool sync);
static void vlv_init_panel_power_sequencer(struct intel_encoder *encoder,
					   const struct intel_crtc_state *crtc_state);
static void vlv_steal_power_sequencer(struct drm_i915_private *dev_priv,
				      enum pipe pipe);
static void intel_dp_unset_edid(struct intel_dp *intel_dp);


static void
intel_dp_set_source_rates(struct intel_dp *intel_dp)
{
	/* The values must be in increasing order */
	static const int cnl_rates[] = {
		162000, 216000, 270000, 324000, 432000, 540000, 648000, 810000
	};
	static const int bxt_rates[] = {
		162000, 216000, 243000, 270000, 324000, 432000, 540000
	};
	static const int skl_rates[] = {
		162000, 216000, 270000, 324000, 432000, 540000
	};
	static const int hsw_rates[] = {
		162000, 270000, 540000
	};
	static const int g4x_rates[] = {
		162000, 270000
	};
	struct intel_digital_port *dig_port = dp_to_dig_port(intel_dp);
	struct drm_i915_private *dev_priv = to_i915(dig_port->base.base.dev);
	const struct ddi_vbt_port_info *info =
		&dev_priv->vbt.ddi_port_info[dig_port->base.port];
	const int *source_rates;
	int size, max_rate = 0, vbt_max_rate = info->dp_max_link_rate;

	/* This should only be done once */
	WARN_ON(intel_dp->source_rates || intel_dp->num_source_rates);

    source_rates = hsw_rates;
    size = ARRAY_SIZE(hsw_rates);

	intel_dp->source_rates = source_rates;
	intel_dp->num_source_rates = size;
}



static void intel_dp_unpack_aux(uint32_t src, uint8_t *dst, int dst_bytes)
{
	int i;
	if (dst_bytes > 4)
		dst_bytes = 4;
	for (i = 0; i < dst_bytes; i++)
		dst[i] = src >> ((3-i) * 8);
}

static void
intel_dp_init_panel_power_sequencer(struct intel_dp *intel_dp);
static void
intel_dp_init_panel_power_sequencer_registers(struct intel_dp *intel_dp,
					      bool force_disable_vdd);
static void
intel_dp_pps_init(struct intel_dp *intel_dp);

static void pps_lock(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));

	/*
	 * See intel_power_sequencer_reset() why we need
	 * a power domain reference here.
	 */
	intel_display_power_get(dev_priv, intel_dp->aux_power_domain);

	mutex_lock(&dev_priv->pps_mutex);
}

static void pps_unlock(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));

	mutex_unlock(&dev_priv->pps_mutex);

	intel_display_power_put(dev_priv, intel_dp->aux_power_domain);
}



static void
intel_dp_check_edp(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));

	if (!intel_dp_is_edp(intel_dp))
		return;
}

static uint32_t
intel_dp_aux_wait_done(struct intel_dp *intel_dp, bool has_aux_irq)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	i915_reg_t ch_ctl = intel_dp->aux_ch_ctl_reg(intel_dp);
	uint32_t status;
	bool done;

#define C (((status = I915_READ_NOTRACE(ch_ctl)) & DP_AUX_CH_CTL_SEND_BUSY) == 0)
	if (has_aux_irq)
		done = wait_event_timeout(dev_priv->gmbus_wait_queue, C,
					  msecs_to_jiffies_timeout(10));
	else
		done = wait_for(C, 10) == 0;
	if (!done)
		DRM_ERROR("dp aux hw did not signal timeout (has irq: %i)!\n",
			  has_aux_irq);
#undef C

	return status;
}


static uint32_t hsw_get_aux_clock_divider(struct intel_dp *intel_dp, int index)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));

	if (intel_dp->aux_ch != AUX_CH_A && HAS_PCH_LPT_H(dev_priv)) {
		/* Workaround for non-ULT HSW */
		switch (index) {
		case 0: return 63;
		case 1: return 72;
		default: return 0;
		}
	}
    return 0;
}

static uint32_t g4x_get_aux_send_ctl(struct intel_dp *intel_dp,
				     bool has_aux_irq,
				     int send_bytes,
				     uint32_t aux_clock_divider)
{
	struct intel_digital_port *intel_dig_port = dp_to_dig_port(intel_dp);
	struct drm_i915_private *dev_priv =
			to_i915(intel_dig_port->base.base.dev);
	uint32_t precharge, timeout;

	if (IS_GEN6(dev_priv))
		precharge = 3;
	else
		precharge = 5;

	if (IS_BROADWELL(dev_priv))
		timeout = DP_AUX_CH_CTL_TIME_OUT_600us;
	else
		timeout = DP_AUX_CH_CTL_TIME_OUT_400us;

	return DP_AUX_CH_CTL_SEND_BUSY |
	       DP_AUX_CH_CTL_DONE |
	       (has_aux_irq ? DP_AUX_CH_CTL_INTERRUPT : 0) |
	       DP_AUX_CH_CTL_TIME_OUT_ERROR |
	       timeout |
	       DP_AUX_CH_CTL_RECEIVE_ERROR |
	       (send_bytes << DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT) |
	       (precharge << DP_AUX_CH_CTL_PRECHARGE_2US_SHIFT) |
	       (aux_clock_divider << DP_AUX_CH_CTL_BIT_CLOCK_2X_SHIFT);
}

static uint32_t skl_get_aux_send_ctl(struct intel_dp *intel_dp,
				      bool has_aux_irq,
				      int send_bytes,
				      uint32_t unused)
{
	return DP_AUX_CH_CTL_SEND_BUSY |
	       DP_AUX_CH_CTL_DONE |
	       (has_aux_irq ? DP_AUX_CH_CTL_INTERRUPT : 0) |
	       DP_AUX_CH_CTL_TIME_OUT_ERROR |
	       DP_AUX_CH_CTL_TIME_OUT_MAX |
	       DP_AUX_CH_CTL_RECEIVE_ERROR |
	       (send_bytes << DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT) |
	       DP_AUX_CH_CTL_FW_SYNC_PULSE_SKL(32) |
	       DP_AUX_CH_CTL_SYNC_PULSE_SKL(32);
}
int intel_dp_retrain_link(struct intel_encoder *encoder,struct drm_modeset_acquire_ctx *ctx) {
}
uint32_t intel_dp_pack_aux(const uint8_t *src, int src_bytes)
{
    int i;
    uint32_t v = 0;

    if (src_bytes > 4)
        src_bytes = 4;
    for (i = 0; i < src_bytes; i++)
        v |= ((uint32_t) src[i]) << ((3-i) * 8);
    return v;
}


static int
intel_dp_aux_xfer(struct intel_dp *intel_dp,
		  const uint8_t *send, int send_bytes,
		  uint8_t *recv, int recv_size,
		  u32 aux_send_ctl_flags)
{
	struct intel_digital_port *intel_dig_port = dp_to_dig_port(intel_dp);
	struct drm_i915_private *dev_priv =
			to_i915(intel_dig_port->base.base.dev);
	i915_reg_t ch_ctl, ch_data[5];
	uint32_t aux_clock_divider;
	int i, ret, recv_bytes;
	uint32_t status;
	int try, clock = 0;
	bool has_aux_irq = HAS_AUX_IRQ(dev_priv);
	bool vdd;

	ch_ctl = intel_dp->aux_ch_ctl_reg(intel_dp);
	for (i = 0; i < ARRAY_SIZE(ch_data); i++)
		ch_data[i] = intel_dp->aux_ch_data_reg(intel_dp, i);

	pps_lock(intel_dp);

	/*
	 * We will be called with VDD already enabled for dpcd/edid/oui reads.
	 * In such cases we want to leave VDD enabled and it's up to upper layers
	 * to turn it off. But for eg. i2c-dev access we need to turn it on/off
	 * ourselves.
	 */
	vdd = edp_panel_vdd_on(intel_dp);

	/* dp aux is extremely sensitive to irq latency, hence request the
	 * lowest possible wakeup latency and so prevent the cpu from going into
	 * deep sleep states.
	 */
	pm_qos_update_request(&dev_priv->pm_qos, 0);

	intel_dp_check_edp(intel_dp);

	/* Try to wait for any previous AUX channel activity */
	for (try = 0; try < 3; try++) {
		status = I915_READ_NOTRACE(ch_ctl);
		if ((status & DP_AUX_CH_CTL_SEND_BUSY) == 0)
			break;
		msleep(1);
	}

	if (try == 3) {
		static u32 last_status = -1;
		const u32 status = I915_READ(ch_ctl);

		if (status != last_status) {
			WARN(1, "dp_aux_ch not started status 0x%08x\n",
			     status);
			last_status = status;
		}

		ret = -EBUSY;
		goto out;
	}

	/* Only 5 data registers! */
	if (WARN_ON(send_bytes > 20 || recv_size > 20)) {
		ret = -E2BIG;
		goto out;
	}

	while ((aux_clock_divider = intel_dp->get_aux_clock_divider(intel_dp, clock++))) {
		u32 send_ctl = intel_dp->get_aux_send_ctl(intel_dp,
							  has_aux_irq,
							  send_bytes,
							  aux_clock_divider);

		send_ctl |= aux_send_ctl_flags;

		/* Must try at least 3 times according to DP spec */
		for (try = 0; try < 5; try++) {
			/* Load the send data into the aux channel data registers */
			for (i = 0; i < send_bytes; i += 4)
				I915_WRITE(ch_data[i >> 2],
					   intel_dp_pack_aux(send + i,
							     send_bytes - i));

			/* Send the command and wait for it to complete */
			I915_WRITE(ch_ctl, send_ctl);

			status = intel_dp_aux_wait_done(intel_dp, has_aux_irq);

			/* Clear done status and any errors */
			I915_WRITE(ch_ctl,
				   status |
				   DP_AUX_CH_CTL_DONE |
				   DP_AUX_CH_CTL_TIME_OUT_ERROR |
				   DP_AUX_CH_CTL_RECEIVE_ERROR);

			/* DP CTS 1.2 Core Rev 1.1, 4.2.1.1 & 4.2.1.2
			 *   400us delay required for errors and timeouts
			 *   Timeout errors from the HW already meet this
			 *   requirement so skip to next iteration
			 */
			if (status & DP_AUX_CH_CTL_TIME_OUT_ERROR)
				continue;

			if (status & DP_AUX_CH_CTL_RECEIVE_ERROR) {
				usleep_range(400, 500);
				continue;
			}
			if (status & DP_AUX_CH_CTL_DONE)
				goto done;
		}
	}

	if ((status & DP_AUX_CH_CTL_DONE) == 0) {
		DRM_ERROR("dp_aux_ch not done status 0x%08x\n", status);
		ret = -EBUSY;
		goto out;
	}

done:
	/* Check for timeout or receive error.
	 * Timeouts occur when the sink is not connected
	 */
	if (status & DP_AUX_CH_CTL_RECEIVE_ERROR) {
		DRM_ERROR("dp_aux_ch receive error status 0x%08x\n", status);
		ret = -EIO;
		goto out;
	}

	/* Timeouts occur when the device isn't connected, so they're
	 * "normal" -- don't fill the kernel log with these */
	if (status & DP_AUX_CH_CTL_TIME_OUT_ERROR) {
		DRM_DEBUG_KMS("dp_aux_ch timeout status 0x%08x\n", status);
		ret = -ETIMEDOUT;
		goto out;
	}

	/* Unload any bytes sent back from the other side */
	recv_bytes = ((status & DP_AUX_CH_CTL_MESSAGE_SIZE_MASK) >>
		      DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT);

	/*
	 * By BSpec: "Message sizes of 0 or >20 are not allowed."
	 * We have no idea of what happened so we return -EBUSY so
	 * drm layer takes care for the necessary retries.
	 */
	if (recv_bytes == 0 || recv_bytes > 20) {
		DRM_DEBUG_KMS("Forbidden recv_bytes = %d on aux transaction\n",
			      recv_bytes);
		ret = -EBUSY;
		goto out;
	}

	if (recv_bytes > recv_size)
		recv_bytes = recv_size;

	for (i = 0; i < recv_bytes; i += 4)
		intel_dp_unpack_aux(I915_READ(ch_data[i >> 2]),
				    recv + i, recv_bytes - i);

	ret = recv_bytes;
out:
	pm_qos_update_request(&dev_priv->pm_qos, PM_QOS_DEFAULT_VALUE);

	if (vdd)
		edp_panel_vdd_off(intel_dp, false);

	pps_unlock(intel_dp);

	return ret;
}

#define BARE_ADDRESS_SIZE	3
#define HEADER_SIZE		(BARE_ADDRESS_SIZE + 1)

static void
intel_dp_aux_header(u8 txbuf[HEADER_SIZE],
		    const struct drm_dp_aux_msg *msg)
{
	txbuf[0] = (msg->request << 4) | ((msg->address >> 16) & 0xf);
	txbuf[1] = (msg->address >> 8) & 0xff;
	txbuf[2] = msg->address & 0xff;
	txbuf[3] = msg->size - 1;
}

static ssize_t
intel_dp_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg)
{
	struct intel_dp *intel_dp = container_of(aux, struct intel_dp, aux);
	uint8_t txbuf[20], rxbuf[20];
	size_t txsize, rxsize;
	int ret;

	intel_dp_aux_header(txbuf, msg);

	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
	case DP_AUX_I2C_WRITE_STATUS_UPDATE:
		txsize = msg->size ? HEADER_SIZE + msg->size : BARE_ADDRESS_SIZE;
		rxsize = 2; /* 0 or 1 data bytes */

		if (WARN_ON(txsize > 20))
			return -E2BIG;

		WARN_ON(!msg->buffer != !msg->size);

		if (msg->buffer)
			memcpy(txbuf + HEADER_SIZE, msg->buffer, msg->size);

		ret = intel_dp_aux_xfer(intel_dp, txbuf, txsize,
					rxbuf, rxsize, 0);
		if (ret > 0) {
			msg->reply = rxbuf[0] >> 4;

			if (ret > 1) {
				/* Number of bytes written in a short write. */
				ret = clamp_t(int, rxbuf[1], 0, msg->size);
			} else {
				/* Return payload size. */
				ret = msg->size;
			}
		}
		break;

	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		txsize = msg->size ? HEADER_SIZE : BARE_ADDRESS_SIZE;
		rxsize = msg->size + 1;

		if (WARN_ON(rxsize > 20))
			return -E2BIG;

		ret = intel_dp_aux_xfer(intel_dp, txbuf, txsize,
					rxbuf, rxsize, 0);
		if (ret > 0) {
			msg->reply = rxbuf[0] >> 4;
			/*
			 * Assume happy day, and copy the data. The caller is
			 * expected to check msg->reply before touching it.
			 *
			 * Return payload size.
			 */
			ret--;
			memcpy(msg->buffer, rxbuf + 1, ret);
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum aux_ch intel_aux_ch(struct intel_dp *intel_dp)
{
	struct intel_encoder *encoder = &dp_to_dig_port(intel_dp)->base;
	struct drm_i915_private *dev_priv = to_i915(encoder->base.dev);
	enum port port = encoder->port;
	const struct ddi_vbt_port_info *info =
		&dev_priv->vbt.ddi_port_info[port];
	enum aux_ch aux_ch;

	if (!info->alternate_aux_channel) {
		aux_ch = (enum aux_ch) port;

		DRM_DEBUG_KMS("using AUX %c for port %c (platform default)\n",
			      aux_ch_name(aux_ch), port_name(port));
		return aux_ch;
	}

	switch (info->alternate_aux_channel) {
	case DP_AUX_A:
		aux_ch = AUX_CH_A;
		break;
	case DP_AUX_B:
		aux_ch = AUX_CH_B;
		break;
	case DP_AUX_C:
		aux_ch = AUX_CH_C;
		break;
	case DP_AUX_D:
		aux_ch = AUX_CH_D;
		break;
	case DP_AUX_F:
		aux_ch = AUX_CH_F;
		break;
	default:
		MISSING_CASE(info->alternate_aux_channel);
		aux_ch = AUX_CH_A;
		break;
	}

	DRM_DEBUG_KMS("using AUX %c for port %c (VBT)\n",
		      aux_ch_name(aux_ch), port_name(port));

	return aux_ch;
}

static enum intel_display_power_domain
intel_aux_power_domain(struct intel_dp *intel_dp)
{
	switch (intel_dp->aux_ch) {
	case AUX_CH_A:
		return POWER_DOMAIN_AUX_A;
	case AUX_CH_B:
		return POWER_DOMAIN_AUX_B;
	case AUX_CH_C:
		return POWER_DOMAIN_AUX_C;
	case AUX_CH_D:
		return POWER_DOMAIN_AUX_D;
	case AUX_CH_F:
		return POWER_DOMAIN_AUX_F;
	default:
		MISSING_CASE(intel_dp->aux_ch);
		return POWER_DOMAIN_AUX_A;
	}
}

static i915_reg_t g4x_aux_ctl_reg(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	enum aux_ch aux_ch = intel_dp->aux_ch;

	switch (aux_ch) {
	case AUX_CH_B:
	case AUX_CH_C:
	case AUX_CH_D:
		return DP_AUX_CH_CTL(aux_ch);
	default:
		MISSING_CASE(aux_ch);
		return DP_AUX_CH_CTL(AUX_CH_B);
	}
}

static i915_reg_t g4x_aux_data_reg(struct intel_dp *intel_dp, int index)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	enum aux_ch aux_ch = intel_dp->aux_ch;

	switch (aux_ch) {
	case AUX_CH_B:
	case AUX_CH_C:
	case AUX_CH_D:
		return DP_AUX_CH_DATA(aux_ch, index);
	default:
		MISSING_CASE(aux_ch);
		return DP_AUX_CH_DATA(AUX_CH_B, index);
	}
}

static i915_reg_t ilk_aux_ctl_reg(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	enum aux_ch aux_ch = intel_dp->aux_ch;

	switch (aux_ch) {
	case AUX_CH_A:
		return DP_AUX_CH_CTL(aux_ch);
	case AUX_CH_B:
	case AUX_CH_C:
	case AUX_CH_D:
		return PCH_DP_AUX_CH_CTL(aux_ch);
	default:
		MISSING_CASE(aux_ch);
		return DP_AUX_CH_CTL(AUX_CH_A);
	}
}

static i915_reg_t ilk_aux_data_reg(struct intel_dp *intel_dp, int index)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	enum aux_ch aux_ch = intel_dp->aux_ch;

	switch (aux_ch) {
	case AUX_CH_A:
		return DP_AUX_CH_DATA(aux_ch, index);
	case AUX_CH_B:
	case AUX_CH_C:
	case AUX_CH_D:
		return PCH_DP_AUX_CH_DATA(aux_ch, index);
	default:
		MISSING_CASE(aux_ch);
		return DP_AUX_CH_DATA(AUX_CH_A, index);
	}
}

static i915_reg_t skl_aux_ctl_reg(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	enum aux_ch aux_ch = intel_dp->aux_ch;

	switch (aux_ch) {
	case AUX_CH_A:
	case AUX_CH_B:
	case AUX_CH_C:
	case AUX_CH_D:
	case AUX_CH_F:
		return DP_AUX_CH_CTL(aux_ch);
	default:
		MISSING_CASE(aux_ch);
		return DP_AUX_CH_CTL(AUX_CH_A);
	}
}

static i915_reg_t skl_aux_data_reg(struct intel_dp *intel_dp, int index)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	enum aux_ch aux_ch = intel_dp->aux_ch;

	switch (aux_ch) {
	case AUX_CH_A:
	case AUX_CH_B:
	case AUX_CH_C:
	case AUX_CH_D:
	case AUX_CH_F:
		return DP_AUX_CH_DATA(aux_ch, index);
	default:
		MISSING_CASE(aux_ch);
		return DP_AUX_CH_DATA(AUX_CH_A, index);
	}
}

static void
intel_dp_aux_fini(struct intel_dp *intel_dp)
{
	kfree(intel_dp->aux.name);
}

static void
intel_dp_aux_init(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	struct intel_encoder *encoder = &dp_to_dig_port(intel_dp)->base;

	intel_dp->aux_ch = intel_aux_ch(intel_dp);
	intel_dp->aux_power_domain = intel_aux_power_domain(intel_dp);

    intel_dp->aux_ch_ctl_reg = ilk_aux_ctl_reg;
    intel_dp->aux_ch_data_reg = ilk_aux_data_reg;

    intel_dp->get_aux_clock_divider = hsw_get_aux_clock_divider;

    intel_dp->get_aux_send_ctl = g4x_get_aux_send_ctl;

	drm_dp_aux_init(&intel_dp->aux);

	/* Failure to allocate our preferred name is not critical */
	intel_dp->aux.name = kasprintf(GFP_KERNEL, "DPDDC-%c",
				       port_name(encoder->port));
	intel_dp->aux.transfer = intel_dp_aux_transfer;
}

/*
 * Must be paired with edp_panel_vdd_off().
 * Must hold pps_mutex around the whole on/off sequence.
 * Can be nested with intel_edp_panel_vdd_{on,off}() calls.
 */
static bool edp_panel_vdd_on(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	struct intel_digital_port *intel_dig_port = dp_to_dig_port(intel_dp);
	u32 pp;
	i915_reg_t pp_stat_reg, pp_ctrl_reg;
	bool need_to_disable = !intel_dp->want_panel_vdd;

	lockdep_assert_held(&dev_priv->pps_mutex);

    return false;

}

static void intel_disable_dp(struct intel_encoder *encoder,
			     const struct intel_crtc_state *old_crtc_state,
			     const struct drm_connector_state *old_conn_state)
{
	struct intel_dp *intel_dp = enc_to_intel_dp(&encoder->base);

	intel_dp->link_trained = false;


	/* Make sure the panel is off before trying to change the mode. But also
	 * ensure that we have vdd while we switch off the panel. */
	intel_edp_panel_vdd_on(intel_dp);
	intel_edp_backlight_off(old_conn_state);
	intel_dp_sink_dpms(intel_dp, DRM_MODE_DPMS_OFF);
	intel_edp_panel_off(intel_dp);
}


static void intel_dp_enable_port(struct intel_dp *intel_dp,
				 const struct intel_crtc_state *old_crtc_state)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));

	/* enable with pattern 1 (as per spec) */

	intel_dp_program_link_training_pattern(intel_dp, DP_TRAINING_PATTERN_1);

	/*
	 * Magic for VLV/CHV. We _must_ first set up the register
	 * without actually enabling the port, and then do another
	 * write to enable the port. Otherwise link training will
	 * fail when the power sequencer is freshly used for this port.
	 */
	intel_dp->DP |= DP_PORT_EN;
	if (old_crtc_state->has_audio)
		intel_dp->DP |= DP_AUDIO_OUTPUT_ENABLE;

	I915_WRITE(intel_dp->output_reg, intel_dp->DP);
	POSTING_READ(intel_dp->output_reg);
}

/*
 * Fetch AUX CH registers 0x202 - 0x207 which contain
 * link status information
 */
bool
intel_dp_get_link_status(struct intel_dp *intel_dp, uint8_t link_status[DP_LINK_STATUS_SIZE])
{
	return drm_dp_dpcd_read(&intel_dp->aux, DP_LANE0_1_STATUS, link_status,
				DP_LINK_STATUS_SIZE) == DP_LINK_STATUS_SIZE;
}

/* These are source-specific values. */
uint8_t
intel_dp_voltage_max(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	enum port port = dp_to_dig_port(intel_dp)->base.port;

	if (INTEL_GEN(dev_priv) >= 9) {
		struct intel_encoder *encoder = &dp_to_dig_port(intel_dp)->base;
		return intel_ddi_dp_voltage_max(encoder);
	} else if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv))
		return DP_TRAIN_VOLTAGE_SWING_LEVEL_3;
	else if (IS_GEN7(dev_priv) && port == PORT_A)
		return DP_TRAIN_VOLTAGE_SWING_LEVEL_2;
	else if (HAS_PCH_CPT(dev_priv) && port != PORT_A)
		return DP_TRAIN_VOLTAGE_SWING_LEVEL_3;
	else
		return DP_TRAIN_VOLTAGE_SWING_LEVEL_2;
}

uint8_t
intel_dp_pre_emphasis_max(struct intel_dp *intel_dp, uint8_t voltage_swing)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	enum port port = dp_to_dig_port(intel_dp)->base.port;

	if (INTEL_GEN(dev_priv) >= 9) {
		switch (voltage_swing & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			return DP_TRAIN_PRE_EMPH_LEVEL_3;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			return DP_TRAIN_PRE_EMPH_LEVEL_2;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
			return DP_TRAIN_PRE_EMPH_LEVEL_1;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_3:
			return DP_TRAIN_PRE_EMPH_LEVEL_0;
		default:
			return DP_TRAIN_PRE_EMPH_LEVEL_0;
		}
	} else if (IS_HASWELL(dev_priv) || IS_BROADWELL(dev_priv)) {
		switch (voltage_swing & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			return DP_TRAIN_PRE_EMPH_LEVEL_3;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			return DP_TRAIN_PRE_EMPH_LEVEL_2;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
			return DP_TRAIN_PRE_EMPH_LEVEL_1;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_3:
		default:
			return DP_TRAIN_PRE_EMPH_LEVEL_0;
		}
	} else if (IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) {
		switch (voltage_swing & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			return DP_TRAIN_PRE_EMPH_LEVEL_3;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			return DP_TRAIN_PRE_EMPH_LEVEL_2;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
			return DP_TRAIN_PRE_EMPH_LEVEL_1;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_3:
		default:
			return DP_TRAIN_PRE_EMPH_LEVEL_0;
		}
	} else if (IS_GEN7(dev_priv) && port == PORT_A) {
		switch (voltage_swing & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			return DP_TRAIN_PRE_EMPH_LEVEL_2;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
			return DP_TRAIN_PRE_EMPH_LEVEL_1;
		default:
			return DP_TRAIN_PRE_EMPH_LEVEL_0;
		}
	} else {
		switch (voltage_swing & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			return DP_TRAIN_PRE_EMPH_LEVEL_2;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			return DP_TRAIN_PRE_EMPH_LEVEL_2;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
			return DP_TRAIN_PRE_EMPH_LEVEL_1;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_3:
		default:
			return DP_TRAIN_PRE_EMPH_LEVEL_0;
		}
	}
}

static uint32_t vlv_signal_levels(struct intel_dp *intel_dp)
{
	struct intel_encoder *encoder = &dp_to_dig_port(intel_dp)->base;
	unsigned long demph_reg_value, preemph_reg_value,
		uniqtranscale_reg_value;
	uint8_t train_set = intel_dp->train_set[0];

	switch (train_set & DP_TRAIN_PRE_EMPHASIS_MASK) {
	case DP_TRAIN_PRE_EMPH_LEVEL_0:
		preemph_reg_value = 0x0004000;
		switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			demph_reg_value = 0x2B405555;
			uniqtranscale_reg_value = 0x552AB83A;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			demph_reg_value = 0x2B404040;
			uniqtranscale_reg_value = 0x5548B83A;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
			demph_reg_value = 0x2B245555;
			uniqtranscale_reg_value = 0x5560B83A;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_3:
			demph_reg_value = 0x2B405555;
			uniqtranscale_reg_value = 0x5598DA3A;
			break;
		default:
			return 0;
		}
		break;
	case DP_TRAIN_PRE_EMPH_LEVEL_1:
		preemph_reg_value = 0x0002000;
		switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			demph_reg_value = 0x2B404040;
			uniqtranscale_reg_value = 0x5552B83A;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			demph_reg_value = 0x2B404848;
			uniqtranscale_reg_value = 0x5580B83A;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
			demph_reg_value = 0x2B404040;
			uniqtranscale_reg_value = 0x55ADDA3A;
			break;
		default:
			return 0;
		}
		break;
	case DP_TRAIN_PRE_EMPH_LEVEL_2:
		preemph_reg_value = 0x0000000;
		switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			demph_reg_value = 0x2B305555;
			uniqtranscale_reg_value = 0x5570B83A;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			demph_reg_value = 0x2B2B4040;
			uniqtranscale_reg_value = 0x55ADDA3A;
			break;
		default:
			return 0;
		}
		break;
	case DP_TRAIN_PRE_EMPH_LEVEL_3:
		preemph_reg_value = 0x0006000;
		switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			demph_reg_value = 0x1B405555;
			uniqtranscale_reg_value = 0x55ADDA3A;
			break;
		default:
			return 0;
		}
		break;
	default:
		return 0;
	}

	vlv_set_phy_signal_level(encoder, demph_reg_value, preemph_reg_value,
				 uniqtranscale_reg_value, 0);

	return 0;
}

static uint32_t chv_signal_levels(struct intel_dp *intel_dp)
{
	struct intel_encoder *encoder = &dp_to_dig_port(intel_dp)->base;
	u32 deemph_reg_value, margin_reg_value;
	bool uniq_trans_scale = false;
	uint8_t train_set = intel_dp->train_set[0];

	switch (train_set & DP_TRAIN_PRE_EMPHASIS_MASK) {
	case DP_TRAIN_PRE_EMPH_LEVEL_0:
		switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			deemph_reg_value = 128;
			margin_reg_value = 52;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			deemph_reg_value = 128;
			margin_reg_value = 77;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
			deemph_reg_value = 128;
			margin_reg_value = 102;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_3:
			deemph_reg_value = 128;
			margin_reg_value = 154;
			uniq_trans_scale = true;
			break;
		default:
			return 0;
		}
		break;
	case DP_TRAIN_PRE_EMPH_LEVEL_1:
		switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			deemph_reg_value = 85;
			margin_reg_value = 78;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			deemph_reg_value = 85;
			margin_reg_value = 116;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
			deemph_reg_value = 85;
			margin_reg_value = 154;
			break;
		default:
			return 0;
		}
		break;
	case DP_TRAIN_PRE_EMPH_LEVEL_2:
		switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			deemph_reg_value = 64;
			margin_reg_value = 104;
			break;
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
			deemph_reg_value = 64;
			margin_reg_value = 154;
			break;
		default:
			return 0;
		}
		break;
	case DP_TRAIN_PRE_EMPH_LEVEL_3:
		switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
			deemph_reg_value = 43;
			margin_reg_value = 154;
			break;
		default:
			return 0;
		}
		break;
	default:
		return 0;
	}

	chv_set_phy_signal_level(encoder, deemph_reg_value,
				 margin_reg_value, uniq_trans_scale);

	return 0;
}

static uint32_t
gen4_signal_levels(uint8_t train_set)
{
	uint32_t	signal_levels = 0;

	switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_0:
	default:
		signal_levels |= DP_VOLTAGE_0_4;
		break;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_1:
		signal_levels |= DP_VOLTAGE_0_6;
		break;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_2:
		signal_levels |= DP_VOLTAGE_0_8;
		break;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_3:
		signal_levels |= DP_VOLTAGE_1_2;
		break;
	}
	switch (train_set & DP_TRAIN_PRE_EMPHASIS_MASK) {
	case DP_TRAIN_PRE_EMPH_LEVEL_0:
	default:
		signal_levels |= DP_PRE_EMPHASIS_0;
		break;
	case DP_TRAIN_PRE_EMPH_LEVEL_1:
		signal_levels |= DP_PRE_EMPHASIS_3_5;
		break;
	case DP_TRAIN_PRE_EMPH_LEVEL_2:
		signal_levels |= DP_PRE_EMPHASIS_6;
		break;
	case DP_TRAIN_PRE_EMPH_LEVEL_3:
		signal_levels |= DP_PRE_EMPHASIS_9_5;
		break;
	}
	return signal_levels;
}

/* Gen6's DP voltage swing and pre-emphasis control */
static uint32_t
gen6_edp_signal_levels(uint8_t train_set)
{
	int signal_levels = train_set & (DP_TRAIN_VOLTAGE_SWING_MASK |
					 DP_TRAIN_PRE_EMPHASIS_MASK);
	switch (signal_levels) {
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_0 | DP_TRAIN_PRE_EMPH_LEVEL_0:
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_1 | DP_TRAIN_PRE_EMPH_LEVEL_0:
		return EDP_LINK_TRAIN_400_600MV_0DB_SNB_B;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_0 | DP_TRAIN_PRE_EMPH_LEVEL_1:
		return EDP_LINK_TRAIN_400MV_3_5DB_SNB_B;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_0 | DP_TRAIN_PRE_EMPH_LEVEL_2:
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_1 | DP_TRAIN_PRE_EMPH_LEVEL_2:
		return EDP_LINK_TRAIN_400_600MV_6DB_SNB_B;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_1 | DP_TRAIN_PRE_EMPH_LEVEL_1:
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_2 | DP_TRAIN_PRE_EMPH_LEVEL_1:
		return EDP_LINK_TRAIN_600_800MV_3_5DB_SNB_B;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_2 | DP_TRAIN_PRE_EMPH_LEVEL_0:
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_3 | DP_TRAIN_PRE_EMPH_LEVEL_0:
		return EDP_LINK_TRAIN_800_1200MV_0DB_SNB_B;
	default:
		DRM_DEBUG_KMS("Unsupported voltage swing/pre-emphasis level:"
			      "0x%x\n", signal_levels);
		return EDP_LINK_TRAIN_400_600MV_0DB_SNB_B;
	}
}

/* Gen7's DP voltage swing and pre-emphasis control */
static uint32_t
gen7_edp_signal_levels(uint8_t train_set)
{
	int signal_levels = train_set & (DP_TRAIN_VOLTAGE_SWING_MASK |
					 DP_TRAIN_PRE_EMPHASIS_MASK);
	switch (signal_levels) {
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_0 | DP_TRAIN_PRE_EMPH_LEVEL_0:
		return EDP_LINK_TRAIN_400MV_0DB_IVB;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_0 | DP_TRAIN_PRE_EMPH_LEVEL_1:
		return EDP_LINK_TRAIN_400MV_3_5DB_IVB;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_0 | DP_TRAIN_PRE_EMPH_LEVEL_2:
		return EDP_LINK_TRAIN_400MV_6DB_IVB;

	case DP_TRAIN_VOLTAGE_SWING_LEVEL_1 | DP_TRAIN_PRE_EMPH_LEVEL_0:
		return EDP_LINK_TRAIN_600MV_0DB_IVB;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_1 | DP_TRAIN_PRE_EMPH_LEVEL_1:
		return EDP_LINK_TRAIN_600MV_3_5DB_IVB;

	case DP_TRAIN_VOLTAGE_SWING_LEVEL_2 | DP_TRAIN_PRE_EMPH_LEVEL_0:
		return EDP_LINK_TRAIN_800MV_0DB_IVB;
	case DP_TRAIN_VOLTAGE_SWING_LEVEL_2 | DP_TRAIN_PRE_EMPH_LEVEL_1:
		return EDP_LINK_TRAIN_800MV_3_5DB_IVB;

	default:
		DRM_DEBUG_KMS("Unsupported voltage swing/pre-emphasis level:"
			      "0x%x\n", signal_levels);
		return EDP_LINK_TRAIN_500MV_0DB_IVB;
	}
}

void
intel_dp_set_signal_levels(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	struct intel_digital_port *intel_dig_port = dp_to_dig_port(intel_dp);
	enum port port = intel_dig_port->base.port;
	uint32_t signal_levels, mask = 0;
	uint8_t train_set = intel_dp->train_set[0];

		signal_levels = gen7_edp_signal_levels(train_set);
		mask = EDP_LINK_TRAIN_VOL_EMP_MASK_IVB;


	if (mask)
		DRM_DEBUG_KMS("Using signal levels %08x\n", signal_levels);

	DRM_DEBUG_KMS("Using vswing level %d\n",
		train_set & DP_TRAIN_VOLTAGE_SWING_MASK);
	DRM_DEBUG_KMS("Using pre-emphasis level %d\n",
		(train_set & DP_TRAIN_PRE_EMPHASIS_MASK) >>
			DP_TRAIN_PRE_EMPHASIS_SHIFT);

	intel_dp->DP = (intel_dp->DP & ~mask) | signal_levels;

	I915_WRITE(intel_dp->output_reg, intel_dp->DP);
	POSTING_READ(intel_dp->output_reg);
}

bool
intel_dp_read_dpcd(struct intel_dp *intel_dp)
{
	if (drm_dp_dpcd_read(&intel_dp->aux, 0x000, intel_dp->dpcd,
			     sizeof(intel_dp->dpcd)) < 0)
		return false; /* aux transfer failed */

	DRM_DEBUG_KMS("DPCD: %*ph\n", (int) sizeof(intel_dp->dpcd), intel_dp->dpcd);

	return intel_dp->dpcd[DP_DPCD_REV] != 0;
}



static bool
intel_dp_get_dpcd(struct intel_dp *intel_dp)
{
	u8 sink_count;

		return false;


}

static bool
intel_dp_can_mst(struct intel_dp *intel_dp)
{
	u8 mstm_cap;

	if (!i915_modparams.enable_dp_mst)
		return false;

	if (!intel_dp->can_mst)
		return false;

	if (intel_dp->dpcd[DP_DPCD_REV] < 0x12)
		return false;

	if (drm_dp_dpcd_readb(&intel_dp->aux, DP_MSTM_CAP, &mstm_cap) != 1)
		return false;

	return mstm_cap & DP_MST_CAP;
}

static void
intel_dp_configure_mst(struct intel_dp *intel_dp)
{
	if (!i915_modparams.enable_dp_mst)
		return;

	if (!intel_dp->can_mst)
		return;

	intel_dp->is_mst = intel_dp_can_mst(intel_dp);

	if (intel_dp->is_mst)
		DRM_DEBUG_KMS("Sink is MST capable\n");
	else
		DRM_DEBUG_KMS("Sink is not MST capable\n");

	drm_dp_mst_topology_mgr_set_mst(&intel_dp->mst_mgr,
					intel_dp->is_mst);
}

static int intel_dp_sink_crc_stop(struct intel_dp *intel_dp,
				  struct intel_crtc_state *crtc_state, bool disable_wa)
{
	struct intel_digital_port *dig_port = dp_to_dig_port(intel_dp);
	struct drm_i915_private *dev_priv = to_i915(dig_port->base.base.dev);
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc_state->base.crtc);
	u8 buf;
	int ret = 0;
	int count = 0;
	int attempts = 10;

	if (drm_dp_dpcd_readb(&intel_dp->aux, DP_TEST_SINK, &buf) < 0) {
		DRM_DEBUG_KMS("Sink CRC couldn't be stopped properly\n");
		ret = -EIO;
		goto out;
	}

	if (drm_dp_dpcd_writeb(&intel_dp->aux, DP_TEST_SINK,
			       buf & ~DP_TEST_SINK_START) < 0) {
		DRM_DEBUG_KMS("Sink CRC couldn't be stopped properly\n");
		ret = -EIO;
		goto out;
	}

	do {
		intel_wait_for_vblank(dev_priv, intel_crtc->pipe);

		if (drm_dp_dpcd_readb(&intel_dp->aux,
				      DP_TEST_SINK_MISC, &buf) < 0) {
			ret = -EIO;
			goto out;
		}
		count = buf & DP_TEST_COUNT_MASK;
	} while (--attempts && count);

	if (attempts == 0) {
		DRM_DEBUG_KMS("TIMEOUT: Sink CRC counter is not zeroed after calculation is stopped\n");
		ret = -ETIMEDOUT;
	}

 out:
	if (disable_wa)
		hsw_enable_ips(crtc_state);
	return ret;
}

static int intel_dp_sink_crc_start(struct intel_dp *intel_dp,
				   struct intel_crtc_state *crtc_state)
{
	struct intel_digital_port *dig_port = dp_to_dig_port(intel_dp);
	struct drm_i915_private *dev_priv = to_i915(dig_port->base.base.dev);
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc_state->base.crtc);
	u8 buf;
	int ret;

	if (drm_dp_dpcd_readb(&intel_dp->aux, DP_TEST_SINK_MISC, &buf) < 0)
		return -EIO;

	if (!(buf & DP_TEST_CRC_SUPPORTED))
		return -ENOTTY;

	if (drm_dp_dpcd_readb(&intel_dp->aux, DP_TEST_SINK, &buf) < 0)
		return -EIO;

	if (buf & DP_TEST_SINK_START) {
		ret = intel_dp_sink_crc_stop(intel_dp, crtc_state, false);
		if (ret)
			return ret;
	}

	hsw_disable_ips(crtc_state);

	if (drm_dp_dpcd_writeb(&intel_dp->aux, DP_TEST_SINK,
			       buf | DP_TEST_SINK_START) < 0) {
		hsw_enable_ips(crtc_state);
		return -EIO;
	}

	intel_wait_for_vblank(dev_priv, intel_crtc->pipe);
	return 0;
}

int intel_dp_sink_crc(struct intel_dp *intel_dp, struct intel_crtc_state *crtc_state, u8 *crc)
{
	struct intel_digital_port *dig_port = dp_to_dig_port(intel_dp);
	struct drm_i915_private *dev_priv = to_i915(dig_port->base.base.dev);
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc_state->base.crtc);
	u8 buf;
	int count, ret;
	int attempts = 6;

	ret = intel_dp_sink_crc_start(intel_dp, crtc_state);
	if (ret)
		return ret;

	do {
		intel_wait_for_vblank(dev_priv, intel_crtc->pipe);

		if (drm_dp_dpcd_readb(&intel_dp->aux,
				      DP_TEST_SINK_MISC, &buf) < 0) {
			ret = -EIO;
			goto stop;
		}
		count = buf & DP_TEST_COUNT_MASK;

	} while (--attempts && count == 0);

	if (attempts == 0) {
		DRM_ERROR("Panel is unable to calculate any CRC after 6 vblanks\n");
		ret = -ETIMEDOUT;
		goto stop;
	}

	if (drm_dp_dpcd_read(&intel_dp->aux, DP_TEST_CRC_R_CR, crc, 6) < 0) {
		ret = -EIO;
		goto stop;
	}

stop:
	intel_dp_sink_crc_stop(intel_dp, crtc_state, true);
	return ret;
}

static bool
intel_dp_get_sink_irq(struct intel_dp *intel_dp, u8 *sink_irq_vector)
{
	return drm_dp_dpcd_readb(&intel_dp->aux, DP_DEVICE_SERVICE_IRQ_VECTOR,
				 sink_irq_vector) == 1;
}

static bool
intel_dp_get_sink_irq_esi(struct intel_dp *intel_dp, u8 *sink_irq_vector)
{
	return drm_dp_dpcd_read(&intel_dp->aux, DP_SINK_COUNT_ESI,
				sink_irq_vector, DP_DPRX_ESI_LEN) ==
		DP_DPRX_ESI_LEN;
}

/* XXX this is probably wrong for multiple downstream ports */
static enum drm_connector_status
intel_dp_detect_dpcd(struct intel_dp *intel_dp)
{
	struct intel_lspcon *lspcon = dp_to_lspcon(intel_dp);
	uint8_t *dpcd = intel_dp->dpcd;
	uint8_t type;

	if (lspcon->active)
		lspcon_resume(lspcon);

	if (!intel_dp_get_dpcd(intel_dp))
		return connector_status_disconnected;

	if (intel_dp_is_edp(intel_dp))
		return connector_status_connected;

	/* if there's no downstream port, we're done */
	if (!drm_dp_is_branch(dpcd))
		return connector_status_connected;

	/* If we're HPD-aware, SINK_COUNT changes dynamically */
	if (intel_dp->dpcd[DP_DPCD_REV] >= 0x11 &&
	    intel_dp->downstream_ports[0] & DP_DS_PORT_HPD) {

		return intel_dp->sink_count ?
		connector_status_connected : connector_status_disconnected;
	}

	if (intel_dp_can_mst(intel_dp))
		return connector_status_connected;

	/* If no HPD, poke DDC gently */
	if (drm_probe_ddc(&intel_dp->aux.ddc))
		return connector_status_connected;

	/* Well we tried, say unknown for unreliable port types */
	if (intel_dp->dpcd[DP_DPCD_REV] >= 0x11) {
		type = intel_dp->downstream_ports[0] & DP_DS_PORT_TYPE_MASK;
		if (type == DP_DS_PORT_TYPE_VGA ||
		    type == DP_DS_PORT_TYPE_NON_EDID)
			return connector_status_unknown;
	} else {
		type = intel_dp->dpcd[DP_DOWNSTREAMPORT_PRESENT] &
			DP_DWN_STRM_PORT_TYPE_MASK;
		if (type == DP_DWN_STRM_PORT_TYPE_ANALOG ||
		    type == DP_DWN_STRM_PORT_TYPE_OTHER)
			return connector_status_unknown;
	}

	/* Anything else is out of spec, warn and ignore */
	DRM_DEBUG_KMS("Broken DP branch device, ignoring\n");
	return connector_status_disconnected;
}

static enum drm_connector_status
edp_detect(struct intel_dp *intel_dp)
{
	struct drm_i915_private *dev_priv = to_i915(intel_dp_to_dev(intel_dp));
	enum drm_connector_status status;

	status = intel_panel_detect(dev_priv);
	if (status == connector_status_unknown)
		status = connector_status_connected;

	return status;
}


static bool cpt_digital_port_connected(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = to_i915(encoder->base.dev);
	u32 bit;

	switch (encoder->hpd_pin) {
	case HPD_PORT_B:
		bit = SDE_PORTB_HOTPLUG_CPT;
		break;
	case HPD_PORT_C:
		bit = SDE_PORTC_HOTPLUG_CPT;
		break;
	case HPD_PORT_D:
		bit = SDE_PORTD_HOTPLUG_CPT;
		break;
	default:
		MISSING_CASE(encoder->hpd_pin);
		return false;
	}

	return I915_READ(SDEISR) & bit;
}

static bool ivb_digital_port_connected(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = to_i915(encoder->base.dev);

	if (encoder->hpd_pin == HPD_PORT_A)
		return I915_READ(DEISR) & DE_DP_A_HOTPLUG_IVB;
	else
		return cpt_digital_port_connected(encoder);
}

/*
 * intel_digital_port_connected - is the specified port connected?
 * @encoder: intel_encoder
 *
 * Return %true if port is connected, %false otherwise.
 */
bool intel_digital_port_connected(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = to_i915(encoder->base.dev);

    return ivb_digital_port_connected(encoder);

}


static void
intel_dp_unset_edid(struct intel_dp *intel_dp)
{
	struct intel_connector *intel_connector = intel_dp->attached_connector;

	kfree(intel_connector->detect_edid);
	intel_connector->detect_edid = NULL;

	intel_dp->has_audio = false;
}

static int
intel_dp_long_pulse(struct intel_connector *connector)
{
	struct drm_i915_private *dev_priv = to_i915(connector->base.dev);
	struct intel_dp *intel_dp = intel_attached_dp(&connector->base);
	enum drm_connector_status status;
	u8 sink_irq_vector = 0;

	WARN_ON(!drm_modeset_is_locked(&dev_priv->drm.mode_config.connection_mutex));

	intel_display_power_get(dev_priv, intel_dp->aux_power_domain);

	/* Can't disconnect eDP, but you can close the lid... */
	if (intel_dp_is_edp(intel_dp))
		status = edp_detect(intel_dp);
	else if (intel_digital_port_connected(&dp_to_dig_port(intel_dp)->base))
		status = intel_dp_detect_dpcd(intel_dp);
	else
		status = connector_status_disconnected;

	if (status == connector_status_disconnected) {
		memset(&intel_dp->compliance, 0, sizeof(intel_dp->compliance));

		if (intel_dp->is_mst) {
			DRM_DEBUG_KMS("MST device may have disappeared %d vs %d\n",
				      intel_dp->is_mst,
				      intel_dp->mst_mgr.mst_state);
			intel_dp->is_mst = false;
			drm_dp_mst_topology_mgr_set_mst(&intel_dp->mst_mgr,
							intel_dp->is_mst);
		}

		goto out;
	}


out:
	if (status != connector_status_connected && !intel_dp->is_mst)
		intel_dp_unset_edid(intel_dp);

	intel_display_power_put(dev_priv, intel_dp->aux_power_domain);
	return status;
}


/* check the VBT to see whether the eDP is on another port */
bool intel_dp_is_port_edp(struct drm_i915_private *dev_priv, enum port port)
{
	/*
	 * eDP not supported on g4x. so bail out early just
	 * for a bit extra safety in case the VBT is bonkers.
	 */
	if (INTEL_GEN(dev_priv) < 5)
		return false;

	if (INTEL_GEN(dev_priv) < 9 && port == PORT_A)
		return true;

	return intel_bios_is_port_edp(dev_priv, port);
}

static void
intel_dp_add_properties(struct intel_dp *intel_dp, struct drm_connector *connector)
{
	struct drm_i915_private *dev_priv = to_i915(connector->dev);
	enum port port = dp_to_dig_port(intel_dp)->base.port;

	if (!IS_G4X(dev_priv) && port != PORT_A)
		intel_attach_force_audio_property(connector);

	intel_attach_broadcast_rgb_property(connector);

	if (intel_dp_is_edp(intel_dp)) {
		u32 allowed_scalers;

		allowed_scalers = BIT(DRM_MODE_SCALE_ASPECT) | BIT(DRM_MODE_SCALE_FULLSCREEN);
		if (!HAS_GMCH_DISPLAY(dev_priv))
			allowed_scalers |= BIT(DRM_MODE_SCALE_CENTER);

		drm_connector_attach_scaling_mode_property(connector, allowed_scalers);

		connector->state->scaling_mode = DRM_MODE_SCALE_ASPECT;

	}
}

/**
 * intel_edp_drrs_flush - Restart Idleness DRRS
 * @dev_priv: i915 device
 * @frontbuffer_bits: frontbuffer plane tracking bits
 *
 * This function gets called every time rendering on the given planes has
 * completed or flip on a crtc is completed. So DRRS should be upclocked
 * (LOW_RR -> HIGH_RR). And also Idleness detection should be started again,
 * if no other planes are dirty.
 *
 * Dirty frontbuffers relevant to DRRS are tracked in busy_frontbuffer_bits.
 */
void intel_edp_drrs_flush(struct drm_i915_private *dev_priv,
			  unsigned int frontbuffer_bits)
{
	struct drm_crtc *crtc;
	enum pipe pipe;


}


static bool intel_edp_init_connector(struct intel_dp *intel_dp,
				     struct intel_connector *intel_connector)
{
	struct drm_device *dev = intel_dp_to_dev(intel_dp);

    return true;

}


static int
intel_dp_connector_register(struct drm_connector *connector)
{
    struct intel_dp *intel_dp = intel_attached_dp(connector);
    int ret;

    ret = intel_connector_register(connector);
    if (ret)
        return ret;


    intel_dp->aux.dev = connector->kdev;
    return drm_dp_aux_register(&intel_dp->aux);
}

static void 
intel_dp_connector_unregister(struct drm_connector *connector)
{
    drm_dp_aux_unregister(&intel_attached_dp(connector)->aux);
    intel_connector_unregister(connector);
}

static void
intel_dp_connector_destroy(struct drm_connector *connector)
{
printk("destrpoy");
}
static int intel_dp_get_modes(struct drm_connector *connector)
{
return 0;
}
int disable_connector(struct drm_connector *connector,
                        uint32_t maxX, uint32_t maxY)
{
    return 0;
}

 static const struct drm_connector_funcs intel_dp_connector_funcs = {
         .force = NULL,
         .fill_modes = disable_connector,
         .atomic_get_property = intel_digital_connector_atomic_get_property,
         .atomic_set_property = intel_digital_connector_atomic_set_property,
         .late_register = intel_dp_connector_register,
         .early_unregister = intel_dp_connector_unregister,
         .destroy = intel_dp_connector_destroy,
         .atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
         .atomic_duplicate_state = intel_digital_connector_duplicate_state,
 };

 static const struct drm_connector_helper_funcs intel_dp_connector_helper_funcs = {
         .detect_ctx = NULL,
         .get_modes = intel_dp_get_modes,
         .mode_valid = NULL,
         .atomic_check = intel_digital_connector_atomic_check,
 };

bool
intel_dp_init_connector(struct intel_digital_port *intel_dig_port,
			struct intel_connector *intel_connector)
{
	struct drm_connector *connector = &intel_connector->base;
	struct intel_dp *intel_dp = &intel_dig_port->dp;
	struct intel_encoder *intel_encoder = &intel_dig_port->base;
	struct drm_device *dev = intel_encoder->base.dev;
	struct drm_i915_private *dev_priv = to_i915(dev);
	enum port port = intel_encoder->port;
	int type;


	if (WARN(intel_dig_port->max_lanes < 1,
		 "Not enough lanes (%d) for DP on port %c\n",
		 intel_dig_port->max_lanes, port_name(port)))
		return false;

	intel_dp_set_source_rates(intel_dp);

	intel_dp->reset_link_params = true;
	intel_dp->pps_pipe = INVALID_PIPE;
	intel_dp->active_pipe = INVALID_PIPE;

	/* intel_dp vfuncs */
	if (HAS_DDI(dev_priv))
		intel_dp->prepare_link_retrain = intel_ddi_prepare_link_retrain;

	/* Preserve the current hw state. */
	intel_dp->DP = I915_READ(intel_dp->output_reg);
	intel_dp->attached_connector = intel_connector;

	if (intel_dp_is_port_edp(dev_priv, port))
		type = DRM_MODE_CONNECTOR_eDP;
	else
		type = DRM_MODE_CONNECTOR_DisplayPort;


	/*
	 * For eDP we always set the encoder type to INTEL_OUTPUT_EDP, but
	 * for DP the encoder type can be set by the caller to
	 * INTEL_OUTPUT_UNKNOWN for DDI, so don't rewrite it.
	 */

	/* eDP only on port B and/or C on vlv/chv */
	if (WARN_ON((IS_VALLEYVIEW(dev_priv) || IS_CHERRYVIEW(dev_priv)) &&
		    intel_dp_is_edp(intel_dp) &&
		    port != PORT_B && port != PORT_C))
		return false;

	DRM_DEBUG_KMS("Adding %s connector on port %c\n",
			type == DRM_MODE_CONNECTOR_eDP ? "eDP" : "DP",
			port_name(port));

	drm_connector_init(dev, connector, &intel_dp_connector_funcs, type);
	drm_connector_helper_add(connector, &intel_dp_connector_helper_funcs);

	if (!IS_VALLEYVIEW(dev_priv) && !IS_CHERRYVIEW(dev_priv))
		connector->interlace_allowed = true;
	connector->doublescan_allowed = 0;

	intel_encoder->hpd_pin = intel_hpd_pin_default(dev_priv, port);

	intel_dp_aux_init(intel_dp);


	intel_connector_attach_encoder(intel_connector, intel_encoder);

    intel_connector->get_hw_state = intel_ddi_connector_get_hw_state;


	/* init MST on ports that can support it */
	if (HAS_DP_MST(dev_priv) && !intel_dp_is_edp(intel_dp) &&
	    (port == PORT_B || port == PORT_C ||
	     port == PORT_D || port == PORT_F))
		intel_dp_mst_encoder_init(intel_dig_port,
					  intel_connector->base.base.id);

	if (!intel_edp_init_connector(intel_dp, intel_connector)) {
		intel_dp_aux_fini(intel_dp);
		intel_dp_mst_encoder_cleanup(intel_dig_port);
		goto fail;
	}

	intel_dp_add_properties(intel_dp, connector);

	return true;

fail:

	return false;
}
