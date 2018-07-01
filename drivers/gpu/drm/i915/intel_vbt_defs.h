/*
 * Copyright © 2006-2016 Intel Corporation
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

/*
 * This information is private to VBT parsing in intel_bios.c.
 *
 * Please do NOT include anywhere else.
 */


#ifndef _INTEL_VBT_DEFS_H_
#define _INTEL_VBT_DEFS_H_

#include "intel_bios.h"



/* Add the device class for LFP, TV, HDMI */
#define DEVICE_TYPE_INT_LFP		0x1022
#define DEVICE_TYPE_INT_TV		0x1009
#define DEVICE_TYPE_HDMI		0x60D2
#define DEVICE_TYPE_DP			0x68C6
#define DEVICE_TYPE_DP_DUAL_MODE	0x60D6
#define DEVICE_TYPE_eDP			0x78C6

#define DEVICE_TYPE_CLASS_EXTENSION	(1 << 15)
#define DEVICE_TYPE_POWER_MANAGEMENT	(1 << 14)
#define DEVICE_TYPE_HOTPLUG_SIGNALING	(1 << 13)
#define DEVICE_TYPE_INTERNAL_CONNECTOR	(1 << 12)
#define DEVICE_TYPE_NOT_HDMI_OUTPUT	(1 << 11)
#define DEVICE_TYPE_MIPI_OUTPUT		(1 << 10)
#define DEVICE_TYPE_COMPOSITE_OUTPUT	(1 << 9)
#define DEVICE_TYPE_DUAL_CHANNEL	(1 << 8)
#define DEVICE_TYPE_HIGH_SPEED_LINK	(1 << 6)
#define DEVICE_TYPE_LVDS_SIGNALING	(1 << 5)
#define DEVICE_TYPE_TMDS_DVI_SIGNALING	(1 << 4)
#define DEVICE_TYPE_VIDEO_SIGNALING	(1 << 3)
#define DEVICE_TYPE_DISPLAYPORT_OUTPUT	(1 << 2)
#define DEVICE_TYPE_DIGITAL_OUTPUT	(1 << 1)
#define DEVICE_TYPE_ANALOG_OUTPUT	(1 << 0)

/*
 * Bits we care about when checking for DEVICE_TYPE_eDP. Depending on the
 * system, the other bits may or may not be set for eDP outputs.
 */
#define DEVICE_TYPE_eDP_BITS \
        (DEVICE_TYPE_INTERNAL_CONNECTOR |	\
         DEVICE_TYPE_MIPI_OUTPUT |		\
         DEVICE_TYPE_COMPOSITE_OUTPUT |		\
         DEVICE_TYPE_DUAL_CHANNEL |		\
         DEVICE_TYPE_LVDS_SIGNALING |		\
         DEVICE_TYPE_TMDS_DVI_SIGNALING |	\
         DEVICE_TYPE_VIDEO_SIGNALING |		\
         DEVICE_TYPE_DISPLAYPORT_OUTPUT |	\
         DEVICE_TYPE_ANALOG_OUTPUT)

#define DEVICE_TYPE_DP_DUAL_MODE_BITS \
        (DEVICE_TYPE_INTERNAL_CONNECTOR |	\
         DEVICE_TYPE_MIPI_OUTPUT |		\
         DEVICE_TYPE_COMPOSITE_OUTPUT |		\
         DEVICE_TYPE_LVDS_SIGNALING |		\
         DEVICE_TYPE_TMDS_DVI_SIGNALING |	\
         DEVICE_TYPE_VIDEO_SIGNALING |		\
         DEVICE_TYPE_DISPLAYPORT_OUTPUT |	\
         DEVICE_TYPE_DIGITAL_OUTPUT |		\
         DEVICE_TYPE_ANALOG_OUTPUT)

#define DEVICE_CFG_NONE		0x00
#define DEVICE_CFG_12BIT_DVOB	0x01
#define DEVICE_CFG_12BIT_DVOC	0x02
#define DEVICE_CFG_24BIT_DVOBC	0x09
#define DEVICE_CFG_24BIT_DVOCB	0x0a
#define DEVICE_CFG_DUAL_DVOB	0x11
#define DEVICE_CFG_DUAL_DVOC	0x12
#define DEVICE_CFG_DUAL_DVOBC	0x13
#define DEVICE_CFG_DUAL_LINK_DVOBC	0x19
#define DEVICE_CFG_DUAL_LINK_DVOCB	0x1a

#define DEVICE_WIRE_NONE	0x00
#define DEVICE_WIRE_DVOB	0x01
#define DEVICE_WIRE_DVOC	0x02
#define DEVICE_WIRE_DVOBC	0x03
#define DEVICE_WIRE_DVOBB	0x05
#define DEVICE_WIRE_DVOCC	0x06
#define DEVICE_WIRE_DVOB_MASTER 0x0d
#define DEVICE_WIRE_DVOC_MASTER 0x0e

/* dvo_port pre BDB 155 */
#define DEVICE_PORT_DVOA	0x00 /* none on 845+ */
#define DEVICE_PORT_DVOB	0x01
#define DEVICE_PORT_DVOC	0x02

/* dvo_port BDB 155+ */
#define DVO_PORT_HDMIA		0
#define DVO_PORT_HDMIB		1
#define DVO_PORT_HDMIC		2
#define DVO_PORT_HDMID		3
#define DVO_PORT_LVDS		4
#define DVO_PORT_TV		5
#define DVO_PORT_CRT		6
#define DVO_PORT_DPB		7
#define DVO_PORT_DPC		8
#define DVO_PORT_DPD		9
#define DVO_PORT_DPA		10
#define DVO_PORT_DPE		11				/* 193 */
#define DVO_PORT_HDMIE		12				/* 193 */
#define DVO_PORT_DPF		13				/* N/A */
#define DVO_PORT_HDMIF		14				/* N/A */
#define DVO_PORT_MIPIA		21				/* 171 */
#define DVO_PORT_MIPIB		22				/* 171 */
#define DVO_PORT_MIPIC		23				/* 171 */
#define DVO_PORT_MIPID		24				/* 171 */

#define HDMI_MAX_DATA_RATE_PLATFORM	0			/* 204 */
#define HDMI_MAX_DATA_RATE_297		1			/* 204 */
#define HDMI_MAX_DATA_RATE_165		2			/* 204 */


/*
 * The child device config, aka the display device data structure, provides a
 * description of a port and its configuration on the platform.
 *
 * The child device config size has been increased, and fields have been added
 * and their meaning has changed over time. Care must be taken when accessing
 * basically any of the fields to ensure the correct interpretation for the BDB
 * version in question.
 *
 * When we copy the child device configs to dev_priv->vbt.child_dev, we reserve
 * space for the full structure below, and initialize the tail not actually
 * present in VBT to zeros. Accessing those fields is fine, as long as the
 * default zero is taken into account, again according to the BDB version.
 *
 * BDB versions 155 and below are considered legacy, and version 155 seems to be
 * a baseline for some of the VBT documentation. When adding new fields, please
 * include the BDB version when the field was added, if it's above that.
 */
struct child_device_config {
	u16 handle;
	u16 device_type; /* See DEVICE_TYPE_* above */

	union {
		u8  device_id[10]; /* ascii string */
		struct {
			u8 i2c_speed;
			u8 dp_onboard_redriver;			/* 158 */
			u8 dp_ondock_redriver;			/* 158 */
			u8 hdmi_level_shifter_value:5;		/* 169 */
			u8 hdmi_max_data_rate:3;		/* 204 */
			u16 dtd_buf_ptr;			/* 161 */
			u8 edidless_efp:1;			/* 161 */
			u8 compression_enable:1;		/* 198 */
			u8 compression_method:1;		/* 198 */
			u8 ganged_edp:1;			/* 202 */
			u8 reserved0:4;
			u8 compression_structure_index:4;	/* 198 */
			u8 reserved1:4;
			u8 slave_port;				/* 202 */
			u8 reserved2;
		} __packed;
	} __packed;

	u16 addin_offset;
	u8 dvo_port; /* See DEVICE_PORT_* and DVO_PORT_* above */
	u8 i2c_pin;
	u8 slave_addr;
	u8 ddc_pin;
	u16 edid_ptr;
	u8 dvo_cfg; /* See DEVICE_CFG_* above */

	union {
		struct {
			u8 dvo2_port;
			u8 i2c2_pin;
			u8 slave2_addr;
			u8 ddc2_pin;
		} __packed;
		struct {
			u8 efp_routed:1;			/* 158 */
			u8 lane_reversal:1;			/* 184 */
			u8 lspcon:1;				/* 192 */
			u8 iboost:1;				/* 196 */
			u8 hpd_invert:1;			/* 196 */
			u8 flag_reserved:3;
			u8 hdmi_support:1;			/* 158 */
			u8 dp_support:1;			/* 158 */
			u8 tmds_support:1;			/* 158 */
			u8 support_reserved:5;
			u8 aux_channel;
			u8 dongle_detect;
		} __packed;
	} __packed;

	u8 pipe_cap:2;
	u8 sdvo_stall:1;					/* 158 */
	u8 hpd_status:2;
	u8 integrated_encoder:1;
	u8 capabilities_reserved:2;
	u8 dvo_wiring; /* See DEVICE_WIRE_* above */

	union {
		u8 dvo2_wiring;
		u8 mipi_bridge_type;				/* 171 */
	} __packed;

	u16 extended_type;
	u8 dvo_function;
	u8 dp_usb_type_c:1;					/* 195 */
	u8 flags2_reserved:7;					/* 195 */
	u8 dp_gpio_index;					/* 195 */
	u16 dp_gpio_pin_num;					/* 195 */
	u8 dp_iboost_level:4;					/* 196 */
	u8 hdmi_iboost_level:4;					/* 196 */
	u8 dp_max_link_rate:2;					/* 216 CNL+ */
	u8 dp_max_link_rate_reserved:6;				/* 216 */
} __packed;

/* Mask for DRRS / Panel Channel / SSC / BLT control bits extraction */
#define MODE_MASK		0x3


#endif /* _INTEL_VBT_DEFS_H_ */
