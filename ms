using card '/dev/dri/card0'
[   12.671920] [drm:drm_stub_open] 
[   12.672157] [drm:drm_open] pid = 29, minor = 0
[   12.672493] [drm:i915_gem_open] 
[   12.672719] [drm:drm_open] 
[   12.672895] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_GET_CAP
[   12.673234] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETRESOURCES
[   12.673633] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETRESOURCES
[   12.674008] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETCONNECTOR
[   12.674388] [drm:drm_helper_probe_single_connector_modes] [CONNECTOR:64:VGA-1]
[   12.674740] [drm:intel_crt_detect] [CONNECTOR:64:VGA-1] force=1
[   12.675029] [drm:intel_power_well_enable] enabling always-on
[   12.675306] [drm:intel_power_well_enable] enabling display
[   12.675585] [drm:intel_crt_detect] trigger hotplug detect cycle: adpa=0xf40000
[   12.685207] [drm:intel_crt_detect] ironlake hotplug adpa=0xf40000, result 0
[   12.685624] [drm:intel_crt_detect] CRT not detected via hotplug
[   12.686157] [drm:do_gmbus_xfer] GMBUS [i915 gmbus vga] NAK for addr: 0050 w(1)
[   12.686585] [drm:do_gmbus_xfer] GMBUS [i915 gmbus vga] NAK on first message, retry
[   12.687184] [drm:do_gmbus_xfer] GMBUS [i915 gmbus vga] NAK for addr: 0050 w(1)
[   12.687598] [drm:drm_do_probe_ddc_edid] drm: skipping non-existent adapter i915 gmbus vga
[   12.688076] [drm:intel_crt_get_edid] CRT GMBUS EDID read failed, retry using GPIO bit-banging
[   12.688557] [drm:intel_gmbus_force_bit] enabling bit-banging on i915 gmbus vga. force bit now 1
[   12.689785] [drm:drm_do_probe_ddc_edid] drm: skipping non-existent adapter i915 gmbus vga
[   12.690288] [drm:intel_gmbus_force_bit] disabling bit-banging on i915 gmbus vga. force bit now 0
[   12.690828] [drm:intel_crt_detect_ddc] CRT not detected via DDC:0x50 [no valid EDID found]
[   12.691455] [drm:intel_power_well_disable] disabling display
[   12.691798] [drm:intel_power_well_disable] disabling always-on
[   12.692140] [drm:drm_helper_probe_single_connector_modes] [CONNECTOR:64:VGA-1] status updated from unknown to disconnected
[   12.692792] [drm:drm_sysfs_hotplug_event] generating hotplug event
[   12.693158] [drm:drm_helper_probe_single_connector_modes] [CONNECTOR:64:VGA-1] disconnected
[   12.693643] [drm:drm_mode_object_put.part.2] OBJ ID: 64 (2)
[   12.693978] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETCONNECTOR
[   12.694428] [drm:drm_mode_object_put.part.2] OBJ ID: 64 (2)
ignoring unused connector 64
[   12.694932] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETCONNECTOR
[   12.695366] [drm:drm_helper_probe_single_connector_modes] [CONNECTOR:67:HDMI-A-1]
[   12.695901] [drm:intel_hdmi_detect] [CONNECTOR:67:HDMI-A-1]
[   12.696317] [drm:intel_power_well_enable] enabling always-on
[   12.696902] [drm:do_gmbus_xfer] GMBUS [i915 gmbus dpc] NAK for addr: 0050 w(1)
[   12.697418] [drm:do_gmbus_xfer] GMBUS [i915 gmbus dpc] NAK on first message, retry
[   12.698134] [drm:do_gmbus_xfer] GMBUS [i915 gmbus dpc] NAK for addr: 0050 w(1)
[   12.698663] [drm:drm_do_probe_ddc_edid] drm: skipping non-existent adapter i915 gmbus dpc
[   12.699206] [drm:intel_hdmi_set_edid] HDMI GMBUS EDID read failed, retry using GPIO bit-banging
[   12.699707] [drm:intel_gmbus_force_bit] enabling bit-banging on i915 gmbus dpc. force bit now 1
[   12.700953] [drm:drm_do_probe_ddc_edid] drm: skipping non-existent adapter i915 gmbus dpc
[   12.701413] [drm:intel_gmbus_force_bit] disabling bit-banging on i915 gmbus dpc. force bit now 0
[   12.702066] [drm:do_gmbus_xfer] GMBUS [i915 gmbus dpc] NAK for addr: 0040 w(1)
[   12.702474] [drm:do_gmbus_xfer] GMBUS [i915 gmbus dpc] NAK on first message, retry
[   12.703077] [drm:do_gmbus_xfer] GMBUS [i915 gmbus dpc] NAK for addr: 0040 w(1)
[   12.703487] [drm:drm_dp_dual_mode_detect] DP dual mode HDMI ID:  (err -6)
[   12.703870] [drm:intel_power_well_disable] disabling always-on
[   12.704197] [drm:drm_helper_probe_single_connector_modes] [CONNECTOR:67:HDMI-A-1] status updated from unknown to disconnected
[   12.704820] [drm:drm_helper_probe_single_connector_modes] [CONNECTOR:67:HDMI-A-1] disconnected
[   12.705296] [drm:drm_mode_object_put.part.2] OBJ ID: 67 (2)
[   12.705611] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETCONNECTOR
[   12.706029] [drm:drm_mode_object_put.part.2] OBJ ID: 67 (2)
ignoring unused connector 67
[   12.706503] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETCONNECTOR
[   12.707005] [drm:drm_helper_probe_single_connector_modes] [CONNECTOR:73:HDMI-A-2]
[   12.707467] [drm:intel_hdmi_detect] [CONNECTOR:73:HDMI-A-2]
[   12.707803] [drm:intel_power_well_enable] enabling always-on
[   12.734165] [drm:do_gmbus_xfer] GMBUS [i915 gmbus dpd] NAK for addr: 0040 w(1)
[   12.734585] [drm:do_gmbus_xfer] GMBUS [i915 gmbus dpd] NAK on first message, retry
[   12.735214] [drm:do_gmbus_xfer] GMBUS [i915 gmbus dpd] NAK for addr: 0040 w(1)
[   12.735620] [drm:drm_dp_dual_mode_detect] DP dual mode HDMI ID:  (err -6)
[   12.736004] [drm:drm_detect_monitor_audio] Monitor has basic audio support
[   12.736392] [drm:intel_power_well_disable] disabling always-on
[   12.736722] [drm:drm_add_display_info] non_desktop set to 0
[   12.737034] [drm:drm_add_display_info] HDMI: DVI dual 0, max TMDS clock 0 kHz
[   12.737434] [drm:drm_add_edid_modes] ELD monitor 27MU67
[   12.737739] [drm:drm_add_edid_modes] HDMI: latency present 0 0, video latency 0 0, audio latency 0 0
[   12.738263] [drm:drm_add_edid_modes] ELD size 32, SAD count 1
[   12.738586] [drm:drm_add_display_info] non_desktop set to 0
[   12.738890] [drm:drm_add_display_info] HDMI: DVI dual 0, max TMDS clock 0 kHz
[   12.739358] [drm:drm_helper_probe_single_connector_modes] [CONNECTOR:73:HDMI-A-2] probed modes :
[   12.739851] [drm:drm_mode_debug_printmodeline] Modeline 76:"1920x2160" 60 277250 1920 1968 2000 2080 2160 2163 2173 2222 0x48 0xa
[   12.740560] [drm:drm_mode_debug_printmodeline] Modeline 77:"1920x1080" 60 148500 1920 2008 2052 2200 1080 1084 1089 1125 0x40 0x5
[   12.741172] [drm:drm_mode_debug_printmodeline] Modeline 90:"1920x1080" 60 148352 1920 2008 2052 2200 1080 1084 1089 1125 0x40 0x5
[   12.741767] [drm:drm_mode_debug_printmodeline] Modeline 82:"1600x900" 60 108000 1600 1624 1704 1800 900 901 904 1000 0x40 0x5
[   12.742366] [drm:drm_mode_debug_printmodeline] Modeline 80:"1280x1024" 60 108000 1280 1328 1440 1688 1024 1025 1028 1066 0x40 0x5
[   12.742970] [drm:drm_mode_debug_printmodeline] Modeline 79:"1152x864" 60 81579 1152 1216 1336 1520 864 865 868 895 0x0 0x6
[   12.743541] [drm:drm_mode_debug_printmodeline] Modeline 81:"1280x720" 60 74250 1280 1390 1430 1650 720 725 730 750 0x40 0x5
[   12.744084] [drm:drm_mode_debug_printmodeline] Modeline 92:"1280x720" 60 74176 1280 1390 1430 1650 720 725 730 750 0x40 0x5
[   12.744643] [drm:drm_mode_debug_printmodeline] Modeline 85:"1024x768" 60 65000 1024 1048 1184 1344 768 771 777 806 0x40 0xa
[   12.745216] [drm:drm_mode_debug_printmodeline] Modeline 83:"800x600" 60 40000 800 840 968 1056 600 601 605 628 0x40 0x5
[   12.745782] [drm:drm_mode_debug_printmodeline] Modeline 96:"720x480" 60 27027 720 736 798 858 480 489 495 525 0x40 0xa
[   12.746311] [drm:drm_mode_debug_printmodeline] Modeline 88:"720x480" 60 27000 720 736 798 858 480 489 495 525 0x40 0xa
[   12.746868] [drm:drm_mode_debug_printmodeline] Modeline 93:"640x480" 60 25200 640 656 752 800 480 490 492 525 0x40 0xa
[   12.747413] [drm:drm_mode_debug_printmodeline] Modeline 84:"640x480" 60 25175 640 656 752 800 480 490 492 525 0x40 0xa
[   12.747965] [drm:drm_mode_object_put.part.2] OBJ ID: 73 (2)
[   12.748265] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETCONNECTOR
[   12.748692] [drm:drm_mode_object_put.part.2] OBJ ID: 73 (2)
mode for connector 73 is 1920x2160
[   12.749180] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETENCODER
[   12.749589] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_CREATE_DUMB
[   12.749993] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_ADDFB
[   12.750366] [drm:drm_mode_addfb2] [FB:78]
[   12.750591] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_MAP_DUMB
[   13.566638] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_GETCRTC
[   13.567055] [drm:drm_ioctl] pid=29, dev=0xe200, auth=1, DRM_IOCTL_MODE_SETCRTC
[   13.567433] [drm:drm_mode_setcrtc] [CRTC:39:pipe A]
[   13.567676] [drm:drm_mode_setcrtc] [CONNECTOR:73:HDMI-A-2]
[   13.567965] [drm:drm_atomic_state_init] Allocated atomic state         (ptrval)
[   13.568348] [drm:drm_atomic_get_crtc_state] Added [CRTC:39:pipe A]         (ptrval) state to         (ptrval)
[   13.568846] [drm:drm_atomic_get_plane_state] Added [PLANE:28:primary A]         (ptrval) state to         (ptrval)
[   13.569373] [drm:drm_atomic_set_mode_for_crtc] Set [MODE:1920x2160] for CRTC state         (ptrval)
[   13.569853] [drm:drm_atomic_set_crtc_for_plane] Link plane state         (ptrval) to [CRTC:39:pipe A]
[   13.570382] [drm:drm_atomic_set_fb_for_plane] Set [FB:78] for plane state         (ptrval)
[   13.570986] [drm:drm_mode_object_get] OBJ ID: 78 (2)
[   13.571354] [drm:drm_atomic_add_affected_connectors] Adding all current connectors for [CRTC:39:pipe A] to         (ptrval)
[   13.572169] [drm:drm_mode_object_get] OBJ ID: 73 (2)
[   13.572507] [drm:drm_atomic_get_connector_state] Added [CONNECTOR:73:HDMI-A-2]         (ptrval) state to         (ptrval)
[   13.573152] [drm:drm_mode_object_get] OBJ ID: 73 (3)
[   13.573446] [drm:drm_atomic_set_crtc_for_connector] Link connector state         (ptrval) to [CRTC:39:pipe A]
[   13.574013] [drm:drm_atomic_check_only] checking         (ptrval)
[   13.574364] [drm:drm_atomic_helper_check_modeset] [CRTC:39:pipe A] mode changed
[   13.574782] [drm:drm_atomic_helper_check_modeset] [CRTC:39:pipe A] enable changed
[   13.575210] [drm:drm_atomic_helper_check_modeset] [CRTC:39:pipe A] active changed
[   13.575633] [drm:drm_atomic_helper_check_modeset] Updating routing for [CONNECTOR:73:HDMI-A-2]
[   13.576117] [drm:drm_atomic_helper_check_modeset] [CONNECTOR:73:HDMI-A-2] using [ENCODER:72:DDI D] on [CRTC:39:pipe A]
[   13.576712] [drm:drm_atomic_helper_check_modeset] [CRTC:39:pipe A] needs all connectors, enable: y, active: y
[   13.577211] [drm:drm_atomic_add_affected_connectors] Adding all current connectors for [CRTC:39:pipe A] to         (ptrval)
[   13.577786] [drm:intel_atomic_check] [CONNECTOR:73:HDMI-A-2] checking for sink bpp constrains
[   13.578332] [drm:intel_atomic_check] clamping display bpp (was 36) to EDID reported max of 24
[   13.578959] [drm:intel_hdmi_compute_config] picking bpc to 8 for HDMI output
[   13.579497] [drm:intel_hdmi_compute_config] forcing pipe bpp to 24 for HDMI
[   13.580049] [drm:intel_atomic_check] hw max bpp: 36, pipe bpp: 24, dithering: 0
[   13.580609] [drm:intel_dump_pipe_config] [CRTC:39:pipe A][modeset]
[   13.581088] [drm:intel_dump_pipe_config] output_types: HDMI (0x40)
[   13.581567] [drm:intel_dump_pipe_config] cpu_transcoder: A, pipe bpp: 24, dithering: 0
[   13.582177] [drm:intel_dump_pipe_config] audio: 1, infoframes: 1
[   13.582647] [drm:intel_dump_pipe_config] requested mode:
[   13.583066] [drm:drm_mode_debug_printmodeline] Modeline 0:"1920x2160" 60 277250 1920 1968 2000 2080 2160 2163 2173 2222 0x48 0xa
[   13.583955] [drm:intel_dump_pipe_config] adjusted mode:
[   13.584364] [drm:drm_mode_debug_printmodeline] Modeline 0:"1920x2160" 60 277250 1920 1968 2000 2080 2160 2163 2173 2222 0x48 0xa
[   13.585253] [drm:intel_dump_pipe_config] crtc timings: 277250 1920 1968 2000 2080 2160 2163 2173 2222, type: 0x48 flags: 0xa
[   13.586118] [drm:intel_dump_pipe_config] port clock: 277250, pipe src size: 1920x2160, pixel rate 277250
[   13.586854] [drm:intel_dump_pipe_config] pch pfit: pos: 0x00000000, size: 0x00000000, disabled
[   13.587521] [drm:intel_dump_pipe_config] ips: 0, double wide: 0
[   13.587982] [drm:hsw_dump_hw_state] dpll_hw_state: wrpll: 0x0 spll: 0x0
[   13.588494] [drm:intel_dump_pipe_config] planes on this crtc
[   13.588935] [drm:intel_dump_pipe_config] [PLANE:28:primary A] disabled, scaler_id = 0
[   13.589539] [drm:intel_dump_pipe_config] [PLANE:31:sprite A] disabled, scaler_id = 0
[   13.590143] [drm:intel_dump_pipe_config] [PLANE:36:cursor A] disabled, scaler_id = 0
[   13.590759] [drm:drm_atomic_get_crtc_state] Added [CRTC:51:pipe B]         (ptrval) state to         (ptrval)
[   13.591510] [drm:drm_atomic_get_crtc_state] Added [CRTC:63:pipe C]         (ptrval) state to         (ptrval)
[   13.592275] [drm:intel_plane_atomic_calc_changes] [CRTC:39:pipe A] has [PLANE:28:primary A] with fb 78
[   13.592991] [drm:intel_plane_atomic_calc_changes] [PLANE:28:primary A] visible 0 -> 1, off 0, on 1, ms 1
[   13.594306] [drm:intel_find_shared_dpll] [CRTC:39:pipe A] allocated WRPLL 1
[   13.594829] [drm:intel_reference_shared_dpll] using WRPLL 1 for pipe A
[   13.595337] [drm:drm_atomic_commit] committing         (ptrval)
[   13.599389] [drm:intel_power_well_enable] enabling always-on
[   13.599817] [drm:intel_power_well_enable] enabling display
[   13.600266] [drm:drm_calc_timestamping_constants] crtc 39: hwmode: htotal 2080, vtotal 2222, vdisplay 2160
[   13.601000] [drm:drm_calc_timestamping_constants] crtc 39: clock 277250 kHz framedur 16670009 linedur 7502
[   13.601743] [drm:intel_atomic_commit_tail] [ENCODER:65:CRT]
[   13.602172] [drm:intel_atomic_commit_tail] [ENCODER:66:DDI C]
[   13.602619] [drm:intel_atomic_commit_tail] [ENCODER:72:DDI D]
[   13.603072] [drm:verify_single_dpll_state.isra.125] WRPLL 1
[   13.603506] [drm:verify_single_dpll_state.isra.125] WRPLL 2
[   13.603938] [drm:verify_single_dpll_state.isra.125] SPLL
[   13.604359] [drm:verify_single_dpll_state.isra.125] LCPLL 810
[   13.604811] [drm:verify_single_dpll_state.isra.125] LCPLL 1350
[   13.605260] [drm:verify_single_dpll_state.isra.125] LCPLL 2700
[   13.605718] [drm:intel_enable_shared_dpll] enable WRPLL 1 (active 1, on? 0) for crtc 39
[   13.606245] [drm:intel_enable_shared_dpll] enabling WRPLL 1
[   13.606751] [drm:intel_enable_pipe] enabling pipe A
[   13.607025] [drm:drm_crtc_vblank_on] crtc 0, vblank enabled 0, inmodeset 1
[   13.607741] [drm:drm_calc_vbltimestamp_from_scanoutpos] crtc 0: Noisy timestamp 110 us > 20 us [3 reps].
[   13.608266] [drm:drm_calc_vbltimestamp_from_scanoutpos] crtc 0 : v p(0,2)@ 13.607740 -> 13.607725 [e 110 us, 3 rep]
[   13.608829] [drm:intel_audio_codec_enable] ELD on [CONNECTOR:73:HDMI-A-2], [ENCODER:72:DDI D]
[   13.609468] [drm:hsw_audio_codec_enable] Enable audio codec on pipe A, 32 bytes ELD
[   13.609997] [drm:audio_config_hdmi_pixel_clock] HDMI audio pixel clock setting for 277250 not found, falling back to defaults
[   13.610758] [drm:audio_config_hdmi_pixel_clock] Configuring HDMI audio for pixel clock 25200 (0x00010000)
[   13.611392] [drm:hsw_audio_config_update] using automatic N
[   13.611774] [drm:drm_calc_vbltimestamp_from_scanoutpos] crtc 0 : v p(0,328)@ 13.611773 -> 13.609312 [e 0 us, 0 rep]
[   13.612471] [drm:drm_vblank_restore] missed 0 vblanks in 1587351 ns, frame duration=16670009 ns, hw_diff=1
[   13.613229] [drm:drm_vblank_enable] enabling vblank on crtc 0, ret: 0
[   13.613763] [drm:drm_calc_vbltimestamp_from_scanoutpos] crtc 0 : v p(0,594)@ 13.613762 -> 13.609306 [e 0 us, 0 rep]
[   13.614570] [drm:drm_update_vblank_count] updating vblank count on crtc 0: current=1, diff=0, hw=1 hw_last=1
[   13.615315] [drm:drm_calc_vbltimestamp_from_scanoutpos] crtc 0 : v p(0,801)@ 13.615314 -> 13.609305 [e 0 us, 0 rep]
[   13.616102] [drm:drm_update_vblank_count] updating vblank count on crtc 0: current=1, diff=0, hw=1 hw_last=1
[   13.625531] [drm:drm_calc_vbltimestamp_from_scanoutpos] crtc 0 : v p(0,-59)@ 13.625527 -> 13.625970 [e 0 us, 0 rep]
[   13.626240] [drm:drm_update_vblank_count] updating vblank count on crtc 0: current=1, diff=1, hw=2 hw_last=1
[   13.626773] [drm:drm_handle_vblank] vblank event on 2, current 2
[   13.627121] random: fast init done
[   13.627335] [drm:verify_connector_state.isra.124] [CONNECTOR:73:HDMI-A-2]
[   13.627725] [drm:intel_atomic_commit_tail] [CRTC:39:pipe A]
[   13.628054] [drm:verify_single_dpll_state.isra.125] WRPLL 1
[   13.628371] [drm:drm_atomic_state_default_clear] Clearing atomic state         (ptrval)
[   13.628815] [drm:drm_mode_object_put.part.2] OBJ ID: 73 (4)
[   13.629125] [drm:__drm_atomic_state_free] Freeing atomic state         (ptrval)
[   13.629530] [drm:drm_mode_object_get] OBJ ID: 78 (3)
[   13.629813] [drm:drm_mode_object_put.part.2] OBJ ID: 78 (4)
[   13.630131] [drm:drm_mode_object_put.part.2] OBJ ID: 73 (3)
[   13.642174] [drm:drm_calc_vbltimestamp_from_scanoutpos] crtc 0 : v p(0,-60)@ 13.642171 -> 13.642621 [e 0 us, 0 rep]
[   13.642787] [drm:drm_update_vblank_count] updating vblank count on crtc 0: current=2, diff=1, hw=3 hw_last=2
[   13.643337] [drm:vblank_disable_fn] disabling vblank on crtc 0
[   13.643662] [drm:drm_calc_vbltimestamp_from_scanoutpos] crtc 0 : v p(0,139)@ 13.643662 -> 13.642619 [e 0 us, 0 rep]
[   13.644235] [drm:drm_update_vblank_count] updating vblank count on crtc 0: current=3, diff=0, hw=3 hw_last=3
