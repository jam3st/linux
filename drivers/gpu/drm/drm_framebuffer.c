/*
 * Copyright (c) 2016 Intel Corporation
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

#include <linux/export.h>
#include <drm/drmP.h>
#include <drm/drm_auth.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_atomic.h>
#include <drm/drm_print.h>

#include "drm_internal.h"
#include "drm_crtc_internal.h"


static int fb_plane_width(int width,
			  const struct drm_format_info *format, int plane)
{
	if (plane == 0)
		return width;

	return DIV_ROUND_UP(width, format->hsub);
}

static int fb_plane_height(int height,
			   const struct drm_format_info *format, int plane)
{
	if (plane == 0)
		return height;

	return DIV_ROUND_UP(height, format->vsub);
}

static int framebuffer_check(struct drm_device *dev,
			     const struct drm_mode_fb_cmd2 *r)
{
	const struct drm_format_info *info;
	int i;

	/* check if the format is supported at all */
	info = __drm_format_info(r->pixel_format & ~DRM_FORMAT_BIG_ENDIAN);
	if (!info) {
		struct drm_format_name_buf format_name;

		DRM_DEBUG_KMS("bad framebuffer format %s\n",
			      drm_get_format_name(r->pixel_format,
						  &format_name));
		return -EINVAL;
	}

	/* now let the driver pick its own format info */
	info = drm_get_format_info(dev, r);

	if (r->width == 0) {
		DRM_DEBUG_KMS("bad framebuffer width %u\n", r->width);
		return -EINVAL;
	}

	if (r->height == 0) {
		DRM_DEBUG_KMS("bad framebuffer height %u\n", r->height);
		return -EINVAL;
	}

	for (i = 0; i < info->num_planes; i++) {
		unsigned int width = fb_plane_width(r->width, info, i);
		unsigned int height = fb_plane_height(r->height, info, i);
		unsigned int cpp = info->cpp[i];

		if (!r->handles[i]) {
			DRM_DEBUG_KMS("no buffer object handle for plane %d\n", i);
			return -EINVAL;
		}

		if ((uint64_t) width * cpp > UINT_MAX)
			return -ERANGE;

		if ((uint64_t) height * r->pitches[i] + r->offsets[i] > UINT_MAX)
			return -ERANGE;

		if (r->pitches[i] < width * cpp) {
			DRM_DEBUG_KMS("bad pitch %u for plane %d\n", r->pitches[i], i);
			return -EINVAL;
		}

		if (r->modifier[i] && !(r->flags & DRM_MODE_FB_MODIFIERS)) {
			DRM_DEBUG_KMS("bad fb modifier %llu for plane %d\n",
				      r->modifier[i], i);
			return -EINVAL;
		}

		if (r->flags & DRM_MODE_FB_MODIFIERS &&
		    r->modifier[i] != r->modifier[0]) {
			DRM_DEBUG_KMS("bad fb modifier %llu for plane %d\n",
				      r->modifier[i], i);
			return -EINVAL;
		}

		/* modifier specific checks: */
		switch (r->modifier[i]) {
		case DRM_FORMAT_MOD_SAMSUNG_64_32_TILE:
			/* NOTE: the pitch restriction may be lifted later if it turns
			 * out that no hw has this restriction:
			 */
			if (r->pixel_format != DRM_FORMAT_NV12 ||
					width % 128 || height % 32 ||
					r->pitches[i] % 128) {
				DRM_DEBUG_KMS("bad modifier data for plane %d\n", i);
				return -EINVAL;
			}
			break;

		default:
			break;
		}
	}

	for (i = info->num_planes; i < 4; i++) {
		if (r->modifier[i]) {
			DRM_DEBUG_KMS("non-zero modifier for unused plane %d\n", i);
			return -EINVAL;
		}

		/* Pre-FB_MODIFIERS userspace didn't clear the structs properly. */
		if (!(r->flags & DRM_MODE_FB_MODIFIERS))
			continue;

		if (r->handles[i]) {
			DRM_DEBUG_KMS("buffer object handle for unused plane %d\n", i);
			return -EINVAL;
		}

		if (r->pitches[i]) {
			DRM_DEBUG_KMS("non-zero pitch for unused plane %d\n", i);
			return -EINVAL;
		}

		if (r->offsets[i]) {
			DRM_DEBUG_KMS("non-zero offset for unused plane %d\n", i);
			return -EINVAL;
		}
	}

	return 0;
}

struct drm_framebuffer *
drm_internal_framebuffer_create(struct drm_device *dev,
				const struct drm_mode_fb_cmd2 *r,
				struct drm_file *file_priv)
{
	struct drm_mode_config *config = &dev->mode_config;
	struct drm_framebuffer *fb;
	int ret;

    printk("drm_internal_framebuffer_createdrm_internal_framebuffer_create xxxxxxxxxxxxxx");
	if (r->flags & ~(DRM_MODE_FB_INTERLACED | DRM_MODE_FB_MODIFIERS)) {
		DRM_DEBUG_KMS("bad framebuffer flags 0x%08x\n", r->flags);
		return ERR_PTR(-EINVAL);
	}

	if ((config->min_width > r->width) || (r->width > config->max_width)) {
		DRM_DEBUG_KMS("bad framebuffer width %d, should be >= %d && <= %d\n",
			  r->width, config->min_width, config->max_width);
		return ERR_PTR(-EINVAL);
	}
	if ((config->min_height > r->height) || (r->height > config->max_height)) {
		DRM_DEBUG_KMS("bad framebuffer height %d, should be >= %d && <= %d\n",
			  r->height, config->min_height, config->max_height);
		return ERR_PTR(-EINVAL);
	}

	if (r->flags & DRM_MODE_FB_MODIFIERS &&
	    !dev->mode_config.allow_fb_modifiers) {
		DRM_DEBUG_KMS("driver does not support fb modifiers\n");
		return ERR_PTR(-EINVAL);
	}

	ret = framebuffer_check(dev, r);
	if (ret)
		return ERR_PTR(ret);

	fb = dev->mode_config.funcs->fb_create(dev, file_priv, r);
	if (IS_ERR(fb)) {
		DRM_DEBUG_KMS("could not create framebuffer\n");
		return fb;
	}

	return fb;
}

/**
 * drm_mode_addfb2 - add an FB to the graphics configuration
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 * Add a new FB to the specified CRTC, given a user request with format. This is
 * the 2nd version of the addfb ioctl, which supports multi-planar framebuffers
 * and uses fourcc codes as pixel format specifiers.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_addfb2(struct drm_device *dev,
		    void *data, struct drm_file *file_priv)
{
	struct drm_mode_fb_cmd2 *r = data;
	struct drm_framebuffer *fb;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	fb = drm_internal_framebuffer_create(dev, r, file_priv);
	if (IS_ERR(fb))
		return PTR_ERR(fb);

	DRM_DEBUG_KMS("[FB:%d]\n", fb->base.id);
	r->fb_id = fb->base.id;

	/* Transfer ownership to the filp for reaping on close */
	mutex_lock(&file_priv->fbs_lock);
	list_add(&fb->filp_head, &file_priv->fbs);
	mutex_unlock(&file_priv->fbs_lock);

	return 0;
}

struct drm_mode_rmfb_work {
	struct work_struct work;
	struct list_head fbs;
};



/**
 * drm_mode_dirtyfb_ioctl - flush frontbuffer rendering on an FB
 * @dev: drm device for the ioctl
 * @data: data pointer for the ioctl
 * @file_priv: drm file for the ioctl call
 *
 * Lookup the FB and flush out the damaged area supplied by userspace as a clip
 * rectangle list. Generic userspace which does frontbuffer rendering must call
 * this ioctl to flush out the changes on manual-update display outputs, e.g.
 * usb display-link, mipi manual update panels or edp panel self refresh modes.
 *
 * Modesetting drivers which always update the frontbuffer do not need to
 * implement the corresponding &drm_framebuffer_funcs.dirty callback.
 *
 * Called by the user via ioctl.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_mode_dirtyfb_ioctl(struct drm_device *dev,
			   void *data, struct drm_file *file_priv)
{
	struct drm_clip_rect __user *clips_ptr;
	struct drm_clip_rect *clips = NULL;
	struct drm_mode_fb_dirty_cmd *r = data;
	struct drm_framebuffer *fb;
	unsigned flags;
	int num_clips;
	int ret;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -EINVAL;

	fb = drm_framebuffer_lookup(dev, file_priv, r->fb_id);
	if (!fb)
		return -ENOENT;

	num_clips = r->num_clips;
	clips_ptr = (struct drm_clip_rect __user *)(unsigned long)r->clips_ptr;

	if (!num_clips != !clips_ptr) {
		ret = -EINVAL;
		goto out_err1;
	}

	flags = DRM_MODE_FB_DIRTY_FLAGS & r->flags;

	/* If userspace annotates copy, clips must come in pairs */
	if (flags & DRM_MODE_FB_DIRTY_ANNOTATE_COPY && (num_clips % 2)) {
		ret = -EINVAL;
		goto out_err1;
	}

	if (num_clips && clips_ptr) {
		if (num_clips < 0 || num_clips > DRM_MODE_FB_DIRTY_MAX_CLIPS) {
			ret = -EINVAL;
			goto out_err1;
		}
		clips = kcalloc(num_clips, sizeof(*clips), GFP_KERNEL);
		if (!clips) {
			ret = -ENOMEM;
			goto out_err1;
		}

		ret = copy_from_user(clips, clips_ptr,
				     num_clips * sizeof(*clips));
		if (ret) {
			ret = -EFAULT;
			goto out_err2;
		}
	}

	if (fb->funcs->dirty) {
		ret = fb->funcs->dirty(fb, file_priv, flags, r->color,
				       clips, num_clips);
	} else {
		ret = -ENOSYS;
	}

out_err2:
	kfree(clips);
out_err1:
	drm_framebuffer_put(fb);

	return ret;
}

void drm_framebuffer_free(struct kref *kref)
{
	struct drm_framebuffer *fb =
			container_of(kref, struct drm_framebuffer, base.refcount);
	struct drm_device *dev = fb->dev;

	/*
	 * The lookup idr holds a weak reference, which has not necessarily been
	 * removed at this point. Check for that.
	 */
	drm_mode_object_unregister(dev, &fb->base);

	fb->funcs->destroy(fb);
}

/**
 * drm_framebuffer_init - initialize a framebuffer
 * @dev: DRM device
 * @fb: framebuffer to be initialized
 * @funcs: ... with these functions
 *
 * Allocates an ID for the framebuffer's parent mode object, sets its mode
 * functions & device file and adds it to the master fd list.
 *
 * IMPORTANT:
 * This functions publishes the fb and makes it available for concurrent access
 * by other users. Which means by this point the fb _must_ be fully set up -
 * since all the fb attributes are invariant over its lifetime, no further
 * locking but only correct reference counting is required.
 *
 * Returns:
 * Zero on success, error code on failure.
 */
int drm_framebuffer_init(struct drm_device *dev, struct drm_framebuffer *fb,
			 const struct drm_framebuffer_funcs *funcs)
{
	int ret;

	if (WARN_ON_ONCE(fb->dev != dev || !fb->format))
		return -EINVAL;

	INIT_LIST_HEAD(&fb->filp_head);

	fb->funcs = funcs;
	strcpy(fb->comm, current->comm);

	ret = __drm_mode_object_add(dev, &fb->base, DRM_MODE_OBJECT_FB,
				    false, drm_framebuffer_free);

    if (ret)
		goto out;

	mutex_lock(&dev->mode_config.fb_lock);
	dev->mode_config.num_fb++;
	list_add(&fb->head, &dev->mode_config.fb_list);
	mutex_unlock(&dev->mode_config.fb_lock);

	drm_mode_object_register(dev, &fb->base);
out:
	return ret;
}
EXPORT_SYMBOL(drm_framebuffer_init);

/**
 * drm_framebuffer_lookup - look up a drm framebuffer and grab a reference
 * @dev: drm device
 * @file_priv: drm file to check for lease against.
 * @id: id of the fb object
 *
 * If successful, this grabs an additional reference to the framebuffer -
 * callers need to make sure to eventually unreference the returned framebuffer
 * again, using drm_framebuffer_put().
 */
struct drm_framebuffer *drm_framebuffer_lookup(struct drm_device *dev,
					       struct drm_file *file_priv,
					       uint32_t id)
{
	struct drm_mode_object *obj;
	struct drm_framebuffer *fb = NULL;

	obj = __drm_mode_object_find(dev, file_priv, id, DRM_MODE_OBJECT_FB);
	if (obj)
		fb = obj_to_fb(obj);
	return fb;
}
EXPORT_SYMBOL(drm_framebuffer_lookup);

/**
 * drm_framebuffer_unregister_private - unregister a private fb from the lookup idr
 * @fb: fb to unregister
 *
 * Drivers need to call this when cleaning up driver-private framebuffers, e.g.
 * those used for fbdev. Note that the caller must hold a reference of it's own,
 * i.e. the object may not be destroyed through this call (since it'll lead to a
 * locking inversion).
 *
 * NOTE: This function is deprecated. For driver-private framebuffers it is not
 * recommended to embed a framebuffer struct info fbdev struct, instead, a
 * framebuffer pointer is preferred and drm_framebuffer_put() should be called
 * when the framebuffer is to be cleaned up.
 */
void drm_framebuffer_unregister_private(struct drm_framebuffer *fb)
{
	struct drm_device *dev;

	if (!fb)
		return;

	dev = fb->dev;

	/* Mark fb as reaped and drop idr ref. */
	drm_mode_object_unregister(dev, &fb->base);
}
EXPORT_SYMBOL(drm_framebuffer_unregister_private);

/**
 * drm_framebuffer_cleanup - remove a framebuffer object
 * @fb: framebuffer to remove
 *
 * Cleanup framebuffer. This function is intended to be used from the drivers
 * &drm_framebuffer_funcs.destroy callback. It can also be used to clean up
 * driver private framebuffers embedded into a larger structure.
 *
 * Note that this function does not remove the fb from active usage - if it is
 * still used anywhere, hilarity can ensue since userspace could call getfb on
 * the id and get back -EINVAL. Obviously no concern at driver unload time.
 *
 * Also, the framebuffer will not be removed from the lookup idr - for
 * user-created framebuffers this will happen in in the rmfb ioctl. For
 * driver-private objects (e.g. for fbdev) drivers need to explicitly call
 * drm_framebuffer_unregister_private.
 */
void drm_framebuffer_cleanup(struct drm_framebuffer *fb)
{
	struct drm_device *dev = fb->dev;

	mutex_lock(&dev->mode_config.fb_lock);
	list_del(&fb->head);
	dev->mode_config.num_fb--;
	mutex_unlock(&dev->mode_config.fb_lock);
}
EXPORT_SYMBOL(drm_framebuffer_cleanup);

static int atomic_remove_fb(struct drm_framebuffer *fb)
{
	struct drm_modeset_acquire_ctx ctx;
	struct drm_device *dev = fb->dev;
	struct drm_atomic_state *state;
	struct drm_plane *plane;
	struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	int i, ret;
	unsigned plane_mask;
	bool disable_crtcs = false;

retry_disable:
	drm_modeset_acquire_init(&ctx, 0);

	state = drm_atomic_state_alloc(dev);
	if (!state) {
		ret = -ENOMEM;
		goto out;
	}
	state->acquire_ctx = &ctx;

retry:
	plane_mask = 0;
	ret = drm_modeset_lock_all_ctx(dev, &ctx);
	if (ret)
		goto unlock;

	drm_for_each_plane(plane, dev) {
		struct drm_plane_state *plane_state;

		if (plane->state->fb != fb)
			continue;

		plane_state = drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state)) {
			ret = PTR_ERR(plane_state);
			goto unlock;
		}

		if (disable_crtcs && plane_state->crtc->primary == plane) {
			struct drm_crtc_state *crtc_state;

			crtc_state = drm_atomic_get_existing_crtc_state(state, plane_state->crtc);

			ret = drm_atomic_add_affected_connectors(state, plane_state->crtc);
			if (ret)
				goto unlock;

			crtc_state->active = false;
			ret = drm_atomic_set_mode_for_crtc(crtc_state, NULL);
			if (ret)
				goto unlock;
		}

		drm_atomic_set_fb_for_plane(plane_state, NULL);
		ret = drm_atomic_set_crtc_for_plane(plane_state, NULL);
		if (ret)
			goto unlock;

		plane_mask |= BIT(drm_plane_index(plane));

		plane->old_fb = plane->fb;
	}

	/* This list is only filled when disable_crtcs is set. */
	for_each_new_connector_in_state(state, conn, conn_state, i) {
		ret = drm_atomic_set_crtc_for_connector(conn_state, NULL);

		if (ret)
			goto unlock;
	}

	if (plane_mask)
		ret = drm_atomic_commit(state);

unlock:
	if (plane_mask)
		drm_atomic_clean_old_fb(dev, plane_mask, ret);

	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		drm_modeset_backoff(&ctx);
		goto retry;
	}

	drm_atomic_state_put(state);

out:
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	if (ret == -EINVAL && !disable_crtcs) {
		disable_crtcs = true;
		goto retry_disable;
	}

	return ret;
}



/**
 * drm_framebuffer_plane_width - width of the plane given the first plane
 * @width: width of the first plane
 * @fb: the framebuffer
 * @plane: plane index
 *
 * Returns:
 * The width of @plane, given that the width of the first plane is @width.
 */
int drm_framebuffer_plane_width(int width,
				const struct drm_framebuffer *fb, int plane)
{
	if (plane >= fb->format->num_planes)
		return 0;

	return fb_plane_width(width, fb->format, plane);
}
EXPORT_SYMBOL(drm_framebuffer_plane_width);

/**
 * drm_framebuffer_plane_height - height of the plane given the first plane
 * @height: height of the first plane
 * @fb: the framebuffer
 * @plane: plane index
 *
 * Returns:
 * The height of @plane, given that the height of the first plane is @height.
 */
int drm_framebuffer_plane_height(int height,
				 const struct drm_framebuffer *fb, int plane)
{
	if (plane >= fb->format->num_planes)
		return 0;

	return fb_plane_height(height, fb->format, plane);
}
EXPORT_SYMBOL(drm_framebuffer_plane_height);

void drm_framebuffer_print_info(struct drm_printer *p, unsigned int indent,
				const struct drm_framebuffer *fb)
{
	struct drm_format_name_buf format_name;
	unsigned int i;

	drm_printf_indent(p, indent, "allocated by = %s\n", fb->comm);
	drm_printf_indent(p, indent, "refcount=%u\n",
			  drm_framebuffer_read_refcount(fb));
	drm_printf_indent(p, indent, "format=%s\n",
			  drm_get_format_name(fb->format->format, &format_name));
	drm_printf_indent(p, indent, "modifier=0x%llx\n", fb->modifier);
	drm_printf_indent(p, indent, "size=%ux%u\n", fb->width, fb->height);
	drm_printf_indent(p, indent, "layers:\n");

	for (i = 0; i < fb->format->num_planes; i++) {
		drm_printf_indent(p, indent + 1, "size[%u]=%dx%d\n", i,
				  drm_framebuffer_plane_width(fb->width, fb, i),
				  drm_framebuffer_plane_height(fb->height, fb, i));
		drm_printf_indent(p, indent + 1, "pitch[%u]=%u\n", i, fb->pitches[i]);
		drm_printf_indent(p, indent + 1, "offset[%u]=%u\n", i, fb->offsets[i]);
		drm_printf_indent(p, indent + 1, "obj[%u]:%s\n", i,
				  fb->obj[i] ? "" : "(null)");
		if (fb->obj[i])
			drm_gem_print_info(p, indent + 2, fb->obj[i]);
	}
}
