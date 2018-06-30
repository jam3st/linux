/*
 * Copyright © 2008-2015 Intel Corporation
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
 *    Eric Anholt <eric@anholt.net>
 *
 */

#include <drm/drmP.h>
#include <drm/drm_vma_manager.h>
#include <drm/i915_drm.h>
#include "i915_drv.h"
#include "i915_gem_clflush.h"
#include "i915_trace.h"
#include "intel_drv.h"
#include "intel_frontbuffer.h"
#include <linux/dma-fence-array.h>
#include <linux/kthread.h>
#include <linux/reservation.h>
#include <linux/shmem_fs.h>
#include <linux/slab.h>
#include <linux/stop_machine.h>
#include <linux/swap.h>
#include <linux/pci.h>
#include <linux/dma-buf.h>



/* some bookkeeping */
static void i915_gem_info_add_obj(struct drm_i915_private *dev_priv,
				  u64 size)
{
	spin_lock(&dev_priv->mm.object_stat_lock);
	dev_priv->mm.object_count++;
	dev_priv->mm.object_memory += size;
	spin_unlock(&dev_priv->mm.object_stat_lock);
}



static void
flush_write_domain(struct drm_i915_gem_object *obj, unsigned int flush_domains)
{
	if (!(obj->write_domain & flush_domains))
		return;

}

 void i915_gem_stolen_remove_node(struct drm_i915_private *dev_priv,
                                  struct drm_mm_node *node)
 {
         mutex_lock(&dev_priv->mm.stolen_lock);
         drm_mm_remove_node(node);
         mutex_unlock(&dev_priv->mm.stolen_lock);
 }

 static void i915_gem_object_bump_inactive_ggtt(struct drm_i915_gem_object *obj)
 {
         struct drm_i915_private *i915;
         struct list_head *list;
         struct i915_vma *vma;

         GEM_BUG_ON(!i915_gem_object_has_pinned_pages(obj));

         for_each_ggtt_vma(vma, obj) {
                 if (i915_vma_is_active(vma))
                         continue;

                 if (!drm_mm_node_allocated(&vma->node))
                         continue;

                 list_move_tail(&vma->vm_link, &vma->vm->inactive_list);
         }

         i915 = to_i915(obj->base.dev);
         spin_lock(&i915->mm.obj_lock);
         list = obj->bind_count ? &i915->mm.bound_list : &i915->mm.unbound_list;
         list_move_tail(&obj->mm.link, list);
         spin_unlock(&i915->mm.obj_lock);
 }


void __i915_gem_object_set_pages(struct drm_i915_gem_object *obj,
				 struct sg_table *pages,
				 unsigned int sg_page_sizes)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	unsigned long supported = INTEL_INFO(i915)->page_sizes;
	int i;

printk("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx __i915_gem_object_set_pages xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
	lockdep_assert_held(&obj->mm.lock);

	obj->mm.get_page.sg_pos = pages->sgl;
	obj->mm.get_page.sg_idx = 0;

	obj->mm.pages = pages;

	GEM_BUG_ON(!sg_page_sizes);
	obj->mm.page_sizes.phys = sg_page_sizes;

	/*
	 * Calculate the supported page-sizes which fit into the given
	 * sg_page_sizes. This will give us the page-sizes which we may be able
	 * to use opportunistically when later inserting into the GTT. For
	 * example if phys=2G, then in theory we should be able to use 1G, 2M,
	 * 64K or 4K pages, although in practice this will depend on a number of
	 * other factors.
	 */
	obj->mm.page_sizes.sg = 0;
	for_each_set_bit(i, &supported, ilog2(I915_GTT_MAX_PAGE_SIZE) + 1) {
		if (obj->mm.page_sizes.phys & ~0u << i)
			obj->mm.page_sizes.sg |= BIT(i);
	}
	GEM_BUG_ON(!HAS_PAGE_SIZES(i915, obj->mm.page_sizes.sg));

	spin_lock(&i915->mm.obj_lock);
//	list_add(&obj->mm.link, &i915->mm.unbound_list);
	spin_unlock(&i915->mm.obj_lock);
}



void *i915_gem_object_alloc(struct drm_i915_private *dev_priv)
 {
    printk("i915_gem_object_alloc stolen");

        return kmem_cache_zalloc(dev_priv->objects, GFP_KERNEL);
 }


int i915_gem_object_set_cache_level(struct drm_i915_gem_object *obj,
                                    enum i915_cache_level cache_level)
{
        struct i915_vma *vma;
        int ret;

        lockdep_assert_held(&obj->base.dev->struct_mutex);

        if (obj->cache_level == cache_level)
                return 0;


        i915_gem_object_set_cache_coherency(obj, cache_level);
        obj->cache_dirty = true; /* Always invalidate stale cachelines */

        return 0;
}

/*
 * Prepare buffer for display plane (scanout, cursors, etc).
 * Can be called from an uninterruptible phase (modesetting) and allows
 * any flushes to be pipelined (for pageflips).
 */
struct i915_vma *
i915_gem_object_pin_to_display_plane(struct drm_i915_gem_object *obj,
				     u32 alignment,
				     const struct i915_ggtt_view *view,
				     unsigned int flags)
{
	struct i915_vma *vma;
	int ret;

	lockdep_assert_held(&obj->base.dev->struct_mutex);

	/* Mark the global pin early so that we account for the
	 * display coherency whilst setting up the cache domains.
	 */
	obj->pin_global++;

	/* The display engine is not coherent with the LLC cache on gen6.  As
	 * a result, we make sure that the pinning that is about to occur is
	 * done with uncached PTEs. This is lowest common denominator for all
	 * chipsets.
	 *
	 * However for gen6+, we could do better by using the GFDT bit instead
	 * of uncaching, which would allow us to flush all the LLC-cached data
	 * with that bit in the PTE to main memory with just one PIPE_CONTROL.
	 */
    printk("i915_gem_object_pin_to_display_plane b4");
	ret = i915_gem_object_set_cache_level(obj,
					      HAS_WT(to_i915(obj->base.dev)) ?
					      I915_CACHE_WT : I915_CACHE_NONE);
    printk("i915_gem_object_pin_to_display_plane fter");
	if (ret) {
    printk("i915_gem_object_pin_to_display_plane error");
		vma = ERR_PTR(ret);
		goto err_unpin_global;
    }

	/* As the user may map the buffer once pinned in the display plane
	 * (e.g. libkms for the bootup splash), we have to ensure that we
	 * always use map_and_fenceable for all scanout buffers. However,
	 * it may simply be too big to fit into mappable, in which case
	 * put it anyway and hope that userspace can cope (but always first
	 * try to preserve the existing ABI).
	 */
    printk("i915_gem_object_pin_to_display_plane x0");
	vma = ERR_PTR(-ENOSPC);
	if ((flags & PIN_MAPPABLE) == 0 &&
	    (!view || view->type == I915_GGTT_VIEW_NORMAL))
    printk("i915_gem_object_pin_to_display_plane x1");
		vma = i915_gem_object_ggtt_pin(obj, view, 0, alignment,
					       flags |
					       PIN_MAPPABLE |
					       PIN_NONBLOCK);
    printk("i915_gem_object_pin_to_display_plane x2 %llx", vma);
	if (IS_ERR(vma))
		vma = i915_gem_object_ggtt_pin(obj, view, 0, alignment, flags);
	if (IS_ERR(vma))
		goto err_unpin_global;

	vma->display_alignment = max_t(u64, vma->display_alignment, alignment);

	/* Treat this as an end-of-frame, like intel_user_framebuffer_dirty() */
	intel_fb_obj_flush(obj, ORIGIN_DIRTYFB);

	/* It should now be out of any other write domains, and we can update
	 * the domain values for our changes.
	 */
	obj->read_domains |= I915_GEM_DOMAIN_GTT;


err_unpin_global:
    printk("dma failed err_unpin_global");
	obj->pin_global--;
	return vma;
}

void
i915_gem_object_unpin_from_display_plane(struct i915_vma *vma)
{
	lockdep_assert_held(&vma->vm->i915->drm.struct_mutex);

	if (WARN_ON(vma->obj->pin_global == 0))
		return;

	if (--vma->obj->pin_global == 0)
		vma->display_alignment = I915_GTT_MIN_ALIGNMENT;

	/* Bump the LRU to try and avoid premature eviction whilst flipping  */
	i915_gem_object_bump_inactive_ggtt(vma->obj);

	i915_vma_unpin(vma);
}



struct i915_vma *
i915_gem_object_ggtt_pin(struct drm_i915_gem_object *obj,
			 const struct i915_ggtt_view *view,
			 u64 size,
			 u64 alignment,
			 u64 flags)
{
    struct drm_i915_private *dev_priv = to_i915(obj->base.dev);
    struct i915_address_space *vm = &dev_priv->ggtt.base;
    struct i915_vma *vma;
    int ret;

    lockdep_assert_held(&obj->base.dev->struct_mutex);

    if (flags & PIN_MAPPABLE &&
        (!view || view->type == I915_GGTT_VIEW_NORMAL)) {
        /* If the required space is larger than the available
         * aperture, we will not able to find a slot for the
         * object and unbinding the object now will be in
         * vain. Worse, doing so may cause us to ping-pong
         * the object in and out of the Global GTT and
         * waste a lot of cycles under the mutex.
         */
        if (obj->base.size > dev_priv->ggtt.mappable_end) {
            printk("2big");
            return ERR_PTR(-E2BIG);
            }

        /* If NONBLOCK is set the caller is optimistically
         * trying to cache the full object within the mappable
         * aperture, and *must* have a fallback in place for
         * situations where we cannot bind the object. We
         * can be a little more lax here and use the fallback
         * more often to avoid costly migrations of ourselves
         * and other objects within the aperture.
         *
         * Half-the-aperture is used as a simple heuristic.
         * More interesting would to do search for a free
         * block prior to making the commitment to unbind.
         * That caters for the self-harm case, and with a
         * little more heuristics (e.g. NOFAULT, NOEVICT)
         * we could try to minimise harm to others.
         */
        if (flags & PIN_NONBLOCK &&
            obj->base.size > dev_priv->ggtt.mappable_end / 2) {
            printk("nospace");
            return ERR_PTR(-ENOSPC);
        }
    }

 printk("instance obh");
    vma = i915_vma_instance(obj, vm, view);
 printk("instance obh %llx", vma);
    if (unlikely(IS_ERR(vma))) {
 printk("Is err %llx", vma);
        return vma;
        }


 printk("Is pinxxx %llx", vma);
    ret = i915_vma_pin(vma, size, alignment, flags | PIN_GLOBAL);

    if (ret) {
 printk("Is pinxxx  ret %x", ret);
        return ERR_PTR(ret);
        }

 printk("ret xxx %llx", vma);
    return vma;
}




void i915_gem_object_init(struct drm_i915_gem_object *obj,
			  const struct drm_i915_gem_object_ops *ops)
{
	mutex_init(&obj->mm.lock);

	INIT_LIST_HEAD(&obj->vma_list);
	INIT_LIST_HEAD(&obj->lut_list);
	INIT_LIST_HEAD(&obj->batch_pool_link);

	obj->ops = ops;

	reservation_object_init(&obj->__builtin_resv);
	obj->resv = &obj->__builtin_resv;

	obj->frontbuffer_ggtt_origin = ORIGIN_GTT;

	obj->mm.madv = I915_MADV_WILLNEED;
	INIT_RADIX_TREE(&obj->mm.get_page.radix, GFP_KERNEL | __GFP_NOWARN);
	mutex_init(&obj->mm.get_page.lock);

	i915_gem_info_add_obj(to_i915(obj->base.dev), obj->base.size);
}


void
i915_gem_load_init_fences(struct drm_i915_private *dev_priv)
{
	int i;
    INIT_LIST_HEAD(&dev_priv->mm.fence_list);
    dev_priv->objects = KMEM_CACHE(drm_i915_gem_object, SLAB_HWCACHE_ALIGN);
dev_priv->vmas = KMEM_CACHE(i915_vma, SLAB_HWCACHE_ALIGN);

        dev_priv->num_fence_regs = 32;
printk("i915_gem_load_init_fences start");
//	/* Initialize fence registers to zero */
    for (i = 0; i < dev_priv->num_fence_regs; i++) {
        struct drm_i915_fence_reg *fence = &dev_priv->fence_regs[i];
printk("F %d %x %x", i, fence, dev_priv);
        fence->i915 = dev_priv;
        fence->id = i;
printk("F %d %x %x", i, fence->i915, dev_priv);

        list_add_tail(&fence->link, &dev_priv->mm.fence_list);
    }
    printk("i915_gem_load_init_fences end");
    //i915_gem_restore_fences(dev_priv);

	i915_gem_detect_bit_6_swizzle(dev_priv);
}

static void i915_gem_init__mm(struct drm_i915_private *i915)
{
    spin_lock_init(&i915->mm.object_stat_lock);
    spin_lock_init(&i915->mm.obj_lock);
    spin_lock_init(&i915->mm.free_lock);

    init_llist_head(&i915->mm.free_list);

    INIT_LIST_HEAD(&i915->mm.unbound_list);
    INIT_LIST_HEAD(&i915->mm.bound_list);
    INIT_LIST_HEAD(&i915->mm.fence_list);
    INIT_LIST_HEAD(&i915->mm.userfault_list);

}

static void __start_cpu_write(struct drm_i915_gem_object *obj)
{
    obj->read_domains = I915_GEM_DOMAIN_CPU;
    obj->write_domain = I915_GEM_DOMAIN_CPU;
       obj->cache_dirty = true;
}


static void
__i915_gem_object_release_shmem(struct drm_i915_gem_object *obj,
                struct sg_table *pages,
                bool needs_clflush)
{
    GEM_BUG_ON(obj->mm.madv == __I915_MADV_PURGED);

    if (obj->mm.madv == I915_MADV_DONTNEED)
        obj->mm.dirty = false;

    if (needs_clflush &&
        (obj->read_domains & I915_GEM_DOMAIN_CPU) == 0 &&
        !(obj->cache_coherent & I915_BO_CACHE_COHERENT_FOR_READ))
        drm_clflush_sg(pages);

    __start_cpu_write(obj);
}

/**
 * i915_gem_track_fb - update frontbuffer tracking
 * @old: current GEM buffer for the frontbuffer slots
 * @new: new GEM buffer for the frontbuffer slots
 * @frontbuffer_bits: bitmask of frontbuffer slots
 *
 * This updates the frontbuffer tracking bits @frontbuffer_bits by clearing them
 * from @old and setting them in @new. Both @old and @new can be NULL.
 */
void i915_gem_track_fb(struct drm_i915_gem_object *old,
		       struct drm_i915_gem_object *new,
		       unsigned frontbuffer_bits)
{
	/* Control of individual bits within the mask are guarded by
	 * the owning plane->mutex, i.e. we can never see concurrent
	 * manipulation of individual bits. But since the bitfield as a whole
	 * is updated using RMW, we need to use atomics in order to update
	 * the bits.
	 */
	BUILD_BUG_ON(INTEL_FRONTBUFFER_BITS_PER_PIPE * I915_MAX_PIPES >
		     sizeof(atomic_t) * BITS_PER_BYTE);

	if (old) {
		WARN_ON(!(atomic_read(&old->frontbuffer_bits) & frontbuffer_bits));
		atomic_andnot(frontbuffer_bits, &old->frontbuffer_bits);
	}

	if (new) {
		WARN_ON(atomic_read(&new->frontbuffer_bits) & frontbuffer_bits);
		atomic_or(frontbuffer_bits, &new->frontbuffer_bits);
	}
}
static int i915_gem_object_get_pages_phys(struct drm_i915_gem_object *obj)
{
    struct address_space *mapping = obj->base.filp->f_mapping;
    drm_dma_handle_t *phys;
    struct sg_table *st;
    struct scatterlist *sg;
    char *vaddr;
    int i;
    int err;

    if (WARN_ON(i915_gem_object_needs_bit17_swizzle(obj)))
        return -EINVAL;

    /* Always aligning to the object size, allows a single allocation
     * to handle all possible callers, and given typical object sizes,
     * the alignment of the buddy allocation will naturally match.
     */
    phys = drm_pci_alloc(obj->base.dev,
                 roundup_pow_of_two(obj->base.size),
                 roundup_pow_of_two(obj->base.size));
    if (!phys)
        return -ENOMEM;

    vaddr = phys->vaddr;
    for (i = 0; i < obj->base.size / PAGE_SIZE; i++) {
        struct page *page;
        char *src;

        page = shmem_read_mapping_page(mapping, i);
        if (IS_ERR(page)) {
            err = PTR_ERR(page);
            goto err_phys;
        }

        src = kmap_atomic(page);
        memcpy(vaddr, src, PAGE_SIZE);
        drm_clflush_virt_range(vaddr, PAGE_SIZE);
        kunmap_atomic(src);

        put_page(page);
        vaddr += PAGE_SIZE;
    }

    i915_gem_chipset_flush(to_i915(obj->base.dev));

    st = kmalloc(sizeof(*st), GFP_KERNEL);
    if (!st) {
        err = -ENOMEM;
        goto err_phys;
    }

    if (sg_alloc_table(st, 1, GFP_KERNEL)) {
        kfree(st);
        err = -ENOMEM;
        goto err_phys;
    }

    sg = st->sgl;
    sg->offset = 0;
    sg->length = obj->base.size;

    sg_dma_address(sg) = phys->busaddr;
    sg_dma_len(sg) = obj->base.size;

    obj->phys_handle = phys;

    __i915_gem_object_set_pages(obj, st, sg->length);

    return 0;

err_phys:
    drm_pci_free(obj->base.dev, phys);

    return err;
}
static void
i915_gem_object_put_pages_phys(struct drm_i915_gem_object *obj,
                   struct sg_table *pages)
{
    __i915_gem_object_release_shmem(obj, pages, false);

    if (obj->mm.dirty) {
        struct address_space *mapping = obj->base.filp->f_mapping;
        char *vaddr = obj->phys_handle->vaddr;
        int i;

        for (i = 0; i < obj->base.size / PAGE_SIZE; i++) {
            struct page *page;
            char *dst;

            page = shmem_read_mapping_page(mapping, i);
            if (IS_ERR(page))
                continue;

            dst = kmap_atomic(page);
            drm_clflush_virt_range(vaddr, PAGE_SIZE);
            memcpy(dst, vaddr, PAGE_SIZE);
            kunmap_atomic(dst);

            set_page_dirty(page);
            if (obj->mm.madv == I915_MADV_WILLNEED)
                mark_page_accessed(page);
            put_page(page);
            vaddr += PAGE_SIZE;
        }
        obj->mm.dirty = false;
    }

    sg_free_table(pages);
    kfree(pages);

    drm_pci_free(obj->base.dev, obj->phys_handle);
}

static const struct drm_i915_gem_object_ops i915_gem_phys_ops = {
    .get_pages = i915_gem_object_get_pages_phys,
    .put_pages = i915_gem_object_put_pages_phys,
    .release = NULL,
};

static const struct drm_i915_gem_object_ops i915_gem_object_ops;



static int i915_gem_object_create_shmem(struct drm_device *dev,
                    struct drm_gem_object *obj,
                    size_t size)
{
    struct drm_i915_private *i915 = to_i915(dev);
    unsigned long flags = VM_NORESERVE;
    struct file *filp;

    drm_gem_private_object_init(dev, obj, size);

    if (i915->mm.gemfs)
        filp = shmem_file_setup_with_mnt(i915->mm.gemfs, "i915", size,
                         flags);
    else
        filp = shmem_file_setup("i915", size, flags);

    if (IS_ERR(filp))
        return PTR_ERR(filp);

    obj->filp = filp;

    return 0;
}

struct drm_i915_gem_object *
i915_gem_object_create(struct drm_i915_private *dev_priv, u64 size)
{
    struct drm_i915_gem_object *obj;
    struct address_space *mapping;
    unsigned int cache_level;
    gfp_t mask;
    int ret;

    /* There is a prevalence of the assumption that we fit the object's
     * page count inside a 32bit _signed_ variable. Let's document this and
     * catch if we ever need to fix it. In the meantime, if you do spot
     * such a local variable, please consider fixing!
     */
    if (size >> PAGE_SHIFT > INT_MAX)
        return ERR_PTR(-E2BIG);

    if (overflows_type(size, obj->base.size))
        return ERR_PTR(-E2BIG);

    obj = i915_gem_object_alloc(dev_priv);
    if (obj == NULL)
        return ERR_PTR(-ENOMEM);

    ret = i915_gem_object_create_shmem(&dev_priv->drm, &obj->base, size);
    if (ret)
        goto fail;

    mask = GFP_HIGHUSER | __GFP_RECLAIMABLE;
    if (IS_I965GM(dev_priv) || IS_I965G(dev_priv)) {
        /* 965gm cannot relocate objects above 4GiB. */
        mask &= ~__GFP_HIGHMEM;
        mask |= __GFP_DMA32;
    }

    mapping = obj->base.filp->f_mapping;
    mapping_set_gfp_mask(mapping, mask);
    GEM_BUG_ON(!(mapping_gfp_mask(mapping) & __GFP_RECLAIM));

    i915_gem_object_init(obj, &i915_gem_object_ops);

    obj->write_domain = I915_GEM_DOMAIN_CPU;
    obj->read_domains = I915_GEM_DOMAIN_CPU;

    if (HAS_LLC(dev_priv))
        /* On some devices, we can have the GPU use the LLC (the CPU
         * cache) for about a 10% performance improvement
         * compared to uncached.  Graphics requests other than
         * display scanout are coherent with the CPU in
         * accessing this cache.  This means in this mode we
         * don't need to clflush on the CPU side, and on the
         * GPU side we only need to flush internal caches to
         * get data visible to the CPU.
         *
         * However, we maintain the display planes as UC, and so
         * need to rebind when first used as such.
         */
        cache_level = I915_CACHE_LLC;
    else
        cache_level = I915_CACHE_NONE;

    i915_gem_object_set_cache_coherency(obj, cache_level);

    trace_i915_gem_object_create(obj);

    return obj;

fail:
    return ERR_PTR(ret);
}
