/*
 * Copyright © 2010 Daniel Vetter
 * Copyright © 2011-2014 Intel Corporation
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

#include <linux/slab.h> /* fault-inject.h is not standalone! */

#include <linux/fault-inject.h>
#include <linux/log2.h>
#include <linux/random.h>
#include <linux/seq_file.h>
#include <linux/stop_machine.h>

#include <asm/set_memory.h>

#include <drm/drmP.h>
#include <drm/i915_drm.h>

#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_frontbuffer.h"

#define I915_GFP_DMA (GFP_KERNEL | __GFP_HIGHMEM)


/**
 * DOC: Global GTT views
 *
 * Background and previous state
 *
 * Historically objects could exists (be bound) in global GTT space only as
 * singular instances with a view representing all of the object's backing pages
 * in a linear fashion. This view will be called a normal view.
 *
 * To support multiple views of the same object, where the number of mapped
 * pages is not equal to the backing store, or where the layout of the pages
 * is not linear, concept of a GGTT view was added.
 *
 * One example of an alternative view is a stereo display driven by a single
 * image. In this case we would have a framebuffer looking like this
 * (2x2 pages):
 *
 *    12
 *    34
 *
 * Above would represent a normal GGTT view as normally mapped for GPU or CPU
 * rendering. In contrast, fed to the display engine would be an alternative
 * view which could look something like this:
 *
 *   1212
 *   3434
 *
 * In this example both the size and layout of pages in the alternative view is
 * different from the normal view.
 *
 * Implementation and usage
 *
 * GGTT views are implemented using VMAs and are distinguished via enum
 * i915_ggtt_view_type and struct i915_ggtt_view.
 *
 * A new flavour of core GEM functions which work with GGTT bound objects were
 * added with the _ggtt_ infix, and sometimes with _view postfix to avoid
 * renaming  in large amounts of code. They take the struct i915_ggtt_view
 * parameter encapsulating all metadata required to implement a view.
 *
 * As a helper for callers which are only interested in the normal view,
 * globally const i915_ggtt_view_normal singleton instance exists. All old core
 * GEM API functions, the ones not taking the view parameter, are operating on,
 * or with the normal GGTT view.
 *
 * Code wanting to add or use a new GGTT view needs to:
 *
 * 1. Add a new enum with a suitable name.
 * 2. Extend the metadata in the i915_ggtt_view structure if required.
 * 3. Add support to i915_get_vma_pages().
 *
 * New views are required to build a scatter-gather table from within the
 * i915_get_vma_pages function. This table is stored in the vma.ggtt_view and
 * exists for the lifetime of an VMA.
 *
 * Core API is designed to have copy semantics which means that passed in
 * struct i915_ggtt_view does not need to be persistent (left around after
 * calling the core API functions).
 *
 */
int i915_gem_evict_for_node(struct i915_address_space *vm,
                struct drm_mm_node *target,
                unsigned int flags)
;
int i915_gem_gtt_reserve(struct i915_address_space *vm,
             struct drm_mm_node *node,
             u64 size, u64 offset, unsigned long color,
             unsigned int flags)
{
    int err;

    GEM_BUG_ON(!size);
    GEM_BUG_ON(!IS_ALIGNED(size, I915_GTT_PAGE_SIZE));
    GEM_BUG_ON(!IS_ALIGNED(offset, I915_GTT_MIN_ALIGNMENT));
    GEM_BUG_ON(range_overflows(offset, size, vm->total));
    GEM_BUG_ON(vm == &vm->i915->mm.aliasing_ppgtt->base);
    GEM_BUG_ON(drm_mm_node_allocated(node));

    node->size = size;
    node->start = offset;
    node->color = color;

    err = drm_mm_reserve_node(&vm->mm, node);
    if (err != -ENOSPC)
        return err;

    if (flags & PIN_NOEVICT)
        return -ENOSPC;

    err = i915_gem_evict_for_node(vm, node, flags);
    if (err == 0)
        err = drm_mm_reserve_node(&vm->mm, node);

    return err;
}

static void gen7_ppgtt_enable(struct drm_i915_private *dev_priv)
{
    struct intel_engine_cs *engine;
    u32 ecochk, ecobits;
    enum intel_engine_id id;

    ecobits = I915_READ(GAC_ECO_BITS);
    I915_WRITE(GAC_ECO_BITS, ecobits | ECOBITS_PPGTT_CACHE64B);

    ecochk = I915_READ(GAM_ECOCHK);
    if (IS_HASWELL(dev_priv)) {
        ecochk |= ECOCHK_PPGTT_WB_HSW;
    } else {
        ecochk |= ECOCHK_PPGTT_LLC_IVB;
        ecochk &= ~ECOCHK_PPGTT_GFDT_IVB;
    }
    I915_WRITE(GAM_ECOCHK, ecochk);

    for_each_engine(engine, dev_priv, id) {
        /* GFX_MODE is per-ring on gen7+ */
        I915_WRITE(RING_MODE_GEN7(engine),
               _MASKED_BIT_ENABLE(GFX_PPGTT_ENABLE));
    }
}

int i915_ppgtt_init_hw(struct drm_i915_private *dev_priv) {
    gen7_ppgtt_enable(dev_priv);
}

static int
i915_get_ggtt_vma_pages(struct i915_vma *vma);

static void gen6_ggtt_invalidate(struct drm_i915_private *dev_priv)
{
	/* Note that as an uncached mmio write, this should flush the
	 * WCB of the writes into the GGTT before it triggers the invalidate.
	 */
	I915_WRITE(GFX_FLSH_CNTL_GEN6, GFX_FLSH_CNTL_EN);
}


int intel_sanitize_enable_ppgtt(struct drm_i915_private *dev_priv,
			       	int enable_ppgtt)
{
	bool has_full_ppgtt;
	bool has_full_48bit_ppgtt;

	if (!dev_priv->info.has_aliasing_ppgtt)
		return 0;

	has_full_ppgtt = dev_priv->info.has_full_ppgtt;
	has_full_48bit_ppgtt = dev_priv->info.has_full_48bit_ppgtt;

	/*
	 * We don't allow disabling PPGTT for gen9+ as it's a requirement for
	 * execlists, the sole mechanism available to submit work.
	 */
	if (enable_ppgtt == 0 && INTEL_GEN(dev_priv) < 9)
		return 0;

	if (enable_ppgtt == 1)
		return 1;

	if (enable_ppgtt == 2 && has_full_ppgtt)
		return 2;

	if (enable_ppgtt == 3 && has_full_48bit_ppgtt)
		return 3;

	/* Disable ppgtt on SNB if VT-d is on. */
	if (IS_GEN6(dev_priv) && intel_vtd_active()) {
		DRM_INFO("Disabling PPGTT because VT-d is on\n");
		return 0;
	}

	/* Early VLV doesn't have this */
	if (IS_VALLEYVIEW(dev_priv) && dev_priv->drm.pdev->revision < 0xb) {
		DRM_DEBUG_DRIVER("disabling PPGTT on pre-B3 step VLV\n");
		return 0;
	}

	if (HAS_LOGICAL_RING_CONTEXTS(dev_priv)) {
		if (has_full_48bit_ppgtt)
			return 3;

		if (has_full_ppgtt)
			return 2;
	}

	return 1;
}
static gen6_pte_t hsw_pte_encode(dma_addr_t addr,
				 enum i915_cache_level level,
				 u32 unused)
{
	gen6_pte_t pte = GEN6_PTE_VALID;
//    printk("hsw_pte_encode %x", addr);
	pte |= HSW_PTE_ADDR_ENCODE(addr);

	if (level != I915_CACHE_NONE)
		pte |= HSW_WB_LLC_AGE3;

	return pte;
}

static int
setup_scratch_page(struct i915_address_space *vm, gfp_t gfp)
{
	unsigned long size;

	/*
	 * In order to utilize 64K pages for an object with a size < 2M, we will
	 * need to support a 64K scratch page, given that every 16th entry for a
	 * page-table operating in 64K mode must point to a properly aligned 64K
	 * region, including any PTEs which happen to point to scratch.
	 *
	 * This is only relevant for the 48b PPGTT where we support
	 * huge-gtt-pages, see also i915_vma_insert().
	 *
	 * TODO: we should really consider write-protecting the scratch-page and
	 * sharing between ppgtt
	 */
	size = I915_GTT_PAGE_SIZE_4K;
	if (i915_vm_is_48bit(vm) &&
	    HAS_PAGE_SIZES(vm->i915, I915_GTT_PAGE_SIZE_64K)) {
		size = I915_GTT_PAGE_SIZE_64K;
		gfp |= __GFP_NOWARN;
	}
	gfp |= __GFP_ZERO | __GFP_RETRY_MAYFAIL;

	do {
		int order = get_order(size);
		struct page *page;
		dma_addr_t addr;

		page = alloc_pages(gfp, order);
		if (unlikely(!page))
			goto skip;

		addr = dma_map_page(vm->dma, page, 0, size,
				    PCI_DMA_BIDIRECTIONAL);
		if (unlikely(dma_mapping_error(vm->dma, addr)))
			goto free_page;

		if (unlikely(!IS_ALIGNED(addr, size)))
			goto unmap_page;

		vm->scratch_page.page = page;
		vm->scratch_page.daddr = addr;
		vm->scratch_page.order = order;
		return 0;

unmap_page:
		dma_unmap_page(vm->dma, addr, size, PCI_DMA_BIDIRECTIONAL);
free_page:
		__free_pages(page, order);
skip:
		if (size == I915_GTT_PAGE_SIZE_4K)
			return -ENOMEM;

		size = I915_GTT_PAGE_SIZE_4K;
		gfp &= ~__GFP_NOWARN;
	} while (1);
}


static void i915_address_space_init(struct i915_address_space *vm,
				    struct drm_i915_private *dev_priv,
				    const char *name)
{
    i915_gem_timeline_init(dev_priv, &vm->timeline, name);

    drm_mm_init(&vm->mm, 0, vm->total);
    vm->mm.head_node.color = I915_COLOR_UNEVICTABLE;

    INIT_LIST_HEAD(&vm->active_list);
    INIT_LIST_HEAD(&vm->inactive_list);
    INIT_LIST_HEAD(&vm->unbound_list);

    list_add_tail(&vm->global_link, &dev_priv->vm_list);
    pagevec_init(&vm->free_pages);
}


/*
 * Binds an object into the global gtt with the specified cache level. The object
 * will be accessible to the GPU via commands whose operands reference offsets
 * within the global GTT as well as accessible by the GPU through the GMADR
 * mapped BAR (dev_priv->mm.gtt->gtt).
 */
static void gen6_ggtt_insert_entries(struct i915_address_space *vm,
				     struct i915_vma *vma,
				     enum i915_cache_level level,
				     u32 flags)
{
	struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
	gen6_pte_t __iomem *entries = (gen6_pte_t __iomem *)ggtt->gsm;
	unsigned int i = vma->node.start >> PAGE_SHIFT;
	struct sgt_iter iter;
	dma_addr_t addr;
 //   printk("gen6_ggtt_insert_entries abort vma-pagex %x", vma->pages);
	for_each_sgt_dma(addr, iter, vma->pages) {
 //   printk("gen6_ggtt_insert_entries abort %x", addr);
		iowrite32(vm->pte_encode(addr, level, flags), &entries[i++]);
        }
	wmb();

	/* This next bit makes the above posting read even more important. We
	 * want to flush the TLBs only after we're certain all the PTE updates
	 * have finished.
	 */
	ggtt->invalidate(vm->i915);
}


static int ggtt_bind_vma(struct i915_vma *vma,
			 enum i915_cache_level cache_level,
			 u32 flags)
{
    struct drm_i915_private *i915;
	struct drm_i915_gem_object *obj;
	u32 pte_flags;
    printk("ggtt_bind_vma  pages %x", vma->pages);
    i915 = vma->vm->i915;
	obj = vma->obj;

	/* Currently applicable only to VLV */
	pte_flags = 0;
	if (obj->gt_ro)
		pte_flags |= PTE_READ_ONLY;

    printk("ggtt_bind_vma pre insert");
	vma->vm->insert_entries(vma->vm, vma, cache_level, pte_flags);
    printk("ggtt_bind_vma post insert");

	vma->page_sizes.gtt = I915_GTT_PAGE_SIZE;

	/*
	 * Without aliasing PPGTT there's no difference between
	 * GLOBAL/LOCAL_BIND, it's all the same ptes. Hence unconditionally
	 * upgrade to both bound if we bind either to avoid double-binding.
	 */
	vma->flags |= I915_VMA_GLOBAL_BIND | I915_VMA_LOCAL_BIND;

	return 0;
}


static int ggtt_set_pages(struct i915_vma *vma)
{
	int ret;

	GEM_BUG_ON(vma->pages);
 printk("ggtt_bind_set  pages %x                 xxxxxxxxxxxxxxxxxxxxxxxxx", vma->pages);

      ret = i915_get_ggtt_vma_pages(vma);
    if (ret)
        return ret;

    vma->page_sizes = vma->obj->mm.page_sizes;


   printk("ggtt_bind_set  pages %x                 xxxxxxxxxxxxxxxxxxxxxxxxx", vma->pages);

	return 0;
}
static unsigned int gen6_get_total_gtt_size(u16 snb_gmch_ctl)
{
	snb_gmch_ctl >>= SNB_GMCH_GGMS_SHIFT;
	snb_gmch_ctl &= SNB_GMCH_GGMS_MASK;
	return snb_gmch_ctl << 20;
}



static int ggtt_probe_common(struct i915_ggtt *ggtt, u64 size)
{
	struct drm_i915_private *dev_priv = ggtt->base.i915;
	struct pci_dev *pdev = dev_priv->drm.pdev;
	phys_addr_t phys_addr;
	int ret;

	/* For Modern GENs the PTEs and register space are split in the BAR */
	phys_addr = pci_resource_start(pdev, 0) + pci_resource_len(pdev, 0) / 2;

	/*
	 * On BXT+/CNL+ writes larger than 64 bit to the GTT pagetable range
	 * will be dropped. For WC mappings in general we have 64 byte burst
	 * writes when the WC buffer is flushed, so we can't use it, but have to
	 * resort to an uncached mapping. The WC issue is easily caught by the
	 * readback check when writing GTT PTE entries.
	 */
	if (IS_GEN9_LP(dev_priv) || INTEL_GEN(dev_priv) >= 10)
		ggtt->gsm = ioremap_nocache(phys_addr, size);
	else
		ggtt->gsm = ioremap_wc(phys_addr, size);
	if (!ggtt->gsm) {
		DRM_ERROR("Failed to map the ggtt page table\n");
		return -ENOMEM;
	}

	ret = setup_scratch_page(&ggtt->base, GFP_DMA32);
	if (ret) {
		DRM_ERROR("Scratch setup failed\n");
		/* iounmap will also get called at remove, but meh */
		iounmap(ggtt->gsm);
		return ret;
	}

	return 0;
}
static void gen6_ggtt_insert_page(struct i915_address_space *vm, 
                  dma_addr_t addr,
                  u64 offset,
                  enum i915_cache_level level,
                  u32 flags)
{
    struct i915_ggtt *ggtt = i915_vm_to_ggtt(vm);
    gen6_pte_t __iomem *pte =
        (gen6_pte_t __iomem *)ggtt->gsm + (offset >> PAGE_SHIFT);

printk("Insert PTE");
    iowrite32(vm->pte_encode(addr, level, flags), pte);

    ggtt->invalidate(vm->i915);
}


static int gen6_gmch_probe(struct i915_ggtt *ggtt)
{
	struct drm_i915_private *dev_priv = ggtt->base.i915;
	struct pci_dev *pdev = dev_priv->drm.pdev;
	unsigned int size;
	u16 snb_gmch_ctl;
	int err;

	ggtt->gmadr =
		(struct resource) DEFINE_RES_MEM(pci_resource_start(pdev, 2),
						 pci_resource_len(pdev, 2));
	ggtt->mappable_end = resource_size(&ggtt->gmadr);

	/* 64/512MB is the current min/max we actually know of, but this is just
	 * a coarse sanity check.
	 */
	if (ggtt->mappable_end < (64<<20) || ggtt->mappable_end > (512<<20)) {
		DRM_ERROR("Unknown GMADR size (%pa)\n", &ggtt->mappable_end);
		return -ENXIO;
	}

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(40));
	if (!err)
		err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(40));
	if (err)
		DRM_ERROR("Can't set DMA mask/consistent mask (%d)\n", err);
	pci_read_config_word(pdev, SNB_GMCH_CTRL, &snb_gmch_ctl);

	size = gen6_get_total_gtt_size(snb_gmch_ctl);
	ggtt->base.total = (size / sizeof(gen6_pte_t)) << PAGE_SHIFT;

    ggtt->base.clear_range = NULL;
    ggtt->base.insert_page = gen6_ggtt_insert_page;
	ggtt->base.insert_entries = gen6_ggtt_insert_entries;
	ggtt->base.bind_vma = ggtt_bind_vma;
    ggtt->base.unbind_vma = NULL;
	ggtt->base.set_pages = ggtt_set_pages;
    ggtt->base.clear_pages = NULL;
    ggtt->base.cleanup = NULL;

	ggtt->invalidate = gen6_ggtt_invalidate;

    ggtt->base.pte_encode = hsw_pte_encode;

	return ggtt_probe_common(ggtt, size);
}

/**
 * i915_ggtt_probe_hw - Probe GGTT hardware location
 * @dev_priv: i915 device
 */
int i915_ggtt_probe_hw(struct drm_i915_private *dev_priv)
{
	struct i915_ggtt *ggtt = &dev_priv->ggtt;
	int ret;

	ggtt->base.i915 = dev_priv;
	ggtt->base.dma = &dev_priv->drm.pdev->dev;

    ret = gen6_gmch_probe(ggtt);
	if (ret)
		return ret;

	/* Trim the GGTT to fit the GuC mappable upper range (when enabled).
	 * This is easier than doing range restriction on the fly, as we
	 * currently don't have any bits spare to pass in this upper
	 * restriction!
	 */


	if ((ggtt->base.total - 1) >> 32) {
		DRM_ERROR("We never expected a Global GTT with more than 32bits"
			  " of address space! Found %lldM!\n",
			  ggtt->base.total >> 20);
		ggtt->base.total = 1ULL << 32;
		ggtt->mappable_end = min_t(u64, ggtt->mappable_end, ggtt->base.total);
	}

	if (ggtt->mappable_end > ggtt->base.total) {
		DRM_ERROR("mappable aperture extends past end of GGTT,"
			  " aperture=%pa, total=%llx\n",
			  &ggtt->mappable_end, ggtt->base.total);
		ggtt->mappable_end = ggtt->base.total;
	}

	/* GMADR is the PCI mmio aperture into the global GTT. */
	DRM_DEBUG_DRIVER("GGTT size = %lluM\n", ggtt->base.total >> 20);
	DRM_DEBUG_DRIVER("GMADR size = %lluM\n", (u64)ggtt->mappable_end >> 20);
	DRM_DEBUG_DRIVER("DSM size = %lluM\n",
			 (u64)resource_size(&intel_graphics_stolen_res) >> 20);
	if (intel_vtd_active())
		DRM_INFO("VT-d active for gfx access\n");

	return 0;
}

/**
 * i915_ggtt_init_hw - Initialize GGTT hardware
 * @dev_priv: i915 device
 */
int i915_ggtt_init_hw(struct drm_i915_private *dev_priv)
{
	struct i915_ggtt *ggtt = &dev_priv->ggtt;
	int ret;

	INIT_LIST_HEAD(&dev_priv->vm_list);
printk("i915_ggtt_init_hw i915_ggtt_init_hw i915_ggtt_init_hw i915_ggtt_init_hw i915_ggtt_init_hw i915_ggtt_init_hw i915_ggtt_init_hw i915_ggtt_init_hw");
	/* Note that we use page colouring to enforce a guard page at the
	 * end of the address space. This is required as the CS may prefetch
	 * beyond the end of the batch buffer, across the page boundary,
	 * and beyond the end of the GTT if we do not provide a guard.
	 */
	mutex_lock(&dev_priv->drm.struct_mutex);
	i915_address_space_init(&ggtt->base, dev_priv, "[global]");
	mutex_unlock(&dev_priv->drm.struct_mutex);

	if (!io_mapping_init_wc(&dev_priv->ggtt.iomap,
				dev_priv->ggtt.gmadr.start,
				dev_priv->ggtt.mappable_end)) {
		ret = -EIO;
		goto out_gtt_cleanup;
	}

	ggtt->mtrr = arch_phys_wc_add(ggtt->gmadr.start, ggtt->mappable_end);

	/*
	 * Initialise stolen early so that we may reserve preallocated
	 * objects for the BIOS to KMS transition.
	 */
	ret = i915_gem_init_stolen(dev_priv);
	if (ret)
		goto out_gtt_cleanup;

	return 0;

out_gtt_cleanup:
	ggtt->base.cleanup(&ggtt->base);
	return ret;
}

int i915_ggtt_enable_hw(struct drm_i915_private *dev_priv)
{
	if (INTEL_GEN(dev_priv) < 6 && !intel_enable_gtt())
		return -EIO;

	return 0;
}

static int
i915_get_ggtt_vma_pages(struct i915_vma *vma)
{
	int ret;

	/* The vma->pages are only valid within the lifespan of the borrowed
	 * obj->mm.pages. When the obj->mm.pages sg_table is regenerated, so
	 * must be the vma->pages. A simple rule is that vma->pages must only
	 * be accessed when the obj->mm.pages are pinned.
	 */
	GEM_BUG_ON(!i915_gem_object_has_pinned_pages(vma->obj));

    vma->pages = vma->obj->mm.pages;
	return 0;
}

int i915_gem_gtt_insert(struct i915_address_space *vm,
			struct drm_mm_node *node,
			u64 size, u64 alignment, unsigned long color,
			u64 start, u64 end, unsigned int flags)
{
	enum drm_mm_insert_mode mode;
	u64 offset;
	int err;

	lockdep_assert_held(&vm->i915->drm.struct_mutex);
	GEM_BUG_ON(!size);
	GEM_BUG_ON(!IS_ALIGNED(size, I915_GTT_PAGE_SIZE));
	GEM_BUG_ON(alignment && !is_power_of_2(alignment));
	GEM_BUG_ON(alignment && !IS_ALIGNED(alignment, I915_GTT_MIN_ALIGNMENT));
	GEM_BUG_ON(start >= end);
	GEM_BUG_ON(start > 0  && !IS_ALIGNED(start, I915_GTT_PAGE_SIZE));
	GEM_BUG_ON(end < U64_MAX && !IS_ALIGNED(end, I915_GTT_PAGE_SIZE));
	GEM_BUG_ON(vm == &vm->i915->mm.aliasing_ppgtt->base);
	GEM_BUG_ON(drm_mm_node_allocated(node));

	if (unlikely(range_overflows(start, size, end)))
		return -ENOSPC;

	if (unlikely(round_up(start, alignment) > round_down(end - size, alignment)))
		return -ENOSPC;

	mode = DRM_MM_INSERT_BEST;
	if (flags & PIN_HIGH)
		mode = DRM_MM_INSERT_HIGH;
	if (flags & PIN_MAPPABLE)
		mode = DRM_MM_INSERT_LOW;

	/* We only allocate in PAGE_SIZE/GTT_PAGE_SIZE (4096) chunks,
	 * so we know that we always have a minimum alignment of 4096.
	 * The drm_mm range manager is optimised to return results
	 * with zero alignment, so where possible use the optimal
	 * path.
	 */
	BUILD_BUG_ON(I915_GTT_MIN_ALIGNMENT > I915_GTT_PAGE_SIZE);
	if (alignment <= I915_GTT_MIN_ALIGNMENT)
		alignment = 0;

	err = drm_mm_insert_node_in_range(&vm->mm, node,
					  size, alignment, color,
					  start, end, mode);
	if (err != -ENOSPC)
		return err;

    return -ENOSPC;
}

