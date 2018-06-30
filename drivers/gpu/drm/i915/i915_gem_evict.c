#include <drm/drmP.h>
#include <drm/i915_drm.h>

#include "i915_drv.h"
#include "intel_drv.h"
#include "i915_trace.h"


int i915_gem_evict_for_node(struct i915_address_space *vm,
			    struct drm_mm_node *target,
			    unsigned int flags)
{
	LIST_HEAD(eviction_list);
	struct drm_mm_node *node;
	u64 start = target->start;
	u64 end = start + target->size;
	struct i915_vma *vma, *next;
	bool check_color;
	int ret = 0;

	lockdep_assert_held(&vm->i915->drm.struct_mutex);
	GEM_BUG_ON(!IS_ALIGNED(start, I915_GTT_PAGE_SIZE));
	GEM_BUG_ON(!IS_ALIGNED(end, I915_GTT_PAGE_SIZE));

	trace_i915_gem_evict_node(vm, target, flags);

	/* Retire before we search the active list. Although we have
	 * reasonable accuracy in our retirement lists, we may have
	 * a stray pin (preventing eviction) that can only be resolved by
	 * retiring.
	 */

	check_color = vm->mm.color_adjust;
	if (check_color) {
		/* Expand search to cover neighbouring guard pages (or lack!) */
		if (start)
			start -= I915_GTT_PAGE_SIZE;

		/* Always look at the page afterwards to avoid the end-of-GTT */
		end += I915_GTT_PAGE_SIZE;
	}
	GEM_BUG_ON(start >= end);

	drm_mm_for_each_node_in_range(node, &vm->mm, start, end) {
		/* If we find any non-objects (!vma), we cannot evict them */
		if (node->color == I915_COLOR_UNEVICTABLE) {
			ret = -ENOSPC;
			break;
		}

		GEM_BUG_ON(!node->allocated);
		vma = container_of(node, typeof(*vma), node);

		/* If we are using coloring to insert guard pages between
		 * different cache domains within the address space, we have
		 * to check whether the objects on either side of our range
		 * abutt and conflict. If they are in conflict, then we evict
		 * those as well to make room for our guard pages.
		 */
		if (check_color) {
			if (node->start + node->size == target->start) {
				if (node->color == target->color)
					continue;
			}
			if (node->start == target->start + target->size) {
				if (node->color == target->color)
					continue;
			}
		}

		if (flags & PIN_NONBLOCK &&
		    (i915_vma_is_pinned(vma) || i915_vma_is_active(vma))) {
			ret = -ENOSPC;
			break;
		}

		if (flags & PIN_NONFAULT && i915_vma_has_userfault(vma)) {
			ret = -ENOSPC;
			break;
		}

		/* Overlap of objects in the same batch? */
		if (i915_vma_is_pinned(vma)) {
			ret = -ENOSPC;
			if (vma->exec_flags &&
			    *vma->exec_flags & EXEC_OBJECT_PINNED)
				ret = -EINVAL;
			break;
		}

		/* Never show fear in the face of dragons!
		 *
		 * We cannot directly remove this node from within this
		 * iterator and as with i915_gem_evict_something() we employ
		 * the vma pin_count in order to prevent the action of
		 * unbinding one vma from freeing (by dropping its active
		 * reference) another in our eviction list.
		 */
		__i915_vma_pin(vma);
		list_add(&vma->evict_link, &eviction_list);
	}

	list_for_each_entry_safe(vma, next, &eviction_list, evict_link) {
		__i915_vma_unpin(vma);
	//	if (ret == 0)
		//	ret = i915_vma_unbind(vma);
	}

	return ret;
}

