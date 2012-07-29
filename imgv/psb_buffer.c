/**************************************************************************
 * Copyright (c) 2011, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 **************************************************************************/
/*
 * Authors: Thomas Hellstrom <thomas-at-tungstengraphics.com>
 */
#include "ttm/ttm_placement.h"
#include "ttm/ttm_execbuf_util.h"
#include "ttm/ttm_page_alloc.h"
#include "psb_ttm_fence_api.h"
#include <drm/drmP.h>
#include "psb_drv.h"
#include "psb_schedule.h"

#define DRM_MEM_TTM       26

struct drm_psb_ttm_tt {
	struct ttm_dma_tt ttm;
	/* struct page **pages; */
	unsigned int desired_tile_stride;
	unsigned int hw_tile_stride;
	int mem_type;
	unsigned long offset;
	/* unsigned long num_pages; */
};

/*
 * MSVDX/TOPAZ GPU virtual space looks like this
 * (We currently use only one MMU context).
 * PSB_MEM_MMU_START: from 0x00000000~0xe000000, for generic buffers
 * TTM_PL_CI: from 0xe0000000+half GTT space, for camear/video buffer sharing
 * TTM_PL_RAR: from TTM_PL_CI+CI size, for RAR/video buffer sharing
 * TTM_PL_TT: from TTM_PL_RAR+RAR size, for buffers need to mapping into GTT
 */
static int psb_init_mem_type(struct ttm_bo_device *bdev, uint32_t type,
			     struct ttm_mem_type_manager *man)
{

	struct drm_psb_private *dev_priv =
	    container_of(bdev, struct drm_psb_private, bdev);
	struct psb_gtt *pg = dev_priv->pg;

	switch (type) {
	case TTM_PL_SYSTEM:
		man->flags = TTM_MEMTYPE_FLAG_MAPPABLE;
		man->available_caching = TTM_PL_FLAG_CACHED |
		    TTM_PL_FLAG_UNCACHED | TTM_PL_FLAG_WC;
		man->default_caching = TTM_PL_FLAG_CACHED;
		break;
	case DRM_PSB_MEM_MMU:
		man->func = &ttm_bo_manager_func;
		man->flags = TTM_MEMTYPE_FLAG_MAPPABLE |
		    TTM_MEMTYPE_FLAG_CMA;
		man->gpu_offset = PSB_MEM_MMU_START;
		man->available_caching = TTM_PL_FLAG_CACHED |
		    TTM_PL_FLAG_UNCACHED | TTM_PL_FLAG_WC;
		man->default_caching = TTM_PL_FLAG_WC;
		break;
	case TTM_PL_CI:
		man->func = &ttm_bo_manager_func;
		man->flags = TTM_MEMTYPE_FLAG_MAPPABLE |
			TTM_MEMTYPE_FLAG_FIXED;
		man->gpu_offset = pg->mmu_gatt_start + (pg->ci_start);
		man->available_caching = TTM_PL_FLAG_UNCACHED;
		man->default_caching = TTM_PL_FLAG_UNCACHED;
		break;
	case TTM_PL_RAR:	/* Unmappable RAR memory */
		man->func = &ttm_bo_manager_func;
		man->flags = TTM_MEMTYPE_FLAG_MAPPABLE |
			TTM_MEMTYPE_FLAG_FIXED;
		man->available_caching = TTM_PL_FLAG_UNCACHED;
		man->default_caching = TTM_PL_FLAG_UNCACHED;
		man->gpu_offset = pg->mmu_gatt_start + (pg->rar_start);
		break;
	case TTM_PL_TT:	/* Mappable GATT memory */
		man->func = &ttm_bo_manager_func;
#ifdef PSB_WORKING_HOST_MMU_ACCESS
		man->flags = TTM_MEMTYPE_FLAG_MAPPABLE;
#else
		man->flags = TTM_MEMTYPE_FLAG_MAPPABLE |
		    TTM_MEMTYPE_FLAG_CMA;
#endif
		man->available_caching = TTM_PL_FLAG_CACHED |
		    TTM_PL_FLAG_UNCACHED | TTM_PL_FLAG_WC;
		man->default_caching = TTM_PL_FLAG_WC;
		man->gpu_offset = pg->mmu_gatt_start + (pg->rar_start + dev_priv->rar_region_size);
		break;
	default:
		DRM_ERROR("Unsupported memory type %u\n", (unsigned) type);
		return -EINVAL;
	}
	return 0;
}


static void psb_evict_mask(struct ttm_buffer_object *bo, struct ttm_placement* placement)
{
	static uint32_t cur_placement;

	cur_placement = bo->mem.placement & ~TTM_PL_MASK_MEM;
	cur_placement |= TTM_PL_FLAG_SYSTEM;

	placement->fpfn = 0;
	placement->lpfn = 0;
	placement->num_placement = 1;
	placement->placement = &cur_placement;
	placement->num_busy_placement = 0;
	placement->busy_placement = NULL;
	
	/* all buffers evicted to system memory */
	/* return cur_placement | TTM_PL_FLAG_SYSTEM; */
}

static int psb_invalidate_caches(struct ttm_bo_device *bdev,
				 uint32_t placement)
{
	return 0;
}

static int psb_move_blit(struct ttm_buffer_object *bo,
			 bool evict, bool no_wait,
			 struct ttm_mem_reg *new_mem)
{
	BUG();
	return 0;
}

/*
 * Flip destination ttm into GATT,
 * then blit and subsequently move out again.
 */

static int psb_move_flip(struct ttm_buffer_object *bo,
			 bool evict, bool interruptible, bool no_wait,
			 struct ttm_mem_reg *new_mem)
{
	/*struct ttm_bo_device *bdev = bo->bdev;*/
	struct ttm_mem_reg tmp_mem;
	int ret;
	struct ttm_placement placement;
	uint32_t flags = TTM_PL_FLAG_TT;

	tmp_mem = *new_mem;
	tmp_mem.mm_node = NULL;

	placement.fpfn = 0;
	placement.lpfn = 0;
	placement.num_placement = 1;
	placement.placement = &flags;
	placement.num_busy_placement = 0; /* FIXME */
	placement.busy_placement = NULL;

	ret = ttm_bo_mem_space(bo, &placement, &tmp_mem, interruptible, false, no_wait);
	if (ret)
		return ret;
	ret = ttm_tt_bind(bo->ttm, &tmp_mem);
	if (ret)
		goto out_cleanup;
	ret = psb_move_blit(bo, true, no_wait, &tmp_mem);
	if (ret)
		goto out_cleanup;

	ret = ttm_bo_move_ttm(bo, evict, false, no_wait, new_mem);
out_cleanup:
	if (tmp_mem.mm_node) {
		/*spin_lock(&bdev->lru_lock);*/ // lru_lock is removed from upstream TTM
		drm_mm_put_block(tmp_mem.mm_node);
		tmp_mem.mm_node = NULL;
		/*spin_unlock(&bdev->lru_lock);*/
	}
	return ret;
}

static int psb_move(struct ttm_buffer_object *bo,
		    bool evict, bool interruptible, bool no_wait_reserve,
		    bool no_wait, struct ttm_mem_reg *new_mem)
{
	struct ttm_mem_reg *old_mem = &bo->mem;

	if ((old_mem->mem_type == TTM_PL_RAR) ||
	    (new_mem->mem_type == TTM_PL_RAR)) {
		if(old_mem->mm_node) {
			spin_lock(&bo->glob->lru_lock);
			drm_mm_put_block(old_mem->mm_node);
			spin_unlock(&bo->glob->lru_lock);
		}
		old_mem->mm_node = NULL;
		*old_mem = *new_mem;
	} else if (old_mem->mem_type == TTM_PL_SYSTEM) {
		return ttm_bo_move_memcpy(bo, evict, false, no_wait, new_mem);
	} else if (new_mem->mem_type == TTM_PL_SYSTEM) {
		int ret = psb_move_flip(bo, evict, interruptible,
					no_wait, new_mem);
		if (unlikely(ret != 0)) {
			if (ret == -ERESTART)
				return ret;
			else
				return ttm_bo_move_memcpy(bo, evict, false, no_wait,
							  new_mem);
		}
	} else {
		if (psb_move_blit(bo, evict, no_wait, new_mem))
			return ttm_bo_move_memcpy(bo, evict, false, no_wait,
						  new_mem);
	}
	return 0;
}

/* static int drm_psb_tbe_populate(struct ttm_tt *backend, */
/* 				unsigned long num_pages, */
/* 				struct page **pages, */
/* 				struct page *dummy_read_page, */
/* 				dma_addr_t *dma_addrs) */
/* { */
/* 	struct drm_psb_ttm_backend *psb_be = */
/* 	    container_of(backend, struct drm_psb_ttm_tt, ttm); */

/* 	psb_be->pages = pages; */
/* 	return 0; */
/* } */

static int drm_psb_tbe_populate(struct ttm_tt *ttm)
{
	struct ttm_bo_device *bdev = ttm->bdev;
	struct drm_psb_private *dev_priv =
	    container_of(bdev, struct drm_psb_private, bdev);
	struct drm_psb_ttm_tt *psb_ttm = (void*) ttm;
	unsigned i;
	int r;

	if (ttm->state != tt_unpopulated)
		return 0;

	r = ttm_pool_populate(ttm);
	if (r) {
		return r;
	}

	for (i = 0; i < ttm->num_pages; i++) {
		psb_ttm->ttm.dma_address[i] = pci_map_page(dev_priv->dev->pdev,
							   ttm->pages[i],
							   0, PAGE_SIZE,
							   PCI_DMA_BIDIRECTIONAL);
		if (pci_dma_mapping_error(dev_priv->dev->pdev,
					  psb_ttm->ttm.dma_address[i])) {
			while (--i) {
				pci_unmap_page(dev_priv->dev->pdev,
					       psb_ttm->ttm.dma_address[i],
					       PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
				psb_ttm->ttm.dma_address[i] = 0;
			}
			ttm_pool_unpopulate(ttm);
			return -EFAULT;
		}
	}
	return 0;
}

static int drm_psb_tbe_unbind(struct ttm_tt *ttm)
{
	struct ttm_bo_device *bdev = ttm->bdev;
	struct drm_psb_private *dev_priv =
	    container_of(bdev, struct drm_psb_private, bdev);
	struct drm_psb_ttm_tt *psb_ttm = (void*) ttm;
	    /* container_of(ttm, struct drm_psb_ttm_tt, ttm); */
	struct psb_mmu_pd *pd = psb_mmu_get_default_pd(dev_priv->mmu);
	/* struct ttm_mem_type_manager *man = &bdev->man[psb_be->mem_type]; */

	if (psb_ttm->mem_type == TTM_PL_TT) {
		uint32_t gatt_p_offset =
		    (psb_ttm->offset - dev_priv->pg->mmu_gatt_start) >> PAGE_SHIFT;

		(void) psb_gtt_remove_pages(dev_priv->pg, gatt_p_offset,
					    ttm->num_pages,
					    psb_ttm->desired_tile_stride,
					    psb_ttm->hw_tile_stride, 0);
	}

	psb_mmu_remove_pages(pd, psb_ttm->offset,
			     ttm->num_pages,
			     psb_ttm->desired_tile_stride,
			     psb_ttm->hw_tile_stride);

	return 0;
}

static int drm_psb_tbe_bind(struct ttm_tt *ttm,
			    struct ttm_mem_reg *bo_mem)
{
	struct ttm_bo_device *bdev = ttm->bdev;
	struct drm_psb_private *dev_priv =
	    container_of(bdev, struct drm_psb_private, bdev);
	struct drm_psb_ttm_tt *psb_ttm = (void*) ttm;
	    /* container_of(ttm, struct drm_psb_ttm_tt, ttm); */
	struct psb_mmu_pd *pd = psb_mmu_get_default_pd(dev_priv->mmu);
	struct ttm_mem_type_manager *man = &bdev->man[bo_mem->mem_type];
	int type;
	int ret = 0;

	psb_ttm->mem_type = bo_mem->mem_type;
	ttm->num_pages = bo_mem->num_pages;
	psb_ttm->desired_tile_stride = 0;
	psb_ttm->hw_tile_stride = 0;
	psb_ttm->offset = (bo_mem->start << PAGE_SHIFT) +
	    man->gpu_offset;

	type =
	    (bo_mem->
	     placement & TTM_PL_FLAG_CACHED) ? PSB_MMU_CACHED_MEMORY : 0;

	if (psb_ttm->mem_type == TTM_PL_TT) {
		uint32_t gatt_p_offset =
		    (psb_ttm->offset - dev_priv->pg->mmu_gatt_start) >> PAGE_SHIFT;

		ret = psb_gtt_insert_pages(dev_priv->pg, ttm->pages,
					   gatt_p_offset,
					   ttm->num_pages,
					   psb_ttm->desired_tile_stride,
					   psb_ttm->hw_tile_stride, type);
	}

	ret = psb_mmu_insert_pages(pd, ttm->pages,
				   psb_ttm->offset, ttm->num_pages,
				   psb_ttm->desired_tile_stride,
				   psb_ttm->hw_tile_stride, type);
	if (ret)
		goto out_err;

	return 0;
out_err:
	drm_psb_tbe_unbind(ttm);
	return ret;

}

static void drm_psb_tbe_unpopulate(struct ttm_tt *ttm)
{
	struct ttm_bo_device *bdev = ttm->bdev;
	struct drm_psb_private *dev_priv =
	    container_of(bdev, struct drm_psb_private, bdev);
	struct drm_psb_ttm_tt *psb_ttm = (void*) ttm;
	unsigned i;

#ifdef CONFIG_SWIOTLB
	if (swiotlb_nr_tbl()) {
		ttm_dma_unpopulate(&psb_ttm->ttm, dev_priv->dev->pdev);
		return;
	}
#endif

	for (i = 0; i < ttm->num_pages; i++) {
		if (psb_ttm->ttm.dma_address[i]) {
			pci_unmap_page(dev_priv->dev->pdev,
				       psb_ttm->ttm.dma_address[i],
				       PAGE_SIZE, PCI_DMA_BIDIRECTIONAL);
		}
	}

	ttm_pool_unpopulate(ttm);
}


static void drm_psb_tbe_destroy(struct ttm_tt *ttm)
{
	struct drm_psb_ttm_tt *psb_ttm = (void*) ttm;
	    /* container_of(ttm, struct drm_psb_ttm_tt, ttm); */

	if (ttm) {
		ttm_dma_tt_fini(&psb_ttm->ttm);
		kfree(psb_ttm);
	}
}

static struct ttm_backend_func psb_ttm_backend = {
	.bind = drm_psb_tbe_bind,
	.unbind = drm_psb_tbe_unbind,
	.destroy = drm_psb_tbe_destroy,
};

static struct ttm_tt *drm_psb_tbe_init(struct ttm_bo_device *bdev,
				       unsigned long size, uint32_t page_flags,
				       struct page *dummy_read_page)
{
	struct drm_psb_ttm_tt *psb_ttm;

	psb_ttm = kzalloc(sizeof(*psb_ttm), GFP_KERNEL);
	if (!psb_ttm)
		return NULL;
	psb_ttm->ttm.ttm.func = &psb_ttm_backend;
	psb_ttm->ttm.ttm.pages = NULL;

	if(ttm_dma_tt_init(&psb_ttm->ttm, bdev, size, page_flags, dummy_read_page)) {
		return NULL;
	}
	return &psb_ttm->ttm.ttm;
}

static int psb_ttm_io_mem_reserve(struct ttm_bo_device *bdev, struct ttm_mem_reg *mem)
{
	struct ttm_mem_type_manager *man = &bdev->man[mem->mem_type];
	struct drm_psb_private *dev_priv =
	    container_of(bdev, struct drm_psb_private, bdev);
	struct psb_gtt *pg = dev_priv->pg;

	mem->bus.addr = NULL;
	mem->bus.offset = 0;
	mem->bus.size = mem->num_pages << PAGE_SHIFT;
	mem->bus.base = 0;
	mem->bus.is_iomem = false;
	if (!(man->flags & TTM_MEMTYPE_FLAG_MAPPABLE))
		return -EINVAL;
	switch (mem->mem_type) {
	case TTM_PL_SYSTEM:
		/* system memory */
		return 0;
	case TTM_PL_TT:
		mem->bus.offset = mem->start << PAGE_SHIFT;
		mem->bus.base = pg->gatt_start;
		mem->bus.is_iomem = false; /* Don't know whether it is IO_MEM, this flag used in vm_fault handle */
		break;
	case DRM_PSB_MEM_MMU:
		mem->bus.offset = mem->start << PAGE_SHIFT;
		mem->bus.base = 0x00000000;
		break;
	case TTM_PL_CI:
		mem->bus.offset = mem->start << PAGE_SHIFT;
		mem->bus.base = dev_priv->ci_region_start;;
		mem->bus.is_iomem = true;
		break;
	case TTM_PL_RAR:
		mem->bus.offset = mem->start << PAGE_SHIFT;
		mem->bus.base = dev_priv->rar_region_start;;
		mem->bus.is_iomem = true;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void psb_ttm_io_mem_free(struct ttm_bo_device *bdev, struct ttm_mem_reg *mem)
{
}

/*
 * Use this memory type priority if no eviction is needed.
 */
/*
static uint32_t psb_mem_prios[] = {
	TTM_PL_CI,
	TTM_PL_RAR,
	TTM_PL_TT,
	DRM_PSB_MEM_MMU,
	TTM_PL_SYSTEM
};
*/
/*
 * Use this memory type priority if need to evict.
 */
/*
static uint32_t psb_busy_prios[] = {
	TTM_PL_TT,
	TTM_PL_CI,
	TTM_PL_RAR,
	DRM_PSB_MEM_MMU,
	TTM_PL_SYSTEM
};
*/
struct ttm_bo_driver psb_ttm_bo_driver = {
/*
	.mem_type_prio = psb_mem_prios,
	.mem_busy_prio = psb_busy_prios,
	.num_mem_type_prio = ARRAY_SIZE(psb_mem_prios),
	.num_mem_busy_prio = ARRAY_SIZE(psb_busy_prios),
*/
	.ttm_tt_create = &drm_psb_tbe_init,
	.ttm_tt_populate = &drm_psb_tbe_populate,
	.ttm_tt_unpopulate = &drm_psb_tbe_unpopulate,
	.invalidate_caches = &psb_invalidate_caches,
	.init_mem_type = &psb_init_mem_type,
	.evict_flags = &psb_evict_mask,
	.move = &psb_move,
	.verify_access = &psb_verify_access,
	.sync_obj_signaled = &ttm_fence_sync_obj_signaled,
	.sync_obj_wait = &ttm_fence_sync_obj_wait,
	.sync_obj_flush = &ttm_fence_sync_obj_flush,
	.sync_obj_unref = &ttm_fence_sync_obj_unref,
	.sync_obj_ref = &ttm_fence_sync_obj_ref,
	.io_mem_reserve = &psb_ttm_io_mem_reserve,
	.io_mem_free = &psb_ttm_io_mem_free
};
