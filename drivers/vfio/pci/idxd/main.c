// SPDX-License-Identifier: GPL-2.0-only
/* Copyright(c) 2026 Intel Corporation */

#include <linux/capability.h>
#include <linux/bitmap.h>
#include <linux/container_of.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iommufd.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include <linux/vfio.h>
#include <linux/vfio_pci_core.h>

#include <uapi/linux/vfio.h>

#include "idxd.h"

#define IDXD_VDEV_BAR0_SIZE		SZ_64K
#define IDXD_VDEV_PORTALS_PER_WQ	2
#define IDXD_VDEV_WQCFG_OFFSET		0x100

#define IDXD_VDEV_CMDCAP (BIT(IDXD_CMD_ENABLE_DEVICE) |		\
			  BIT(IDXD_CMD_DISABLE_DEVICE) |		\
			  BIT(IDXD_CMD_DRAIN_ALL) |		\
			  BIT(IDXD_CMD_ABORT_ALL) |		\
			  BIT(IDXD_CMD_RESET_DEVICE) |		\
			  BIT(IDXD_CMD_ENABLE_WQ) |		\
			  BIT(IDXD_CMD_DISABLE_WQ) |		\
			  BIT(IDXD_CMD_DRAIN_WQ) |		\
			  BIT(IDXD_CMD_ABORT_WQ) |		\
			  BIT(IDXD_CMD_RESET_WQ) |		\
			  BIT(IDXD_CMD_DRAIN_PASID) |		\
			  BIT(IDXD_CMD_ABORT_PASID))

#define IDXD_VDEV_WQCFG_IDX2_WR_MASK (BIT(0) | GENMASK(27, 8) |	\
				       BIT(28) | BIT(29))

struct idxd_vfio_device {
	struct vfio_device vdev;
	struct idxd_vdev *ivdev;
	u8 config[PCI_CFG_SPACE_EXP_SIZE];

	struct mutex bar0_lock;	/* protects the virtual BAR0 state */
	enum idxd_device_status_state state;
	union wqcfg *wqcfg;
	unsigned long *wq_enable_map;
	struct grpcfg grpcfg;
	u32 genctrl;
	u32 gencfg;
	u32 intcause;
	u32 cmdsts;
	u64 evlcfg[2];
	u64 evlstatus;

	struct mutex pasid_lock;	/* protects default_host_pasid */
	u32 default_host_pasid;
	bool pasid_attached;
};

static size_t idxd_vfio_bar2_size(struct idxd_vfio_device *vfio_dev)
{
	return vfio_dev->ivdev->num_wqs * IDXD_VDEV_PORTALS_PER_WQ * PAGE_SIZE;
}

static u32 idxd_vfio_wqcfg_offset(struct idxd_vfio_device *vfio_dev)
{
	return IDXD_VDEV_WQCFG_OFFSET;
}

static u32 idxd_vfio_wqcfg_size(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_vdev *ivdev = vfio_dev->ivdev;

	return ivdev->num_wqs * ivdev->idxd->wqcfg_size;
}

static u32 idxd_vfio_grpcfg_offset(struct idxd_vfio_device *vfio_dev)
{
	return ALIGN(idxd_vfio_wqcfg_offset(vfio_dev) +
		     idxd_vfio_wqcfg_size(vfio_dev), IDXD_TABLE_MULT);
}

static u32 idxd_vfio_msix_perm_offset(struct idxd_vfio_device *vfio_dev)
{
	return idxd_vfio_grpcfg_offset(vfio_dev) + IDXD_TABLE_MULT;
}

static u32 idxd_vfio_ims_offset(struct idxd_vfio_device *vfio_dev)
{
	return idxd_vfio_msix_perm_offset(vfio_dev) + IDXD_TABLE_MULT;
}

static void idxd_vfio_config_writew(struct idxd_vfio_device *vfio_dev,
				    unsigned int pos, u16 val)
{
	vfio_dev->config[pos] = val;
	vfio_dev->config[pos + 1] = val >> 8;
}

static void idxd_vfio_config_writeb(struct idxd_vfio_device *vfio_dev,
				    unsigned int pos, u8 val)
{
	vfio_dev->config[pos] = val;
}

static void idxd_vfio_init_config(struct idxd_vfio_device *vfio_dev)
{
	struct pci_dev *pdev = vfio_dev->ivdev->idxd->pdev;

	memset(vfio_dev->config, 0, sizeof(vfio_dev->config));
	idxd_vfio_config_writew(vfio_dev, PCI_VENDOR_ID, PCI_VENDOR_ID_INTEL);
	idxd_vfio_config_writew(vfio_dev, PCI_DEVICE_ID, pdev->device);
	idxd_vfio_config_writew(vfio_dev, PCI_COMMAND, PCI_COMMAND_MEMORY);
	idxd_vfio_config_writew(vfio_dev, PCI_CLASS_DEVICE,
				PCI_CLASS_ACCELERATOR_PROCESSING);
	idxd_vfio_config_writeb(vfio_dev, PCI_HEADER_TYPE, PCI_HEADER_TYPE_NORMAL);
}

static struct idxd_vwq *idxd_vfio_vwq(struct idxd_vdev *ivdev, unsigned int id)
{
	struct idxd_vwq *vwq;

	list_for_each_entry(vwq, &ivdev->wqs, node) {
		if (vwq->id == id)
			return vwq;
	}

	return NULL;
}

static unsigned int idxd_vfio_mmap_area_count(struct idxd_vdev *ivdev)
{
	struct idxd_vwq *vwq;
	unsigned int count = 0;

	list_for_each_entry(vwq, &ivdev->wqs, node) {
		count++;
		if (!vwq->shared)
			count++;
	}

	return count;
}

static u64 idxd_vfio_read_wqcap(struct idxd_vdev *ivdev)
{
	struct idxd_device *idxd = ivdev->idxd;
	union wq_cap_reg wqcap = idxd->hw.wq_cap;
	struct idxd_vwq *vwq;

	wqcap.total_wq_size = 0;
	wqcap.num_wqs = ivdev->num_wqs;
	wqcap.dedicated_mode = 0;
	wqcap.shared_mode = 0;

	list_for_each_entry(vwq, &ivdev->wqs, node) {
		wqcap.total_wq_size += vwq->wq->size;
		if (vwq->shared) {
			wqcap.shared_mode = 1;
		} else {
			/*
			 * A single-guest assignment can be exposed to the guest
			 * as either a dedicated or shared WQ.
			 */
			wqcap.dedicated_mode = 1;
			wqcap.shared_mode = 1;
		}
	}

	return wqcap.bits;
}

static u64 idxd_vfio_read_gencap(struct idxd_vdev *ivdev)
{
	union gen_cap_reg gencap = ivdev->idxd->hw.gen_cap;

	gencap.cmd_cap = 1;
	gencap.evl_support = 0;
	gencap.max_ims_mult = 0;
	gencap.config_en = 0;
	return gencap.bits;
}

static u64 idxd_vfio_read_groupcap(struct idxd_vdev *ivdev)
{
	union group_cap_reg groupcap = ivdev->idxd->hw.group_cap;

	groupcap.num_groups = 1;
	return groupcap.bits;
}

static u64 idxd_vfio_read_enginecap(struct idxd_vdev *ivdev)
{
	union engine_cap_reg enginecap = ivdev->idxd->hw.engine_cap;

	enginecap.num_engines = min_t(unsigned int, ivdev->num_wqs,
				      enginecap.num_engines);
	return enginecap.bits;
}

static u64 idxd_vfio_read_table(struct idxd_vfio_device *vfio_dev,
				unsigned int idx)
{
	union offsets_reg offsets = {};

	offsets.grpcfg = idxd_vfio_grpcfg_offset(vfio_dev) / IDXD_TABLE_MULT;
	offsets.wqcfg = idxd_vfio_wqcfg_offset(vfio_dev) / IDXD_TABLE_MULT;
	offsets.msix_perm = idxd_vfio_msix_perm_offset(vfio_dev) / IDXD_TABLE_MULT;
	offsets.ims = idxd_vfio_ims_offset(vfio_dev) / IDXD_TABLE_MULT;

	return offsets.bits[idx];
}

static bool idxd_vfio_wqcfg_readb(struct idxd_vfio_device *vfio_dev,
				  loff_t pos, u8 *val)
{
	struct idxd_vdev *ivdev = vfio_dev->ivdev;
	struct idxd_device *idxd = ivdev->idxd;
	struct idxd_vwq *vwq;
	union wqcfg wqcfg;
	u32 wqcfg_offset = idxd_vfio_wqcfg_offset(vfio_dev);
	unsigned int id;
	u32 offset;

	if (pos < wqcfg_offset ||
	    pos >= wqcfg_offset + idxd_vfio_wqcfg_size(vfio_dev))
		return false;

	offset = pos - wqcfg_offset;
	id = offset / idxd->wqcfg_size;
	if (id >= ivdev->num_wqs)
		return false;

	vwq = idxd_vfio_vwq(ivdev, id);
	if (!vwq)
		return false;

	memcpy(&wqcfg, &vfio_dev->wqcfg[id], sizeof(wqcfg));
	wqcfg.mode_support = !vwq->shared;
	wqcfg.wq_state = test_bit(id, vfio_dev->wq_enable_map) ?
		IDXD_WQ_ENABLED : IDXD_WQ_DISABLED;
	offset %= idxd->wqcfg_size;
	if (offset >= sizeof(wqcfg))
		*val = 0;
	else
		*val = ((u8 *)&wqcfg)[offset];

	return true;
}

static bool idxd_vfio_grpcfg_readb(struct idxd_vfio_device *vfio_dev,
				   loff_t pos, u8 *val)
{
	u32 grpcfg_offset = idxd_vfio_grpcfg_offset(vfio_dev);
	u32 offset;

	if (pos < grpcfg_offset || pos >= grpcfg_offset + GRPCFG_SIZE)
		return false;

	offset = pos - grpcfg_offset;
	if (offset >= sizeof(vfio_dev->grpcfg))
		*val = 0;
	else
		*val = ((u8 *)&vfio_dev->grpcfg)[offset];

	return true;
}

static u64 idxd_vfio_bar0_readq(struct idxd_vfio_device *vfio_dev, loff_t pos)
{
	struct idxd_vdev *ivdev = vfio_dev->ivdev;
	struct idxd_device *idxd = ivdev->idxd;
	union gensts_reg gensts = {};

	switch (pos) {
	case IDXD_VER_OFFSET:
		return idxd->hw.version;
	case IDXD_GENCAP_OFFSET:
		return idxd_vfio_read_gencap(ivdev);
	case IDXD_WQCAP_OFFSET:
		return idxd_vfio_read_wqcap(ivdev);
	case IDXD_GRPCAP_OFFSET:
		return idxd_vfio_read_groupcap(ivdev);
	case IDXD_ENGCAP_OFFSET:
		return idxd_vfio_read_enginecap(ivdev);
	case IDXD_OPCAP_OFFSET:
	case IDXD_OPCAP_OFFSET + sizeof(u64):
	case IDXD_OPCAP_OFFSET + 2 * sizeof(u64):
	case IDXD_OPCAP_OFFSET + 3 * sizeof(u64):
		return idxd->hw.opcap.bits[(pos - IDXD_OPCAP_OFFSET) /
					    sizeof(u64)];
	case IDXD_TABLE_OFFSET:
		return idxd_vfio_read_table(vfio_dev, 0);
	case IDXD_TABLE_OFFSET + sizeof(u64):
		return idxd_vfio_read_table(vfio_dev, 1);
	case IDXD_GENCFG_OFFSET:
		return vfio_dev->gencfg;
	case IDXD_GENCTRL_OFFSET:
		return vfio_dev->genctrl;
	case IDXD_GENSTATS_OFFSET:
		gensts.state = vfio_dev->state;
		return gensts.bits;
	case IDXD_INTCAUSE_OFFSET:
		return vfio_dev->intcause;
	case IDXD_CMDSTS_OFFSET:
		return vfio_dev->cmdsts;
	case IDXD_CMDCAP_OFFSET:
		return IDXD_VDEV_CMDCAP;
	case IDXD_EVLCFG_OFFSET:
		return vfio_dev->evlcfg[0];
	case IDXD_EVLCFG_OFFSET + sizeof(u64):
		return vfio_dev->evlcfg[1];
	case IDXD_EVLSTATUS_OFFSET:
		return vfio_dev->evlstatus;
	case IDXD_SWERR_OFFSET:
	case IDXD_SWERR_OFFSET + sizeof(u64):
	case IDXD_SWERR_OFFSET + 2 * sizeof(u64):
	case IDXD_SWERR_OFFSET + 3 * sizeof(u64):
	case IDXD_IAACAP_OFFSET:
		return 0;
	default:
		return 0;
	}
}

static u8 idxd_vfio_bar0_readb(struct idxd_vfio_device *vfio_dev, loff_t pos)
{
	u64 val;
	u8 byte;

	if (idxd_vfio_wqcfg_readb(vfio_dev, pos, &byte))
		return byte;

	if (idxd_vfio_grpcfg_readb(vfio_dev, pos, &byte))
		return byte;

	val = idxd_vfio_bar0_readq(vfio_dev, pos & ~0x7ULL);
	return val >> ((pos & 0x7) * 8);
}

static ssize_t idxd_vfio_bar0_read(struct idxd_vfio_device *vfio_dev,
				   char __user *buf, size_t count, loff_t pos)
{
	size_t done;

	if (pos < 0 || pos >= IDXD_VDEV_BAR0_SIZE)
		return -EINVAL;

	count = min_t(size_t, count, IDXD_VDEV_BAR0_SIZE - pos);
	mutex_lock(&vfio_dev->bar0_lock);
	for (done = 0; done < count; done++) {
		u8 val = idxd_vfio_bar0_readb(vfio_dev, pos + done);

		if (copy_to_user(buf + done, &val, sizeof(val))) {
			mutex_unlock(&vfio_dev->bar0_lock);
			return -EFAULT;
		}
	}
	mutex_unlock(&vfio_dev->bar0_lock);

	return count;
}

static void idxd_vfio_init_wqcfg(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_vdev *ivdev = vfio_dev->ivdev;
	struct idxd_vwq *vwq;

	list_for_each_entry(vwq, &ivdev->wqs, node) {
		union wqcfg *wqcfg = &vfio_dev->wqcfg[vwq->id];

		memcpy(wqcfg, vwq->wq->wqcfg, sizeof(*wqcfg));
		wqcfg->pasid = 0;
		wqcfg->pasid_en = 0;
		wqcfg->mode_support = !vwq->shared;
		wqcfg->wq_state = IDXD_WQ_DISABLED;
		if (vwq->shared)
			wqcfg->mode = 0;
		else
			wqcfg->mode = 1;
	}
}

static void idxd_vfio_init_grpcfg(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_vdev *ivdev = vfio_dev->ivdev;
	struct idxd_vwq *vwq;
	unsigned int engines;

	memset(&vfio_dev->grpcfg, 0, sizeof(vfio_dev->grpcfg));
	list_for_each_entry(vwq, &ivdev->wqs, node)
		vfio_dev->grpcfg.wqs[vwq->id / 64] |= BIT_ULL(vwq->id % 64);

	engines = min_t(unsigned int, ivdev->num_wqs,
			ivdev->idxd->hw.engine_cap.num_engines);
	engines = min_t(unsigned int, engines, 64);
	if (engines)
		vfio_dev->grpcfg.engines = GENMASK_ULL(engines - 1, 0);
	vfio_dev->grpcfg.flags.tc_a = 0;
	vfio_dev->grpcfg.flags.tc_b = 1;
}

static void idxd_vfio_reset_bar0(struct idxd_vfio_device *vfio_dev)
{
	vfio_dev->state = IDXD_DEVICE_STATE_DISABLED;
	vfio_dev->genctrl = 0;
	vfio_dev->gencfg = 0;
	vfio_dev->intcause = 0;
	vfio_dev->cmdsts = IDXD_CMDSTS_SUCCESS;
	vfio_dev->evlcfg[0] = 0;
	vfio_dev->evlcfg[1] = 0;
	vfio_dev->evlstatus = 0;
	bitmap_zero(vfio_dev->wq_enable_map, vfio_dev->ivdev->num_wqs);
	idxd_vfio_init_wqcfg(vfio_dev);
	idxd_vfio_init_grpcfg(vfio_dev);
}

static u32 idxd_vfio_cmdsts(u8 err, u16 result)
{
	union cmdsts_reg cmdsts = {};

	cmdsts.err = err;
	cmdsts.result = result;
	return cmdsts.bits;
}

static void idxd_vfio_complete_cmd(struct idxd_vfio_device *vfio_dev,
				   union idxd_command_reg cmd, u8 err,
				   u16 result)
{
	vfio_dev->cmdsts = idxd_vfio_cmdsts(err, result);
	if (cmd.int_req)
		vfio_dev->intcause |= IDXD_INTC_CMD;
}

static u8 idxd_vfio_enable_wq(struct idxd_vfio_device *vfio_dev, u32 operand)
{
	struct idxd_vwq *vwq;
	union wqcfg *wqcfg;

	if (operand >= vfio_dev->ivdev->num_wqs)
		return IDXD_CMDSTS_INVAL_WQIDX;
	if (vfio_dev->state != IDXD_DEVICE_STATE_ENABLED)
		return IDXD_CMDSTS_ERR_DEV_NOTEN;
	if (test_bit(operand, vfio_dev->wq_enable_map))
		return IDXD_CMDSTS_ERR_WQ_ENABLED;

	vwq = idxd_vfio_vwq(vfio_dev->ivdev, operand);
	if (!vwq)
		return IDXD_CMDSTS_INVAL_WQIDX;

	wqcfg = &vfio_dev->wqcfg[operand];
	if (vwq->shared && wqcfg->mode != 0)
		return IDXD_CMDSTS_ERR_WQ_MODE;
	if (wqcfg->wq_size == 0)
		return IDXD_CMDSTS_ERR_WQ_SIZE;
	if (!wqcfg->mode && wqcfg->wq_thresh == 0)
		return IDXD_CMDSTS_ERR_WQ_SIZE;

	set_bit(operand, vfio_dev->wq_enable_map);
	return IDXD_CMDSTS_SUCCESS;
}

static u8 idxd_vfio_disable_wq(struct idxd_vfio_device *vfio_dev, u32 operand)
{
	unsigned long mask = operand & GENMASK(15, 0);
	unsigned int bit, base = (operand >> 16) * 16;

	if (!mask)
		return IDXD_CMDSTS_INVAL_WQIDX;
	if (vfio_dev->state != IDXD_DEVICE_STATE_ENABLED)
		return IDXD_CMDSTS_ERR_DEV_NOT_EN;

	for_each_set_bit(bit, &mask, 16) {
		unsigned int id = base + bit;

		if (id >= vfio_dev->ivdev->num_wqs)
			return IDXD_CMDSTS_INVAL_WQIDX;
		clear_bit(id, vfio_dev->wq_enable_map);
	}

	return IDXD_CMDSTS_SUCCESS;
}

static u8 idxd_vfio_reset_wq(struct idxd_vfio_device *vfio_dev, u32 operand)
{
	struct idxd_vwq *vwq;
	unsigned long mask = operand & GENMASK(15, 0);
	unsigned int bit, base = (operand >> 16) * 16;

	if (!mask)
		return IDXD_CMDSTS_INVAL_WQIDX;
	if (vfio_dev->state != IDXD_DEVICE_STATE_ENABLED)
		return IDXD_CMDSTS_ERR_DEV_NOT_EN;

	for_each_set_bit(bit, &mask, 16) {
		unsigned int id = base + bit;

		if (id >= vfio_dev->ivdev->num_wqs)
			return IDXD_CMDSTS_INVAL_WQIDX;

		vwq = idxd_vfio_vwq(vfio_dev->ivdev, id);
		if (!vwq)
			return IDXD_CMDSTS_INVAL_WQIDX;

		clear_bit(id, vfio_dev->wq_enable_map);
		memcpy(&vfio_dev->wqcfg[id], vwq->wq->wqcfg,
		       sizeof(vfio_dev->wqcfg[id]));
		vfio_dev->wqcfg[id].pasid = 0;
		vfio_dev->wqcfg[id].pasid_en = 0;
		vfio_dev->wqcfg[id].mode_support = !vwq->shared;
		vfio_dev->wqcfg[id].wq_state = IDXD_WQ_DISABLED;
		vfio_dev->wqcfg[id].mode = vwq->shared ? 0 : 1;
	}

	return IDXD_CMDSTS_SUCCESS;
}

static u8 idxd_vfio_check_wq_mask(struct idxd_vfio_device *vfio_dev, u32 operand)
{
	unsigned long mask = operand & GENMASK(15, 0);
	unsigned int bit, base = (operand >> 16) * 16;

	if (!mask)
		return IDXD_CMDSTS_INVAL_WQIDX;
	if (vfio_dev->state != IDXD_DEVICE_STATE_ENABLED)
		return IDXD_CMDSTS_ERR_DEV_NOT_EN;

	for_each_set_bit(bit, &mask, 16) {
		if (base + bit >= vfio_dev->ivdev->num_wqs)
			return IDXD_CMDSTS_INVAL_WQIDX;
	}

	return IDXD_CMDSTS_SUCCESS;
}

static void idxd_vfio_exec_cmd(struct idxd_vfio_device *vfio_dev, u32 val)
{
	union idxd_command_reg cmd = { .bits = val };
	u8 status = IDXD_CMDSTS_SUCCESS;

	switch (cmd.cmd) {
	case IDXD_CMD_ENABLE_DEVICE:
		if (vfio_dev->state == IDXD_DEVICE_STATE_ENABLED)
			status = IDXD_CMDSTS_ERR_DEV_ENABLED;
		else
			vfio_dev->state = IDXD_DEVICE_STATE_ENABLED;
		break;
	case IDXD_CMD_DISABLE_DEVICE:
		vfio_dev->state = IDXD_DEVICE_STATE_DISABLED;
		bitmap_zero(vfio_dev->wq_enable_map, vfio_dev->ivdev->num_wqs);
		break;
	case IDXD_CMD_RESET_DEVICE:
		idxd_vfio_reset_bar0(vfio_dev);
		break;
	case IDXD_CMD_ENABLE_WQ:
		status = idxd_vfio_enable_wq(vfio_dev, cmd.operand);
		break;
	case IDXD_CMD_DISABLE_WQ:
		status = idxd_vfio_disable_wq(vfio_dev, cmd.operand);
		break;
	case IDXD_CMD_DRAIN_WQ:
	case IDXD_CMD_ABORT_WQ:
		status = idxd_vfio_check_wq_mask(vfio_dev, cmd.operand);
		break;
	case IDXD_CMD_RESET_WQ:
		status = idxd_vfio_reset_wq(vfio_dev, cmd.operand);
		break;
	case IDXD_CMD_DRAIN_ALL:
	case IDXD_CMD_ABORT_ALL:
	case IDXD_CMD_DRAIN_PASID:
	case IDXD_CMD_ABORT_PASID:
		break;
	default:
		status = IDXD_CMDSTS_INVAL_CMD;
		break;
	}

	idxd_vfio_complete_cmd(vfio_dev, cmd, status, 0);
}

static bool idxd_vfio_wqcfg_write(struct idxd_vfio_device *vfio_dev,
				  loff_t pos, u32 val)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	struct idxd_vwq *vwq;
	union wqcfg *wqcfg;
	u32 wqcfg_offset = idxd_vfio_wqcfg_offset(vfio_dev);
	u32 offset;
	unsigned int id, idx;
	u32 mask;

	if (pos < wqcfg_offset ||
	    pos >= wqcfg_offset + idxd_vfio_wqcfg_size(vfio_dev))
		return false;
	if (!IS_ALIGNED(pos, sizeof(u32)))
		return true;

	offset = pos - wqcfg_offset;
	id = offset / idxd->wqcfg_size;
	idx = (offset % idxd->wqcfg_size) / sizeof(u32);
	if (id >= vfio_dev->ivdev->num_wqs ||
	    idx >= ARRAY_SIZE(vfio_dev->wqcfg[0].bits))
		return true;

	vwq = idxd_vfio_vwq(vfio_dev->ivdev, id);
	if (!vwq)
		return true;

	wqcfg = &vfio_dev->wqcfg[id];
	if (vwq->shared || test_bit(id, vfio_dev->wq_enable_map))
		return true;

	switch (idx) {
	case 1:
		mask = GENMASK(15, 0);
		break;
	case 2:
		mask = IDXD_VDEV_WQCFG_IDX2_WR_MASK;
		break;
	default:
		mask = 0;
		break;
	}

	wqcfg->bits[idx] = (wqcfg->bits[idx] & ~mask) | (val & mask);
	wqcfg->mode_support = 1;
	return true;
}

static ssize_t idxd_vfio_bar0_write(struct idxd_vfio_device *vfio_dev,
				    const char __user *buf, size_t count,
				    loff_t pos)
{
	u32 val;
	u64 val64;

	if (pos < 0 || pos >= IDXD_VDEV_BAR0_SIZE)
		return -EINVAL;

	mutex_lock(&vfio_dev->bar0_lock);
	if (count == sizeof(val)) {
		if (copy_from_user(&val, buf, sizeof(val)))
			goto err_fault;
		if (idxd_vfio_wqcfg_write(vfio_dev, pos, val))
			goto out_unlock;
		if (pos == IDXD_GENCFG_OFFSET)
			vfio_dev->gencfg = val;
		else if (pos == IDXD_GENCTRL_OFFSET)
			vfio_dev->genctrl = val;
		else if (pos == IDXD_INTCAUSE_OFFSET)
			vfio_dev->intcause &= ~val;
		else if (pos == IDXD_CMD_OFFSET)
			idxd_vfio_exec_cmd(vfio_dev, val);
		else if (pos == IDXD_EVLSTATUS_OFFSET)
			vfio_dev->evlstatus = val;
	} else if (count == sizeof(val64)) {
		if (copy_from_user(&val64, buf, sizeof(val64)))
			goto err_fault;
		if (pos == IDXD_EVLCFG_OFFSET)
			vfio_dev->evlcfg[0] = val64;
		else if (pos == IDXD_EVLCFG_OFFSET + sizeof(u64))
			vfio_dev->evlcfg[1] = val64;
	}

out_unlock:
	mutex_unlock(&vfio_dev->bar0_lock);
	return min_t(size_t, count, IDXD_VDEV_BAR0_SIZE - pos);

err_fault:
	mutex_unlock(&vfio_dev->bar0_lock);
	return -EFAULT;
}

static int idxd_vfio_bar2_pos(struct idxd_vfio_device *vfio_dev, loff_t pos,
			      struct idxd_vwq **vwq,
			      enum idxd_portal_prot *prot, u64 *portal_offset)
{
	unsigned int vportal, vwq_id;

	if (pos < 0 || pos >= idxd_vfio_bar2_size(vfio_dev))
		return -EINVAL;

	vportal = pos >> PAGE_SHIFT;
	vwq_id = vportal / IDXD_VDEV_PORTALS_PER_WQ;
	*vwq = idxd_vfio_vwq(vfio_dev->ivdev, vwq_id);
	if (!*vwq)
		return -EINVAL;

	*prot = (vportal % IDXD_VDEV_PORTALS_PER_WQ) ?
		IDXD_PORTAL_UNLIMITED : IDXD_PORTAL_LIMITED;
	*portal_offset = offset_in_page(pos);

	return 0;
}

static ssize_t idxd_vfio_bar2_write(struct idxd_vfio_device *vfio_dev,
				    const char __user *buf, size_t count,
				    loff_t pos)
{
	struct idxd_vwq *vwq;
	enum idxd_portal_prot prot;
	struct dsa_hw_desc desc;
	void __iomem *portal;
	u64 portal_offset;
	phys_addr_t paddr;
	int rc;

	mutex_lock(&vfio_dev->pasid_lock);
	if (!vfio_dev->pasid_attached) {
		mutex_unlock(&vfio_dev->pasid_lock);
		return -EIO;
	}
	mutex_unlock(&vfio_dev->pasid_lock);

	rc = idxd_vfio_bar2_pos(vfio_dev, pos, &vwq, &prot, &portal_offset);
	if (rc)
		return rc;

	if (prot != IDXD_PORTAL_UNLIMITED)
		return -EINVAL;

	if (portal_offset + count > PAGE_SIZE ||
	    !IS_ALIGNED(portal_offset, sizeof(desc)) ||
	    count != sizeof(desc))
		return -EINVAL;

	if (copy_from_user(&desc, buf, sizeof(desc)))
		return -EFAULT;

	paddr = pci_resource_start(vwq->wq->idxd->pdev, IDXD_WQ_BAR);
	paddr += idxd_get_wq_portal_full_offset(vwq->wq->id,
						IDXD_PORTAL_UNLIMITED);

	portal = ioremap(paddr, PAGE_SIZE);
	if (!portal)
		return -ENOMEM;

	iosubmit_cmds512(portal + portal_offset, &desc, 1);
	iounmap(portal);

	return count;
}

static ssize_t idxd_vfio_read(struct vfio_device *vdev, char __user *buf,
			      size_t count, loff_t *ppos)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	unsigned int index = VFIO_PCI_OFFSET_TO_INDEX(*ppos);
	loff_t pos = *ppos & VFIO_PCI_OFFSET_MASK;
	ssize_t ret;

	switch (index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
		ret = idxd_vfio_bar0_read(vfio_dev, buf, count, pos);
		break;
	case VFIO_PCI_CONFIG_REGION_INDEX:
		if (pos < 0 || pos >= sizeof(vfio_dev->config))
			return -EINVAL;
		ret = min_t(size_t, count, sizeof(vfio_dev->config) - pos);
		if (copy_to_user(buf, vfio_dev->config + pos, ret))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	if (ret > 0)
		*ppos += ret;
	return ret;
}

static ssize_t idxd_vfio_write(struct vfio_device *vdev, const char __user *buf,
			       size_t count, loff_t *ppos)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	unsigned int index = VFIO_PCI_OFFSET_TO_INDEX(*ppos);
	loff_t pos = *ppos & VFIO_PCI_OFFSET_MASK;
	ssize_t ret;

	switch (index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
		ret = idxd_vfio_bar0_write(vfio_dev, buf, count, pos);
		break;
	case VFIO_PCI_BAR2_REGION_INDEX:
		ret = idxd_vfio_bar2_write(vfio_dev, buf, count, pos);
		break;
	case VFIO_PCI_CONFIG_REGION_INDEX:
		if (pos < 0 || pos >= sizeof(vfio_dev->config))
			return -EINVAL;
		ret = min_t(size_t, count, sizeof(vfio_dev->config) - pos);
		if (copy_from_user(vfio_dev->config + pos, buf, ret))
			return -EFAULT;
		break;
	default:
		return -EINVAL;
	}

	if (ret > 0)
		*ppos += ret;
	return ret;
}

static int
idxd_vfio_add_sparse_mmap_cap(struct idxd_vfio_device *vfio_dev,
			      struct vfio_region_info *info, unsigned long arg,
			      unsigned long minsz)
{
	struct vfio_region_info_cap_sparse_mmap *sparse;
	struct idxd_vdev *ivdev = vfio_dev->ivdev;
	struct idxd_vwq *vwq;
	unsigned int nr_areas;
	unsigned int i = 0;
	size_t capsz;

	nr_areas = idxd_vfio_mmap_area_count(ivdev);
	capsz = struct_size(sparse, areas, nr_areas);
	info->flags |= VFIO_REGION_INFO_FLAG_CAPS;
	if (info->argsz < minsz + capsz) {
		info->argsz = minsz + capsz;
		info->cap_offset = 0;
		return 0;
	}

	sparse = kzalloc(capsz, GFP_KERNEL);
	if (!sparse)
		return -ENOMEM;

	sparse->header.id = VFIO_REGION_INFO_CAP_SPARSE_MMAP;
	sparse->header.version = 1;
	sparse->nr_areas = nr_areas;

	list_for_each_entry(vwq, &ivdev->wqs, node) {
		sparse->areas[i].offset =
			(vwq->id * IDXD_VDEV_PORTALS_PER_WQ) * PAGE_SIZE;
		sparse->areas[i].size = PAGE_SIZE;
		i++;

		if (!vwq->shared) {
			sparse->areas[i].offset =
				(vwq->id * IDXD_VDEV_PORTALS_PER_WQ + 1) *
				PAGE_SIZE;
			sparse->areas[i].size = PAGE_SIZE;
			i++;
		}
	}

	info->cap_offset = minsz;

	if (copy_to_user((void __user *)arg + minsz, sparse, capsz)) {
		kfree(sparse);
		return -EFAULT;
	}

	kfree(sparse);
	return 0;
}

static int idxd_vfio_get_region_info(struct idxd_vfio_device *vfio_dev,
				     unsigned long arg)
{
	struct vfio_region_info info;
	unsigned long minsz;
	int ret;

	minsz = offsetofend(struct vfio_region_info, offset);
	if (copy_from_user(&info, (void __user *)arg, minsz))
		return -EFAULT;
	if (info.argsz < minsz)
		return -EINVAL;

	info.flags = 0;
	info.cap_offset = 0;
	info.size = 0;
	info.offset = VFIO_PCI_INDEX_TO_OFFSET(info.index);

	switch (info.index) {
	case VFIO_PCI_BAR0_REGION_INDEX:
		info.flags = VFIO_REGION_INFO_FLAG_READ |
			     VFIO_REGION_INFO_FLAG_WRITE;
		info.size = IDXD_VDEV_BAR0_SIZE;
		break;
	case VFIO_PCI_BAR2_REGION_INDEX:
		info.flags = VFIO_REGION_INFO_FLAG_WRITE |
			     VFIO_REGION_INFO_FLAG_MMAP;
		info.size = idxd_vfio_bar2_size(vfio_dev);
		ret = idxd_vfio_add_sparse_mmap_cap(vfio_dev, &info, arg,
						    minsz);
		if (ret)
			return ret;
		break;
	case VFIO_PCI_CONFIG_REGION_INDEX:
		info.flags = VFIO_REGION_INFO_FLAG_READ |
			     VFIO_REGION_INFO_FLAG_WRITE;
		info.size = sizeof(vfio_dev->config);
		break;
	default:
		if (info.index >= VFIO_PCI_NUM_REGIONS)
			return -EINVAL;
		break;
	}

	return copy_to_user((void __user *)arg, &info, minsz) ? -EFAULT : 0;
}

static int idxd_vfio_mmap(struct vfio_device *vdev, struct vm_area_struct *vma)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	unsigned int index = vma->vm_pgoff >> (VFIO_PCI_OFFSET_SHIFT - PAGE_SHIFT);
	u64 pos = (vma->vm_pgoff &
		   ((1U << (VFIO_PCI_OFFSET_SHIFT - PAGE_SHIFT)) - 1)) <<
		  PAGE_SHIFT;
	struct idxd_vwq *vwq;
	enum idxd_portal_prot prot;
	u64 portal_offset;
	phys_addr_t paddr;
	int rc;

	if (index != VFIO_PCI_BAR2_REGION_INDEX)
		return -EINVAL;

	mutex_lock(&vfio_dev->pasid_lock);
	if (!vfio_dev->pasid_attached) {
		mutex_unlock(&vfio_dev->pasid_lock);
		return -EIO;
	}
	mutex_unlock(&vfio_dev->pasid_lock);

	if ((vma->vm_end - vma->vm_start) != PAGE_SIZE)
		return -EINVAL;

	rc = idxd_vfio_bar2_pos(vfio_dev, pos, &vwq, &prot, &portal_offset);
	if (rc)
		return rc;

	if (portal_offset || (prot == IDXD_PORTAL_UNLIMITED && vwq->shared))
		return -EINVAL;

	if (!vwq->wq->idxd->user_submission_safe && !capable(CAP_SYS_RAWIO))
		return -EPERM;

	paddr = pci_resource_start(vwq->wq->idxd->pdev, IDXD_WQ_BAR);
	paddr += idxd_get_wq_portal_full_offset(vwq->wq->id, prot);

	vm_flags_set(vma, VM_DONTCOPY);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start, paddr >> PAGE_SHIFT,
			       PAGE_SIZE, vma->vm_page_prot);
}

static int idxd_vfio_bind_iommufd(struct vfio_device *vdev,
				  struct iommufd_ctx *ictx,
				  u32 *out_device_id)
{
	return vfio_iommufd_emulated_bind(vdev, ictx, out_device_id);
}

static void idxd_vfio_unbind_iommufd(struct vfio_device *vdev)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);

	mutex_lock(&vfio_dev->pasid_lock);
	vfio_dev->pasid_attached = false;
	mutex_unlock(&vfio_dev->pasid_lock);
	vfio_iommufd_emulated_unbind(vdev);
}

static int idxd_vfio_attach_ioas(struct vfio_device *vdev, u32 *pt_id)
{
	return -EOPNOTSUPP;
}

static void idxd_vfio_detach_ioas(struct vfio_device *vdev)
{
}

static int idxd_vfio_pasid_attach_ioas(struct vfio_device *vdev, u32 pasid,
				       u32 *pt_id)
{
	return -EOPNOTSUPP;
}

static void idxd_vfio_pasid_detach_ioas(struct vfio_device *vdev, u32 pasid)
{
}

static long idxd_vfio_ioctl(struct vfio_device *vdev, unsigned int cmd,
			    unsigned long arg)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	unsigned long minsz;

	switch (cmd) {
	case VFIO_DEVICE_GET_INFO: {
		struct vfio_device_info info;

		minsz = offsetofend(struct vfio_device_info, num_irqs);
		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;
		if (info.argsz < minsz)
			return -EINVAL;

		info.flags = VFIO_DEVICE_FLAGS_PCI;
		info.num_regions = VFIO_PCI_NUM_REGIONS;
		info.num_irqs = 0;
		info.cap_offset = 0;
		info.pad = 0;

		return copy_to_user((void __user *)arg, &info, minsz) ?
			-EFAULT : 0;
	}
	case VFIO_DEVICE_GET_REGION_INFO:
		return idxd_vfio_get_region_info(vfio_dev, arg);
	default:
		return -ENOTTY;
	}
}

static int idxd_vfio_match(struct vfio_device *vdev, char *buf)
{
	struct idxd_vfio_device *ivdev =
		container_of(vdev, struct idxd_vfio_device, vdev);

	return sysfs_streq(buf, dev_name(vdev_confdev(ivdev->ivdev)));
}

static const struct vfio_device_ops idxd_vfio_ops = {
	.name = "idxd-vfio-pci",
	.ioctl = idxd_vfio_ioctl,
	.read = idxd_vfio_read,
	.write = idxd_vfio_write,
	.mmap = idxd_vfio_mmap,
	.match = idxd_vfio_match,
	.bind_iommufd = idxd_vfio_bind_iommufd,
	.unbind_iommufd = idxd_vfio_unbind_iommufd,
	.attach_ioas = idxd_vfio_attach_ioas,
	.detach_ioas = idxd_vfio_detach_ioas,
	.pasid_attach_ioas = idxd_vfio_pasid_attach_ioas,
	.pasid_detach_ioas = idxd_vfio_pasid_detach_ioas,
};

static int idxd_vfio_init_bar0(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_vdev *ivdev = vfio_dev->ivdev;

	vfio_dev->wqcfg = kcalloc(ivdev->num_wqs, sizeof(*vfio_dev->wqcfg),
				  GFP_KERNEL);
	if (!vfio_dev->wqcfg)
		return -ENOMEM;

	vfio_dev->wq_enable_map = bitmap_zalloc(ivdev->num_wqs, GFP_KERNEL);
	if (!vfio_dev->wq_enable_map) {
		kfree(vfio_dev->wqcfg);
		vfio_dev->wqcfg = NULL;
		return -ENOMEM;
	}

	idxd_vfio_reset_bar0(vfio_dev);
	return 0;
}

static void idxd_vfio_free_bar0(struct idxd_vfio_device *vfio_dev)
{
	bitmap_free(vfio_dev->wq_enable_map);
	kfree(vfio_dev->wqcfg);
}

static int idxd_vfio_probe(struct idxd_dev *idxd_dev)
{
	struct idxd_vdev *ivdev = idxd_dev_to_vdev(idxd_dev);
	struct device *dev = vdev_confdev(ivdev);
	struct idxd_vfio_device *vfio_dev;
	int rc;

	vfio_dev = vfio_alloc_device(idxd_vfio_device, vdev, dev,
				     &idxd_vfio_ops);
	if (IS_ERR(vfio_dev))
		return PTR_ERR(vfio_dev);

	vfio_dev->ivdev = ivdev;
	mutex_init(&vfio_dev->bar0_lock);
	mutex_init(&vfio_dev->pasid_lock);
	idxd_vfio_init_config(vfio_dev);
	dev_set_drvdata(dev, vfio_dev);

	rc = idxd_vfio_init_bar0(vfio_dev);
	if (rc)
		goto err_drvdata;

	rc = vfio_register_emulated_iommu_dev(&vfio_dev->vdev);
	if (rc)
		goto err_bar0;

	dev_info(dev, "registered VFIO VDEV with %u WQ(s)\n", ivdev->num_wqs);
	return 0;

err_bar0:
	idxd_vfio_free_bar0(vfio_dev);
err_drvdata:
	dev_set_drvdata(dev, NULL);
	mutex_destroy(&vfio_dev->pasid_lock);
	mutex_destroy(&vfio_dev->bar0_lock);
	vfio_put_device(&vfio_dev->vdev);
	return rc;
}

static void idxd_vfio_remove(struct idxd_dev *idxd_dev)
{
	struct idxd_vdev *ivdev = idxd_dev_to_vdev(idxd_dev);
	struct device *dev = vdev_confdev(ivdev);
	struct idxd_vfio_device *vfio_dev = dev_get_drvdata(dev);

	dev_set_drvdata(dev, NULL);
	vfio_unregister_group_dev(&vfio_dev->vdev);
	idxd_vfio_free_bar0(vfio_dev);
	mutex_destroy(&vfio_dev->pasid_lock);
	mutex_destroy(&vfio_dev->bar0_lock);
	vfio_put_device(&vfio_dev->vdev);
}

static enum idxd_dev_type idxd_vfio_dev_types[] = {
	IDXD_DEV_VFIO,
	IDXD_DEV_NONE,
};

static struct idxd_device_driver idxd_vfio_driver = {
	.probe = idxd_vfio_probe,
	.remove = idxd_vfio_remove,
	.name = "idxd_vfio_pci",
	.type = idxd_vfio_dev_types,
};

module_idxd_driver(idxd_vfio_driver);

MODULE_ALIAS_IDXD_DEVICE(IDXD_DEV_VFIO);
MODULE_IMPORT_NS("IDXD");
MODULE_IMPORT_NS("IOMMUFD");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel DSA VFIO PCI support");
