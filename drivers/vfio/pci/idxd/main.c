// SPDX-License-Identifier: GPL-2.0-only
/* Copyright(c) 2026 Intel Corporation */

#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/capability.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/eventfd.h>
#include <linux/io.h>
#include <linux/iommufd.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/sizes.h>
#include <linux/uaccess.h>
#include <linux/vfio.h>
#include <linux/vfio_pci_core.h>
#include <linux/workqueue.h>
#include <linux/xarray.h>

#include <uapi/linux/vfio.h>

#include "idxd.h"

#define IDXD_VDEV_BAR0_SIZE		SZ_64K
#define IDXD_VDEV_PORTALS_PER_WQ	4
#define IDXD_VDEV_WQCFG_OFFSET		0x100
#define IDXD_VDEV_PCIE_CAP_OFFSET	0x40
#define IDXD_VDEV_MSIX_CAP_OFFSET	0x80
#define IDXD_VDEV_MSIX_TABLE_OFFSET	0xf000
#define IDXD_VDEV_ATS_EXT_CAP_OFFSET	0x100
#define IDXD_VDEV_PRI_EXT_CAP_OFFSET	0x108
#define IDXD_VDEV_PASID_EXT_CAP_OFFSET	0x118
#define IDXD_VDEV_PASID_BITS		20
#define IDXD_VDEV_PRI_MAX_REQS		32

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
#define IDXD_VFIO_BAR2_DESC_SIZE	sizeof(struct dsa_raw_desc)
#define IDXD_VFIO_BAR2_DESC_SLOTS	(PAGE_SIZE / IDXD_VFIO_BAR2_DESC_SIZE)
#define IDXD_VFIO_BAR2_FORWARD_RETRIES	100000U

enum idxd_vfio_bar2_portal {
	IDXD_VFIO_BAR2_UNLIMITED = 0,
	IDXD_VFIO_BAR2_LIMITED,
	IDXD_VFIO_BAR2_MSIX,
	IDXD_VFIO_BAR2_IMS,
};

static bool allow_unsafe_silicon;
module_param(allow_unsafe_silicon, bool, 0644);
MODULE_PARM_DESC(allow_unsafe_silicon,
		 "Allow VFIO portal mapping on DSA silicon affected by unsafe user submission behavior");

static unsigned int bar2_forward_retry_limit = IDXD_VFIO_BAR2_FORWARD_RETRIES;
module_param(bar2_forward_retry_limit, uint, 0644);
MODULE_PARM_DESC(bar2_forward_retry_limit,
		 "Maximum host unlimited portal Retry loops for one trapped BAR2 descriptor, 0 means unlimited");

struct idxd_vfio_wq_pasid {
	u32 saved_wqcfg_pasid;
	ioasid_t guest_pasid;
	ioasid_t host_pasid;
	bool saved_wqcfg_valid;
	bool uses_default_pasid;
	bool programmed;
};

struct idxd_vfio_pasid_entry {
	ioasid_t host_pasid;
};

struct idxd_vfio_guest_pasid_entry {
	ioasid_t guest_pasid;
	ioasid_t host_pasid;
};

struct idxd_vfio_trapped_desc {
	struct dsa_raw_desc desc __aligned(64);
	u64 byte_mask;
};

struct idxd_vfio_bar2_forward_entry {
	struct list_head node;
	struct idxd_vwq *vwq;
	u64 portal_offset;
	struct dsa_raw_desc desc __aligned(64);
};

struct idxd_vfio_bar2_stats {
	atomic64_t writes;
	atomic64_t trapped_unlimited_writes;
	atomic64_t full_writes;
	atomic64_t split_writes;
	atomic64_t assembled_descs;
	atomic64_t trapped_unlimited_descs;
	atomic64_t forward_queued;
	atomic64_t forward_started;
	atomic64_t forward_completed;
	atomic64_t forward_retry_loops;
	atomic64_t forward_retry_exhausted;
	atomic64_t forward_dropped;
	atomic64_t submitted_descs;
	atomic64_t host_unlimited_retries;
	atomic64_t host_unlimited_retry_failures;
	atomic64_t forced_host_unlimited_retries;
	atomic64_t submit_errors;
	atomic64_t rejected_writes;
	atomic64_t pasid_translated;
	atomic64_t pasid_default;
	atomic64_t validation_errors;
	atomic64_t live_wq_errors;
};

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

	struct mutex irq_lock;	/* protects virtual MSI-X eventfds */
	struct eventfd_ctx **msix_trigger;

	struct mutex bar2_lock;	/* protects trapped BAR2 descriptor assembly */
	void __iomem **shared_unlimited_portals;
	struct idxd_vfio_trapped_desc **shared_unlimited_descs;
	spinlock_t bar2_forward_lock;	/* protects BAR2 forward queue */
	struct list_head bar2_forward_list;
	struct work_struct bar2_forward_work;
	atomic_t bar2_force_unlimited_retries;
	struct idxd_vfio_bar2_stats bar2_stats;
	bool bar2_forward_work_active;
	bool bar2_forward_stopping;

	struct iommufd_device *idev;
	u32 iommufd_device_id;
	struct idxd_vfio_wq_pasid *wq_pasid;
	struct xarray pasid_xa;
	struct xarray guest_pasid_xa;
	struct mutex pasid_lock;	/* protects iommufd/PASID attachment state */
	ioasid_t default_host_pasid;
	bool pasid_attached;
};

static void idxd_vfio_msix_signal(struct idxd_vfio_device *vfio_dev,
				  unsigned int vector);
static void idxd_vfio_bar2_forward_work(struct work_struct *work);

static void idxd_vfio_reset_bar2_stats(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_vfio_bar2_stats *stats = &vfio_dev->bar2_stats;

	atomic64_set(&stats->writes, 0);
	atomic64_set(&stats->trapped_unlimited_writes, 0);
	atomic64_set(&stats->full_writes, 0);
	atomic64_set(&stats->split_writes, 0);
	atomic64_set(&stats->assembled_descs, 0);
	atomic64_set(&stats->trapped_unlimited_descs, 0);
	atomic64_set(&stats->forward_queued, 0);
	atomic64_set(&stats->forward_started, 0);
	atomic64_set(&stats->forward_completed, 0);
	atomic64_set(&stats->forward_retry_loops, 0);
	atomic64_set(&stats->forward_retry_exhausted, 0);
	atomic64_set(&stats->forward_dropped, 0);
	atomic64_set(&stats->submitted_descs, 0);
	atomic64_set(&stats->host_unlimited_retries, 0);
	atomic64_set(&stats->host_unlimited_retry_failures, 0);
	atomic64_set(&stats->forced_host_unlimited_retries, 0);
	atomic64_set(&stats->submit_errors, 0);
	atomic64_set(&stats->rejected_writes, 0);
	atomic64_set(&stats->pasid_translated, 0);
	atomic64_set(&stats->pasid_default, 0);
	atomic64_set(&stats->validation_errors, 0);
	atomic64_set(&stats->live_wq_errors, 0);
}

static ssize_t bar2_trap_stats_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct idxd_vfio_device *vfio_dev = dev_get_drvdata(dev);
	struct idxd_vfio_bar2_stats *stats;

	if (!vfio_dev)
		return -ENODEV;

	stats = &vfio_dev->bar2_stats;
	return sysfs_emit(buf,
			  "writes %lld\n"
			  "trapped_unlimited_writes %lld\n"
			  "full_writes %lld\n"
			  "split_writes %lld\n"
			  "assembled_descs %lld\n"
			  "trapped_unlimited_descs %lld\n"
			  "forward_queued %lld\n"
			  "forward_started %lld\n"
			  "forward_completed %lld\n"
			  "forward_retry_loops %lld\n"
			  "forward_retry_exhausted %lld\n"
			  "forward_dropped %lld\n"
			  "submitted_descs %lld\n"
			  "host_unlimited_retries %lld\n"
			  "host_unlimited_retry_failures %lld\n"
			  "forced_host_unlimited_retries %lld\n"
			  "submit_errors %lld\n"
			  "rejected_writes %lld\n"
			  "pasid_translated %lld\n"
			  "pasid_default %lld\n"
			  "validation_errors %lld\n"
			  "live_wq_errors %lld\n",
			  (long long)atomic64_read(&stats->writes),
			  (long long)atomic64_read(&stats->trapped_unlimited_writes),
			  (long long)atomic64_read(&stats->full_writes),
			  (long long)atomic64_read(&stats->split_writes),
			  (long long)atomic64_read(&stats->assembled_descs),
			  (long long)atomic64_read(&stats->trapped_unlimited_descs),
			  (long long)atomic64_read(&stats->forward_queued),
			  (long long)atomic64_read(&stats->forward_started),
			  (long long)atomic64_read(&stats->forward_completed),
			  (long long)atomic64_read(&stats->forward_retry_loops),
			  (long long)atomic64_read(&stats->forward_retry_exhausted),
			  (long long)atomic64_read(&stats->forward_dropped),
			  (long long)atomic64_read(&stats->submitted_descs),
			  (long long)atomic64_read(&stats->host_unlimited_retries),
			  (long long)atomic64_read(&stats->host_unlimited_retry_failures),
			  (long long)atomic64_read(&stats->forced_host_unlimited_retries),
			  (long long)atomic64_read(&stats->submit_errors),
			  (long long)atomic64_read(&stats->rejected_writes),
			  (long long)atomic64_read(&stats->pasid_translated),
			  (long long)atomic64_read(&stats->pasid_default),
			  (long long)atomic64_read(&stats->validation_errors),
			  (long long)atomic64_read(&stats->live_wq_errors));
}
static DEVICE_ATTR_RO(bar2_trap_stats);

static ssize_t bar2_trap_stats_reset_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct idxd_vfio_device *vfio_dev = dev_get_drvdata(dev);
	bool reset;
	int rc;

	if (!vfio_dev)
		return -ENODEV;

	rc = kstrtobool(buf, &reset);
	if (rc)
		return rc;

	if (reset)
		idxd_vfio_reset_bar2_stats(vfio_dev);

	return count;
}
static DEVICE_ATTR_WO(bar2_trap_stats_reset);

static ssize_t bar2_force_unlimited_retries_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct idxd_vfio_device *vfio_dev = dev_get_drvdata(dev);

	if (!vfio_dev)
		return -ENODEV;

	return sysfs_emit(buf, "%d\n",
			  atomic_read(&vfio_dev->bar2_force_unlimited_retries));
}

static ssize_t bar2_force_unlimited_retries_store(struct device *dev,
						  struct device_attribute *attr,
						  const char *buf, size_t count)
{
	struct idxd_vfio_device *vfio_dev = dev_get_drvdata(dev);
	unsigned int retries;
	int rc;

	if (!vfio_dev)
		return -ENODEV;

	rc = kstrtouint(buf, 0, &retries);
	if (rc)
		return rc;

	atomic_set(&vfio_dev->bar2_force_unlimited_retries, retries);
	return count;
}
static DEVICE_ATTR_RW(bar2_force_unlimited_retries);

static int idxd_vfio_create_sysfs(struct idxd_vfio_device *vfio_dev)
{
	struct device *dev = vdev_confdev(vfio_dev->ivdev);
	int rc;

	rc = device_create_file(dev, &dev_attr_bar2_trap_stats);
	if (rc)
		return rc;

	rc = device_create_file(dev, &dev_attr_bar2_trap_stats_reset);
	if (rc)
		goto err_stats;

	rc = device_create_file(dev, &dev_attr_bar2_force_unlimited_retries);
	if (rc)
		goto err_reset;

	return rc;

err_reset:
	device_remove_file(dev, &dev_attr_bar2_trap_stats_reset);
err_stats:
	device_remove_file(dev, &dev_attr_bar2_trap_stats);
	return rc;
}

static void idxd_vfio_remove_sysfs(struct idxd_vfio_device *vfio_dev)
{
	struct device *dev = vdev_confdev(vfio_dev->ivdev);

	device_remove_file(dev, &dev_attr_bar2_force_unlimited_retries);
	device_remove_file(dev, &dev_attr_bar2_trap_stats_reset);
	device_remove_file(dev, &dev_attr_bar2_trap_stats);
}

static unsigned int idxd_vfio_msix_count(struct idxd_vfio_device *vfio_dev)
{
	return vfio_dev->ivdev->num_wqs + 1;
}

static u32 idxd_vfio_msix_pba_offset(struct idxd_vfio_device *vfio_dev)
{
	return ALIGN(IDXD_VDEV_MSIX_TABLE_OFFSET +
		     idxd_vfio_msix_count(vfio_dev) * PCI_MSIX_ENTRY_SIZE, 8);
}

static size_t idxd_vfio_bar2_size(struct idxd_vfio_device *vfio_dev)
{
	size_t size = vfio_dev->ivdev->num_wqs * IDXD_VDEV_PORTALS_PER_WQ *
		      PAGE_SIZE;

	return roundup_pow_of_two(size);
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

static void idxd_vfio_config_writel(struct idxd_vfio_device *vfio_dev,
				    unsigned int pos, u32 val)
{
	vfio_dev->config[pos] = val;
	vfio_dev->config[pos + 1] = val >> 8;
	vfio_dev->config[pos + 2] = val >> 16;
	vfio_dev->config[pos + 3] = val >> 24;
}

static void idxd_vfio_config_write_ext_cap(struct idxd_vfio_device *vfio_dev,
					   unsigned int pos, u16 id,
					   unsigned int next)
{
	idxd_vfio_config_writel(vfio_dev, pos,
				id | (1 << 16) | (next << 20));
}

static void idxd_vfio_init_config(struct idxd_vfio_device *vfio_dev)
{
	struct pci_dev *pdev = vfio_dev->ivdev->idxd->pdev;
	unsigned int msix_count = idxd_vfio_msix_count(vfio_dev);

	memset(vfio_dev->config, 0, sizeof(vfio_dev->config));
	idxd_vfio_config_writew(vfio_dev, PCI_VENDOR_ID, PCI_VENDOR_ID_INTEL);
	idxd_vfio_config_writew(vfio_dev, PCI_DEVICE_ID, pdev->device);
	idxd_vfio_config_writew(vfio_dev, PCI_COMMAND, PCI_COMMAND_MEMORY);
	idxd_vfio_config_writew(vfio_dev, PCI_STATUS, PCI_STATUS_CAP_LIST);
	idxd_vfio_config_writew(vfio_dev, PCI_CLASS_DEVICE,
				PCI_CLASS_ACCELERATOR_PROCESSING);
	idxd_vfio_config_writeb(vfio_dev, PCI_CAPABILITY_LIST,
				IDXD_VDEV_PCIE_CAP_OFFSET);
	idxd_vfio_config_writeb(vfio_dev, PCI_HEADER_TYPE, PCI_HEADER_TYPE_NORMAL);

	idxd_vfio_config_writeb(vfio_dev,
				IDXD_VDEV_PCIE_CAP_OFFSET + PCI_CAP_LIST_ID,
				PCI_CAP_ID_EXP);
	idxd_vfio_config_writeb(vfio_dev,
				IDXD_VDEV_PCIE_CAP_OFFSET + PCI_CAP_LIST_NEXT,
				IDXD_VDEV_MSIX_CAP_OFFSET);
	idxd_vfio_config_writew(vfio_dev,
				IDXD_VDEV_PCIE_CAP_OFFSET + PCI_EXP_FLAGS,
				(PCI_EXP_TYPE_RC_END << 4) | 2);
	idxd_vfio_config_writel(vfio_dev,
				IDXD_VDEV_PCIE_CAP_OFFSET + PCI_EXP_DEVCAP2,
				PCI_EXP_DEVCAP2_EE_PREFIX);

	idxd_vfio_config_writeb(vfio_dev,
				IDXD_VDEV_MSIX_CAP_OFFSET + PCI_CAP_LIST_ID,
				PCI_CAP_ID_MSIX);
	idxd_vfio_config_writeb(vfio_dev,
				IDXD_VDEV_MSIX_CAP_OFFSET + PCI_CAP_LIST_NEXT,
				0);
	idxd_vfio_config_writew(vfio_dev,
				IDXD_VDEV_MSIX_CAP_OFFSET + PCI_MSIX_FLAGS,
				msix_count - 1);
	idxd_vfio_config_writel(vfio_dev,
				IDXD_VDEV_MSIX_CAP_OFFSET + PCI_MSIX_TABLE,
				IDXD_VDEV_MSIX_TABLE_OFFSET |
				VFIO_PCI_BAR0_REGION_INDEX);
	idxd_vfio_config_writel(vfio_dev,
				IDXD_VDEV_MSIX_CAP_OFFSET + PCI_MSIX_PBA,
				idxd_vfio_msix_pba_offset(vfio_dev) |
				VFIO_PCI_BAR0_REGION_INDEX);

	idxd_vfio_config_write_ext_cap(vfio_dev, IDXD_VDEV_ATS_EXT_CAP_OFFSET,
				       PCI_EXT_CAP_ID_ATS,
				       IDXD_VDEV_PRI_EXT_CAP_OFFSET);
	idxd_vfio_config_writew(vfio_dev,
				IDXD_VDEV_ATS_EXT_CAP_OFFSET + PCI_ATS_CAP,
				PCI_ATS_CAP_PAGE_ALIGNED);

	idxd_vfio_config_write_ext_cap(vfio_dev, IDXD_VDEV_PRI_EXT_CAP_OFFSET,
				       PCI_EXT_CAP_ID_PRI,
				       IDXD_VDEV_PASID_EXT_CAP_OFFSET);
	idxd_vfio_config_writew(vfio_dev,
				IDXD_VDEV_PRI_EXT_CAP_OFFSET + PCI_PRI_STATUS,
				PCI_PRI_STATUS_STOPPED |
				PCI_PRI_STATUS_PASID);
	idxd_vfio_config_writel(vfio_dev,
				IDXD_VDEV_PRI_EXT_CAP_OFFSET + PCI_PRI_MAX_REQ,
				IDXD_VDEV_PRI_MAX_REQS);

	idxd_vfio_config_write_ext_cap(vfio_dev,
				       IDXD_VDEV_PASID_EXT_CAP_OFFSET,
				       PCI_EXT_CAP_ID_PASID, 0);
	idxd_vfio_config_writew(vfio_dev,
				IDXD_VDEV_PASID_EXT_CAP_OFFSET + PCI_PASID_CAP,
				PCI_PASID_CAP_EXEC | PCI_PASID_CAP_PRIV |
				(IDXD_VDEV_PASID_BITS << 8));
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
	wqcap.occupancy_int = 0;

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

	/*
	 * Initial interrupt virtualization scope is polling-only. Do not expose
	 * event log or IMS capabilities until virtual IMS is implemented.
	 */
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

	return offsets.bits[idx];
}

static void idxd_vfio_sync_host_wqcfg(struct idxd_vfio_device *vfio_dev,
				      struct idxd_vwq *vwq,
				      union wqcfg *wqcfg)
{
	struct idxd_wq *wq = vwq->wq;

	memcpy(wqcfg, wq->wqcfg, sizeof(*wqcfg));
	wqcfg->wq_size = wq->size;
	wqcfg->wq_thresh = wq->threshold;
	wqcfg->priority = wq->priority;
	wqcfg->bof = test_bit(WQ_FLAG_BLOCK_ON_FAULT, &wq->flags);
	wqcfg->wq_ats_disable = test_bit(WQ_FLAG_ATS_DISABLE, &wq->flags);
	wqcfg->wq_prs_disable = test_bit(WQ_FLAG_PRS_DISABLE, &wq->flags);
	if (wq->max_xfer_bytes)
		wqcfg->max_xfer_shift = ilog2(wq->max_xfer_bytes);
	if (wq->max_batch_size)
		idxd_wqcfg_set_max_batch_shift(wq->idxd->data->type, wqcfg,
						ilog2(wq->max_batch_size));

	wqcfg->pasid = 0;
	wqcfg->pasid_en = 0;
	wqcfg->mode_support = !vwq->shared;
	wqcfg->wq_state = test_bit(vwq->id, vfio_dev->wq_enable_map) ?
		IDXD_WQ_ENABLED : IDXD_WQ_DISABLED;
	wqcfg->mode = vwq->shared ? 0 : 1;
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

	if (vwq->shared)
		idxd_vfio_sync_host_wqcfg(vfio_dev, vwq,
					  &vfio_dev->wqcfg[id]);
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

		idxd_vfio_sync_host_wqcfg(vfio_dev, vwq, wqcfg);
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

static int idxd_vfio_msix_set_vector_signal(struct idxd_vfio_device *vfio_dev,
					    unsigned int vector, s32 fd)
{
	struct eventfd_ctx *trigger = NULL;
	struct eventfd_ctx *old;

	if (vector >= idxd_vfio_msix_count(vfio_dev))
		return -EINVAL;

	if (fd >= 0) {
		trigger = eventfd_ctx_fdget(fd);
		if (IS_ERR(trigger))
			return PTR_ERR(trigger);
	}

	mutex_lock(&vfio_dev->irq_lock);
	old = vfio_dev->msix_trigger[vector];
	vfio_dev->msix_trigger[vector] = trigger;
	mutex_unlock(&vfio_dev->irq_lock);

	if (old)
		eventfd_ctx_put(old);

	return 0;
}

static int idxd_vfio_msix_set_block(struct idxd_vfio_device *vfio_dev,
				    unsigned int start, unsigned int count,
				    s32 *fds)
{
	unsigned int i;
	int rc;

	for (i = 0; i < count; i++) {
		rc = idxd_vfio_msix_set_vector_signal(vfio_dev, start + i,
						      fds ? fds[i] : -1);
		if (rc)
			goto err_unwind;
	}

	return 0;

err_unwind:
	while (i--)
		idxd_vfio_msix_set_vector_signal(vfio_dev, start + i, -1);
	return rc;
}

static void idxd_vfio_msix_disable(struct idxd_vfio_device *vfio_dev)
{
	idxd_vfio_msix_set_block(vfio_dev, 0,
				 idxd_vfio_msix_count(vfio_dev), NULL);
}

static void idxd_vfio_msix_signal(struct idxd_vfio_device *vfio_dev,
				  unsigned int vector)
{
	struct eventfd_ctx *trigger;

	if (vector >= idxd_vfio_msix_count(vfio_dev))
		return;

	mutex_lock(&vfio_dev->irq_lock);
	trigger = vfio_dev->msix_trigger[vector];
	if (trigger)
		eventfd_signal(trigger);
	mutex_unlock(&vfio_dev->irq_lock);
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
	if (cmd.int_req) {
		vfio_dev->intcause |= IDXD_INTC_CMD;
		idxd_vfio_msix_signal(vfio_dev, 0);
	}
}

static bool idxd_vfio_get_default_pasid(struct idxd_vfio_device *vfio_dev,
					ioasid_t *pasid)
{
	bool attached;

	mutex_lock(&vfio_dev->pasid_lock);
	attached = vfio_dev->pasid_attached;
	if (attached && pasid)
		*pasid = vfio_dev->default_host_pasid;
	mutex_unlock(&vfio_dev->pasid_lock);

	return attached;
}

static bool idxd_vfio_host_pasid_attached_locked(struct idxd_vfio_device *vfio_dev,
						 ioasid_t host_pasid)
{
	return xa_load(&vfio_dev->pasid_xa, host_pasid);
}

static bool idxd_vfio_lookup_host_pasid(struct idxd_vfio_device *vfio_dev,
					ioasid_t guest_pasid,
					ioasid_t *host_pasid)
{
	struct idxd_vfio_guest_pasid_entry *guest_entry;
	struct idxd_vfio_pasid_entry *host_entry;
	bool found = false;

	mutex_lock(&vfio_dev->pasid_lock);
	guest_entry = xa_load(&vfio_dev->guest_pasid_xa, guest_pasid);
	if (guest_entry) {
		if (host_pasid)
			*host_pasid = guest_entry->host_pasid;
		found = true;
	} else {
		host_entry = xa_load(&vfio_dev->pasid_xa, guest_pasid);
		if (host_entry) {
			if (host_pasid)
				*host_pasid = host_entry->host_pasid;
			found = true;
		}
	}
	mutex_unlock(&vfio_dev->pasid_lock);

	return found;
}

static void
idxd_vfio_free_guest_pasid_entries_locked(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_vfio_guest_pasid_entry *entry;
	unsigned long index = 0;

	while ((entry = xa_find(&vfio_dev->guest_pasid_xa, &index,
				ULONG_MAX, XA_PRESENT))) {
		xa_erase(&vfio_dev->guest_pasid_xa, index);
		kfree(entry);
		index = 0;
	}
}

static void
idxd_vfio_remove_guest_maps_for_host_locked(struct idxd_vfio_device *vfio_dev,
					    ioasid_t host_pasid)
{
	struct idxd_vfio_guest_pasid_entry *entry;
	unsigned long index = 0;

	while ((entry = xa_find(&vfio_dev->guest_pasid_xa, &index,
				ULONG_MAX, XA_PRESENT))) {
		if (entry->host_pasid == host_pasid) {
			xa_erase(&vfio_dev->guest_pasid_xa, index);
			kfree(entry);
			index = 0;
			continue;
		}
		if (index == ULONG_MAX)
			break;
		index++;
	}
}

static void idxd_vfio_free_pasid_entries_locked(struct idxd_vfio_device *vfio_dev,
						struct iommufd_device *idev)
{
	struct idxd_vfio_pasid_entry *entry;
	unsigned long index = 0;

	while ((entry = xa_find(&vfio_dev->pasid_xa, &index, ULONG_MAX,
				XA_PRESENT))) {
		xa_erase(&vfio_dev->pasid_xa, index);
		if (idev)
			iommufd_device_detach(idev, entry->host_pasid);
		kfree(entry);
		index = 0;
	}
}

static int idxd_vfio_restore_wq_pasid(struct idxd_vfio_device *vfio_dev,
				      unsigned int id)
{
	struct idxd_vfio_wq_pasid *state;
	struct idxd_vwq *vwq;
	union wqcfg saved = {};
	int rc;

	if (!vfio_dev->wq_pasid || id >= vfio_dev->ivdev->num_wqs)
		return 0;

	state = &vfio_dev->wq_pasid[id];
	if (!state->programmed)
		return 0;

	vwq = idxd_vfio_vwq(vfio_dev->ivdev, id);
	if (!vwq)
		return -EINVAL;

	if (WARN_ON(!state->saved_wqcfg_valid))
		return -EINVAL;

	saved.bits[WQCFG_PASID_IDX] = state->saved_wqcfg_pasid;

	mutex_lock(&vwq->wq->wq_lock);
	if (saved.pasid_en)
		rc = idxd_wq_set_pasid(vwq->wq, saved.pasid);
	else
		rc = idxd_wq_disable_pasid(vwq->wq);
	mutex_unlock(&vwq->wq->wq_lock);

	if (rc)
		return rc;

	state->guest_pasid = IOMMU_PASID_INVALID;
	state->host_pasid = IOMMU_PASID_INVALID;
	state->saved_wqcfg_valid = false;
	state->uses_default_pasid = false;
	state->programmed = false;
	return 0;
}

static int idxd_vfio_restore_all_wq_pasids(struct idxd_vfio_device *vfio_dev)
{
	unsigned int i;
	int rc, first = 0;

	for (i = 0; i < vfio_dev->ivdev->num_wqs; i++) {
		rc = idxd_vfio_restore_wq_pasid(vfio_dev, i);
		if (rc && !first)
			first = rc;
	}

	return first;
}

static int idxd_vfio_restore_wqs_for_pasid(struct idxd_vfio_device *vfio_dev,
					   ioasid_t pasid)
{
	unsigned int i;
	int rc, first = 0;

	if (!vfio_dev->wq_pasid)
		return 0;

	for (i = 0; i < vfio_dev->ivdev->num_wqs; i++) {
		if (!vfio_dev->wq_pasid[i].programmed ||
		    vfio_dev->wq_pasid[i].host_pasid != pasid)
			continue;

		rc = idxd_vfio_restore_wq_pasid(vfio_dev, i);
		if (rc) {
			if (!first)
				first = rc;
		} else {
			clear_bit(i, vfio_dev->wq_enable_map);
		}
	}

	return first;
}

static bool idxd_vfio_guest_pasid_in_use(struct idxd_vfio_device *vfio_dev,
					 ioasid_t guest_pasid)
{
	unsigned int i;

	if (!vfio_dev->wq_pasid)
		return false;

	for (i = 0; i < vfio_dev->ivdev->num_wqs; i++) {
		if (vfio_dev->wq_pasid[i].programmed &&
		    !vfio_dev->wq_pasid[i].uses_default_pasid &&
		    vfio_dev->wq_pasid[i].guest_pasid == guest_pasid)
			return true;
	}

	return false;
}

static bool idxd_vfio_default_pasid_in_use(struct idxd_vfio_device *vfio_dev)
{
	unsigned int i;

	if (!vfio_dev->wq_pasid)
		return false;

	for (i = 0; i < vfio_dev->ivdev->num_wqs; i++) {
		if (vfio_dev->wq_pasid[i].programmed &&
		    vfio_dev->wq_pasid[i].uses_default_pasid)
			return true;
	}

	return false;
}

static int idxd_vfio_program_wq_pasid(struct idxd_vfio_device *vfio_dev,
				      struct idxd_vwq *vwq,
				      ioasid_t guest_pasid,
				      ioasid_t host_pasid,
				      bool uses_default)
{
	struct idxd_vfio_wq_pasid *state;
	int rc;

	if (!vfio_dev->wq_pasid || vwq->id >= vfio_dev->ivdev->num_wqs)
		return -EINVAL;

	state = &vfio_dev->wq_pasid[vwq->id];
	if (state->programmed && state->host_pasid == host_pasid &&
	    state->guest_pasid == guest_pasid &&
	    state->uses_default_pasid == uses_default)
		return 0;

	if (state->programmed) {
		rc = idxd_vfio_restore_wq_pasid(vfio_dev, vwq->id);
		if (rc)
			return rc;
	}

	mutex_lock(&vwq->wq->wq_lock);
	state->saved_wqcfg_pasid =
		vwq->wq->wqcfg->bits[WQCFG_PASID_IDX];
	state->saved_wqcfg_valid = true;
	rc = idxd_wq_set_pasid(vwq->wq, host_pasid);
	mutex_unlock(&vwq->wq->wq_lock);
	if (rc) {
		state->saved_wqcfg_valid = false;
		return rc;
	}

	state->guest_pasid = guest_pasid;
	state->host_pasid = host_pasid;
	state->uses_default_pasid = uses_default;
	state->programmed = true;
	return 0;
}

static u8 idxd_vfio_prepare_wq_pasid(struct idxd_vfio_device *vfio_dev,
				     struct idxd_vwq *vwq,
				     const union wqcfg *wqcfg)
{
	ioasid_t host_pasid;
	ioasid_t guest_pasid = IOMMU_PASID_INVALID;
	bool uses_default = false;
	int rc;

	if (wqcfg->pasid_en) {
		guest_pasid = wqcfg->pasid;
		if (!idxd_vfio_lookup_host_pasid(vfio_dev, wqcfg->pasid,
						 &host_pasid))
			return IDXD_CMDSTS_ERR_PASID_INVAL;
	} else if (!idxd_vfio_get_default_pasid(vfio_dev, &host_pasid)) {
		return IDXD_CMDSTS_ERR_PASID_EN;
	} else {
		uses_default = true;
	}

	/*
	 * A WQ shared by multiple VDEVs keeps the host-configured physical
	 * shared-mode PASID state. Exclusive WQs program the resolved host
	 * PASID, either from an explicit guest mapping or the VM default.
	 */
	if (vwq->shared)
		return IDXD_CMDSTS_SUCCESS;

	rc = idxd_vfio_program_wq_pasid(vfio_dev, vwq, guest_pasid,
					host_pasid, uses_default);
	if (rc) {
		dev_warn(vdev_confdev(vfio_dev->ivdev),
			 "failed to program host PASID %u for vWQ%u: %d\n",
			 host_pasid, vwq->id, rc);
		return IDXD_CMDSTS_ERR_PASID_EN;
	}

	return IDXD_CMDSTS_SUCCESS;
}

static u8 idxd_vfio_enable_wq(struct idxd_vfio_device *vfio_dev, u32 operand)
{
	struct idxd_vwq *vwq;
	union wqcfg *wqcfg;
	u8 status;

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
	if (vwq->shared)
		idxd_vfio_sync_host_wqcfg(vfio_dev, vwq, wqcfg);
	if (vwq->shared && wqcfg->mode != 0)
		return IDXD_CMDSTS_ERR_WQ_MODE;
	if (wqcfg->wq_size == 0)
		return IDXD_CMDSTS_ERR_WQ_SIZE;
	if (!wqcfg->mode && wqcfg->wq_thresh == 0)
		return IDXD_CMDSTS_ERR_WQ_SIZE;

	status = idxd_vfio_prepare_wq_pasid(vfio_dev, vwq, wqcfg);
	if (status)
		return status;

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
		int rc;

		if (id >= vfio_dev->ivdev->num_wqs)
			return IDXD_CMDSTS_INVAL_WQIDX;
		clear_bit(id, vfio_dev->wq_enable_map);
		rc = idxd_vfio_restore_wq_pasid(vfio_dev, id);
		if (rc) {
			dev_warn(vdev_confdev(vfio_dev->ivdev),
				 "failed to restore PASID for vWQ%u: %d\n",
				 id, rc);
			return IDXD_CMDSTS_HW_ERR;
		}
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
		int rc;

		if (id >= vfio_dev->ivdev->num_wqs)
			return IDXD_CMDSTS_INVAL_WQIDX;

		vwq = idxd_vfio_vwq(vfio_dev->ivdev, id);
		if (!vwq)
			return IDXD_CMDSTS_INVAL_WQIDX;

		clear_bit(id, vfio_dev->wq_enable_map);
		rc = idxd_vfio_restore_wq_pasid(vfio_dev, id);
		if (rc) {
			dev_warn(vdev_confdev(vfio_dev->ivdev),
				 "failed to restore PASID for vWQ%u: %d\n",
				 id, rc);
			return IDXD_CMDSTS_HW_ERR;
		}
		idxd_vfio_sync_host_wqcfg(vfio_dev, vwq,
					  &vfio_dev->wqcfg[id]);
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
		if (idxd_vfio_restore_all_wq_pasids(vfio_dev))
			status = IDXD_CMDSTS_HW_ERR;
		bitmap_zero(vfio_dev->wq_enable_map, vfio_dev->ivdev->num_wqs);
		break;
	case IDXD_CMD_RESET_DEVICE:
		if (idxd_vfio_restore_all_wq_pasids(vfio_dev))
			status = IDXD_CMDSTS_HW_ERR;
		else
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
	if (vwq->shared) {
		idxd_vfio_sync_host_wqcfg(vfio_dev, vwq, wqcfg);
		return true;
	}
	if (test_bit(id, vfio_dev->wq_enable_map))
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
		if (pos == IDXD_GENCFG_OFFSET) {
			union gencfg_reg gencfg = { .bits = val };

			gencfg.user_int_en = 0;
			gencfg.evl_en = 0;
			vfio_dev->gencfg = gencfg.bits;
		} else if (pos == IDXD_GENCTRL_OFFSET) {
			vfio_dev->genctrl = 0;
		} else if (pos == IDXD_INTCAUSE_OFFSET) {
			vfio_dev->intcause &= ~val;
		} else if (pos == IDXD_CMD_OFFSET) {
			idxd_vfio_exec_cmd(vfio_dev, val);
		} else if (pos == IDXD_EVLSTATUS_OFFSET) {
			vfio_dev->evlstatus = val;
		}
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
			      enum idxd_vfio_bar2_portal *portal,
			      u64 *portal_offset)
{
	unsigned int vportal, vwq_id;

	if (pos < 0 || pos >= idxd_vfio_bar2_size(vfio_dev))
		return -EINVAL;

	vportal = pos >> PAGE_SHIFT;
	vwq_id = vportal / IDXD_VDEV_PORTALS_PER_WQ;
	*vwq = idxd_vfio_vwq(vfio_dev->ivdev, vwq_id);
	if (!*vwq)
		return -EINVAL;

	*portal = vportal % IDXD_VDEV_PORTALS_PER_WQ;
	*portal_offset = offset_in_page(pos);

	return 0;
}

static bool idxd_vfio_bar2_mmap_prot(struct idxd_vwq *vwq,
				     enum idxd_vfio_bar2_portal portal,
				     enum idxd_portal_prot *prot)
{
	switch (portal) {
	case IDXD_VFIO_BAR2_LIMITED:
		*prot = IDXD_PORTAL_LIMITED;
		return true;
	case IDXD_VFIO_BAR2_UNLIMITED:
		if (vwq->shared)
			return false;
		*prot = IDXD_PORTAL_UNLIMITED;
		return true;
	case IDXD_VFIO_BAR2_MSIX:
	case IDXD_VFIO_BAR2_IMS:
	default:
		return false;
	}
}

static bool idxd_vfio_portal_access_allowed(struct idxd_wq *wq)
{
	return wq->idxd->user_submission_safe || allow_unsafe_silicon ||
	       capable(CAP_SYS_RAWIO);
}

static int idxd_vfio_validate_trapped_desc(struct idxd_wq *wq,
					   const struct dsa_raw_desc *raw)
{
	struct idxd_dev *idxd_dev = &wq->idxd->idxd_dev;
	const struct dsa_hw_desc *desc = (const struct dsa_hw_desc *)raw;

	if (!is_dsa_dev(idxd_dev))
		return 0;

	if (desc->completion_addr &&
	    !IS_ALIGNED(desc->completion_addr, wq->idxd->data->align))
		return -EINVAL;

	if (desc->opcode == DSA_OPCODE_BATCH &&
	    wq->idxd->hw.version == DEVICE_VERSION_1 &&
	    !wq->idxd->user_submission_safe && !allow_unsafe_silicon)
		return -EINVAL;

	return 0;
}

static int idxd_vfio_get_live_wq(struct idxd_wq *wq)
{
	if (wq->idxd->state != IDXD_DEV_ENABLED || wq->state != IDXD_WQ_ENABLED)
		return -EIO;

	if (!percpu_ref_tryget_live(&wq->wq_active)) {
		wait_for_completion(&wq->wq_resurrect);
		if (!percpu_ref_tryget_live(&wq->wq_active))
			return -ENXIO;
	}

	if (wq->idxd->state != IDXD_DEV_ENABLED || wq->state != IDXD_WQ_ENABLED) {
		percpu_ref_put(&wq->wq_active);
		return -EIO;
	}

	return 0;
}

static int idxd_vfio_prepare_shared_unlimited(struct idxd_vfio_device *vfio_dev,
					      struct idxd_vwq *vwq,
					      struct dsa_raw_desc *desc)
{
	struct idxd_wq *wq = vwq->wq;
	struct dsa_hw_desc *hw = (struct dsa_hw_desc *)desc;
	struct idxd_vfio_bar2_stats *stats = &vfio_dev->bar2_stats;
	ioasid_t host_pasid;
	int rc;

	if (is_dsa_dev(&wq->idxd->idxd_dev)) {
		if (hw->pasid) {
			if (!idxd_vfio_lookup_host_pasid(vfio_dev, hw->pasid,
							 &host_pasid)) {
				atomic64_inc(&stats->submit_errors);
				return -EACCES;
			}
			atomic64_inc(&stats->pasid_translated);
		} else if (idxd_vfio_get_default_pasid(vfio_dev,
						       &host_pasid)) {
			atomic64_inc(&stats->pasid_default);
		} else {
			atomic64_inc(&stats->submit_errors);
			return -EACCES;
		}
		hw->pasid = host_pasid;
	} else if (!idxd_vfio_get_default_pasid(vfio_dev, &host_pasid)) {
		atomic64_inc(&stats->submit_errors);
		return -EACCES;
	} else {
		atomic64_inc(&stats->pasid_default);
	}

	rc = idxd_vfio_validate_trapped_desc(wq, desc);
	if (rc) {
		atomic64_inc(&stats->validation_errors);
		atomic64_inc(&stats->submit_errors);
		return rc;
	}

	rc = idxd_vfio_get_live_wq(wq);
	if (rc) {
		atomic64_inc(&stats->live_wq_errors);
		atomic64_inc(&stats->submit_errors);
		return rc;
	}

	return 0;
}

static bool idxd_vfio_consume_forced_unlimited_retry(struct idxd_vfio_device *vfio_dev)
{
	int retries;

	retries = atomic_dec_if_positive(&vfio_dev->bar2_force_unlimited_retries);
	if (retries < 0)
		return false;

	atomic64_inc(&vfio_dev->bar2_stats.forced_host_unlimited_retries);
	return true;
}

static int
idxd_vfio_forward_shared_unlimited(struct idxd_vfio_device *vfio_dev,
				   struct idxd_vfio_bar2_forward_entry *entry)
{
	struct idxd_vwq *vwq = entry->vwq;
	struct idxd_wq *wq = vwq->wq;
	void __iomem *portal = vfio_dev->shared_unlimited_portals[vwq->id];
	struct idxd_vfio_bar2_stats *stats = &vfio_dev->bar2_stats;
	unsigned int retry_loops = 0;
	int rc;

	atomic64_inc(&stats->forward_started);
	if (!portal) {
		atomic64_inc(&stats->submit_errors);
		atomic64_inc(&stats->forward_dropped);
		return -ENODEV;
	}

	/*
	 * The guest descriptor was copied into normal memory. Flush it before
	 * ringing the host-owned unlimited portal. Shared WQs use ENQCMDS so
	 * the host still observes Retry instead of silently dropping a forwarded
	 * descriptor.
	 */
	wmb();
	for (;;) {
		unsigned int retries = 0;

		if (idxd_vfio_consume_forced_unlimited_retry(vfio_dev)) {
			rc = -EAGAIN;
			retries = 1;
		} else {
			rc = idxd_enqcmds_with_retry_count(wq,
							   portal + entry->portal_offset,
							   &entry->desc,
							   &retries);
		}

		if (retries)
			atomic64_add(retries, &stats->host_unlimited_retries);
		if (!rc) {
			atomic64_inc(&stats->submitted_descs);
			atomic64_inc(&stats->forward_completed);
			return 0;
		}

		if (rc != -EAGAIN) {
			atomic64_inc(&stats->submit_errors);
			atomic64_inc(&stats->forward_dropped);
			return rc;
		}

		retry_loops++;
		atomic64_inc(&stats->forward_retry_loops);
		if (bar2_forward_retry_limit &&
		    retry_loops >= bar2_forward_retry_limit) {
			atomic64_inc(&stats->host_unlimited_retry_failures);
			atomic64_inc(&stats->forward_retry_exhausted);
			atomic64_inc(&stats->forward_dropped);
			atomic64_inc(&stats->submit_errors);
			return -EAGAIN;
		}

		if (retry_loops < 16)
			cpu_relax();
		else
			usleep_range(10, 50);
		cond_resched();
	}
}

static int idxd_vfio_queue_shared_unlimited(struct idxd_vfio_device *vfio_dev,
					    struct idxd_vwq *vwq,
					    struct dsa_raw_desc *desc,
					    u64 portal_offset)
{
	struct idxd_vfio_bar2_forward_entry *entry;
	bool queue = false;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry) {
		percpu_ref_put(&vwq->wq->wq_active);
		atomic64_inc(&vfio_dev->bar2_stats.submit_errors);
		return -ENOMEM;
	}

	entry->vwq = vwq;
	entry->portal_offset = portal_offset;
	memcpy(&entry->desc, desc, sizeof(entry->desc));
	INIT_LIST_HEAD(&entry->node);

	spin_lock(&vfio_dev->bar2_forward_lock);
	if (vfio_dev->bar2_forward_stopping) {
		spin_unlock(&vfio_dev->bar2_forward_lock);
		percpu_ref_put(&vwq->wq->wq_active);
		kfree(entry);
		atomic64_inc(&vfio_dev->bar2_stats.submit_errors);
		return -ENODEV;
	}

	list_add_tail(&entry->node, &vfio_dev->bar2_forward_list);
	atomic64_inc(&vfio_dev->bar2_stats.forward_queued);
	if (!vfio_dev->bar2_forward_work_active) {
		vfio_dev->bar2_forward_work_active = true;
		queue = true;
	}
	spin_unlock(&vfio_dev->bar2_forward_lock);

	if (queue)
		queue_work(system_wq, &vfio_dev->bar2_forward_work);
	return 0;
}

static void idxd_vfio_bar2_forward_work(struct work_struct *work)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(work, struct idxd_vfio_device, bar2_forward_work);
	struct idxd_vfio_bar2_forward_entry *entry;

	for (;;) {
		spin_lock(&vfio_dev->bar2_forward_lock);
		entry = list_first_entry_or_null(&vfio_dev->bar2_forward_list,
						 struct idxd_vfio_bar2_forward_entry,
						 node);
		if (entry) {
			list_del_init(&entry->node);
		} else {
			vfio_dev->bar2_forward_work_active = false;
		}
		spin_unlock(&vfio_dev->bar2_forward_lock);

		if (!entry)
			break;

		idxd_vfio_forward_shared_unlimited(vfio_dev, entry);
		percpu_ref_put(&entry->vwq->wq->wq_active);
		kfree(entry);
	}
}

static void idxd_vfio_drop_bar2_forward_list(struct idxd_vfio_device *vfio_dev,
					     struct list_head *list)
{
	struct idxd_vfio_bar2_forward_entry *entry, *tmp;

	list_for_each_entry_safe(entry, tmp, list, node) {
		list_del_init(&entry->node);
		percpu_ref_put(&entry->vwq->wq->wq_active);
		kfree(entry);
		atomic64_inc(&vfio_dev->bar2_stats.forward_dropped);
	}
}

static void idxd_vfio_flush_bar2_forward(struct idxd_vfio_device *vfio_dev)
{
	flush_work(&vfio_dev->bar2_forward_work);
}

static void idxd_vfio_stop_bar2_forward(struct idxd_vfio_device *vfio_dev)
{
	LIST_HEAD(pending);

	spin_lock(&vfio_dev->bar2_forward_lock);
	vfio_dev->bar2_forward_stopping = true;
	list_splice_init(&vfio_dev->bar2_forward_list, &pending);
	spin_unlock(&vfio_dev->bar2_forward_lock);

	flush_work(&vfio_dev->bar2_forward_work);
	spin_lock(&vfio_dev->bar2_forward_lock);
	vfio_dev->bar2_forward_work_active = false;
	spin_unlock(&vfio_dev->bar2_forward_lock);
	idxd_vfio_drop_bar2_forward_list(vfio_dev, &pending);
}

static void idxd_vfio_reset_trapped_desc(struct idxd_vfio_device *vfio_dev,
					 struct idxd_vwq *vwq,
					 u64 portal_offset)
{
	struct idxd_vfio_trapped_desc *slots;
	struct idxd_vfio_trapped_desc *slot;

	if (!vfio_dev->shared_unlimited_descs ||
	    !vfio_dev->shared_unlimited_descs[vwq->id])
		return;

	slots = vfio_dev->shared_unlimited_descs[vwq->id];
	slot = &slots[portal_offset / IDXD_VFIO_BAR2_DESC_SIZE];
	memset(slot, 0, sizeof(*slot));
}

static int idxd_vfio_buffer_trapped_desc(struct idxd_vfio_device *vfio_dev,
					 struct idxd_vwq *vwq,
					 const u8 *chunk, size_t count,
					 u64 portal_offset,
					 struct dsa_raw_desc *desc)
{
	struct idxd_vfio_trapped_desc *slots, *slot;
	u64 desc_offset = portal_offset % IDXD_VFIO_BAR2_DESC_SIZE;
	u64 byte_mask;
	bool complete;

	if (!vfio_dev->shared_unlimited_descs)
		return -ENODEV;

	slots = vfio_dev->shared_unlimited_descs[vwq->id];
	if (!slots)
		return -ENODEV;

	if (!count || count > sizeof(u64) ||
	    desc_offset + count > IDXD_VFIO_BAR2_DESC_SIZE)
		return -EINVAL;

	byte_mask = ((1ULL << count) - 1) << desc_offset;

	mutex_lock(&vfio_dev->bar2_lock);
	slot = &slots[portal_offset / IDXD_VFIO_BAR2_DESC_SIZE];
	if (!desc_offset && slot->byte_mask)
		memset(slot, 0, sizeof(*slot));

	memcpy((u8 *)&slot->desc + desc_offset, chunk, count);
	slot->byte_mask |= byte_mask;

	complete = slot->byte_mask == ~0ULL;
	if (complete) {
		memcpy(desc, &slot->desc, sizeof(*desc));
		memset(slot, 0, sizeof(*slot));
	}
	mutex_unlock(&vfio_dev->bar2_lock);

	return complete;
}

static ssize_t idxd_vfio_bar2_write(struct idxd_vfio_device *vfio_dev,
				    const char __user *buf, size_t count,
				    loff_t pos)
{
	struct idxd_vwq *vwq;
	enum idxd_vfio_bar2_portal portal;
	struct dsa_raw_desc desc __aligned(64);
	u8 chunk[sizeof(u64)];
	u64 portal_offset;
	int rc;

	rc = idxd_vfio_bar2_pos(vfio_dev, pos, &vwq, &portal, &portal_offset);
	if (rc) {
		atomic64_inc(&vfio_dev->bar2_stats.rejected_writes);
		return rc;
	}

	if (portal != IDXD_VFIO_BAR2_UNLIMITED || !vwq->shared) {
		atomic64_inc(&vfio_dev->bar2_stats.rejected_writes);
		return -EINVAL;
	}

	if (!idxd_vfio_get_default_pasid(vfio_dev, NULL)) {
		atomic64_inc(&vfio_dev->bar2_stats.rejected_writes);
		return -EACCES;
	}

	if (!idxd_vfio_portal_access_allowed(vwq->wq)) {
		atomic64_inc(&vfio_dev->bar2_stats.rejected_writes);
		return -EPERM;
	}

	if (portal_offset + count > PAGE_SIZE) {
		atomic64_inc(&vfio_dev->bar2_stats.rejected_writes);
		return -EINVAL;
	}

	if (count == sizeof(desc)) {
		if (!IS_ALIGNED(portal_offset, sizeof(desc))) {
			atomic64_inc(&vfio_dev->bar2_stats.rejected_writes);
			return -EINVAL;
		}

		if (copy_from_user(&desc, buf, sizeof(desc)))
			return -EFAULT;

		atomic64_inc(&vfio_dev->bar2_stats.writes);
		atomic64_inc(&vfio_dev->bar2_stats.trapped_unlimited_writes);
		atomic64_inc(&vfio_dev->bar2_stats.full_writes);

		mutex_lock(&vfio_dev->bar2_lock);
		idxd_vfio_reset_trapped_desc(vfio_dev, vwq, portal_offset);
		mutex_unlock(&vfio_dev->bar2_lock);
	} else {
		if (count > sizeof(chunk)) {
			atomic64_inc(&vfio_dev->bar2_stats.rejected_writes);
			return -EINVAL;
		}
		if (copy_from_user(chunk, buf, count))
			return -EFAULT;

		atomic64_inc(&vfio_dev->bar2_stats.writes);
		atomic64_inc(&vfio_dev->bar2_stats.trapped_unlimited_writes);
		atomic64_inc(&vfio_dev->bar2_stats.split_writes);

		rc = idxd_vfio_buffer_trapped_desc(vfio_dev, vwq, chunk,
						   count, portal_offset, &desc);
		if (rc < 0) {
			atomic64_inc(&vfio_dev->bar2_stats.rejected_writes);
			return rc;
		}
		if (!rc)
			return count;
		atomic64_inc(&vfio_dev->bar2_stats.assembled_descs);
	}

	atomic64_inc(&vfio_dev->bar2_stats.trapped_unlimited_descs);
	rc = idxd_vfio_prepare_shared_unlimited(vfio_dev, vwq, &desc);
	if (rc)
		return rc;

	rc = idxd_vfio_queue_shared_unlimited(vfio_dev, vwq, &desc,
					      ALIGN_DOWN(portal_offset,
							 IDXD_VFIO_BAR2_DESC_SIZE));
	if (rc)
		return rc;

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
			(vwq->id * IDXD_VDEV_PORTALS_PER_WQ +
			 IDXD_VFIO_BAR2_LIMITED) * PAGE_SIZE;
		sparse->areas[i].size = PAGE_SIZE;
		i++;

		if (!vwq->shared) {
			sparse->areas[i].offset =
				(vwq->id * IDXD_VDEV_PORTALS_PER_WQ +
				 IDXD_VFIO_BAR2_UNLIMITED) *
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

static int idxd_vfio_get_irq_info(struct idxd_vfio_device *vfio_dev,
				  unsigned long arg)
{
	unsigned long minsz = offsetofend(struct vfio_irq_info, count);
	struct vfio_irq_info info;

	if (copy_from_user(&info, (void __user *)arg, minsz))
		return -EFAULT;
	if (info.argsz < minsz || info.index >= VFIO_PCI_NUM_IRQS)
		return -EINVAL;

	info.flags = 0;
	info.count = 0;
	if (info.index == VFIO_PCI_MSIX_IRQ_INDEX) {
		info.flags = VFIO_IRQ_INFO_EVENTFD | VFIO_IRQ_INFO_NORESIZE;
		info.count = idxd_vfio_msix_count(vfio_dev);
	}

	return copy_to_user((void __user *)arg, &info, minsz) ? -EFAULT : 0;
}

static int idxd_vfio_set_msix_trigger(struct idxd_vfio_device *vfio_dev,
				      unsigned int start, unsigned int count,
				      u32 flags, void *data)
{
	unsigned int i;

	if (!count && (flags & VFIO_IRQ_SET_DATA_NONE)) {
		idxd_vfio_msix_disable(vfio_dev);
		return 0;
	}

	if (flags & VFIO_IRQ_SET_DATA_EVENTFD)
		return idxd_vfio_msix_set_block(vfio_dev, start, count, data);

	for (i = start; i < start + count; i++) {
		if (flags & VFIO_IRQ_SET_DATA_NONE) {
			idxd_vfio_msix_signal(vfio_dev, i);
		} else if (flags & VFIO_IRQ_SET_DATA_BOOL) {
			u8 *bools = data;

			if (bools[i - start])
				idxd_vfio_msix_signal(vfio_dev, i);
		}
	}

	return 0;
}

static int idxd_vfio_set_irqs(struct idxd_vfio_device *vfio_dev,
			      unsigned long arg)
{
	unsigned long minsz = offsetofend(struct vfio_irq_set, count);
	struct vfio_irq_set __user *uarg = (void __user *)arg;
	struct vfio_irq_set hdr;
	size_t data_size = 0;
	u8 *data = NULL;
	int rc;

	if (copy_from_user(&hdr, uarg, minsz))
		return -EFAULT;
	if (hdr.index != VFIO_PCI_MSIX_IRQ_INDEX)
		return -ENOTTY;

	rc = vfio_set_irqs_validate_and_prepare(&hdr,
						idxd_vfio_msix_count(vfio_dev),
						VFIO_PCI_NUM_IRQS, &data_size);
	if (rc)
		return rc;

	if ((hdr.flags & VFIO_IRQ_SET_ACTION_TYPE_MASK) !=
	    VFIO_IRQ_SET_ACTION_TRIGGER)
		return -ENOTTY;

	if (data_size) {
		data = memdup_user(uarg->data, data_size);
		if (IS_ERR(data))
			return PTR_ERR(data);
	}

	rc = idxd_vfio_set_msix_trigger(vfio_dev, hdr.start, hdr.count,
					hdr.flags, data);
	kfree(data);
	return rc;
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
	enum idxd_vfio_bar2_portal portal;
	enum idxd_portal_prot prot;
	u64 portal_offset;
	phys_addr_t paddr;
	int rc;

	if (index != VFIO_PCI_BAR2_REGION_INDEX)
		return -EINVAL;

	if (!(vma->vm_flags & VM_SHARED) || vma->vm_end < vma->vm_start)
		return -EINVAL;
	if ((vma->vm_end - vma->vm_start) != PAGE_SIZE)
		return -EINVAL;

	rc = idxd_vfio_bar2_pos(vfio_dev, pos, &vwq, &portal, &portal_offset);
	if (rc)
		return rc;

	if (portal_offset || !idxd_vfio_bar2_mmap_prot(vwq, portal, &prot))
		return -EINVAL;

	if (!idxd_vfio_get_default_pasid(vfio_dev, NULL))
		return -EACCES;

	if (!idxd_vfio_portal_access_allowed(vwq->wq))
		return -EPERM;

	paddr = pci_resource_start(vwq->wq->idxd->pdev, IDXD_WQ_BAR);
	paddr += idxd_get_wq_portal_full_offset(vwq->wq->id, prot);

	vm_flags_set(vma, VM_ALLOW_ANY_UNCACHED | VM_IO | VM_PFNMAP |
		     VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return io_remap_pfn_range(vma, vma->vm_start, paddr >> PAGE_SHIFT,
				  PAGE_SIZE, vma->vm_page_prot);
}

static int idxd_vfio_bind_iommufd(struct vfio_device *vdev,
				  struct iommufd_ctx *ictx,
				  u32 *out_device_id)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	struct device *dev = &vfio_dev->ivdev->idxd->pdev->dev;
	struct iommufd_device *idev;

	idev = iommufd_device_pasid_bind(ictx, dev, out_device_id);
	if (IS_ERR(idev))
		return PTR_ERR(idev);

	mutex_lock(&vfio_dev->pasid_lock);
	vfio_dev->idev = idev;
	vfio_dev->iommufd_device_id = *out_device_id;
	vfio_dev->default_host_pasid = IOMMU_PASID_INVALID;
	vfio_dev->pasid_attached = false;
	mutex_unlock(&vfio_dev->pasid_lock);

	return 0;
}

static void idxd_vfio_unbind_iommufd(struct vfio_device *vdev)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	struct iommufd_device *idev;
	int rc;

	mutex_lock(&vfio_dev->pasid_lock);
	idev = vfio_dev->idev;
	vfio_dev->idev = NULL;
	vfio_dev->iommufd_device_id = 0;
	vfio_dev->default_host_pasid = IOMMU_PASID_INVALID;
	vfio_dev->pasid_attached = false;
	mutex_unlock(&vfio_dev->pasid_lock);

	mutex_lock(&vfio_dev->bar0_lock);
	rc = idxd_vfio_restore_all_wq_pasids(vfio_dev);
	if (rc)
		dev_warn(vdev_confdev(vfio_dev->ivdev),
			 "failed to restore all WQ PASIDs during unbind: %d\n",
			 rc);
	bitmap_zero(vfio_dev->wq_enable_map, vfio_dev->ivdev->num_wqs);
	mutex_unlock(&vfio_dev->bar0_lock);

	if (idev) {
		mutex_lock(&vfio_dev->pasid_lock);
		idxd_vfio_free_guest_pasid_entries_locked(vfio_dev);
		idxd_vfio_free_pasid_entries_locked(vfio_dev, idev);
		mutex_unlock(&vfio_dev->pasid_lock);
		iommufd_device_unbind(idev);
	}
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
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	struct idxd_vfio_pasid_entry *entry, *old;
	bool inserted = false;
	int rc;

	if (pasid == IOMMU_NO_PASID || pasid == IOMMU_PASID_INVALID)
		return -EINVAL;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;
	entry->host_pasid = pasid;

	mutex_lock(&vfio_dev->pasid_lock);
	if (!vfio_dev->idev) {
		rc = -EINVAL;
	} else {
		old = xa_load(&vfio_dev->pasid_xa, pasid);
		if (old) {
			rc = iommufd_device_replace(vfio_dev->idev, pasid,
						    pt_id);
		} else {
			rc = iommufd_device_attach(vfio_dev->idev, pasid,
						   pt_id);
			if (!rc) {
				rc = xa_insert(&vfio_dev->pasid_xa, pasid,
					       entry, GFP_KERNEL);
				if (rc)
					iommufd_device_detach(vfio_dev->idev,
							      pasid);
				else
					inserted = true;
			}
		}

		if (!rc && inserted && !vfio_dev->pasid_attached) {
			vfio_dev->default_host_pasid = pasid;
			vfio_dev->pasid_attached = true;
		}
	}
	mutex_unlock(&vfio_dev->pasid_lock);

	if (!inserted)
		kfree(entry);

	return rc;
}

static void idxd_vfio_pasid_detach_ioas(struct vfio_device *vdev, u32 pasid)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	struct idxd_vfio_pasid_entry *entry;
	struct iommufd_device *idev;
	int rc;

	mutex_lock(&vfio_dev->pasid_lock);
	idev = vfio_dev->idev;
	entry = xa_erase(&vfio_dev->pasid_xa, pasid);
	if (entry && vfio_dev->pasid_attached &&
	    vfio_dev->default_host_pasid == entry->host_pasid) {
		vfio_dev->default_host_pasid = IOMMU_PASID_INVALID;
		vfio_dev->pasid_attached = false;
	}
	if (entry)
		idxd_vfio_remove_guest_maps_for_host_locked(vfio_dev,
							    entry->host_pasid);
	mutex_unlock(&vfio_dev->pasid_lock);

	if (!entry || !idev) {
		kfree(entry);
		return;
	}

	mutex_lock(&vfio_dev->bar0_lock);
	rc = idxd_vfio_restore_wqs_for_pasid(vfio_dev, entry->host_pasid);
	if (rc)
		dev_warn(vdev_confdev(vfio_dev->ivdev),
			 "failed to restore WQ PASIDs for PASID %u during detach: %d\n",
			 entry->host_pasid, rc);
	mutex_unlock(&vfio_dev->bar0_lock);

	iommufd_device_detach(idev, entry->host_pasid);
	kfree(entry);
}

static bool idxd_vfio_valid_pasid(ioasid_t pasid)
{
	return pasid != IOMMU_NO_PASID && pasid != IOMMU_PASID_INVALID;
}

static int idxd_vfio_set_guest_pasid_map(struct idxd_vfio_device *vfio_dev,
					 ioasid_t guest_pasid,
					 ioasid_t host_pasid)
{
	struct idxd_vfio_guest_pasid_entry *entry, *old;
	int rc = 0;

	if (!idxd_vfio_valid_pasid(guest_pasid) ||
	    !idxd_vfio_valid_pasid(host_pasid))
		return -EINVAL;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;
	entry->guest_pasid = guest_pasid;
	entry->host_pasid = host_pasid;

	mutex_lock(&vfio_dev->bar0_lock);
	if (idxd_vfio_guest_pasid_in_use(vfio_dev, guest_pasid)) {
		rc = -EBUSY;
		goto out_bar0;
	}

	mutex_lock(&vfio_dev->pasid_lock);
	if (!vfio_dev->idev ||
	    !idxd_vfio_host_pasid_attached_locked(vfio_dev, host_pasid)) {
		rc = -ENOENT;
	} else {
		old = xa_store(&vfio_dev->guest_pasid_xa, guest_pasid, entry,
			       GFP_KERNEL);
		if (xa_is_err(old)) {
			rc = xa_err(old);
		} else {
			kfree(old);
			entry = NULL;
		}
	}
	mutex_unlock(&vfio_dev->pasid_lock);

out_bar0:
	mutex_unlock(&vfio_dev->bar0_lock);
	kfree(entry);
	return rc;
}

static int idxd_vfio_unmap_guest_pasid(struct idxd_vfio_device *vfio_dev,
				       ioasid_t guest_pasid)
{
	struct idxd_vfio_guest_pasid_entry *entry;
	int rc = 0;

	if (!idxd_vfio_valid_pasid(guest_pasid))
		return -EINVAL;

	mutex_lock(&vfio_dev->bar0_lock);
	if (idxd_vfio_guest_pasid_in_use(vfio_dev, guest_pasid)) {
		rc = -EBUSY;
		goto out_unlock_bar0;
	}

	mutex_lock(&vfio_dev->pasid_lock);
	entry = xa_erase(&vfio_dev->guest_pasid_xa, guest_pasid);
	mutex_unlock(&vfio_dev->pasid_lock);
	if (!entry)
		rc = -ENOENT;
	kfree(entry);

out_unlock_bar0:
	mutex_unlock(&vfio_dev->bar0_lock);
	return rc;
}

static int idxd_vfio_set_default_pasid(struct idxd_vfio_device *vfio_dev,
				       ioasid_t host_pasid)
{
	int rc = 0;

	if (!idxd_vfio_valid_pasid(host_pasid))
		return -EINVAL;

	mutex_lock(&vfio_dev->bar0_lock);
	mutex_lock(&vfio_dev->pasid_lock);
	if (!vfio_dev->idev ||
	    !idxd_vfio_host_pasid_attached_locked(vfio_dev, host_pasid)) {
		rc = -ENOENT;
	} else if (vfio_dev->pasid_attached &&
		   vfio_dev->default_host_pasid == host_pasid) {
		rc = 0;
	} else if (idxd_vfio_default_pasid_in_use(vfio_dev)) {
		rc = -EBUSY;
	} else {
		vfio_dev->default_host_pasid = host_pasid;
		vfio_dev->pasid_attached = true;
	}
	mutex_unlock(&vfio_dev->pasid_lock);
	mutex_unlock(&vfio_dev->bar0_lock);

	return rc;
}

static int idxd_vfio_clear_default_pasid(struct idxd_vfio_device *vfio_dev)
{
	int rc = 0;

	mutex_lock(&vfio_dev->bar0_lock);
	if (idxd_vfio_default_pasid_in_use(vfio_dev)) {
		rc = -EBUSY;
		goto out_unlock;
	}

	mutex_lock(&vfio_dev->pasid_lock);
	vfio_dev->default_host_pasid = IOMMU_PASID_INVALID;
	vfio_dev->pasid_attached = false;
	mutex_unlock(&vfio_dev->pasid_lock);

out_unlock:
	mutex_unlock(&vfio_dev->bar0_lock);
	return rc;
}

static int idxd_vfio_pasid_feature(struct vfio_device *vdev, u32 flags,
				   void __user *arg, size_t argsz)
{
	size_t minsz =
		offsetofend(struct vfio_device_feature_idxd_siov_pasid,
			    __reserved);
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	struct vfio_device_feature_idxd_siov_pasid ctrl;
	ioasid_t host_pasid;
	int rc;

	rc = vfio_check_feature(flags, argsz,
				VFIO_DEVICE_FEATURE_GET |
				VFIO_DEVICE_FEATURE_SET,
				minsz);
	if (rc != 1)
		return rc;

	if (copy_from_user(&ctrl, arg, minsz))
		return -EFAULT;
	if (ctrl.__reserved)
		return -EINVAL;

	if (flags & VFIO_DEVICE_FEATURE_SET) {
		switch (ctrl.op) {
		case VFIO_DEVICE_FEATURE_IDXD_SIOV_PASID_MAP:
			return idxd_vfio_set_guest_pasid_map(vfio_dev,
							     ctrl.guest_pasid,
							     ctrl.host_pasid);
		case VFIO_DEVICE_FEATURE_IDXD_SIOV_PASID_UNMAP:
			return idxd_vfio_unmap_guest_pasid(vfio_dev,
							   ctrl.guest_pasid);
		case VFIO_DEVICE_FEATURE_IDXD_SIOV_PASID_SET_DEFAULT:
			return idxd_vfio_set_default_pasid(vfio_dev,
							   ctrl.host_pasid);
		case VFIO_DEVICE_FEATURE_IDXD_SIOV_PASID_CLEAR_DEFAULT:
			return idxd_vfio_clear_default_pasid(vfio_dev);
		default:
			return -EINVAL;
		}
	}

	switch (ctrl.op) {
	case VFIO_DEVICE_FEATURE_IDXD_SIOV_PASID_MAP:
		if (!idxd_vfio_valid_pasid(ctrl.guest_pasid))
			return -EINVAL;
		if (!idxd_vfio_lookup_host_pasid(vfio_dev, ctrl.guest_pasid,
						 &host_pasid))
			return -ENOENT;
		ctrl.host_pasid = host_pasid;
		break;
	case VFIO_DEVICE_FEATURE_IDXD_SIOV_PASID_GET_DEFAULT:
		if (!idxd_vfio_get_default_pasid(vfio_dev, &host_pasid))
			return -ENOENT;
		ctrl.host_pasid = host_pasid;
		break;
	default:
		return -EINVAL;
	}

	return copy_to_user(arg, &ctrl, minsz) ? -EFAULT : 0;
}

static int idxd_vfio_ioctl_feature(struct vfio_device *vdev, u32 flags,
				   void __user *arg, size_t argsz)
{
	switch (flags & VFIO_DEVICE_FEATURE_MASK) {
	case VFIO_DEVICE_FEATURE_IDXD_SIOV_PASID:
		return idxd_vfio_pasid_feature(vdev, flags, arg, argsz);
	default:
		return -ENOTTY;
	}
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
		info.num_irqs = VFIO_PCI_NUM_IRQS;
		info.cap_offset = 0;
		info.pad = 0;

		return copy_to_user((void __user *)arg, &info, minsz) ?
			-EFAULT : 0;
	}
	case VFIO_DEVICE_GET_REGION_INFO:
		return idxd_vfio_get_region_info(vfio_dev, arg);
	case VFIO_DEVICE_GET_IRQ_INFO:
		return idxd_vfio_get_irq_info(vfio_dev, arg);
	case VFIO_DEVICE_SET_IRQS:
		return idxd_vfio_set_irqs(vfio_dev, arg);
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

static void idxd_vfio_close_device(struct vfio_device *vdev)
{
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);

	idxd_vfio_flush_bar2_forward(vfio_dev);
	idxd_vfio_msix_disable(vfio_dev);
}

static const struct vfio_device_ops idxd_vfio_ops = {
	.name = "idxd-vfio-pci",
	.close_device = idxd_vfio_close_device,
	.ioctl = idxd_vfio_ioctl,
	.read = idxd_vfio_read,
	.write = idxd_vfio_write,
	.mmap = idxd_vfio_mmap,
	.match = idxd_vfio_match,
	.device_feature = idxd_vfio_ioctl_feature,
	.bind_iommufd = idxd_vfio_bind_iommufd,
	.unbind_iommufd = idxd_vfio_unbind_iommufd,
	.attach_ioas = idxd_vfio_attach_ioas,
	.detach_ioas = idxd_vfio_detach_ioas,
	.pasid_attach_ioas = idxd_vfio_pasid_attach_ioas,
	.pasid_detach_ioas = idxd_vfio_pasid_detach_ioas,
};

static int idxd_vfio_init_irqs(struct idxd_vfio_device *vfio_dev)
{
	vfio_dev->msix_trigger =
		kcalloc(idxd_vfio_msix_count(vfio_dev),
			sizeof(*vfio_dev->msix_trigger), GFP_KERNEL);
	if (!vfio_dev->msix_trigger)
		return -ENOMEM;

	return 0;
}

static void idxd_vfio_free_irqs(struct idxd_vfio_device *vfio_dev)
{
	idxd_vfio_msix_disable(vfio_dev);
	kfree(vfio_dev->msix_trigger);
	vfio_dev->msix_trigger = NULL;
}

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

static int idxd_vfio_init_pasid(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_vdev *ivdev = vfio_dev->ivdev;
	unsigned int i;

	xa_init(&vfio_dev->pasid_xa);
	xa_init(&vfio_dev->guest_pasid_xa);

	vfio_dev->wq_pasid = kcalloc(ivdev->num_wqs,
				     sizeof(*vfio_dev->wq_pasid),
				     GFP_KERNEL);
	if (!vfio_dev->wq_pasid)
		return -ENOMEM;

	for (i = 0; i < ivdev->num_wqs; i++) {
		vfio_dev->wq_pasid[i].guest_pasid = IOMMU_PASID_INVALID;
		vfio_dev->wq_pasid[i].host_pasid = IOMMU_PASID_INVALID;
	}

	vfio_dev->default_host_pasid = IOMMU_PASID_INVALID;
	vfio_dev->pasid_attached = false;
	return 0;
}

static void idxd_vfio_free_pasid(struct idxd_vfio_device *vfio_dev)
{
	idxd_vfio_restore_all_wq_pasids(vfio_dev);
	mutex_lock(&vfio_dev->pasid_lock);
	idxd_vfio_free_guest_pasid_entries_locked(vfio_dev);
	idxd_vfio_free_pasid_entries_locked(vfio_dev, NULL);
	vfio_dev->default_host_pasid = IOMMU_PASID_INVALID;
	vfio_dev->pasid_attached = false;
	mutex_unlock(&vfio_dev->pasid_lock);
	xa_destroy(&vfio_dev->guest_pasid_xa);
	xa_destroy(&vfio_dev->pasid_xa);
	kfree(vfio_dev->wq_pasid);
	vfio_dev->wq_pasid = NULL;
}

static void __iomem *idxd_vfio_ioremap_portal(struct idxd_vwq *vwq,
					      enum idxd_portal_prot prot)
{
	phys_addr_t paddr;

	paddr = pci_resource_start(vwq->wq->idxd->pdev, IDXD_WQ_BAR);
	paddr += idxd_get_wq_portal_full_offset(vwq->wq->id, prot);

	return ioremap(paddr, PAGE_SIZE);
}

static void idxd_vfio_free_bar2(struct idxd_vfio_device *vfio_dev)
{
	unsigned int i;

	idxd_vfio_stop_bar2_forward(vfio_dev);

	if (vfio_dev->shared_unlimited_portals) {
		for (i = 0; i < vfio_dev->ivdev->num_wqs; i++) {
			if (vfio_dev->shared_unlimited_portals[i])
				iounmap(vfio_dev->shared_unlimited_portals[i]);
		}
		kfree(vfio_dev->shared_unlimited_portals);
		vfio_dev->shared_unlimited_portals = NULL;
	}

	if (vfio_dev->shared_unlimited_descs) {
		for (i = 0; i < vfio_dev->ivdev->num_wqs; i++)
			kfree(vfio_dev->shared_unlimited_descs[i]);
		kfree(vfio_dev->shared_unlimited_descs);
		vfio_dev->shared_unlimited_descs = NULL;
	}
}

static int idxd_vfio_init_bar2(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_vdev *ivdev = vfio_dev->ivdev;
	struct idxd_vwq *vwq;

	BUILD_BUG_ON(sizeof(struct dsa_raw_desc) != 64);
	BUILD_BUG_ON(IDXD_VFIO_BAR2_DESC_SLOTS * IDXD_VFIO_BAR2_DESC_SIZE !=
		     PAGE_SIZE);

	vfio_dev->shared_unlimited_portals =
		kcalloc(ivdev->num_wqs,
			sizeof(*vfio_dev->shared_unlimited_portals),
			GFP_KERNEL);
	if (!vfio_dev->shared_unlimited_portals)
		return -ENOMEM;

	vfio_dev->shared_unlimited_descs =
		kcalloc(ivdev->num_wqs,
			sizeof(*vfio_dev->shared_unlimited_descs),
			GFP_KERNEL);
	if (!vfio_dev->shared_unlimited_descs) {
		idxd_vfio_free_bar2(vfio_dev);
		return -ENOMEM;
	}

	list_for_each_entry(vwq, &ivdev->wqs, node) {
		if (!vwq->shared)
			continue;

		vfio_dev->shared_unlimited_descs[vwq->id] =
			kcalloc(IDXD_VFIO_BAR2_DESC_SLOTS,
				sizeof(**vfio_dev->shared_unlimited_descs),
				GFP_KERNEL);
		if (!vfio_dev->shared_unlimited_descs[vwq->id]) {
			idxd_vfio_free_bar2(vfio_dev);
			return -ENOMEM;
		}

		vfio_dev->shared_unlimited_portals[vwq->id] =
			idxd_vfio_ioremap_portal(vwq, IDXD_PORTAL_UNLIMITED);
		if (!vfio_dev->shared_unlimited_portals[vwq->id]) {
			idxd_vfio_free_bar2(vfio_dev);
			return -ENOMEM;
		}
	}

	vfio_dev->bar2_forward_stopping = false;
	return 0;
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
	mutex_init(&vfio_dev->irq_lock);
	mutex_init(&vfio_dev->bar2_lock);
	mutex_init(&vfio_dev->pasid_lock);
	spin_lock_init(&vfio_dev->bar2_forward_lock);
	INIT_LIST_HEAD(&vfio_dev->bar2_forward_list);
	INIT_WORK(&vfio_dev->bar2_forward_work, idxd_vfio_bar2_forward_work);
	atomic_set(&vfio_dev->bar2_force_unlimited_retries, 0);
	idxd_vfio_init_config(vfio_dev);
	dev_set_drvdata(dev, vfio_dev);

	rc = idxd_vfio_init_irqs(vfio_dev);
	if (rc)
		goto err_drvdata;

	rc = idxd_vfio_init_bar0(vfio_dev);
	if (rc)
		goto err_irqs;

	rc = idxd_vfio_init_pasid(vfio_dev);
	if (rc)
		goto err_bar0;

	rc = idxd_vfio_init_bar2(vfio_dev);
	if (rc)
		goto err_pasid;

	rc = idxd_vfio_create_sysfs(vfio_dev);
	if (rc)
		goto err_bar2;

	rc = vfio_register_emulated_iommu_dev(&vfio_dev->vdev);
	if (rc)
		goto err_sysfs;

	dev_info(dev, "registered VFIO VDEV with %u WQ(s)\n", ivdev->num_wqs);
	return 0;

err_sysfs:
	idxd_vfio_remove_sysfs(vfio_dev);
err_bar2:
	idxd_vfio_free_bar2(vfio_dev);
err_pasid:
	idxd_vfio_free_pasid(vfio_dev);
err_bar0:
	idxd_vfio_free_bar0(vfio_dev);
err_irqs:
	idxd_vfio_free_irqs(vfio_dev);
err_drvdata:
	dev_set_drvdata(dev, NULL);
	mutex_destroy(&vfio_dev->pasid_lock);
	mutex_destroy(&vfio_dev->bar2_lock);
	mutex_destroy(&vfio_dev->irq_lock);
	mutex_destroy(&vfio_dev->bar0_lock);
	vfio_put_device(&vfio_dev->vdev);
	return rc;
}

static void idxd_vfio_remove(struct idxd_dev *idxd_dev)
{
	struct idxd_vdev *ivdev = idxd_dev_to_vdev(idxd_dev);
	struct device *dev = vdev_confdev(ivdev);
	struct idxd_vfio_device *vfio_dev = dev_get_drvdata(dev);

	vfio_unregister_group_dev(&vfio_dev->vdev);
	idxd_vfio_remove_sysfs(vfio_dev);
	dev_set_drvdata(dev, NULL);
	idxd_vfio_free_bar2(vfio_dev);
	idxd_vfio_free_pasid(vfio_dev);
	idxd_vfio_free_bar0(vfio_dev);
	idxd_vfio_free_irqs(vfio_dev);
	mutex_destroy(&vfio_dev->pasid_lock);
	mutex_destroy(&vfio_dev->bar2_lock);
	mutex_destroy(&vfio_dev->irq_lock);
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
