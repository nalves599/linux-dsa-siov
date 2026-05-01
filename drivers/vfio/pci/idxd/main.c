// SPDX-License-Identifier: GPL-2.0-only
/* Copyright(c) 2026 Intel Corporation */

#include <linux/atomic.h>
#include <linux/bitmap.h>
#include <linux/capability.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/eventfd.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommufd.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/sizes.h>
#include <linux/sysfs.h>
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
			  BIT(IDXD_CMD_ABORT_PASID) |		\
			  BIT(IDXD_CMD_REQUEST_INT_HANDLE) |	\
			  BIT(IDXD_CMD_RELEASE_INT_HANDLE))

#define IDXD_VDEV_WQCFG_IDX2_WR_MASK (BIT(0) | GENMASK(27, 8) |	\
				       BIT(28) | BIT(29))
#define IDXD_VFIO_BAR2_DESC_SIZE	sizeof(struct dsa_raw_desc)
#define IDXD_VFIO_BAR2_DESC_SLOTS	(PAGE_SIZE / IDXD_VFIO_BAR2_DESC_SIZE)
#define IDXD_VFIO_BAR2_FORWARD_RETRIES	100000U
#define IDXD_VFIO_IMS_ENTRY_SIZE	16
#define IDXD_VFIO_IMS_MSG_ADDR		0
#define IDXD_VFIO_IMS_MSG_DATA		8
#define IDXD_VFIO_IMS_CTRL		12
#define IDXD_VFIO_IMS_CTRL_MASK		BIT(0)
#define IDXD_VFIO_IMS_CTRL_PENDING	BIT(1)
#define IDXD_VFIO_IMS_CTRL_IGNORE	BIT(2)
#define IDXD_VFIO_IMS_CTRL_PASID_EN	BIT(3)
#define IDXD_VFIO_IMS_CTRL_PASID_SHIFT	12
#define IDXD_VFIO_IMS_CTRL_PASID_MASK	GENMASK(31, 12)
#define IDXD_VFIO_IMS_VALID_FLAGS				\
	(VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_MASK |		\
	 VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_IGNORE |		\
	 VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_PASID)

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
	union wqcfg saved_wqcfg;
	enum idxd_wq_state saved_state;
	u32 saved_size;
	u32 saved_threshold;
	u32 saved_priority;
	unsigned long saved_flags;
	u64 saved_max_xfer_bytes;
	u32 saved_max_batch_size;
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

struct idxd_vfio_ims_stats {
	atomic64_t request_cmds;
	atomic64_t request_errors;
	atomic64_t request_vector0_rejects;
	atomic64_t duplicate_requests;
	atomic64_t release_cmds;
	atomic64_t release_errors;
	atomic64_t release_unallocated;
	atomic64_t handles_allocated;
	atomic64_t handles_released;
	atomic64_t program_cmds;
	atomic64_t program_deferred;
	atomic64_t program_errors;
	atomic64_t clear_cmds;
	atomic64_t clear_errors;
	atomic64_t physical_clears;
	atomic64_t pasid_teardown_clears;
	atomic64_t all_teardown_clears;
	atomic64_t irq_requests;
	atomic64_t irq_frees;
	atomic64_t signals;
	atomic64_t pending_signals;
	atomic64_t pending_flushes;
	atomic64_t revoke_cmds;
	atomic64_t revoke_errors;
	atomic64_t revoked_vectors;
	atomic64_t revoked_reprograms;
	atomic64_t assertion_failures;
	atomic64_t selftest_runs;
	atomic64_t selftest_failures;
};

struct idxd_vfio_msix_entry {
	struct idxd_vfio_device *vfio_dev;
	u32 msg_addr_lo;
	u32 msg_addr_hi;
	u32 msg_data;
	u32 vector_ctrl;
	ioasid_t host_pasid;
	unsigned int vector;
	unsigned int host_msix_vector;
	unsigned int ims_index;
	int ims_handle;
	int host_irq;
	bool ims_allocated;
	bool ims_configured;
	bool ims_programmed;
	bool ims_ignore;
	bool host_irq_requested;
	bool pending;
	bool ims_revoked;
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
	struct idxd_vfio_msix_entry *msix_entries;
	struct idxd_vfio_ims_stats ims_stats;

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
static bool idxd_vfio_valid_pasid(ioasid_t pasid);
static int idxd_vfio_alloc_ims_handles(struct idxd_vfio_device *vfio_dev);
static void idxd_vfio_free_unused_ims_irqs(struct idxd_vfio_device *vfio_dev);
static int idxd_vfio_revoke_ims_handles(struct idxd_vfio_device *vfio_dev,
					unsigned int vector);
static int idxd_vfio_run_ims_selftest(struct idxd_vfio_device *vfio_dev);
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

static void idxd_vfio_reset_ims_stats(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_vfio_ims_stats *stats = &vfio_dev->ims_stats;

	atomic64_set(&stats->request_cmds, 0);
	atomic64_set(&stats->request_errors, 0);
	atomic64_set(&stats->request_vector0_rejects, 0);
	atomic64_set(&stats->duplicate_requests, 0);
	atomic64_set(&stats->release_cmds, 0);
	atomic64_set(&stats->release_errors, 0);
	atomic64_set(&stats->release_unallocated, 0);
	atomic64_set(&stats->handles_allocated, 0);
	atomic64_set(&stats->handles_released, 0);
	atomic64_set(&stats->program_cmds, 0);
	atomic64_set(&stats->program_deferred, 0);
	atomic64_set(&stats->program_errors, 0);
	atomic64_set(&stats->clear_cmds, 0);
	atomic64_set(&stats->clear_errors, 0);
	atomic64_set(&stats->physical_clears, 0);
	atomic64_set(&stats->pasid_teardown_clears, 0);
	atomic64_set(&stats->all_teardown_clears, 0);
	atomic64_set(&stats->irq_requests, 0);
	atomic64_set(&stats->irq_frees, 0);
	atomic64_set(&stats->signals, 0);
	atomic64_set(&stats->pending_signals, 0);
	atomic64_set(&stats->pending_flushes, 0);
	atomic64_set(&stats->revoke_cmds, 0);
	atomic64_set(&stats->revoke_errors, 0);
	atomic64_set(&stats->revoked_vectors, 0);
	atomic64_set(&stats->revoked_reprograms, 0);
	atomic64_set(&stats->assertion_failures, 0);
	atomic64_set(&stats->selftest_runs, 0);
	atomic64_set(&stats->selftest_failures, 0);
}

static ssize_t ims_stats_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct idxd_vfio_device *vfio_dev = dev_get_drvdata(dev);
	struct idxd_vfio_ims_stats *stats;

	if (!vfio_dev)
		return -ENODEV;

	stats = &vfio_dev->ims_stats;
	return sysfs_emit(buf,
			  "request_cmds %lld\n"
			  "request_errors %lld\n"
			  "request_vector0_rejects %lld\n"
			  "duplicate_requests %lld\n"
			  "release_cmds %lld\n"
			  "release_errors %lld\n"
			  "release_unallocated %lld\n"
			  "handles_allocated %lld\n"
			  "handles_released %lld\n"
			  "program_cmds %lld\n"
			  "program_deferred %lld\n"
			  "program_errors %lld\n"
			  "clear_cmds %lld\n"
			  "clear_errors %lld\n"
			  "physical_clears %lld\n"
			  "pasid_teardown_clears %lld\n"
			  "all_teardown_clears %lld\n"
			  "irq_requests %lld\n"
			  "irq_frees %lld\n"
			  "signals %lld\n"
			  "pending_signals %lld\n"
			  "pending_flushes %lld\n"
			  "revoke_cmds %lld\n"
			  "revoke_errors %lld\n"
			  "revoked_vectors %lld\n"
			  "revoked_reprograms %lld\n"
			  "assertion_failures %lld\n"
			  "selftest_runs %lld\n"
			  "selftest_failures %lld\n",
			  (long long)atomic64_read(&stats->request_cmds),
			  (long long)atomic64_read(&stats->request_errors),
			  (long long)atomic64_read(&stats->request_vector0_rejects),
			  (long long)atomic64_read(&stats->duplicate_requests),
			  (long long)atomic64_read(&stats->release_cmds),
			  (long long)atomic64_read(&stats->release_errors),
			  (long long)atomic64_read(&stats->release_unallocated),
			  (long long)atomic64_read(&stats->handles_allocated),
			  (long long)atomic64_read(&stats->handles_released),
			  (long long)atomic64_read(&stats->program_cmds),
			  (long long)atomic64_read(&stats->program_deferred),
			  (long long)atomic64_read(&stats->program_errors),
			  (long long)atomic64_read(&stats->clear_cmds),
			  (long long)atomic64_read(&stats->clear_errors),
			  (long long)atomic64_read(&stats->physical_clears),
			  (long long)atomic64_read(&stats->pasid_teardown_clears),
			  (long long)atomic64_read(&stats->all_teardown_clears),
			  (long long)atomic64_read(&stats->irq_requests),
			  (long long)atomic64_read(&stats->irq_frees),
			  (long long)atomic64_read(&stats->signals),
			  (long long)atomic64_read(&stats->pending_signals),
			  (long long)atomic64_read(&stats->pending_flushes),
			  (long long)atomic64_read(&stats->revoke_cmds),
			  (long long)atomic64_read(&stats->revoke_errors),
			  (long long)atomic64_read(&stats->revoked_vectors),
			  (long long)atomic64_read(&stats->revoked_reprograms),
			  (long long)atomic64_read(&stats->assertion_failures),
			  (long long)atomic64_read(&stats->selftest_runs),
			  (long long)atomic64_read(&stats->selftest_failures));
}
static DEVICE_ATTR_RO(ims_stats);
static struct device_attribute dev_attr_ims_lifecycle_stats =
	__ATTR(ims_lifecycle_stats, 0444, ims_stats_show, NULL);

static ssize_t ims_stats_reset_store(struct device *dev,
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
		idxd_vfio_reset_ims_stats(vfio_dev);

	return count;
}
static DEVICE_ATTR_WO(ims_stats_reset);
static struct device_attribute dev_attr_ims_lifecycle_stats_reset =
	__ATTR(ims_lifecycle_stats_reset, 0200, NULL, ims_stats_reset_store);

static ssize_t ims_selftest_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct idxd_vfio_device *vfio_dev = dev_get_drvdata(dev);
	bool run;
	int rc;

	if (!vfio_dev)
		return -ENODEV;

	rc = kstrtobool(buf, &run);
	if (rc)
		return rc;
	if (!run)
		return count;

	rc = idxd_vfio_run_ims_selftest(vfio_dev);
	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR_WO(ims_selftest);
static struct device_attribute dev_attr_ims_lifecycle_selftest =
	__ATTR(ims_lifecycle_selftest, 0200, NULL, ims_selftest_store);

static ssize_t ims_revoke_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct idxd_vfio_device *vfio_dev = dev_get_drvdata(dev);
	bool revoke;
	int rc;

	if (!vfio_dev)
		return -ENODEV;

	rc = kstrtobool(buf, &revoke);
	if (rc)
		return rc;
	if (!revoke)
		return count;

	rc = idxd_vfio_revoke_ims_handles(vfio_dev,
					  VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_VECTOR_ALL);
	if (rc)
		return rc;

	return count;
}
static DEVICE_ATTR_WO(ims_revoke);
static struct device_attribute dev_attr_ims_revoke_handles =
	__ATTR(ims_revoke_handles, 0200, NULL, ims_revoke_store);

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

static struct attribute *idxd_vfio_attrs[] = {
	&dev_attr_bar2_trap_stats.attr,
	&dev_attr_bar2_trap_stats_reset.attr,
	&dev_attr_bar2_force_unlimited_retries.attr,
	&dev_attr_ims_stats.attr,
	&dev_attr_ims_stats_reset.attr,
	&dev_attr_ims_selftest.attr,
	&dev_attr_ims_revoke.attr,
	/*
	 * Compatibility aliases for existing test scripts. New scripts should
	 * use ims_stats, ims_stats_reset, ims_selftest, and ims_revoke.
	 */
	&dev_attr_ims_lifecycle_stats.attr,
	&dev_attr_ims_lifecycle_stats_reset.attr,
	&dev_attr_ims_lifecycle_selftest.attr,
	&dev_attr_ims_revoke_handles.attr,
	NULL,
};

static const struct attribute_group idxd_vfio_attr_group = {
	.attrs = idxd_vfio_attrs,
};

static int idxd_vfio_create_sysfs(struct idxd_vfio_device *vfio_dev)
{
	struct device *dev = vdev_confdev(vfio_dev->ivdev);

	return sysfs_create_group(&dev->kobj, &idxd_vfio_attr_group);
}

static void idxd_vfio_remove_sysfs(struct idxd_vfio_device *vfio_dev)
{
	struct device *dev = vdev_confdev(vfio_dev->ivdev);

	sysfs_remove_group(&dev->kobj, &idxd_vfio_attr_group);
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

static u32 idxd_vfio_msix_pba_size(struct idxd_vfio_device *vfio_dev)
{
	return BITS_TO_LONGS(idxd_vfio_msix_count(vfio_dev)) * sizeof(unsigned long);
}

static u16 idxd_vfio_msix_flags(struct idxd_vfio_device *vfio_dev)
{
	return vfio_dev->config[IDXD_VDEV_MSIX_CAP_OFFSET + PCI_MSIX_FLAGS] |
	       (vfio_dev->config[IDXD_VDEV_MSIX_CAP_OFFSET + PCI_MSIX_FLAGS + 1] << 8);
}

static bool idxd_vfio_msix_masked_locked(struct idxd_vfio_device *vfio_dev,
					 unsigned int vector)
{
	struct idxd_vfio_msix_entry *entry = &vfio_dev->msix_entries[vector];

	lockdep_assert_held(&vfio_dev->irq_lock);

	return !(idxd_vfio_msix_flags(vfio_dev) & PCI_MSIX_FLAGS_ENABLE) ||
	       (idxd_vfio_msix_flags(vfio_dev) & PCI_MSIX_FLAGS_MASKALL) ||
	       (entry->vector_ctrl & PCI_MSIX_ENTRY_CTRL_MASKBIT);
}

static bool idxd_vfio_ims_supported(struct idxd_device *idxd)
{
	return idxd->hw.gen_cap.max_ims_mult && idxd->ims_offset;
}

static bool idxd_vfio_msix_vector_uses_ims(unsigned int vector)
{
	/*
	 * Vector 0 is the VDEV command/error vector. Descriptor completion
	 * vectors start at 1 and are backed by physical IMS entries.
	 */
	return vector != 0;
}

static void __iomem *idxd_vfio_ims_entry_addr(struct idxd_device *idxd,
					      unsigned int index,
					      unsigned int offset)
{
	return idxd->reg_base + idxd->ims_offset +
	       index * IDXD_VFIO_IMS_ENTRY_SIZE + offset;
}

static u32 idxd_vfio_ims_entry_ctrl(struct idxd_vfio_msix_entry *entry)
{
	u32 ctrl = 0;

	if (entry->vector_ctrl & PCI_MSIX_ENTRY_CTRL_MASKBIT)
		ctrl |= IDXD_VFIO_IMS_CTRL_MASK;
	if (entry->ims_ignore)
		ctrl |= IDXD_VFIO_IMS_CTRL_IGNORE;
	if (entry->ims_programmed) {
		ctrl |= IDXD_VFIO_IMS_CTRL_PASID_EN;
		ctrl |= (entry->host_pasid << IDXD_VFIO_IMS_CTRL_PASID_SHIFT) &
			IDXD_VFIO_IMS_CTRL_PASID_MASK;
	}

	return ctrl;
}

static irqreturn_t idxd_vfio_ims_irq_thread(int irq, void *data)
{
	struct idxd_vfio_msix_entry *entry = data;

	idxd_vfio_msix_signal(entry->vfio_dev, entry->vector);
	return IRQ_HANDLED;
}

static int idxd_vfio_request_ims_irq_locked(struct idxd_vfio_device *vfio_dev,
					    struct idxd_vfio_msix_entry *entry)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	struct device *dev = vdev_confdev(vfio_dev->ivdev);
	int irq, rc;

	lockdep_assert_held(&vfio_dev->irq_lock);

	if (!idxd_vfio_msix_vector_uses_ims(entry->vector) ||
	    entry->host_irq_requested)
		return 0;
	if (entry->host_msix_vector >= idxd->irq_cnt)
		return -ENOSPC;

	irq = pci_irq_vector(idxd->pdev, entry->host_msix_vector);
	if (irq < 0)
		return irq;

	rc = request_threaded_irq(irq, NULL, idxd_vfio_ims_irq_thread, 0,
				  "idxd-vfio-ims", entry);
	if (rc) {
		dev_err(dev, "failed to request IMS host IRQ for vector %u host vector %u: %d\n",
			entry->vector, entry->host_msix_vector, rc);
		return rc;
	}

	entry->host_irq = irq;
	entry->host_irq_requested = true;
	atomic64_inc(&vfio_dev->ims_stats.irq_requests);
	return 0;
}

static void idxd_vfio_free_ims_irq(struct idxd_vfio_msix_entry *entry)
{
	if (!entry->host_irq_requested)
		return;

	free_irq(entry->host_irq, entry);
	entry->host_irq = -1;
	entry->host_irq_requested = false;
	atomic64_inc(&entry->vfio_dev->ims_stats.irq_frees);
}

static void idxd_vfio_get_ims_msg(struct idxd_vfio_msix_entry *entry,
				  u64 *msg_addr, u32 *msg_data)
{
	if (entry->host_irq_requested) {
		struct msi_msg msg;

		get_cached_msi_msg(entry->host_irq, &msg);
		*msg_addr = ((u64)msg.address_hi << 32) | msg.address_lo;
		*msg_data = msg.data;
		return;
	}

	*msg_addr = ((u64)entry->msg_addr_hi << 32) | entry->msg_addr_lo;
	*msg_data = entry->msg_data;
}

static void idxd_vfio_write_ims_entry(struct idxd_vfio_device *vfio_dev,
				      struct idxd_vfio_msix_entry *entry)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	u32 ctrl = idxd_vfio_ims_entry_ctrl(entry);
	u32 msg_data;
	u64 msg_addr;
	unsigned long flags;

	if (!entry->ims_allocated)
		return;

	idxd_vfio_get_ims_msg(entry, &msg_addr, &msg_data);

	spin_lock_irqsave(&idxd->dev_lock, flags);
	iowrite32(lower_32_bits(msg_addr),
		  idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
					   IDXD_VFIO_IMS_MSG_ADDR));
	iowrite32(upper_32_bits(msg_addr),
		  idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
					   IDXD_VFIO_IMS_MSG_ADDR + sizeof(u32)));
	iowrite32(msg_data,
		  idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
					   IDXD_VFIO_IMS_MSG_DATA));
	iowrite32(ctrl, idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
						 IDXD_VFIO_IMS_CTRL));
	spin_unlock_irqrestore(&idxd->dev_lock, flags);
}

static void idxd_vfio_clear_ims_entry_hw(struct idxd_vfio_device *vfio_dev,
					 struct idxd_vfio_msix_entry *entry)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	unsigned long flags;

	if (!entry->ims_allocated)
		return;

	atomic64_inc(&vfio_dev->ims_stats.physical_clears);
	spin_lock_irqsave(&idxd->dev_lock, flags);
	iowrite32(IDXD_VFIO_IMS_CTRL_MASK | IDXD_VFIO_IMS_CTRL_IGNORE,
		  idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
					   IDXD_VFIO_IMS_CTRL));
	iowrite32(0, idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
					      IDXD_VFIO_IMS_MSG_ADDR));
	iowrite32(0, idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
					      IDXD_VFIO_IMS_MSG_ADDR +
					      sizeof(u32)));
	iowrite32(0, idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
					      IDXD_VFIO_IMS_MSG_DATA));
	spin_unlock_irqrestore(&idxd->dev_lock, flags);
}

static void idxd_vfio_forget_ims_handle(struct idxd_vfio_msix_entry *entry)
{
	entry->ims_allocated = false;
	entry->ims_index = UINT_MAX;
	entry->ims_handle = INVALID_INT_HANDLE;
	entry->host_msix_vector = UINT_MAX;
}

static void idxd_vfio_clear_ims_entry(struct idxd_vfio_device *vfio_dev,
				      struct idxd_vfio_msix_entry *entry)
{
	entry->ims_configured = false;
	entry->ims_programmed = false;
	entry->ims_ignore = true;
	entry->host_pasid = IOMMU_PASID_INVALID;
	entry->msg_addr_lo = 0;
	entry->msg_addr_hi = 0;
	entry->msg_data = 0;
	entry->vector_ctrl |= PCI_MSIX_ENTRY_CTRL_MASKBIT;
	entry->pending = false;
	entry->ims_revoked = false;

	idxd_vfio_clear_ims_entry_hw(vfio_dev, entry);
}

static void idxd_vfio_read_ims_entry_hw(struct idxd_vfio_device *vfio_dev,
					struct idxd_vfio_msix_entry *entry,
					u64 *msg_addr, u32 *msg_data,
					u32 *ctrl)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	unsigned long flags;

	*msg_addr = 0;
	*msg_data = 0;
	*ctrl = 0;

	if (!entry->ims_allocated)
		return;

	spin_lock_irqsave(&idxd->dev_lock, flags);
	*msg_addr =
		ioread32(idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
						  IDXD_VFIO_IMS_MSG_ADDR));
	*msg_addr |=
		(u64)ioread32(idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
						       IDXD_VFIO_IMS_MSG_ADDR +
						       sizeof(u32))) << 32;
	*msg_data = ioread32(idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
						      IDXD_VFIO_IMS_MSG_DATA));
	*ctrl = ioread32(idxd_vfio_ims_entry_addr(idxd, entry->ims_index,
						  IDXD_VFIO_IMS_CTRL));
	spin_unlock_irqrestore(&idxd->dev_lock, flags);
}

static u32 idxd_vfio_read_ims_entry_ctrl(struct idxd_vfio_device *vfio_dev,
					 struct idxd_vfio_msix_entry *entry)
{
	u64 msg_addr;
	u32 msg_data, ctrl;

	idxd_vfio_read_ims_entry_hw(vfio_dev, entry, &msg_addr, &msg_data,
				    &ctrl);

	return ctrl;
}

static bool
idxd_vfio_assert_ims_entry_clear_locked(struct idxd_vfio_device *vfio_dev,
					struct idxd_vfio_msix_entry *entry,
					const char *reason,
					bool require_irq_free,
					bool require_handle_free)
{
	struct device *dev = vdev_confdev(vfio_dev->ivdev);
	bool ok = true;
	u64 msg_addr;
	u32 msg_data, ctrl;

	lockdep_assert_held(&vfio_dev->irq_lock);

	if (entry->ims_programmed ||
	    entry->ims_configured ||
	    entry->host_pasid != IOMMU_PASID_INVALID ||
	    !entry->ims_ignore ||
	    entry->pending ||
	    entry->ims_revoked)
		ok = false;

	if (require_irq_free && entry->host_irq_requested)
		ok = false;

	if (require_handle_free &&
	    (entry->ims_allocated ||
	     entry->ims_index != UINT_MAX ||
	     entry->ims_handle != INVALID_INT_HANDLE ||
	     entry->host_msix_vector != UINT_MAX))
		ok = false;

	idxd_vfio_read_ims_entry_hw(vfio_dev, entry, &msg_addr, &msg_data,
				    &ctrl);
	if (entry->ims_allocated &&
	    ((ctrl & IDXD_VFIO_IMS_CTRL_PASID_EN) ||
	     !(ctrl & IDXD_VFIO_IMS_CTRL_MASK) ||
	     !(ctrl & IDXD_VFIO_IMS_CTRL_IGNORE)))
		ok = false;

	if (!ok) {
		atomic64_inc(&vfio_dev->ims_stats.assertion_failures);
		dev_warn(dev,
			 "IMS state assertion failed during %s: vector=%u allocated=%u configured=%u programmed=%u irq_requested=%u handle=%d ims_index=%u host_vector=%u host_pasid=%u pending=%u ignore=%u revoked=%u ctrl=%#x msg=%#llx/%#x\n",
			 reason, entry->vector, entry->ims_allocated,
			 entry->ims_configured, entry->ims_programmed,
			 entry->host_irq_requested, entry->ims_handle,
			 entry->ims_index, entry->host_msix_vector,
			 entry->host_pasid, entry->pending, entry->ims_ignore,
			 entry->ims_revoked, ctrl,
			 (unsigned long long)msg_addr, msg_data);
	}

	return ok;
}

static bool idxd_vfio_assert_ims_entry_clear(struct idxd_vfio_device *vfio_dev,
					     struct idxd_vfio_msix_entry *entry,
					     const char *reason,
					     bool require_irq_free,
					     bool require_handle_free)
{
	bool ok;

	mutex_lock(&vfio_dev->irq_lock);
	ok = idxd_vfio_assert_ims_entry_clear_locked(vfio_dev, entry, reason,
						    require_irq_free,
						    require_handle_free);
	mutex_unlock(&vfio_dev->irq_lock);

	return ok;
}

static void idxd_vfio_assert_all_ims_clear(struct idxd_vfio_device *vfio_dev,
					   const char *reason,
					   bool require_irq_free)
{
	unsigned int i;

	mutex_lock(&vfio_dev->irq_lock);
	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++)
		idxd_vfio_assert_ims_entry_clear_locked(vfio_dev,
							&vfio_dev->msix_entries[i],
							reason,
							require_irq_free,
							false);
	mutex_unlock(&vfio_dev->irq_lock);
}

static void idxd_vfio_assert_no_ims_for_pasid(struct idxd_vfio_device *vfio_dev,
					      ioasid_t host_pasid,
					      const char *reason)
{
	struct device *dev = vdev_confdev(vfio_dev->ivdev);
	unsigned int i;

	mutex_lock(&vfio_dev->irq_lock);
	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		struct idxd_vfio_msix_entry *entry = &vfio_dev->msix_entries[i];

		if (entry->host_pasid != host_pasid ||
		    (!entry->ims_configured && !entry->ims_programmed &&
		     !entry->ims_revoked))
			continue;

		atomic64_inc(&vfio_dev->ims_stats.assertion_failures);
		dev_warn(dev,
			 "IMS PASID teardown assertion failed during %s: vector=%u still references PASID %u configured=%u programmed=%u revoked=%u\n",
			 reason, i, host_pasid, entry->ims_configured,
			 entry->ims_programmed, entry->ims_revoked);
	}
	mutex_unlock(&vfio_dev->irq_lock);
}

static void idxd_vfio_clear_ims_entries_for_pasid(struct idxd_vfio_device *vfio_dev,
						  ioasid_t host_pasid)
{
	unsigned int i;

	mutex_lock(&vfio_dev->irq_lock);
	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		struct idxd_vfio_msix_entry *entry = &vfio_dev->msix_entries[i];

		if (entry->host_pasid != host_pasid ||
		    (!entry->ims_configured && !entry->ims_programmed &&
		     !entry->ims_revoked))
			continue;

		idxd_vfio_clear_ims_entry(vfio_dev, entry);
		atomic64_inc(&vfio_dev->ims_stats.pasid_teardown_clears);
		entry->pending = false;
	}
	mutex_unlock(&vfio_dev->irq_lock);
}

static void idxd_vfio_clear_all_ims_entries(struct idxd_vfio_device *vfio_dev)
{
	unsigned int i;

	mutex_lock(&vfio_dev->irq_lock);
	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		idxd_vfio_clear_ims_entry(vfio_dev, &vfio_dev->msix_entries[i]);
		atomic64_inc(&vfio_dev->ims_stats.all_teardown_clears);
		vfio_dev->msix_entries[i].pending = false;
	}
	mutex_unlock(&vfio_dev->irq_lock);
}

static void idxd_vfio_free_unused_ims_irqs(struct idxd_vfio_device *vfio_dev)
{
	unsigned int i;

	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		struct idxd_vfio_msix_entry *entry = &vfio_dev->msix_entries[i];

		if (!entry->host_irq_requested || entry->ims_programmed)
			continue;

		idxd_vfio_free_ims_irq(entry);
	}
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

static bool idxd_vfio_msix_table_readb(struct idxd_vfio_device *vfio_dev,
				       loff_t pos, u8 *val)
{
	struct idxd_vfio_msix_entry *entry;
	u32 table_size, offset, field;
	unsigned int vector;
	u32 value;

	table_size = idxd_vfio_msix_count(vfio_dev) * PCI_MSIX_ENTRY_SIZE;
	if (pos < IDXD_VDEV_MSIX_TABLE_OFFSET ||
	    pos >= IDXD_VDEV_MSIX_TABLE_OFFSET + table_size)
		return false;

	offset = pos - IDXD_VDEV_MSIX_TABLE_OFFSET;
	vector = offset / PCI_MSIX_ENTRY_SIZE;
	field = offset % PCI_MSIX_ENTRY_SIZE;
	entry = &vfio_dev->msix_entries[vector];

	switch (field & ~0x3U) {
	case PCI_MSIX_ENTRY_LOWER_ADDR:
		value = entry->msg_addr_lo;
		break;
	case PCI_MSIX_ENTRY_UPPER_ADDR:
		value = entry->msg_addr_hi;
		break;
	case PCI_MSIX_ENTRY_DATA:
		value = entry->msg_data;
		break;
	case PCI_MSIX_ENTRY_VECTOR_CTRL:
		value = entry->vector_ctrl;
		break;
	default:
		value = 0;
		break;
	}

	*val = value >> ((field & 0x3) * 8);
	return true;
}

static bool idxd_vfio_msix_pba_readb(struct idxd_vfio_device *vfio_dev,
				     loff_t pos, u8 *val)
{
	u32 pba_offset = idxd_vfio_msix_pba_offset(vfio_dev);
	u32 pba_size = idxd_vfio_msix_pba_size(vfio_dev);
	unsigned int vector;
	u32 offset;
	u8 value = 0;

	if (pos < pba_offset || pos >= pba_offset + pba_size)
		return false;

	offset = pos - pba_offset;
	for (vector = offset * BITS_PER_BYTE;
	     vector < min_t(unsigned int, idxd_vfio_msix_count(vfio_dev),
			    (offset + 1) * BITS_PER_BYTE);
	     vector++) {
		struct idxd_vfio_msix_entry *entry =
			&vfio_dev->msix_entries[vector];
		u32 ctrl = idxd_vfio_read_ims_entry_ctrl(vfio_dev, entry);

		if (entry->pending || (ctrl & IDXD_VFIO_IMS_CTRL_PENDING))
			value |= BIT(vector - offset * BITS_PER_BYTE);
	}

	*val = value;
	return true;
}

static bool idxd_vfio_msix_readb(struct idxd_vfio_device *vfio_dev,
				 loff_t pos, u8 *val)
{
	bool handled;

	mutex_lock(&vfio_dev->irq_lock);
	handled = idxd_vfio_msix_table_readb(vfio_dev, pos, val) ||
		  idxd_vfio_msix_pba_readb(vfio_dev, pos, val);
	mutex_unlock(&vfio_dev->irq_lock);

	return handled;
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

	if (idxd_vfio_msix_readb(vfio_dev, pos, &byte))
		return byte;

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
	unsigned int i;

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

	mutex_lock(&vfio_dev->irq_lock);
	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		vfio_dev->msix_entries[i].msg_addr_lo = 0;
		vfio_dev->msix_entries[i].msg_addr_hi = 0;
		vfio_dev->msix_entries[i].msg_data = 0;
		vfio_dev->msix_entries[i].vector_ctrl =
			PCI_MSIX_ENTRY_CTRL_MASKBIT;
		idxd_vfio_clear_ims_entry(vfio_dev, &vfio_dev->msix_entries[i]);
		vfio_dev->msix_entries[i].pending = false;
	}
	mutex_unlock(&vfio_dev->irq_lock);
}

static void idxd_vfio_msix_flush_pending_locked(struct idxd_vfio_device *vfio_dev,
						unsigned int vector)
{
	struct idxd_vfio_msix_entry *entry;
	struct eventfd_ctx *trigger;

	lockdep_assert_held(&vfio_dev->irq_lock);

	if (vector >= idxd_vfio_msix_count(vfio_dev))
		return;

	entry = &vfio_dev->msix_entries[vector];
	if (!entry->pending || idxd_vfio_msix_masked_locked(vfio_dev, vector))
		return;

	trigger = vfio_dev->msix_trigger[vector];
	if (!trigger)
		return;

	entry->pending = false;
	atomic64_inc(&vfio_dev->ims_stats.pending_flushes);
	eventfd_signal(trigger);
}

static void idxd_vfio_msix_flush_all_pending(struct idxd_vfio_device *vfio_dev)
{
	unsigned int i;

	mutex_lock(&vfio_dev->irq_lock);
	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++)
		idxd_vfio_msix_flush_pending_locked(vfio_dev, i);
	mutex_unlock(&vfio_dev->irq_lock);
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
	idxd_vfio_msix_flush_pending_locked(vfio_dev, vector);
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
	struct idxd_vfio_msix_entry *entry;
	struct eventfd_ctx *trigger;

	if (vector >= idxd_vfio_msix_count(vfio_dev))
		return;

	mutex_lock(&vfio_dev->irq_lock);
	entry = &vfio_dev->msix_entries[vector];
	atomic64_inc(&vfio_dev->ims_stats.signals);
	if (idxd_vfio_msix_masked_locked(vfio_dev, vector)) {
		entry->pending = true;
		atomic64_inc(&vfio_dev->ims_stats.pending_signals);
		mutex_unlock(&vfio_dev->irq_lock);
		return;
	}

	trigger = vfio_dev->msix_trigger[vector];
	if (trigger) {
		entry->pending = false;
		eventfd_signal(trigger);
	} else {
		entry->pending = true;
		atomic64_inc(&vfio_dev->ims_stats.pending_signals);
	}
	mutex_unlock(&vfio_dev->irq_lock);
}

static u32 idxd_vfio_cmdsts(u8 err, u16 result)
{
	return err | ((u32)result << IDXD_CMDSTS_RES_SHIFT);
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

static int idxd_vfio_try_program_ims_locked(struct idxd_vfio_device *vfio_dev,
					    struct idxd_vfio_msix_entry *entry,
					    bool *deferred)
{
	bool attached;
	int rc;

	lockdep_assert_held(&vfio_dev->irq_lock);

	if (deferred)
		*deferred = false;

	if (!idxd_vfio_msix_vector_uses_ims(entry->vector) ||
	    !entry->ims_configured)
		return 0;

	if (!entry->ims_allocated) {
		if (deferred)
			*deferred = true;
		return 0;
	}

	if (!idxd_vfio_valid_pasid(entry->host_pasid))
		return -EINVAL;

	mutex_lock(&vfio_dev->pasid_lock);
	attached = idxd_vfio_host_pasid_attached_locked(vfio_dev,
							entry->host_pasid);
	mutex_unlock(&vfio_dev->pasid_lock);
	if (!attached) {
		if (deferred)
			*deferred = true;
		return 0;
	}

	if (!entry->ims_programmed) {
		rc = idxd_vfio_request_ims_irq_locked(vfio_dev, entry);
		if (rc)
			return rc;
		entry->ims_programmed = true;
	}

	idxd_vfio_write_ims_entry(vfio_dev, entry);
	return 0;
}

static int idxd_vfio_program_pending_ims_for_pasid(struct idxd_vfio_device *vfio_dev,
						   ioasid_t host_pasid)
{
	unsigned int i;
	int first_rc = 0;

	mutex_lock(&vfio_dev->irq_lock);
	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		struct idxd_vfio_msix_entry *entry = &vfio_dev->msix_entries[i];
		int rc;

		if (entry->host_pasid != host_pasid || !entry->ims_configured ||
		    entry->ims_programmed)
			continue;

		rc = idxd_vfio_try_program_ims_locked(vfio_dev, entry, NULL);
		if (!rc)
			continue;

		atomic64_inc(&vfio_dev->ims_stats.program_errors);
		if (!first_rc)
			first_rc = rc;
	}
	mutex_unlock(&vfio_dev->irq_lock);

	return first_rc;
}

static void idxd_vfio_write_wqcfg(struct idxd_wq *wq)
{
	struct idxd_device *idxd = wq->idxd;
	unsigned int i, n;

	n = min_t(unsigned int, WQCFG_STRIDES(idxd),
		  ARRAY_SIZE(wq->wqcfg->bits));

	spin_lock(&idxd->dev_lock);
	for (i = 0; i < n; i++)
		iowrite32(wq->wqcfg->bits[i],
			  idxd->reg_base + WQCFG_OFFSET(idxd, wq->id, i));
	spin_unlock(&idxd->dev_lock);
}

static void idxd_vfio_save_wq_state(struct idxd_vfio_wq_pasid *state,
				    struct idxd_wq *wq)
{
	memcpy(&state->saved_wqcfg, wq->wqcfg, sizeof(state->saved_wqcfg));
	state->saved_state = wq->state;
	state->saved_size = wq->size;
	state->saved_threshold = wq->threshold;
	state->saved_priority = wq->priority;
	state->saved_flags = wq->flags;
	state->saved_max_xfer_bytes = wq->max_xfer_bytes;
	state->saved_max_batch_size = wq->max_batch_size;
	state->saved_wqcfg_valid = true;
}

static void idxd_vfio_clear_wq_pasid_state(struct idxd_vfio_wq_pasid *state)
{
	state->guest_pasid = IOMMU_PASID_INVALID;
	state->host_pasid = IOMMU_PASID_INVALID;
	state->saved_wqcfg_valid = false;
	state->uses_default_pasid = false;
	state->programmed = false;
}

static int idxd_vfio_restore_wq_saved_locked(struct idxd_vfio_wq_pasid *state,
					     struct idxd_wq *wq)
{
	enum idxd_wq_state saved_state = state->saved_state;
	int rc;

	lockdep_assert_held(&wq->wq_lock);

	if (WARN_ON(!state->saved_wqcfg_valid))
		return -EINVAL;

	rc = idxd_wq_disable(wq, false);
	if (rc)
		return rc;

	memcpy(wq->wqcfg, &state->saved_wqcfg, sizeof(*wq->wqcfg));
	wq->size = state->saved_size;
	wq->threshold = state->saved_threshold;
	wq->priority = state->saved_priority;
	wq->flags = state->saved_flags;
	wq->max_xfer_bytes = state->saved_max_xfer_bytes;
	wq->max_batch_size = state->saved_max_batch_size;
	idxd_vfio_write_wqcfg(wq);

	if (saved_state == IDXD_WQ_ENABLED)
		rc = idxd_wq_enable(wq);

	return rc;
}

static bool idxd_vfio_pasid_priv_enabled(struct idxd_device *idxd)
{
	struct pci_dev *pdev = idxd->pdev;

	return pdev->pasid_enabled &&
	       (pdev->pasid_features & PCI_PASID_CAP_PRIV);
}

static int
idxd_vfio_program_wq_dedicated_locked(struct idxd_vfio_wq_pasid *state,
				      struct idxd_vwq *vwq,
				      const union wqcfg *vconfig,
				      ioasid_t host_pasid)
{
	struct idxd_wq *wq = vwq->wq;
	union wqcfg wqcfg = state->saved_wqcfg;
	int rc;

	lockdep_assert_held(&wq->wq_lock);

	rc = idxd_wq_disable(wq, false);
	if (rc)
		return rc;

	wqcfg.wq_size = state->saved_size;
	wqcfg.wq_thresh = 0;
	wqcfg.mode = 1;
	wqcfg.pasid = host_pasid;
	wqcfg.pasid_en = 1;
	wqcfg.priv = vconfig->priv;

	memcpy(wq->wqcfg, &wqcfg, sizeof(*wq->wqcfg));
	wq->threshold = 0;
	wq->flags = state->saved_flags;
	set_bit(WQ_FLAG_DEDICATED, &wq->flags);
	idxd_vfio_write_wqcfg(wq);

	return idxd_wq_enable(wq);
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

	mutex_lock(&vwq->wq->wq_lock);
	rc = idxd_vfio_restore_wq_saved_locked(state, vwq->wq);
	mutex_unlock(&vwq->wq->wq_lock);

	if (rc)
		return rc;

	idxd_vfio_clear_wq_pasid_state(state);
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
				      const union wqcfg *wqcfg,
				      ioasid_t guest_pasid,
				      ioasid_t host_pasid,
				      bool uses_default)
{
	struct idxd_vfio_wq_pasid *state;
	int rc;

	if (!vfio_dev->wq_pasid || vwq->id >= vfio_dev->ivdev->num_wqs)
		return -EINVAL;
	if (host_pasid & ~GENMASK(IDXD_VDEV_PASID_BITS - 1, 0))
		return -EINVAL;

	/*
	 * A single-guest WQ that the guest configures as shared continues to
	 * use descriptor PASIDs. The dedicated path below is the one that
	 * requires the physical WQ PASID field to be programmed.
	 */
	if (!wqcfg->mode)
		return 0;

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
	if (idxd_wq_refcount(vwq->wq) > 1) {
		rc = -EBUSY;
		goto out_unlock_wq;
	}
	if (wqcfg->priv && !idxd_vfio_pasid_priv_enabled(vwq->wq->idxd)) {
		rc = -EOPNOTSUPP;
		goto out_unlock_wq;
	}
	idxd_vfio_save_wq_state(state, vwq->wq);
	state->guest_pasid = guest_pasid;
	state->host_pasid = host_pasid;
	state->uses_default_pasid = uses_default;
	state->programmed = true;
	rc = idxd_vfio_program_wq_dedicated_locked(state, vwq, wqcfg,
						   host_pasid);
out_unlock_wq:
	mutex_unlock(&vwq->wq->wq_lock);

	if (rc) {
		int restore_rc;

		restore_rc = idxd_vfio_restore_wq_pasid(vfio_dev, vwq->id);
		if (restore_rc)
			dev_warn(vdev_confdev(vfio_dev->ivdev),
				 "failed to restore vWQ%u after dedicated programming failure: %d\n",
				 vwq->id, restore_rc);
		return rc;
	}

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

	rc = idxd_vfio_program_wq_pasid(vfio_dev, vwq, wqcfg, guest_pasid,
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

static u8 idxd_vfio_request_int_handle(struct idxd_vfio_device *vfio_dev,
				       u32 operand, u16 *result)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	struct idxd_vfio_msix_entry *entry;
	unsigned int vector = operand & GENMASK(15, 0);
	bool was_revoked, reprogram;
	int handle, rc;

	atomic64_inc(&vfio_dev->ims_stats.request_cmds);

	if (vector >= idxd_vfio_msix_count(vfio_dev)) {
		atomic64_inc(&vfio_dev->ims_stats.request_errors);
		return IDXD_CMDSTS_ERR_INVAL_INT_IDX;
	}
	if (!idxd_vfio_msix_vector_uses_ims(vector)) {
		atomic64_inc(&vfio_dev->ims_stats.request_errors);
		atomic64_inc(&vfio_dev->ims_stats.request_vector0_rejects);
		return IDXD_CMDSTS_ERR_INVAL_INT_IDX;
	}
	if (!idxd_vfio_ims_supported(idxd) || vector >= idxd->irq_cnt) {
		atomic64_inc(&vfio_dev->ims_stats.request_errors);
		return IDXD_CMDSTS_ERR_NO_HANDLE;
	}

	entry = &vfio_dev->msix_entries[vector];

	mutex_lock(&vfio_dev->irq_lock);
	if (entry->ims_allocated) {
		*result = entry->ims_handle;
		atomic64_inc(&vfio_dev->ims_stats.duplicate_requests);
		mutex_unlock(&vfio_dev->irq_lock);
		return IDXD_CMDSTS_SUCCESS;
	}
	mutex_unlock(&vfio_dev->irq_lock);

	if (idxd->request_int_handles) {
		rc = idxd_device_request_int_handle(idxd, vector, &handle,
						    IDXD_IRQ_IMS);
		if (rc) {
			atomic64_inc(&vfio_dev->ims_stats.request_errors);
			return IDXD_CMDSTS_ERR_NO_HANDLE;
		}
	} else {
		handle = vector;
	}

	mutex_lock(&vfio_dev->irq_lock);
	if (entry->ims_allocated) {
		*result = entry->ims_handle;
		atomic64_inc(&vfio_dev->ims_stats.duplicate_requests);
		mutex_unlock(&vfio_dev->irq_lock);
		if (idxd->request_int_handles)
			idxd_device_release_int_handle(idxd, handle,
						       IDXD_IRQ_IMS);
		return IDXD_CMDSTS_SUCCESS;
	}

	was_revoked = entry->ims_revoked;
	reprogram = was_revoked && entry->ims_configured;
	entry->ims_handle = handle;
	entry->ims_index = handle;
	entry->host_msix_vector = vector;
	entry->ims_allocated = true;
	entry->ims_revoked = false;
	if (reprogram) {
		bool deferred = false;

		rc = idxd_vfio_try_program_ims_locked(vfio_dev, entry,
						      &deferred);
		if (rc) {
			idxd_vfio_forget_ims_handle(entry);
			entry->ims_revoked = true;
			mutex_unlock(&vfio_dev->irq_lock);
			if (idxd->request_int_handles)
				idxd_device_release_int_handle(idxd, handle,
							       IDXD_IRQ_IMS);
			atomic64_inc(&vfio_dev->ims_stats.request_errors);
			return IDXD_CMDSTS_ERR_NO_HANDLE;
		}
		if (deferred)
			atomic64_inc(&vfio_dev->ims_stats.program_deferred);
		else
			atomic64_inc(&vfio_dev->ims_stats.revoked_reprograms);
	} else {
		idxd_vfio_clear_ims_entry(vfio_dev, entry);
	}
	atomic64_inc(&vfio_dev->ims_stats.handles_allocated);
	*result = handle;
	mutex_unlock(&vfio_dev->irq_lock);

	return IDXD_CMDSTS_SUCCESS;
}

static u8 idxd_vfio_release_int_handle(struct idxd_vfio_device *vfio_dev,
				       u32 operand)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	unsigned int handle = operand & GENMASK(15, 0);
	unsigned int i;

	atomic64_inc(&vfio_dev->ims_stats.release_cmds);

	mutex_lock(&vfio_dev->irq_lock);
	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		struct idxd_vfio_msix_entry *entry = &vfio_dev->msix_entries[i];

		if (!entry->ims_allocated || entry->ims_handle != handle)
			continue;

		idxd_vfio_clear_ims_entry(vfio_dev, entry);
		idxd_vfio_assert_ims_entry_clear_locked(vfio_dev, entry,
							"release-int-handle-clear",
							false, false);
		idxd_vfio_forget_ims_handle(entry);
		entry->pending = false;
		mutex_unlock(&vfio_dev->irq_lock);

		idxd_vfio_free_ims_irq(entry);
		if (idxd->request_int_handles &&
		    idxd_device_release_int_handle(idxd, handle, IDXD_IRQ_IMS)) {
			atomic64_inc(&vfio_dev->ims_stats.release_errors);
			return IDXD_CMDSTS_HW_ERR;
		}
		atomic64_inc(&vfio_dev->ims_stats.handles_released);
		idxd_vfio_assert_ims_entry_clear(vfio_dev, entry,
						 "release-int-handle",
						 true, true);
		return IDXD_CMDSTS_SUCCESS;
	}
	mutex_unlock(&vfio_dev->irq_lock);

	atomic64_inc(&vfio_dev->ims_stats.release_errors);
	atomic64_inc(&vfio_dev->ims_stats.release_unallocated);
	return IDXD_CMDSTS_ERR_INVAL_INT_IDX;
}

static int idxd_vfio_revoke_ims_vector(struct idxd_vfio_device *vfio_dev,
				       unsigned int vector)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	struct idxd_vfio_msix_entry *entry;
	int handle;

	if (vector >= idxd_vfio_msix_count(vfio_dev) ||
	    !idxd_vfio_msix_vector_uses_ims(vector))
		return -EINVAL;

	entry = &vfio_dev->msix_entries[vector];

	mutex_lock(&vfio_dev->irq_lock);
	if (!entry->ims_allocated) {
		mutex_unlock(&vfio_dev->irq_lock);
		return -ENOENT;
	}

	handle = entry->ims_handle;
	idxd_vfio_clear_ims_entry_hw(vfio_dev, entry);
	entry->ims_programmed = false;
	idxd_vfio_forget_ims_handle(entry);
	entry->pending = false;
	entry->ims_revoked = true;
	mutex_unlock(&vfio_dev->irq_lock);

	idxd_vfio_free_ims_irq(entry);
	if (idxd->request_int_handles &&
	    idxd_device_release_int_handle(idxd, handle, IDXD_IRQ_IMS))
		return -EIO;

	atomic64_inc(&vfio_dev->ims_stats.revoked_vectors);
	return 0;
}

static int idxd_vfio_revoke_ims_handles(struct idxd_vfio_device *vfio_dev,
					unsigned int vector)
{
	unsigned int i, start, end;
	unsigned int revoked = 0;
	int rc = 0;

	atomic64_inc(&vfio_dev->ims_stats.revoke_cmds);

	if (vector == VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_VECTOR_ALL) {
		start = 0;
		end = idxd_vfio_msix_count(vfio_dev);
	} else {
		start = vector;
		end = vector + 1;
	}

	for (i = start; i < end; i++) {
		if (!idxd_vfio_msix_vector_uses_ims(i))
			continue;

		rc = idxd_vfio_revoke_ims_vector(vfio_dev, i);
		if (!rc) {
			revoked++;
			continue;
		}
		if (rc == -ENOENT &&
		    vector == VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_VECTOR_ALL) {
			rc = 0;
			continue;
		}
		break;
	}

	if (rc) {
		atomic64_inc(&vfio_dev->ims_stats.revoke_errors);
		return rc;
	}
	if (!revoked) {
		atomic64_inc(&vfio_dev->ims_stats.revoke_errors);
		return -ENOENT;
	}

	mutex_lock(&vfio_dev->bar0_lock);
	vfio_dev->intcause |= IDXD_INTC_INT_HANDLE_REVOKED;
	mutex_unlock(&vfio_dev->bar0_lock);
	idxd_vfio_msix_signal(vfio_dev, 0);

	return 0;
}

static int
idxd_vfio_ims_selftest_find_idle_vector(struct idxd_vfio_device *vfio_dev,
					unsigned int *vector)
{
	unsigned int i;
	int rc = 0;

	mutex_lock(&vfio_dev->pasid_lock);
	if (vfio_dev->idev)
		rc = -EBUSY;
	mutex_unlock(&vfio_dev->pasid_lock);
	if (rc)
		return rc;

	mutex_lock(&vfio_dev->irq_lock);
	*vector = UINT_MAX;
	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		struct idxd_vfio_msix_entry *entry = &vfio_dev->msix_entries[i];

		if (vfio_dev->msix_trigger[i] || entry->ims_configured ||
		    entry->ims_programmed || entry->host_irq_requested) {
			rc = -EBUSY;
			break;
		}

		if (*vector == UINT_MAX &&
		    idxd_vfio_msix_vector_uses_ims(i) &&
		    entry->ims_allocated)
			*vector = i;
	}
	mutex_unlock(&vfio_dev->irq_lock);

	if (rc)
		return rc;
	if (*vector == UINT_MAX)
		return -ENODEV;

	return 0;
}

static int idxd_vfio_run_ims_selftest(struct idxd_vfio_device *vfio_dev)
{
	struct device *dev = vdev_confdev(vfio_dev->ivdev);
	struct idxd_vfio_msix_entry *entry;
	unsigned int vector;
	u16 handle, duplicate;
	u8 status;
	int rc;

	atomic64_inc(&vfio_dev->ims_stats.selftest_runs);

	rc = idxd_vfio_ims_selftest_find_idle_vector(vfio_dev, &vector);
	if (rc)
		goto fail;

	status = idxd_vfio_request_int_handle(vfio_dev, 0, &handle);
	if (status != IDXD_CMDSTS_ERR_INVAL_INT_IDX) {
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: vector 0 request returned %#x\n",
			 status);
		goto fail;
	}

	status = idxd_vfio_request_int_handle(vfio_dev,
					      idxd_vfio_msix_count(vfio_dev),
					      &handle);
	if (status != IDXD_CMDSTS_ERR_INVAL_INT_IDX) {
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: invalid vector request returned %#x\n",
			 status);
		goto fail;
	}

	status = idxd_vfio_release_int_handle(vfio_dev, INVALID_INT_HANDLE);
	if (status != IDXD_CMDSTS_ERR_INVAL_INT_IDX) {
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: invalid handle release returned %#x\n",
			 status);
		goto fail;
	}

	status = idxd_vfio_request_int_handle(vfio_dev, vector, &handle);
	if (status != IDXD_CMDSTS_SUCCESS) {
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: vector %u request returned %#x\n",
			 vector, status);
		goto fail;
	}

	status = idxd_vfio_request_int_handle(vfio_dev, vector, &duplicate);
	if (status != IDXD_CMDSTS_SUCCESS || duplicate != handle) {
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: duplicate request status %#x handle %u expected %u\n",
			 status, duplicate, handle);
		goto fail;
	}

	entry = &vfio_dev->msix_entries[vector];
	if (!idxd_vfio_assert_ims_entry_clear(vfio_dev, entry,
					      "selftest-duplicate-request",
					      true, false)) {
		rc = -EIO;
		goto fail;
	}

	status = idxd_vfio_release_int_handle(vfio_dev, handle);
	if (status != IDXD_CMDSTS_SUCCESS) {
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: handle %u release returned %#x\n",
			 handle, status);
		goto fail;
	}

	if (!idxd_vfio_assert_ims_entry_clear(vfio_dev, entry,
					      "selftest-release",
					      true, true)) {
		rc = -EIO;
		goto fail;
	}

	status = idxd_vfio_release_int_handle(vfio_dev, handle);
	if (status != IDXD_CMDSTS_ERR_INVAL_INT_IDX) {
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: duplicate release returned %#x\n",
			 status);
		goto fail;
	}

	status = idxd_vfio_request_int_handle(vfio_dev, vector, &handle);
	if (status != IDXD_CMDSTS_SUCCESS) {
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: vector %u re-request returned %#x\n",
			 vector, status);
		goto fail;
	}

	if (!idxd_vfio_assert_ims_entry_clear(vfio_dev, entry,
					      "selftest-rerequest",
					      true, false)) {
		rc = -EIO;
		goto fail;
	}

	rc = idxd_vfio_revoke_ims_handles(vfio_dev, vector);
	if (rc) {
		dev_warn(dev,
			 "IMS selftest failed: vector %u revoke returned %d\n",
			 vector, rc);
		goto fail;
	}

	mutex_lock(&vfio_dev->irq_lock);
	if (!entry->ims_revoked || entry->ims_allocated) {
		mutex_unlock(&vfio_dev->irq_lock);
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: vector %u revoke state invalid\n",
			 vector);
		goto fail;
	}
	mutex_unlock(&vfio_dev->irq_lock);

	mutex_lock(&vfio_dev->bar0_lock);
	if (!(vfio_dev->intcause & IDXD_INTC_INT_HANDLE_REVOKED)) {
		mutex_unlock(&vfio_dev->bar0_lock);
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: revoke did not set INTCAUSE\n");
		goto fail;
	}
	vfio_dev->intcause &= ~IDXD_INTC_INT_HANDLE_REVOKED;
	mutex_unlock(&vfio_dev->bar0_lock);

	mutex_lock(&vfio_dev->irq_lock);
	vfio_dev->msix_entries[0].pending = false;
	mutex_unlock(&vfio_dev->irq_lock);

	status = idxd_vfio_request_int_handle(vfio_dev, vector, &handle);
	if (status != IDXD_CMDSTS_SUCCESS) {
		rc = -EIO;
		dev_warn(dev,
			 "IMS selftest failed: vector %u post-revoke request returned %#x\n",
			 vector, status);
		goto fail;
	}

	if (!idxd_vfio_assert_ims_entry_clear(vfio_dev, entry,
					      "selftest-post-revoke-request",
					      true, false)) {
		rc = -EIO;
		goto fail;
	}

	dev_info(dev, "IMS selftest passed on vector %u handle %u\n",
		 vector, handle);
	return 0;

fail:
	atomic64_inc(&vfio_dev->ims_stats.selftest_failures);
	return rc;
}

static void idxd_vfio_exec_cmd(struct idxd_vfio_device *vfio_dev, u32 val)
{
	union idxd_command_reg cmd = { .bits = val };
	u8 status = IDXD_CMDSTS_SUCCESS;
	u16 result = 0;

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
	case IDXD_CMD_REQUEST_INT_HANDLE:
		status = idxd_vfio_request_int_handle(vfio_dev, cmd.operand,
						      &result);
		break;
	case IDXD_CMD_RELEASE_INT_HANDLE:
		status = idxd_vfio_release_int_handle(vfio_dev, cmd.operand);
		break;
	default:
		status = IDXD_CMDSTS_INVAL_CMD;
		break;
	}

	idxd_vfio_complete_cmd(vfio_dev, cmd, status, result);
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

static bool idxd_vfio_msix_table_write(struct idxd_vfio_device *vfio_dev,
				       loff_t pos, u32 val)
{
	struct idxd_vfio_msix_entry *entry;
	u32 table_size, offset, field;
	unsigned int vector;

	table_size = idxd_vfio_msix_count(vfio_dev) * PCI_MSIX_ENTRY_SIZE;
	if (pos < IDXD_VDEV_MSIX_TABLE_OFFSET ||
	    pos >= IDXD_VDEV_MSIX_TABLE_OFFSET + table_size)
		return false;
	if (!IS_ALIGNED(pos, sizeof(u32)))
		return true;

	offset = pos - IDXD_VDEV_MSIX_TABLE_OFFSET;
	vector = offset / PCI_MSIX_ENTRY_SIZE;
	field = offset % PCI_MSIX_ENTRY_SIZE;
	entry = &vfio_dev->msix_entries[vector];

	mutex_lock(&vfio_dev->irq_lock);
	switch (field) {
	case PCI_MSIX_ENTRY_LOWER_ADDR:
		entry->msg_addr_lo = val;
		if (entry->ims_programmed)
			idxd_vfio_write_ims_entry(vfio_dev, entry);
		break;
	case PCI_MSIX_ENTRY_UPPER_ADDR:
		entry->msg_addr_hi = val;
		if (entry->ims_programmed)
			idxd_vfio_write_ims_entry(vfio_dev, entry);
		break;
	case PCI_MSIX_ENTRY_DATA:
		entry->msg_data = val;
		if (entry->ims_programmed)
			idxd_vfio_write_ims_entry(vfio_dev, entry);
		break;
	case PCI_MSIX_ENTRY_VECTOR_CTRL:
		entry->vector_ctrl = val;
		if (entry->ims_programmed)
			idxd_vfio_write_ims_entry(vfio_dev, entry);
		idxd_vfio_msix_flush_pending_locked(vfio_dev, vector);
		break;
	default:
		break;
	}
	mutex_unlock(&vfio_dev->irq_lock);

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
		if (idxd_vfio_msix_table_write(vfio_dev, pos, val))
			goto out_unlock;
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

static bool idxd_vfio_bar2_mmap_portal(struct idxd_vwq *vwq,
				       enum idxd_vfio_bar2_portal portal,
				       unsigned int *phys_portal)
{
	switch (portal) {
	case IDXD_VFIO_BAR2_LIMITED:
		/*
		 * The VDEV exposes virtual MSI-X portals to the guest but maps
		 * them to physical IMS portals. Keep GENCAP.IMS hidden for now.
		 */
		*phys_portal = IDXD_VFIO_BAR2_IMS;
		return true;
	case IDXD_VFIO_BAR2_UNLIMITED:
		if (vwq->shared)
			return false;
		*phys_portal = IDXD_VFIO_BAR2_IMS;
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

static void idxd_vfio_reset_bar2(struct idxd_vfio_device *vfio_dev)
{
	LIST_HEAD(pending);
	unsigned int i;

	spin_lock(&vfio_dev->bar2_forward_lock);
	vfio_dev->bar2_forward_stopping = true;
	list_splice_init(&vfio_dev->bar2_forward_list, &pending);
	spin_unlock(&vfio_dev->bar2_forward_lock);

	flush_work(&vfio_dev->bar2_forward_work);

	spin_lock(&vfio_dev->bar2_forward_lock);
	vfio_dev->bar2_forward_work_active = false;
	vfio_dev->bar2_forward_stopping = false;
	spin_unlock(&vfio_dev->bar2_forward_lock);

	idxd_vfio_drop_bar2_forward_list(vfio_dev, &pending);

	mutex_lock(&vfio_dev->bar2_lock);
	if (vfio_dev->shared_unlimited_descs) {
		for (i = 0; i < vfio_dev->ivdev->num_wqs; i++) {
			if (!vfio_dev->shared_unlimited_descs[i])
				continue;
			memset(vfio_dev->shared_unlimited_descs[i], 0,
			       IDXD_VFIO_BAR2_DESC_SLOTS *
			       sizeof(**vfio_dev->shared_unlimited_descs));
		}
	}
	mutex_unlock(&vfio_dev->bar2_lock);

	atomic_set(&vfio_dev->bar2_force_unlimited_retries, 0);
	idxd_vfio_reset_bar2_stats(vfio_dev);
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
		if (pos <= IDXD_VDEV_MSIX_CAP_OFFSET + PCI_MSIX_FLAGS + 1 &&
		    pos + ret > IDXD_VDEV_MSIX_CAP_OFFSET + PCI_MSIX_FLAGS)
			idxd_vfio_msix_flush_all_pending(vfio_dev);
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

	if (info.index != VFIO_PCI_MSIX_IRQ_INDEX)
		return -EINVAL;

	info.flags = VFIO_IRQ_INFO_EVENTFD | VFIO_IRQ_INFO_NORESIZE;
	info.count = idxd_vfio_msix_count(vfio_dev);

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
	unsigned int phys_portal;
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

	if (portal_offset ||
	    !idxd_vfio_bar2_mmap_portal(vwq, portal, &phys_portal))
		return -EINVAL;

	if (!idxd_vfio_get_default_pasid(vfio_dev, NULL))
		return -EACCES;

	if (!idxd_vfio_portal_access_allowed(vwq->wq))
		return -EPERM;

	paddr = pci_resource_start(vwq->wq->idxd->pdev, IDXD_WQ_BAR);
	paddr += ((vwq->wq->id * IDXD_VDEV_PORTALS_PER_WQ + phys_portal) <<
		  PAGE_SHIFT);

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

	idxd_vfio_clear_all_ims_entries(vfio_dev);
	idxd_vfio_free_unused_ims_irqs(vfio_dev);
	idxd_vfio_assert_all_ims_clear(vfio_dev, "iommufd-unbind", true);

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
	int ims_rc;
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
	if (!rc) {
		ims_rc = idxd_vfio_program_pending_ims_for_pasid(vfio_dev,
								 pasid);
		if (ims_rc)
			dev_warn(vdev_confdev(vfio_dev->ivdev),
				 "failed to program deferred IMS entries for PASID %u: %d\n",
				 pasid, ims_rc);
	}

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

	idxd_vfio_clear_ims_entries_for_pasid(vfio_dev, entry->host_pasid);
	idxd_vfio_free_unused_ims_irqs(vfio_dev);
	idxd_vfio_assert_no_ims_for_pasid(vfio_dev, entry->host_pasid,
					  "pasid-detach");

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

static int idxd_vfio_program_ims_entry(struct idxd_vfio_device *vfio_dev,
				       struct vfio_device_feature_idxd_siov_ims *ctrl)
{
	struct idxd_vfio_msix_entry *entry;
	ioasid_t host_pasid = ctrl->host_pasid;
	bool deferred = false;
	int rc = 0;

	atomic64_inc(&vfio_dev->ims_stats.program_cmds);

	if (ctrl->vector >= idxd_vfio_msix_count(vfio_dev)) {
		atomic64_inc(&vfio_dev->ims_stats.program_errors);
		return -EINVAL;
	}
	if (ctrl->flags & ~IDXD_VFIO_IMS_VALID_FLAGS) {
		atomic64_inc(&vfio_dev->ims_stats.program_errors);
		return -EINVAL;
	}

	entry = &vfio_dev->msix_entries[ctrl->vector];

	if (!idxd_vfio_msix_vector_uses_ims(ctrl->vector)) {
		mutex_lock(&vfio_dev->irq_lock);
		entry->msg_addr_lo = lower_32_bits(ctrl->msg_addr);
		entry->msg_addr_hi = upper_32_bits(ctrl->msg_addr);
		entry->msg_data = ctrl->msg_data;
		entry->host_pasid = IOMMU_PASID_INVALID;
		entry->ims_ignore =
			!!(ctrl->flags &
			   VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_IGNORE);
		if (ctrl->flags & VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_MASK)
			entry->vector_ctrl |= PCI_MSIX_ENTRY_CTRL_MASKBIT;
		else
			entry->vector_ctrl &= ~PCI_MSIX_ENTRY_CTRL_MASKBIT;
		idxd_vfio_msix_flush_pending_locked(vfio_dev, ctrl->vector);
		mutex_unlock(&vfio_dev->irq_lock);
		return 0;
	}

	if (!(ctrl->flags & VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_PASID)) {
		if (!idxd_vfio_get_default_pasid(vfio_dev, &host_pasid)) {
			atomic64_inc(&vfio_dev->ims_stats.program_errors);
			return -ENOENT;
		}
	} else if (!idxd_vfio_valid_pasid(host_pasid)) {
		atomic64_inc(&vfio_dev->ims_stats.program_errors);
		return -EINVAL;
	}

	mutex_lock(&vfio_dev->irq_lock);
	entry->msg_addr_lo = lower_32_bits(ctrl->msg_addr);
	entry->msg_addr_hi = upper_32_bits(ctrl->msg_addr);
	entry->msg_data = ctrl->msg_data;
	entry->host_pasid = host_pasid;
	entry->ims_configured = true;
	entry->ims_ignore =
		!!(ctrl->flags & VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_IGNORE);
	if (ctrl->flags & VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_MASK)
		entry->vector_ctrl |= PCI_MSIX_ENTRY_CTRL_MASKBIT;
	else
		entry->vector_ctrl &= ~PCI_MSIX_ENTRY_CTRL_MASKBIT;

	rc = idxd_vfio_try_program_ims_locked(vfio_dev, entry, &deferred);
	if (deferred)
		atomic64_inc(&vfio_dev->ims_stats.program_deferred);

	mutex_unlock(&vfio_dev->irq_lock);
	if (rc)
		atomic64_inc(&vfio_dev->ims_stats.program_errors);
	return rc;
}

static int idxd_vfio_clear_ims_vector(struct idxd_vfio_device *vfio_dev,
				      unsigned int vector)
{
	struct idxd_vfio_msix_entry *entry;

	atomic64_inc(&vfio_dev->ims_stats.clear_cmds);

	if (vector >= idxd_vfio_msix_count(vfio_dev)) {
		atomic64_inc(&vfio_dev->ims_stats.clear_errors);
		return -EINVAL;
	}

	entry = &vfio_dev->msix_entries[vector];

	mutex_lock(&vfio_dev->irq_lock);
	idxd_vfio_clear_ims_entry(vfio_dev, entry);
	entry->pending = false;
	mutex_unlock(&vfio_dev->irq_lock);
	idxd_vfio_free_ims_irq(entry);
	idxd_vfio_assert_ims_entry_clear(vfio_dev, entry, "ims-clear",
					 true, false);

	return 0;
}

static int idxd_vfio_get_ims_entry(struct idxd_vfio_device *vfio_dev,
				   struct vfio_device_feature_idxd_siov_ims *ctrl)
{
	struct idxd_vfio_msix_entry *entry;
	u32 ims_ctrl;

	if (ctrl->vector >= idxd_vfio_msix_count(vfio_dev))
		return -EINVAL;

	entry = &vfio_dev->msix_entries[ctrl->vector];

	mutex_lock(&vfio_dev->irq_lock);
	ctrl->msg_addr = ((u64)entry->msg_addr_hi << 32) | entry->msg_addr_lo;
	ctrl->msg_data = entry->msg_data;
	ctrl->host_pasid = entry->host_pasid;
	ctrl->flags = 0;
	if (entry->vector_ctrl & PCI_MSIX_ENTRY_CTRL_MASKBIT)
		ctrl->flags |= VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_MASK;
	if (entry->ims_ignore)
		ctrl->flags |= VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_IGNORE;
	if (entry->ims_configured)
		ctrl->flags |= VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_PASID;
	if (entry->ims_revoked)
		ctrl->flags |= VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_REVOKED;
	ims_ctrl = idxd_vfio_read_ims_entry_ctrl(vfio_dev, entry);
	if (entry->pending || (ims_ctrl & IDXD_VFIO_IMS_CTRL_PENDING))
		ctrl->flags |= VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_F_PENDING;
	mutex_unlock(&vfio_dev->irq_lock);

	return 0;
}

static int idxd_vfio_ims_feature(struct vfio_device *vdev, u32 flags,
				 void __user *arg, size_t argsz)
{
	size_t minsz =
		offsetofend(struct vfio_device_feature_idxd_siov_ims,
			    __reserved);
	struct idxd_vfio_device *vfio_dev =
		container_of(vdev, struct idxd_vfio_device, vdev);
	struct vfio_device_feature_idxd_siov_ims ctrl;
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
		case VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_PROGRAM:
			return idxd_vfio_program_ims_entry(vfio_dev, &ctrl);
		case VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_CLEAR:
			return idxd_vfio_clear_ims_vector(vfio_dev, ctrl.vector);
		case VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_REVOKE:
			return idxd_vfio_revoke_ims_handles(vfio_dev,
							    ctrl.vector);
		default:
			return -EINVAL;
		}
	}

	if (ctrl.op != VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS_GET)
		return -EINVAL;

	rc = idxd_vfio_get_ims_entry(vfio_dev, &ctrl);
	if (rc)
		return rc;

	return copy_to_user(arg, &ctrl, minsz) ? -EFAULT : 0;
}

static int idxd_vfio_ioctl_feature(struct vfio_device *vdev, u32 flags,
				   void __user *arg, size_t argsz)
{
	switch (flags & VFIO_DEVICE_FEATURE_MASK) {
	case VFIO_DEVICE_FEATURE_IDXD_SIOV_PASID:
		return idxd_vfio_pasid_feature(vdev, flags, arg, argsz);
	case VFIO_DEVICE_FEATURE_IDXD_SIOV_IMS:
		return idxd_vfio_ims_feature(vdev, flags, arg, argsz);
	default:
		return -ENOTTY;
	}
}

static int idxd_vfio_reset_device(struct idxd_vfio_device *vfio_dev)
{
	int rc;

	idxd_vfio_reset_bar2(vfio_dev);

	mutex_lock(&vfio_dev->bar0_lock);
	rc = idxd_vfio_restore_all_wq_pasids(vfio_dev);
	if (!rc) {
		idxd_vfio_init_config(vfio_dev);
		idxd_vfio_reset_bar0(vfio_dev);
	}
	mutex_unlock(&vfio_dev->bar0_lock);
	idxd_vfio_free_unused_ims_irqs(vfio_dev);
	idxd_vfio_assert_all_ims_clear(vfio_dev, "device-reset", true);

	return rc;
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

		info.flags = VFIO_DEVICE_FLAGS_PCI | VFIO_DEVICE_FLAGS_RESET;
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
	case VFIO_DEVICE_RESET:
		return idxd_vfio_reset_device(vfio_dev);
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
	idxd_vfio_clear_all_ims_entries(vfio_dev);
	idxd_vfio_free_unused_ims_irqs(vfio_dev);
	idxd_vfio_assert_all_ims_clear(vfio_dev, "device-close", true);
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
	unsigned int i;
	int rc;

	vfio_dev->msix_trigger =
		kcalloc(idxd_vfio_msix_count(vfio_dev),
			sizeof(*vfio_dev->msix_trigger), GFP_KERNEL);
	if (!vfio_dev->msix_trigger)
		return -ENOMEM;

	vfio_dev->msix_entries =
		kcalloc(idxd_vfio_msix_count(vfio_dev),
			sizeof(*vfio_dev->msix_entries), GFP_KERNEL);
	if (!vfio_dev->msix_entries) {
		kfree(vfio_dev->msix_trigger);
		vfio_dev->msix_trigger = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		vfio_dev->msix_entries[i].vfio_dev = vfio_dev;
		vfio_dev->msix_entries[i].vector = i;
		vfio_dev->msix_entries[i].host_msix_vector = UINT_MAX;
		vfio_dev->msix_entries[i].ims_index = UINT_MAX;
		vfio_dev->msix_entries[i].ims_handle = INVALID_INT_HANDLE;
		vfio_dev->msix_entries[i].host_irq = -1;
		vfio_dev->msix_entries[i].vector_ctrl =
			PCI_MSIX_ENTRY_CTRL_MASKBIT;
		vfio_dev->msix_entries[i].host_pasid = IOMMU_PASID_INVALID;
		vfio_dev->msix_entries[i].ims_ignore = true;
	}

	rc = idxd_vfio_alloc_ims_handles(vfio_dev);
	if (rc) {
		kfree(vfio_dev->msix_entries);
		vfio_dev->msix_entries = NULL;
		kfree(vfio_dev->msix_trigger);
		vfio_dev->msix_trigger = NULL;
	}

	return rc;
}

static void idxd_vfio_release_ims_handles(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	unsigned int i;

	if (!vfio_dev->msix_entries)
		return;

	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		struct idxd_vfio_msix_entry *entry = &vfio_dev->msix_entries[i];

		if (!entry->ims_allocated)
			continue;

		idxd_vfio_clear_ims_entry(vfio_dev, entry);
		idxd_vfio_assert_ims_entry_clear(vfio_dev, entry,
						 "release-ims-handle-clear",
						 false, false);
		idxd_vfio_free_ims_irq(entry);
		if (idxd->request_int_handles)
			idxd_device_release_int_handle(idxd, entry->ims_handle,
						       IDXD_IRQ_IMS);
		idxd_vfio_forget_ims_handle(entry);
		entry->pending = false;
		atomic64_inc(&vfio_dev->ims_stats.handles_released);
	}
	idxd_vfio_clear_all_ims_entries(vfio_dev);
	idxd_vfio_assert_all_ims_clear(vfio_dev, "release-ims-handles", true);
}

static int idxd_vfio_alloc_ims_handles(struct idxd_vfio_device *vfio_dev)
{
	struct idxd_device *idxd = vfio_dev->ivdev->idxd;
	struct device *dev = vdev_confdev(vfio_dev->ivdev);
	unsigned int i;
	int rc;

	if (!idxd_vfio_ims_supported(idxd))
		return -EOPNOTSUPP;

	for (i = 0; i < idxd_vfio_msix_count(vfio_dev); i++) {
		struct idxd_vfio_msix_entry *entry = &vfio_dev->msix_entries[i];
		int handle;

		if (!idxd_vfio_msix_vector_uses_ims(i))
			continue;
		if (i >= idxd->irq_cnt) {
			dev_err(dev,
				"no host MSI-X vector for virtual vector %u\n",
				i);
			idxd_vfio_release_ims_handles(vfio_dev);
			return -ENOSPC;
		}

		if (idxd->request_int_handles) {
			rc = idxd_device_request_int_handle(idxd, i, &handle,
							    IDXD_IRQ_IMS);
			if (rc) {
				dev_err(dev,
					"failed to allocate IMS handle for vector %u: %d\n",
					i, rc);
				idxd_vfio_release_ims_handles(vfio_dev);
				return rc;
			}
		} else {
			handle = i;
		}

		entry->ims_handle = handle;
		entry->ims_index = handle;
		entry->host_msix_vector = i;
		entry->ims_allocated = true;
		idxd_vfio_clear_ims_entry(vfio_dev, entry);
		atomic64_inc(&vfio_dev->ims_stats.handles_allocated);
	}

	return 0;
}

static void idxd_vfio_free_irqs(struct idxd_vfio_device *vfio_dev)
{
	idxd_vfio_release_ims_handles(vfio_dev);
	idxd_vfio_msix_disable(vfio_dev);
	kfree(vfio_dev->msix_entries);
	vfio_dev->msix_entries = NULL;
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
