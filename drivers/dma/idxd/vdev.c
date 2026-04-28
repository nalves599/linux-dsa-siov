// SPDX-License-Identifier: GPL-2.0
/* Copyright(c) 2026 Intel Corporation. All rights rsvd. */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "idxd.h"

#define IDXD_VDEV_MODE_SHARED		"shared"
#define IDXD_VDEV_MODE_EXCLUSIVE	"exclusive"

static void idxd_vwq_put(struct idxd_vwq *vwq)
{
	struct idxd_wq *wq = vwq->wq;

	mutex_lock(&wq->wq_lock);
	idxd_wq_put(wq);
	mutex_unlock(&wq->wq_lock);
	put_device(wq_confdev(wq));
	kfree(vwq);
}

static void idxd_vdev_release(struct device *dev)
{
	struct idxd_vdev *vdev = confdev_to_vdev(dev);
	struct idxd_vwq *vwq, *tmp;
	struct idxd_device *idxd = vdev->idxd;

	list_for_each_entry_safe(vwq, tmp, &vdev->wqs, node) {
		list_del(&vwq->node);
		idxd_vwq_put(vwq);
	}

	ida_free(&idxd->vdev_ida, vdev->id);
	mutex_destroy(&vdev->lock);
	kfree(vdev);
}

static const struct device_type idxd_vdev_device_type = {
	.name = "idxd_vfio",
	.release = idxd_vdev_release,
};

static const char *idxd_vdev_wq_type_name(enum idxd_wq_type type)
{
	switch (type) {
	case IDXD_WQT_NONE:
		return "none";
	case IDXD_WQT_KERNEL:
		return "kernel";
	case IDXD_WQT_USER:
		return "user";
	default:
		return "unknown";
	}
}

static const char *idxd_vdev_wq_state_name(enum idxd_wq_state state)
{
	switch (state) {
	case IDXD_WQ_DISABLED:
		return "disabled";
	case IDXD_WQ_ENABLED:
		return "enabled";
	default:
		return "unknown";
	}
}

static const char *idxd_vdev_wq_mode_name(bool shared)
{
	return shared ? IDXD_VDEV_MODE_SHARED : IDXD_VDEV_MODE_EXCLUSIVE;
}

static bool idxd_vdev_has_wq(struct idxd_vdev *vdev, struct idxd_wq *wq)
{
	struct idxd_vwq *vwq;

	list_for_each_entry(vwq, &vdev->wqs, node) {
		if (vwq->wq == wq)
			return true;
	}

	return false;
}

static bool idxd_vdev_wq_conflicts(struct idxd_device *idxd, struct idxd_wq *wq,
				   bool shared, struct idxd_vdev **owner,
				   bool *owner_shared)
{
	struct idxd_vdev *vdev;
	struct idxd_vwq *vwq;

	lockdep_assert_held(&idxd->vdev_lock);

	list_for_each_entry(vdev, &idxd->vdev_list, list) {
		list_for_each_entry(vwq, &vdev->wqs, node) {
			if (vwq->wq != wq)
				continue;
			if (!shared || !vwq->shared) {
				if (owner)
					*owner = vdev;
				if (owner_shared)
					*owner_shared = vwq->shared;
				return true;
			}
		}
	}

	return false;
}

static int idxd_vdev_parse_wq_token(struct idxd_device *idxd, char *token,
				    struct idxd_wq **wq, bool *shared)
{
	char *mode;
	int idxd_id, wq_id;
	int consumed = 0;
	int rc;

	mode = strchr(token, ':');
	if (mode) {
		*mode++ = '\0';
		mode = strim(mode);
	}

	token = strim(token);
	if (sscanf(token, "wq%d.%d%n", &idxd_id, &wq_id, &consumed) != 2 ||
	    token[consumed]) {
		idxd_id = idxd->id;
		rc = kstrtoint(token, 0, &wq_id);
		if (rc) {
			dev_err(idxd_confdev(idxd),
				"invalid VDEV WQ token '%s'\n", token);
			return -EINVAL;
		}
	}

	if (idxd_id != idxd->id) {
		dev_err(idxd_confdev(idxd),
			"cannot assign WQ wq%d.%d from dsa%d to dsa%d VDEV\n",
			idxd_id, wq_id, idxd_id, idxd->id);
		return -EINVAL;
	}

	if (wq_id < 0 || wq_id >= idxd->max_wqs) {
		dev_err(idxd_confdev(idxd),
			"WQ id %d is outside dsa%d WQ range 0-%d\n",
			wq_id, idxd->id, idxd->max_wqs - 1);
		return -EINVAL;
	}

	*wq = idxd->wqs[wq_id];

	if (!mode)
		*shared = wq_shared(*wq);
	else if (sysfs_streq(mode, IDXD_VDEV_MODE_SHARED))
		*shared = true;
	else if (sysfs_streq(mode, IDXD_VDEV_MODE_EXCLUSIVE))
		*shared = false;
	else {
		dev_err(idxd_confdev(idxd),
			"invalid VDEV WQ mode '%s' for wq%d.%d\n",
			mode, idxd->id, wq_id);
		return -EINVAL;
	}

	return 0;
}

static int idxd_vdev_add_wq(struct idxd_vdev *vdev, struct idxd_wq *wq,
			    bool shared)
{
	struct idxd_device *idxd = vdev->idxd;
	struct idxd_vdev *owner = NULL;
	struct idxd_vwq *vwq;
	bool owner_shared = false;
	int rc = 0;

	if (idxd_vdev_has_wq(vdev, wq)) {
		dev_err(idxd_confdev(idxd),
			"wq%d.%d appears more than once in the VDEV request\n",
			idxd->id, wq->id);
		return -EEXIST;
	}

	mutex_lock(&idxd->vdev_lock);
	if (idxd_vdev_wq_conflicts(idxd, wq, shared, &owner, &owner_shared)) {
		dev_err(idxd_confdev(idxd),
			"wq%d.%d is already assigned to vdev%d.%d as %s\n",
			idxd->id, wq->id, idxd->id, owner->id,
			idxd_vdev_wq_mode_name(owner_shared));
		rc = -EBUSY;
		goto out_unlock_vdev;
	}

	mutex_lock(&wq->wq_lock);
	if (wq->type != IDXD_WQT_USER) {
		dev_err(idxd_confdev(idxd),
			"wq%d.%d must be type=user for VDEV assignment, current type=%s\n",
			idxd->id, wq->id, idxd_vdev_wq_type_name(wq->type));
		rc = -EINVAL;
		goto out_unlock_wq;
	}

	if (wq->state != IDXD_WQ_ENABLED) {
		dev_err(idxd_confdev(idxd),
			"wq%d.%d must be enabled for VDEV assignment, current state=%s\n",
			idxd->id, wq->id,
			idxd_vdev_wq_state_name(wq->state));
		rc = -EINVAL;
		goto out_unlock_wq;
	}

	if (shared && !wq_shared(wq)) {
		dev_err(idxd_confdev(idxd),
			"wq%d.%d cannot be shared between VDEVs while physical WQ mode is %s\n",
			idxd->id, wq->id,
			idxd_vdev_wq_mode_name(wq_shared(wq)));
		rc = -EINVAL;
		goto out_unlock_wq;
	}

	vwq = kzalloc(sizeof(*vwq), GFP_KERNEL);
	if (!vwq) {
		rc = -ENOMEM;
		goto out_unlock_wq;
	}

	vwq->wq = wq;
	vwq->id = vdev->num_wqs;
	vwq->shared = shared;
	get_device(wq_confdev(wq));
	idxd_wq_get(wq);
	mutex_unlock(&wq->wq_lock);

	list_add_tail(&vwq->node, &vdev->wqs);
	vdev->num_wqs++;
	mutex_unlock(&idxd->vdev_lock);

	return 0;

out_unlock_wq:
	mutex_unlock(&wq->wq_lock);
out_unlock_vdev:
	mutex_unlock(&idxd->vdev_lock);
	return rc;
}

static void idxd_vdev_remove_from_list(struct idxd_vdev *vdev)
{
	struct idxd_device *idxd = vdev->idxd;

	mutex_lock(&idxd->vdev_lock);
	list_del_init(&vdev->list);
	mutex_unlock(&idxd->vdev_lock);
}

static int idxd_vdev_parse_wqs(struct idxd_vdev *vdev, char *buf)
{
	struct idxd_device *idxd = vdev->idxd;
	char *token;
	int rc;

	while ((token = strsep(&buf, ", \t\n"))) {
		struct idxd_wq *wq;
		bool shared;

		token = strim(token);
		if (!*token)
			continue;

		rc = idxd_vdev_parse_wq_token(idxd, token, &wq, &shared);
		if (rc)
			return rc;

		rc = idxd_vdev_add_wq(vdev, wq, shared);
		if (rc)
			return rc;
	}

	return vdev->num_wqs ? 0 : -EINVAL;
}

int idxd_vdev_create(struct idxd_device *idxd, const char *buf)
{
	struct device *dev = idxd_confdev(idxd);
	struct idxd_vdev *vdev;
	char *kbuf;
	int rc;

	kbuf = kstrdup(buf, GFP_KERNEL);
	if (!kbuf)
		return -ENOMEM;

	vdev = kzalloc(sizeof(*vdev), GFP_KERNEL);
	if (!vdev) {
		rc = -ENOMEM;
		goto out_free_buf;
	}

	idxd_dev_set_type(&vdev->idxd_dev, IDXD_DEV_VFIO);
	vdev->idxd = idxd;
	INIT_LIST_HEAD(&vdev->list);
	INIT_LIST_HEAD(&vdev->wqs);
	mutex_init(&vdev->lock);

	vdev->id = ida_alloc(&idxd->vdev_ida, GFP_KERNEL);
	if (vdev->id < 0) {
		rc = vdev->id;
		goto out_free_vdev;
	}

	device_initialize(vdev_confdev(vdev));
	vdev_confdev(vdev)->parent = dev;
	vdev_confdev(vdev)->bus = &dsa_bus_type;
	vdev_confdev(vdev)->type = &idxd_vdev_device_type;
	rc = dev_set_name(vdev_confdev(vdev), "vdev%d.%d", idxd->id, vdev->id);
	if (rc)
		goto out_put_device;

	rc = idxd_vdev_parse_wqs(vdev, kbuf);
	if (rc)
		goto out_put_device;

	mutex_lock(&idxd->vdev_lock);
	list_add_tail(&vdev->list, &idxd->vdev_list);
	mutex_unlock(&idxd->vdev_lock);

	rc = device_add(vdev_confdev(vdev));
	if (rc) {
		idxd_vdev_remove_from_list(vdev);
		goto out_put_device;
	}

	kfree(kbuf);
	return 0;

out_put_device:
	put_device(vdev_confdev(vdev));
	kfree(kbuf);
	return rc;

out_free_vdev:
	mutex_destroy(&vdev->lock);
	kfree(vdev);
out_free_buf:
	kfree(kbuf);
	return rc;
}
EXPORT_SYMBOL_GPL(idxd_vdev_create);

int idxd_vdev_destroy(struct idxd_device *idxd, unsigned int id)
{
	struct idxd_vdev *vdev;

	mutex_lock(&idxd->vdev_lock);
	list_for_each_entry(vdev, &idxd->vdev_list, list) {
		if (vdev->id != id)
			continue;
		list_del_init(&vdev->list);
		mutex_unlock(&idxd->vdev_lock);
		device_unregister(vdev_confdev(vdev));
		return 0;
	}
	mutex_unlock(&idxd->vdev_lock);

	return -ENOENT;
}
EXPORT_SYMBOL_GPL(idxd_vdev_destroy);

void idxd_vdev_destroy_all(struct idxd_device *idxd)
{
	struct idxd_vdev *vdev, *tmp;
	LIST_HEAD(vdevs);

	mutex_lock(&idxd->vdev_lock);
	list_splice_init(&idxd->vdev_list, &vdevs);
	mutex_unlock(&idxd->vdev_lock);

	list_for_each_entry_safe(vdev, tmp, &vdevs, list) {
		list_del_init(&vdev->list);
		device_unregister(vdev_confdev(vdev));
	}
}
EXPORT_SYMBOL_GPL(idxd_vdev_destroy_all);

ssize_t idxd_vdevs_show(struct idxd_device *idxd, char *buf)
{
	struct idxd_vdev *vdev;
	ssize_t pos = 0;

	mutex_lock(&idxd->vdev_lock);
	list_for_each_entry(vdev, &idxd->vdev_list, list) {
		struct idxd_vwq *vwq;

		pos += sysfs_emit_at(buf, pos, "vdev%d.%d:",
				     idxd->id, vdev->id);
		list_for_each_entry(vwq, &vdev->wqs, node) {
			struct idxd_wq *wq = vwq->wq;

			mutex_lock(&wq->wq_lock);
			pos += sysfs_emit_at(buf, pos,
					     " wq%d.%d[id=%u,assign=%s,phys=%s,state=%s,type=%s,size=%u,threshold=%u,cdev=%u]",
					     idxd->id, wq->id, vwq->id,
					     idxd_vdev_wq_mode_name(vwq->shared),
					     idxd_vdev_wq_mode_name(wq_shared(wq)),
					     idxd_vdev_wq_state_name(wq->state),
					     idxd_vdev_wq_type_name(wq->type),
					     wq->size, wq->threshold,
					     !!wq->idxd_cdev);
			mutex_unlock(&wq->wq_lock);
		}
		pos += sysfs_emit_at(buf, pos, "\n");
	}
	mutex_unlock(&idxd->vdev_lock);

	return pos;
}
EXPORT_SYMBOL_GPL(idxd_vdevs_show);
