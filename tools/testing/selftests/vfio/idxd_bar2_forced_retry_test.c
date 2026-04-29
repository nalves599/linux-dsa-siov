// SPDX-License-Identifier: GPL-2.0-only
#define _GNU_SOURCE

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/idxd.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_WQ "/dev/dsa/wq0.0"
#define DEFAULT_BAR2 "auto"
#define DEFAULT_CHUNK (2U * 1024U * 1024U)
#define DEFAULT_OPS 256U
#define DEFAULT_TRAP_BYTES 4096U
#define AUTO_TRAP_PASID UINT_MAX

static char enqcmds_retries_path[PATH_MAX];
static unsigned int saved_enqcmds_retries;
static bool saved_enqcmds_retries_valid;

struct dsa_op {
	struct dsa_hw_desc desc;
	struct dsa_completion_record comp;
};

static uint64_t monotonic_ns(void)
{
	struct timespec ts;

	if (clock_gettime(CLOCK_MONOTONIC, &ts)) {
		perror("clock_gettime");
		exit(EXIT_FAILURE);
	}

	return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

static int wait_for_completion(struct dsa_completion_record *comp)
{
	uint64_t deadline = monotonic_ns() + 5000000000ull;

	while (__atomic_load_n(&comp->status, __ATOMIC_ACQUIRE) == DSA_COMP_NONE) {
		if (monotonic_ns() > deadline) {
			errno = ETIMEDOUT;
			return -1;
		}
	}

	if (comp->status != DSA_COMP_SUCCESS) {
		errno = EIO;
		return -1;
	}

	return 0;
}

static uint64_t virt_to_phys(int pagemap_fd, const void *addr)
{
	const uint64_t pfn_mask = (1ULL << 55) - 1;
	long page_size = sysconf(_SC_PAGESIZE);
	uintptr_t va = (uintptr_t)addr;
	uint64_t entry;
	off_t offset;
	ssize_t ret;

	if (page_size <= 0) {
		perror("sysconf(_SC_PAGESIZE)");
		exit(EXIT_FAILURE);
	}

	offset = (off_t)((va / (uintptr_t)page_size) * sizeof(entry));
	ret = pread(pagemap_fd, &entry, sizeof(entry), offset);
	if (ret != sizeof(entry)) {
		perror("pread pagemap");
		exit(EXIT_FAILURE);
	}

	if (!(entry & (1ULL << 63))) {
		fprintf(stderr, "virtual address %p is not present\n", addr);
		exit(EXIT_FAILURE);
	}

	if (!(entry & pfn_mask)) {
		fprintf(stderr, "pagemap did not expose a PFN for %p\n", addr);
		exit(EXIT_FAILURE);
	}

	return ((entry & pfn_mask) * (uint64_t)page_size) +
	       (va & ((uintptr_t)page_size - 1));
}

static void write_unlimited_desc(void *bar2, const struct dsa_hw_desc *desc)
{
	volatile uint64_t *portal = bar2;
	uint64_t raw[8];
	unsigned int i;

	memcpy(raw, desc, sizeof(raw));
	for (i = 0; i < 8; i++)
		portal[i] = raw[i];
}

static int read_u32_path(const char *path, unsigned int *val)
{
	FILE *f;
	int rc = 0;

	f = fopen(path, "r");
	if (!f)
		return -errno;

	if (fscanf(f, "%u", val) != 1)
		rc = -EINVAL;

	fclose(f);
	return rc;
}

static int read_hex_path(const char *path, unsigned int *val)
{
	FILE *f;
	int rc = 0;

	f = fopen(path, "r");
	if (!f)
		return -errno;

	if (fscanf(f, "%x", val) != 1)
		rc = -EINVAL;

	fclose(f);
	return rc;
}

static int write_u32_path(const char *path, unsigned int val)
{
	FILE *f;
	int rc = 0;

	f = fopen(path, "w");
	if (!f)
		return -errno;

	if (fprintf(f, "%u\n", val) < 0)
		rc = -EIO;

	fclose(f);
	return rc;
}

static int detect_dsa_bar2(char *path, size_t path_size)
{
	const char *pci_bus = "/sys/bus/pci/devices";
	unsigned int vendor, device, class;
	struct dirent *de;
	char attr[PATH_MAX];
	DIR *dir;
	int rc = -ENOENT;

	dir = opendir(pci_bus);
	if (!dir)
		return -errno;

	while ((de = readdir(dir))) {
		if (de->d_name[0] == '.')
			continue;

		snprintf(attr, sizeof(attr), "%s/%s/vendor", pci_bus,
			 de->d_name);
		if (read_hex_path(attr, &vendor) || vendor != 0x8086)
			continue;

		snprintf(attr, sizeof(attr), "%s/%s/class", pci_bus,
			 de->d_name);
		if (read_hex_path(attr, &class) || (class >> 8) != 0x1200)
			continue;

		snprintf(attr, sizeof(attr), "%s/%s/device", pci_bus,
			 de->d_name);
		if (read_hex_path(attr, &device))
			continue;

		if (device != 0x0b25)
			continue;

		snprintf(path, path_size, "%s/%s/resource2", pci_bus,
			 de->d_name);
		if (access(path, R_OK | W_OK))
			continue;

		rc = 0;
		break;
	}

	closedir(dir);
	return rc;
}

static void restore_enqcmds_retries(void)
{
	if (!saved_enqcmds_retries_valid)
		return;

	write_u32_path(enqcmds_retries_path, saved_enqcmds_retries);
}

static int force_enqcmds_retries_zero(const char *wq_path)
{
	const char *wq_name = strrchr(wq_path, '/');
	int rc;

	wq_name = wq_name ? wq_name + 1 : wq_path;
	snprintf(enqcmds_retries_path, sizeof(enqcmds_retries_path),
		 "/sys/bus/dsa/devices/%s/enqcmds_retries", wq_name);

	rc = read_u32_path(enqcmds_retries_path, &saved_enqcmds_retries);
	if (rc)
		return rc;

	rc = write_u32_path(enqcmds_retries_path, 0);
	if (rc)
		return rc;

	saved_enqcmds_retries_valid = true;
	atexit(restore_enqcmds_retries);
	return 0;
}

static int detect_open_cdev_pasid(unsigned int *pasid)
{
	const char *dsa_bus = "/sys/bus/dsa/devices";
	char path[PATH_MAX];
	struct dirent *de;
	unsigned int pid;
	DIR *dir;
	int rc = -ENOENT;

	dir = opendir(dsa_bus);
	if (!dir)
		return -errno;

	while ((de = readdir(dir))) {
		if (strncmp(de->d_name, "file", 4))
			continue;

		snprintf(path, sizeof(path), "%s/%s/pid", dsa_bus, de->d_name);
		if (read_u32_path(path, &pid) || pid != (unsigned int)getpid())
			continue;

		snprintf(path, sizeof(path), "%s/%s/pasid", dsa_bus, de->d_name);
		rc = read_u32_path(path, pasid);
		break;
	}

	closedir(dir);
	return rc;
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s [--wq PATH] [--bar2 PATH|auto] [--ops N]\n"
		"          [--chunk BYTES] [--trap-bytes BYTES]\n"
		"          [--trap-pasid PASID|auto]\n"
		"\n"
		"Legacy positional form is still accepted:\n"
		"  %s [wq] [bar2] [ops] [chunk] [trap_bytes] [trap_pasid|auto]\n",
		prog, prog);
}

int main(int argc, char **argv)
{
	const char *wq_path = DEFAULT_WQ;
	const char *bar2_path = DEFAULT_BAR2;
	unsigned int ops_count = DEFAULT_OPS;
	unsigned int chunk = DEFAULT_CHUNK;
	unsigned int trap_bytes = DEFAULT_TRAP_BYTES;
	unsigned int trap_pasid = 0;
	unsigned int submitted = 0, busy = 0, trap_after_busy = 0;
	struct dsa_completion_record *trap_comp;
	struct dsa_hw_desc trap_desc __attribute__((aligned(64)));
	struct dsa_op *ops;
	bool *op_submitted;
	uint8_t *src, *dst, *trap_src, *trap_dst;
	char auto_bar2_path[PATH_MAX];
	void *bar2;
	int wq_fd, bar2_fd, pagemap_fd;
	int rc;
	unsigned int i;

	if (argc > 1 && strncmp(argv[1], "--", 2)) {
		wq_path = argv[1];
		if (argc > 2)
			bar2_path = argv[2];
		if (argc > 3)
			ops_count = strtoul(argv[3], NULL, 0);
		if (argc > 4)
			chunk = strtoul(argv[4], NULL, 0);
		if (argc > 5)
			trap_bytes = strtoul(argv[5], NULL, 0);
		if (argc > 6) {
			if (!strcmp(argv[6], "auto"))
				trap_pasid = AUTO_TRAP_PASID;
			else
				trap_pasid = strtoul(argv[6], NULL, 0);
		}
	} else {
		for (i = 1; i < (unsigned int)argc; i++) {
			if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
				usage(argv[0]);
				return EXIT_SUCCESS;
			} else if (!strcmp(argv[i], "--wq") && i + 1 < (unsigned int)argc) {
				wq_path = argv[++i];
			} else if (!strcmp(argv[i], "--bar2") && i + 1 < (unsigned int)argc) {
				bar2_path = argv[++i];
			} else if (!strcmp(argv[i], "--ops") && i + 1 < (unsigned int)argc) {
				ops_count = strtoul(argv[++i], NULL, 0);
			} else if (!strcmp(argv[i], "--chunk") && i + 1 < (unsigned int)argc) {
				chunk = strtoul(argv[++i], NULL, 0);
			} else if (!strcmp(argv[i], "--trap-bytes") && i + 1 < (unsigned int)argc) {
				trap_bytes = strtoul(argv[++i], NULL, 0);
			} else if (!strcmp(argv[i], "--trap-pasid") && i + 1 < (unsigned int)argc) {
				if (!strcmp(argv[i + 1], "auto"))
					trap_pasid = AUTO_TRAP_PASID;
				else
					trap_pasid = strtoul(argv[i + 1], NULL, 0);
				i++;
			} else {
				usage(argv[0]);
				return EXIT_FAILURE;
			}
		}
	}

	if (!strcmp(bar2_path, "auto")) {
		rc = detect_dsa_bar2(auto_bar2_path, sizeof(auto_bar2_path));
		if (rc) {
			errno = -rc;
			perror("detect DSA BAR2");
			return EXIT_FAILURE;
		}
		bar2_path = auto_bar2_path;
	}

	if (!ops_count || !chunk || !trap_bytes) {
		fprintf(stderr, "ops_count, chunk, and trap_bytes must be non-zero\n");
		return EXIT_FAILURE;
	}

	if (!trap_pasid && trap_bytes > (unsigned int)sysconf(_SC_PAGESIZE)) {
		fprintf(stderr, "trap_bytes must fit in one page for GPA mode\n");
		return EXIT_FAILURE;
	}

	op_submitted = calloc(ops_count, sizeof(*op_submitted));
	if (!op_submitted) {
		perror("calloc");
		return EXIT_FAILURE;
	}

	rc = posix_memalign((void **)&ops, 64, sizeof(*ops) * ops_count);
	if (rc) {
		errno = rc;
		perror("alloc ops");
		return EXIT_FAILURE;
	}

	rc = posix_memalign((void **)&src, 4096, (size_t)ops_count * chunk);
	if (rc) {
		errno = rc;
		perror("alloc src");
		return EXIT_FAILURE;
	}

	rc = posix_memalign((void **)&dst, 4096, (size_t)ops_count * chunk);
	if (rc) {
		errno = rc;
		perror("alloc dst");
		return EXIT_FAILURE;
	}

	rc = posix_memalign((void **)&trap_src, 4096, trap_bytes);
	if (rc) {
		errno = rc;
		perror("alloc trap_src");
		return EXIT_FAILURE;
	}

	rc = posix_memalign((void **)&trap_dst, 4096, trap_bytes);
	if (rc) {
		errno = rc;
		perror("alloc trap_dst");
		return EXIT_FAILURE;
	}

	rc = posix_memalign((void **)&trap_comp, 4096, 4096);
	if (rc) {
		errno = rc;
		perror("alloc trap_comp");
		return EXIT_FAILURE;
	}

	memset(ops, 0, sizeof(*ops) * ops_count);
	memset(dst, 0, (size_t)ops_count * chunk);
	for (i = 0; i < (size_t)ops_count * chunk; i++)
		src[i] = (uint8_t)(i * 131u + 0x5au);
	memset(trap_comp, 0, 4096);
	memset(trap_dst, 0, trap_bytes);
	for (i = 0; i < trap_bytes; i++)
		trap_src[i] = (uint8_t)(i * 17u + 0xa5u);

	wq_fd = open(wq_path, O_RDWR | O_CLOEXEC);
	if (wq_fd < 0) {
		perror(wq_path);
		return EXIT_FAILURE;
	}

	rc = force_enqcmds_retries_zero(wq_path);
	if (rc) {
		errno = -rc;
		perror("force enqcmds_retries");
		return EXIT_FAILURE;
	}

	if (trap_pasid == AUTO_TRAP_PASID) {
		rc = detect_open_cdev_pasid(&trap_pasid);
		if (rc) {
			errno = -rc;
			perror("detect cdev PASID");
			return EXIT_FAILURE;
		}
		if (!trap_pasid) {
			fprintf(stderr, "detected invalid cdev PASID 0\n");
			return EXIT_FAILURE;
		}
	}

	pagemap_fd = open("/proc/self/pagemap", O_RDONLY | O_CLOEXEC);
	if (pagemap_fd < 0) {
		perror("/proc/self/pagemap");
		return EXIT_FAILURE;
	}

	bar2_fd = open(bar2_path, O_RDWR | O_SYNC | O_CLOEXEC);
	if (bar2_fd < 0) {
		perror(bar2_path);
		return EXIT_FAILURE;
	}

	bar2 = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, bar2_fd, 0);
	if (bar2 == MAP_FAILED) {
		perror("mmap BAR2");
		return EXIT_FAILURE;
	}

	for (i = 0; i < ops_count; i++) {
		struct dsa_op *op = &ops[i];
		ssize_t written;

		op->desc.flags = IDXD_OP_FLAG_CRAV | IDXD_OP_FLAG_RCR;
		op->desc.opcode = DSA_OPCODE_MEMMOVE;
		op->desc.src_addr = (uintptr_t)src + (size_t)i * chunk;
		op->desc.dst_addr = (uintptr_t)dst + (size_t)i * chunk;
		op->desc.xfer_size = chunk;
		op->desc.completion_addr = (uintptr_t)&op->comp;

		written = write(wq_fd, &op->desc, sizeof(op->desc));
		if (written == (ssize_t)sizeof(op->desc)) {
			op_submitted[i] = true;
			submitted++;
			continue;
		}

		if (written >= 0) {
			fprintf(stderr, "short descriptor write: %zd\n", written);
			return EXIT_FAILURE;
		}

		if (errno != EBUSY && errno != EAGAIN) {
			perror("write descriptor");
			return EXIT_FAILURE;
		}

		busy++;
		if (!trap_after_busy) {
			memset(&trap_desc, 0, sizeof(trap_desc));
			trap_desc.flags = IDXD_OP_FLAG_CRAV | IDXD_OP_FLAG_RCR;
			trap_desc.opcode = DSA_OPCODE_MEMMOVE;
			if (trap_pasid) {
				trap_desc.pasid = trap_pasid;
				trap_desc.src_addr = (uintptr_t)trap_src;
				trap_desc.dst_addr = (uintptr_t)trap_dst;
				trap_desc.completion_addr = (uintptr_t)trap_comp;
			} else {
				trap_desc.src_addr = virt_to_phys(pagemap_fd,
								  trap_src);
				trap_desc.dst_addr = virt_to_phys(pagemap_fd,
								  trap_dst);
				trap_desc.completion_addr =
					virt_to_phys(pagemap_fd, trap_comp);
			}
			trap_desc.xfer_size = trap_bytes;
			write_unlimited_desc(bar2, &trap_desc);
			trap_after_busy = 1;
		}
	}

	if (trap_after_busy && wait_for_completion(trap_comp)) {
		perror("trapped completion");
		return EXIT_FAILURE;
	}

	if (trap_after_busy && memcmp(trap_src, trap_dst, trap_bytes)) {
		fprintf(stderr, "trapped copy verification failed\n");
		return EXIT_FAILURE;
	}

	for (i = 0; i < ops_count; i++) {
		if (!op_submitted[i])
			continue;
		if (wait_for_completion(&ops[i].comp)) {
			perror("completion");
			return EXIT_FAILURE;
		}
	}

	if (!busy) {
		fprintf(stderr, "no limited-portal Retry observed\n");
		return 2;
	}
	if (!trap_after_busy) {
		fprintf(stderr, "BAR2 unlimited trap was not issued\n");
		return 3;
	}

	printf("wq=%s bar2=%s submitted=%u busy=%u trap_after_busy=%u trap_bytes=%u trap_pasid=%u trap_verified=1\n",
	       wq_path, bar2_path, submitted, busy, trap_after_busy, trap_bytes,
	       trap_pasid);
	return 0;
}
