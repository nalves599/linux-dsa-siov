#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0-only
#
# Host-side regression for IDXD SIOV shared-WQ BAR2 forward progress.
# It expects one or more already-running VMs that expose an IDXD VFIO VDEV.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

TEST_BIN="${TEST_BIN:-$SCRIPT_DIR/idxd_bar2_forced_retry_test}"
SSH_PORTS="${SSH_PORTS:-60035 60036}"
VDEVS="${VDEVS:-vdev0.0 vdev0.1}"
WORK_DIR="${WORK_DIR:-/tmp/idxd-bar2-forced-retry-$(date +%Y%m%d_%H%M%S)}"
OPS="${OPS:-256}"
CHUNK="${CHUNK:-2097152}"
TRAP_BYTES="${TRAP_BYTES:-4096}"
WQ="${WQ:-/dev/dsa/wq0.0}"
FORCE_RETRIES="${FORCE_RETRIES:-8}"

die()
{
	echo "error: $*" >&2
	exit 1
}

read_words()
{
	local -n out=$1
	local input=$2

	# shellcheck disable=SC2206
	out=($input)
}

ssh_guest()
{
	local port=$1
	shift

	ssh -p "$port" \
		-o StrictHostKeyChecking=no \
		-o UserKnownHostsFile=/dev/null \
		root@127.0.0.1 "$@"
}

scp_guest()
{
	local port=$1
	local src=$2
	local dst=$3

	scp -P "$port" \
		-o StrictHostKeyChecking=no \
		-o UserKnownHostsFile=/dev/null \
		"$src" "root@127.0.0.1:$dst" >/dev/null
}

wait_guest()
{
	local port=$1
	local i

	for i in $(seq 1 60); do
		if ssh -p "$port" \
			-o StrictHostKeyChecking=no \
			-o UserKnownHostsFile=/dev/null \
			-o ConnectTimeout=2 \
			root@127.0.0.1 true >/dev/null 2>&1; then
			return 0
		fi
		sleep 1
	done

	die "guest SSH did not become ready on port $port"
}

configure_guest_wq()
{
	local port=$1

	ssh_guest "$port" 'set -e
modprobe idxd_user 2>/dev/null || true
for wq in wq0.0; do
	if [ -e "/sys/bus/dsa/devices/$wq/driver/unbind" ]; then
		echo "$wq" > "/sys/bus/dsa/devices/$wq/driver/unbind" || true
	fi
	echo user > "/sys/bus/dsa/devices/$wq/type"
	echo "app_$wq" > "/sys/bus/dsa/devices/$wq/name"
	echo user > "/sys/bus/dsa/devices/$wq/driver_name"
	echo "$wq" > /sys/bus/dsa/drivers/user/bind
done
test -e /dev/dsa/wq0.0'
}

reset_stats()
{
	local vdev=$1
	local reset="/sys/bus/dsa/devices/$vdev/bar2_trap_stats_reset"

	[[ -e "$reset" ]] || die "missing $reset"
	printf '1\n' | sudo tee "$reset" >/dev/null
}

force_host_unlimited_retries()
{
	local vdev=$1
	local force="/sys/bus/dsa/devices/$vdev/bar2_force_unlimited_retries"

	[[ -e "$force" ]] || die "missing $force"
	printf '%s\n' "$FORCE_RETRIES" | sudo tee "$force" >/dev/null
}

read_stat()
{
	local vdev=$1
	local key=$2
	local stats="/sys/bus/dsa/devices/$vdev/bar2_trap_stats"

	[[ -e "$stats" ]] || die "missing $stats"
	awk -v key="$key" '$1 == key { print $2 }' "$stats"
}

dump_stats()
{
	local vdev=$1
	local stats="/sys/bus/dsa/devices/$vdev/bar2_trap_stats"

	echo "=== $vdev BAR2 trap stats ==="
	cat "$stats"
}

require_stat_ge()
{
	local vdev=$1
	local key=$2
	local min=$3
	local val

	val="$(read_stat "$vdev" "$key")"
	[[ -n "$val" ]] || die "missing stat $key for $vdev"
	if (( val < min )); then
		dump_stats "$vdev" >&2
		die "$vdev $key expected >= $min, got $val"
	fi
	printf '%s.%s=%s\n' "$vdev" "$key" "$val"
}

require_stat_eq()
{
	local vdev=$1
	local key=$2
	local expected=$3
	local val

	val="$(read_stat "$vdev" "$key")"
	[[ -n "$val" ]] || die "missing stat $key for $vdev"
	if (( val != expected )); then
		dump_stats "$vdev" >&2
		die "$vdev $key expected $expected, got $val"
	fi
	printf '%s.%s=%s\n' "$vdev" "$key" "$val"
}

prepare_guest()
{
	local port=$1

	wait_guest "$port"
	scp_guest "$port" "$TEST_BIN" /root/idxd_bar2_forced_retry_test
	ssh_guest "$port" chmod +x /root/idxd_bar2_forced_retry_test
	configure_guest_wq "$port"
}

run_guest_test()
{
	local idx=$1
	local port=$2
	local log="$WORK_DIR/guest-$idx.log"

	ssh_guest "$port" \
		"/root/idxd_bar2_forced_retry_test --wq $WQ --bar2 auto --ops $OPS --chunk $CHUNK --trap-bytes $TRAP_BYTES --trap-pasid auto" |
		tee "$log"
	grep -q 'trap_verified=1' "$log" || die "guest $idx did not verify trapped copy"
	grep -Eq 'busy=[1-9][0-9]*' "$log" || die "guest $idx did not force limited portal Retry"
}

check_vdev_stats()
{
	local vdev=$1

	require_stat_ge "$vdev" writes 8
	require_stat_ge "$vdev" trapped_unlimited_writes 8
	require_stat_ge "$vdev" split_writes 8
	require_stat_ge "$vdev" assembled_descs 1
	require_stat_ge "$vdev" trapped_unlimited_descs 1
	require_stat_ge "$vdev" forward_queued 1
	require_stat_ge "$vdev" forward_started 1
	require_stat_ge "$vdev" forward_completed 1
	require_stat_ge "$vdev" submitted_descs 1
	require_stat_ge "$vdev" pasid_translated 1
	if (( FORCE_RETRIES > 0 )); then
		require_stat_ge "$vdev" forced_host_unlimited_retries "$FORCE_RETRIES"
		require_stat_ge "$vdev" host_unlimited_retries "$FORCE_RETRIES"
		require_stat_ge "$vdev" forward_retry_loops "$FORCE_RETRIES"
	fi
	require_stat_eq "$vdev" forward_retry_exhausted 0
	require_stat_eq "$vdev" forward_dropped 0
	require_stat_eq "$vdev" submit_errors 0
	require_stat_eq "$vdev" rejected_writes 0
}

main()
{
	local -a ports
	local -a vdevs
	local -a pids
	local i

	[[ -x "$TEST_BIN" ]] || die "missing executable $TEST_BIN"
	mkdir -p "$WORK_DIR"

	read_words ports "$SSH_PORTS"
	read_words vdevs "$VDEVS"
	[[ "${#ports[@]}" -gt 0 ]] || die "SSH_PORTS is empty"
	[[ "${#ports[@]}" -eq "${#vdevs[@]}" ]] ||
		die "SSH_PORTS and VDEVS must have the same number of entries"

	for i in "${!ports[@]}"; do
		echo "prepare guest index=$i port=${ports[$i]} vdev=${vdevs[$i]}"
		prepare_guest "${ports[$i]}"
		reset_stats "${vdevs[$i]}"
		force_host_unlimited_retries "${vdevs[$i]}"
	done

	for i in "${!ports[@]}"; do
		run_guest_test "$i" "${ports[$i]}" &
		pids[$i]=$!
	done

	for i in "${!pids[@]}"; do
		wait "${pids[$i]}"
	done

	for i in "${!vdevs[@]}"; do
		check_vdev_stats "${vdevs[$i]}"
	done

	echo "PASS: IDXD BAR2 forced-Retry forward-progress regression"
	echo "work_dir=$WORK_DIR"
}

main "$@"
