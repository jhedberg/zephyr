/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/* bt_enable() in its two forms and bt_enable_async(), which go through the
 * same sequences of commands: what each tells and when, which threads stay
 * free while the controller takes its time over a command, a command that the
 * controller refuses or need not support, calls while the callback form ends,
 * a second call while the first is on its way, and the guess at the vendor HCI
 * extensions.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/bluetooth/hci_vs.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "driver.h"
#include "future.h"
#include "hci_core.h"

/* A command that the initialization sends with every configuration of the
 * suite, after the controller has been read
 */
#define HELD_OPCODE BT_HCI_OP_LE_SET_EVENT_MASK

static K_SEM_DEFINE(ready_sem, 0, 1);
static k_tid_t ready_thread;
static int ready_err;

static void ready(int err)
{
	ready_err = err;
	ready_thread = k_current_get();
	k_sem_give(&ready_sem);
}

/* Work on the system workqueue and on the Bluetooth workqueue, which shows
 * that neither waits for the controller
 */
static K_SEM_DEFINE(probe_sem, 0, 1);
static K_SEM_DEFINE(probe_bt_sem, 0, 1);

static void probe_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_sem_give(&probe_sem);
}

static void probe_bt_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_sem_give(&probe_bt_sem);
}

static K_WORK_DEFINE(probe_work, probe_handler);
static K_WORK_DEFINE(probe_bt_work, probe_bt_handler);

static void expect_workqueues_free(void)
{
	k_sem_reset(&probe_sem);
	k_sem_reset(&probe_bt_sem);
	zassert_true(k_work_submit(&probe_work) >= 0, "Submitting work failed");
	zassert_true(bt_work_submit(&probe_bt_work) >= 0, "Submitting work failed");
	zassert_ok(k_sem_take(&probe_sem, K_MSEC(100)), "The system workqueue is held up");
	zassert_ok(k_sem_take(&probe_bt_sem, K_MSEC(100)), "The Bluetooth workqueue is held up");
}

/* The future of the cases with bt_enable_async(). Static, so that an enable
 * that a failed case left on its way does not resolve a future on a stack
 * that is gone; after() lets that enable end.
 */
static struct bt_future enable_fut;

/* Wait until the controller has been given the held command */
static void expect_held(void)
{
	for (int i = 0; i < 100 && test_held_count == 0U; i++) {
		k_sleep(K_MSEC(1));
	}

	zassert_equal(test_held_count, 1U, "The controller got the command %u times",
		      test_held_count);
}

/* bt_enable(NULL) from a work item */
static int sync_err;
static K_SEM_DEFINE(sync_done, 0, 1);

static void sync_enable_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	sync_err = bt_enable(NULL);
	k_sem_give(&sync_done);
}

static K_WORK_DEFINE(sync_enable_work, sync_enable_handler);

/* Work that holds the system workqueue up until the gate is given */
static K_SEM_DEFINE(blocker_started, 0, 1);
static K_SEM_DEFINE(blocker_gate, 0, 1);

static void blocker_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_sem_give(&blocker_started);
	(void)k_sem_take(&blocker_gate, K_FOREVER);
}

static K_WORK_DEFINE(blocker_work, blocker_handler);

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	test_driver_reset();
	k_sem_reset(&ready_sem);
	k_sem_reset(&sync_done);
	k_sem_reset(&blocker_started);
	k_sem_reset(&blocker_gate);
	ready_thread = NULL;
	ready_err = -EINPROGRESS;

	/* Each case enables Bluetooth on its own */
	if (bt_is_ready()) {
		zassert_ok(bt_disable(), "Bluetooth disable failed");
	}
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);

	/* A case that failed may have left the system workqueue held up, or an
	 * enable on its way that the controller holds up: let both go on, and
	 * the enable end, before the next case
	 */
	k_sem_give(&blocker_gate);
	test_held_respond();
	for (int i = 0; i < 1000 && atomic_test_bit(bt_dev.flags, BT_DEV_ENABLING); i++) {
		k_sleep(K_MSEC(1));
	}

	/* The suites that come later expect Bluetooth to be enabled. The
	 * transport stays open after an initialization that failed.
	 */
	test_driver_reset();
	if (!bt_is_ready()) {
		(void)bt_disable();
		zassert_ok(bt_enable(NULL), "Bluetooth init failed");
	}
}

ZTEST_SUITE(enable, NULL, NULL, before, after, NULL);

/* While the controller takes its time over a command, no thread waits for it:
 * bt_enable() has returned and the system workqueue is free. The callback is
 * called from the system workqueue, as it always was, once the controller has
 * responded and the rest is through.
 */
static ZTEST(enable, test_callback)
{
	test_held_opcode = HELD_OPCODE;

	zassert_ok(bt_enable(ready), "Enabling failed");
	expect_held();
	expect_workqueues_free();
	zassert_equal(k_sem_count_get(&ready_sem), 0U, "Told before the controller responded");
	zassert_false(bt_is_ready(), "Ready before the controller responded");

	test_held_respond();

	zassert_ok(k_sem_take(&ready_sem, K_SECONDS(1)), "The callback was not called");
	zassert_ok(ready_err, "The callback got %d", ready_err);
	zassert_equal(ready_thread, k_sys_work_q.thread_id, "Not called from the system workqueue");
	zassert_true(bt_is_ready(), "Bluetooth is not ready");
}

/* bt_enable_async() returns in the same way, and the future is resolved with
 * what bt_enable(NULL) returns once the controller has responded and the rest
 * is through.
 */
static ZTEST(enable, test_async)
{
	test_held_opcode = HELD_OPCODE;

	zassert_ok(bt_enable_async(&enable_fut), "Enabling failed");
	expect_held();
	expect_workqueues_free();
	zassert_false(bt_future_is_done(&enable_fut), "Resolved before the controller responded");
	zassert_false(bt_is_ready(), "Ready before the controller responded");

	test_held_respond();

	zassert_ok(bt_future_wait(&enable_fut, K_SECONDS(1)), "Bluetooth was not enabled");
	zassert_ok(enable_fut.result, "The result is %d", enable_fut.result);
	zassert_true(bt_is_ready(), "Bluetooth is not ready");
}

/* A command of the initialization that the controller refuses ends it, and
 * the callback gets what bt_enable(NULL) returns for that status.
 */
static ZTEST(enable, test_callback_refused)
{
	test_held_opcode = HELD_OPCODE;
	test_held_status = BT_HCI_ERR_CMD_DISALLOWED;

	zassert_ok(bt_enable(ready), "Enabling failed");
	expect_held();
	test_held_respond();

	zassert_ok(k_sem_take(&ready_sem, K_SECONDS(1)), "The callback was not called");
	zassert_equal(ready_err, -EACCES, "The callback got %d", ready_err);
	zassert_false(bt_is_ready(), "Ready after a failure");
}

/* The same for the future */
static ZTEST(enable, test_async_refused)
{
	test_held_opcode = HELD_OPCODE;
	test_held_status = BT_HCI_ERR_CMD_DISALLOWED;

	zassert_ok(bt_enable_async(&enable_fut), "Enabling failed");
	expect_held();
	test_held_respond();

	zassert_ok(bt_future_wait(&enable_fut, K_SECONDS(1)), "The enable was not completed");
	zassert_equal(enable_fut.result, -EACCES, "The result is %d", enable_fut.result);
	zassert_false(bt_is_ready(), "Ready after a failure");
}

/* And for the synchronous form, which waits for the stages in the caller */
static ZTEST(enable, test_sync_refused)
{
	test_held_opcode = HELD_OPCODE;
	test_held_status = BT_HCI_ERR_CMD_DISALLOWED;

	zassert_true(k_work_submit(&sync_enable_work) >= 0, "Submitting work failed");
	expect_held();
	test_held_respond();

	zassert_ok(k_sem_take(&sync_done, K_SECONDS(1)), "bt_enable() did not return");
	zassert_equal(sync_err, -EACCES, "bt_enable() returned %d", sync_err);
	zassert_false(bt_is_ready(), "Ready after a failure");
}

/* The synchronous form keeps working where it did: on the system workqueue,
 * which in one scenario of this suite also sends the commands, and on the
 * Bluetooth workqueue, which the asynchronous forms go from stage to stage on.
 * The waiter does on either what it waits for.
 */
static void expect_sync_enable_on(int (*submit)(struct k_work *work))
{
	test_held_opcode = HELD_OPCODE;

	zassert_true(submit(&sync_enable_work) >= 0, "Submitting work failed");
	expect_held();
	zassert_equal(k_sem_count_get(&sync_done), 0U, "bt_enable() returned early");

	test_held_respond();

	zassert_ok(k_sem_take(&sync_done, K_SECONDS(1)), "bt_enable() did not return");
	zassert_ok(sync_err, "bt_enable() returned %d", sync_err);
	zassert_true(bt_is_ready(), "Bluetooth is not ready");
}

static ZTEST(enable, test_sync_on_system_workqueue)
{
	expect_sync_enable_on(k_work_submit);
}

static ZTEST(enable, test_sync_on_bt_workqueue)
{
	expect_sync_enable_on(bt_work_submit);
}

/* A command that the controller need not support and refuses: here the
 * public address, without which the Host creates a static random identity
 * and gives it to the controller. Both forms succeed, also when the refusal
 * is the last response of its sequence.
 */
static void expect_random_identity(void)
{
	bt_addr_le_t addrs[CONFIG_BT_ID_MAX];
	size_t count = ARRAY_SIZE(addrs);
	bool addr_set = false;

	bt_id_get(addrs, &count);
	zassert_equal(count, 1U, "%zu identities", count);
	zassert_true(BT_ADDR_IS_STATIC(&addrs[0].a) && addrs[0].type == BT_ADDR_LE_RANDOM,
		     "The identity is not a static random one");

	zassert_true(test_log_len <= ARRAY_SIZE(test_log), "More commands than the log holds");
	for (unsigned int i = 0U; i < test_log_len; i++) {
		addr_set = addr_set || test_log[i] == BT_HCI_OP_LE_SET_RANDOM_ADDRESS;
	}

	zassert_true(addr_set, "The controller did not get the random address");
}

static ZTEST(enable, test_optional_refused)
{
	test_held_opcode = BT_HCI_OP_READ_BD_ADDR;
	test_held_status = BT_HCI_ERR_UNKNOWN_CMD;
	test_log_len = 0U;

	zassert_ok(bt_enable(ready), "Enabling failed");
	expect_held();
	test_held_respond();

	zassert_ok(k_sem_take(&ready_sem, K_SECONDS(1)), "The callback was not called");
	zassert_ok(ready_err, "The callback got %d", ready_err);
	expect_random_identity();

	zassert_ok(bt_disable(), "Bluetooth disable failed");
	test_held_count = 0U;
	test_log_len = 0U;

	zassert_true(k_work_submit(&sync_enable_work) >= 0, "Submitting work failed");
	expect_held();
	test_held_respond();

	zassert_ok(k_sem_take(&sync_done, K_SECONDS(1)), "bt_enable() did not return");
	zassert_ok(sync_err, "bt_enable() returned %d", sync_err);
	expect_random_identity();
}

/* The callback form ends on the system workqueue, right before the callback:
 * until then Bluetooth is not ready, bt_disable() asks to be retried and
 * bt_enable() finds an enable on its way. The system workqueue is held up
 * here after the controller has responded to everything.
 */
static ZTEST(enable, test_callback_pending)
{
	if (!IS_ENABLED(CONFIG_BT_TX_PROCESSOR_THREAD)) {
		/* The system workqueue sends the commands */
		ztest_test_skip();
	}

	zassert_true(k_work_submit(&blocker_work) >= 0, "Submitting work failed");
	zassert_ok(k_sem_take(&blocker_started, K_SECONDS(1)), "The blocker did not start");

	zassert_ok(bt_enable(ready), "Enabling failed");

	/* The stages are through once the end of the enable is queued */
	for (int i = 0; i < 1000 && (k_work_busy_get(&bt_dev.init) & K_WORK_QUEUED) == 0; i++) {
		k_sleep(K_MSEC(1));
	}

	zassert_true((k_work_busy_get(&bt_dev.init) & K_WORK_QUEUED) != 0,
		     "The end of the enable was not queued");
	zassert_false(bt_is_ready(), "Ready before the callback");
	zassert_equal(bt_disable(), -EAGAIN, "bt_disable() did not ask to be retried");
	zassert_equal(bt_enable(NULL), -EALREADY, "A second enable was accepted");

	k_sem_give(&blocker_gate);

	zassert_ok(k_sem_take(&ready_sem, K_SECONDS(1)), "The callback was not called");
	zassert_ok(ready_err, "The callback got %d", ready_err);
	zassert_true(bt_is_ready(), "Bluetooth is not ready");
}

/* With CONFIG_BT_HCI_VS_EXT_DETECT the Host sends vendor commands only to a
 * controller of HCI 5.0 or later that has no public address, which is what
 * the Zephyr controller is like
 */
static bool vs_sent(void)
{
	zassert_true(test_log_len <= ARRAY_SIZE(test_log), "More commands than the log holds");
	for (unsigned int i = 0U; i < test_log_len; i++) {
		if (test_log[i] == BT_HCI_OP_VS_READ_VERSION_INFO) {
			return true;
		}
	}

	return false;
}

static ZTEST(enable, test_vendor_guess)
{
	if (!IS_ENABLED(CONFIG_BT_HCI_VS_EXT_DETECT)) {
		ztest_test_skip();
	}

	test_hci_version = BT_HCI_VERSION_5_4;
	test_log_len = 0U;
	zassert_ok(bt_enable(NULL), "Bluetooth init failed");
	zassert_false(vs_sent(), "Vendor commands sent to a controller with a public address");

	zassert_ok(bt_disable(), "Bluetooth disable failed");
	test_hci_version = BT_HCI_VERSION_4_2;
	test_no_public_addr = true;
	test_log_len = 0U;
	zassert_ok(bt_enable(NULL), "Bluetooth init failed");
	zassert_false(vs_sent(), "Vendor commands sent to a controller older than HCI 5.0");

	zassert_ok(bt_disable(), "Bluetooth disable failed");
	test_hci_version = BT_HCI_VERSION_5_4;
	test_log_len = 0U;
	zassert_ok(bt_enable(NULL), "Bluetooth init failed");
	zassert_true(vs_sent(), "No vendor commands sent to a controller without a public address");
}

/* A second call while the first is on its way is refused, in every form. A
 * refused bt_enable_async() leaves its future alone: it may be one that is
 * still in use, the very future of the enable on its way included, which is
 * then still resolved by that enable.
 */
static ZTEST(enable, test_twice)
{
	struct bt_future other;
	struct bt_future copy;

	test_held_opcode = HELD_OPCODE;

	zassert_ok(bt_enable_async(&enable_fut), "Enabling failed");
	expect_held();

	/* Which taking the future again would reset */
	enable_fut.result = -EINPROGRESS;
	zassert_equal(bt_enable_async(&enable_fut), -EALREADY, "The same future was taken again");
	zassert_equal(enable_fut.result, -EINPROGRESS, "The future on its way was touched");
	zassert_equal(bt_enable(ready), -EALREADY, "A callback enable was accepted");
	zassert_equal(bt_enable(NULL), -EALREADY, "A synchronous enable was accepted");

	(void)memset(&other, 0x5a, sizeof(other));
	copy = other;
	zassert_equal(bt_enable_async(&other), -EALREADY, "A second enable was accepted");
	zassert_mem_equal(&other, &copy, sizeof(other), "The future was touched");
	zassert_false(bt_future_is_done(&enable_fut), "Resolved before the controller responded");

	test_held_respond();

	zassert_ok(bt_future_wait(&enable_fut, K_SECONDS(1)), "Bluetooth was not enabled");
	zassert_ok(enable_fut.result, "The result is %d", enable_fut.result);
	zassert_equal(k_sem_count_get(&ready_sem), 0U, "The refused callback was called");
}

/* The forms send the same commands, as they go through the same stages */
static void expect_same_commands(const uint16_t *log, unsigned int len)
{
	zassert_true(len <= ARRAY_SIZE(test_log), "More commands than the log holds");
	zassert_equal(test_log_len, len, "%u commands, not %u", test_log_len, len);
	zassert_mem_equal(test_log, log, len * sizeof(test_log[0]), "The commands differ");
}

static ZTEST(enable, test_same_commands)
{
	uint16_t sync_log[ARRAY_SIZE(test_log)];
	unsigned int sync_len;

	test_log_len = 0U;
	zassert_ok(bt_enable(NULL), "Bluetooth init failed");
	sync_len = test_log_len;
	(void)memcpy(sync_log, test_log, sizeof(sync_log));

	zassert_ok(bt_disable(), "Bluetooth disable failed");
	test_log_len = 0U;

	zassert_ok(bt_enable(ready), "Enabling failed");
	zassert_ok(k_sem_take(&ready_sem, K_SECONDS(1)), "The callback was not called");
	zassert_ok(ready_err, "The callback got %d", ready_err);
	expect_same_commands(sync_log, sync_len);

	zassert_ok(bt_disable(), "Bluetooth disable failed");
	test_log_len = 0U;

	zassert_ok(bt_enable_async(&enable_fut), "Enabling failed");
	zassert_ok(bt_future_wait(&enable_fut, K_SECONDS(1)), "Bluetooth was not enabled");
	zassert_ok(enable_fut.result, "The result is %d", enable_fut.result);
	expect_same_commands(sync_log, sync_len);
}
