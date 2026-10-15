/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/* A sequence of HCI commands that one operation goes through. The cases use
 * what starting a scanner takes as their example: stop it if it runs, set the
 * random address unless the controller has it, set the parameters, enable.
 * They cover the order and the leaving out of steps, the ways in which a
 * sequence ends before its last command and what is on record of the
 * controller then, that nothing gets in between the commands, two callers at
 * once, return parameters that the next step uses, and a caller that waits
 * for the whole of it on a workqueue.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "future.h"
#include "hci_cmd_op.h"
#include "hci_core.h"

#include "driver.h"

#define DEADLINE_MS (10 * MSEC_PER_SEC)

/* What a module that scans keeps of the controller: whether it scans, and
 * the random address that it has. The copy of the address is what the
 * controller has while the flag for it is set.
 */
enum {
	SCAN_FLAG_SCANNING,
	SCAN_FLAG_ADDR_VALID,
	SCAN_NUM_FLAGS,
};

static struct {
	ATOMIC_DEFINE(flags, SCAN_NUM_FLAGS);
	bt_addr_t ctlr_addr;
} scanner;

/* A request to scan with the parameters that it holds. It is the caller's,
 * and it is all that a request takes: the operation that goes through the
 * commands, the future that the caller waits for, and the parameters, which
 * the commands take from here when they are sent.
 */
struct scan_req {
	struct bt_hci_cmd_op_fut cmd;
	bt_addr_t addr;
	uint16_t interval;
	uint16_t window;
};

static struct scan_req *scan_req_of(struct bt_hci_cmd_op *op)
{
	return CONTAINER_OF(op, struct scan_req, cmd.op);
}

/* For a test that has something to do while the first encoder runs */
static K_SEM_DEFINE(disable_encode_sem, 0, 1);
static K_SEM_DEFINE(disable_encode_gate, 0, 1);
static bool disable_encode_block;

/* The parameters cannot be changed while the controller scans */
static int scan_disable_encode(struct net_buf *buf, struct bt_hci_cmd_op *op)
{
	struct bt_hci_cp_le_set_scan_enable *cp;

	ARG_UNUSED(op);

	if (!atomic_test_bit(scanner.flags, SCAN_FLAG_SCANNING)) {
		return BT_HCI_CMD_SKIP;
	}

	if (disable_encode_block) {
		disable_encode_block = false;
		k_sem_give(&disable_encode_sem);
		(void)k_sem_take(&disable_encode_gate, K_FOREVER);
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->enable = BT_HCI_LE_SCAN_DISABLE;
	cp->filter_dup = BT_HCI_LE_SCAN_FILTER_DUP_DISABLE;

	return 0;
}

static int scan_addr_encode(struct net_buf *buf, struct bt_hci_cmd_op *op)
{
	struct scan_req *req = scan_req_of(op);

	if (atomic_test_bit(scanner.flags, SCAN_FLAG_ADDR_VALID) &&
	    bt_addr_eq(&req->addr, &scanner.ctlr_addr)) {
		return BT_HCI_CMD_SKIP;
	}

	/* Not known to be what the controller has until it has responded */
	atomic_clear_bit(scanner.flags, SCAN_FLAG_ADDR_VALID);
	bt_addr_copy(&scanner.ctlr_addr, &req->addr);

	net_buf_add_mem(buf, &req->addr, sizeof(req->addr));

	return 0;
}

static int scan_param_encode(struct net_buf *buf, struct bt_hci_cmd_op *op)
{
	struct scan_req *req = scan_req_of(op);
	struct bt_hci_cp_le_set_scan_param *cp;

	cp = net_buf_add(buf, sizeof(*cp));
	(void)memset(cp, 0, sizeof(*cp));
	cp->scan_type = BT_HCI_LE_SCAN_PASSIVE;
	cp->interval = sys_cpu_to_le16(req->interval);
	cp->window = sys_cpu_to_le16(req->window);
	cp->addr_type = BT_HCI_OWN_ADDR_RANDOM;

	return 0;
}

static int scan_enable_encode(struct net_buf *buf, struct bt_hci_cmd_op *op)
{
	struct bt_hci_cp_le_set_scan_enable *cp;

	ARG_UNUSED(op);

	cp = net_buf_add(buf, sizeof(*cp));
	cp->enable = BT_HCI_LE_SCAN_ENABLE;
	cp->filter_dup = BT_HCI_LE_SCAN_FILTER_DUP_ENABLE;

	return 0;
}

/* The flags follow the controller: each changes when the command that it is
 * about gets its response, wherever the sequence ends.
 */
static const struct bt_hci_cmd_state_set scan_disabled = {
	.target = scanner.flags,
	.bit = SCAN_FLAG_SCANNING,
	.val = false,
};

static const struct bt_hci_cmd_state_set scan_enabled = {
	.target = scanner.flags,
	.bit = SCAN_FLAG_SCANNING,
	.val = true,
};

static const struct bt_hci_cmd_state_set scan_addr_set = {
	.target = scanner.flags,
	.bit = SCAN_FLAG_ADDR_VALID,
	.val = true,
};

enum {
	SCAN_STEP_DISABLE,
	SCAN_STEP_ADDR,
	SCAN_STEP_PARAM,
	SCAN_STEP_ENABLE,
};

/* Starting the scanner and giving a running one other parameters are the
 * same sequence: each step sends what is missing when it is its turn.
 */
static const struct bt_hci_cmd_step scan_start_steps[] = {
	[SCAN_STEP_DISABLE] = {BT_HCI_OP_LE_SET_SCAN_ENABLE, scan_disable_encode, &scan_disabled},
	[SCAN_STEP_ADDR] = {BT_HCI_OP_LE_SET_RANDOM_ADDRESS, scan_addr_encode, &scan_addr_set},
	[SCAN_STEP_PARAM] = {BT_HCI_OP_LE_SET_SCAN_PARAM, scan_param_encode},
	[SCAN_STEP_ENABLE] = {BT_HCI_OP_LE_SET_SCAN_ENABLE, scan_enable_encode, &scan_enabled},
};

static const struct bt_hci_cmd_seq scan_start_seq = BT_HCI_CMD_SEQ_INIT(scan_start_steps, NULL);

/* Start scanning without waiting for it. The parameters are taken over into
 * the request before the call returns, and the future of the request resolves
 * once the controller has responded to the last command, or the one that
 * failed.
 */
static int scan_start_async(struct scan_req *req, const bt_addr_t *addr, uint16_t interval,
			    uint16_t window)
{
	if (bt_hci_cmd_op_is_pending(&req->cmd.op)) {
		return -EBUSY;
	}

	bt_addr_copy(&req->addr, addr);
	req->interval = interval;
	req->window = window;

	return bt_hci_cmd_send_seq_async(&req->cmd, &scan_start_seq);
}

/* The same for a caller that waits: one call, one result, and nothing of the
 * request outlives the call. It returns what a function that sends the
 * commands one by one with bt_hci_cmd_send_sync() returns.
 */
static int scan_start(const bt_addr_t *addr, uint16_t interval, uint16_t window)
{
	struct scan_req req = {0};
	int err;

	err = scan_start_async(&req, addr, interval, window);
	if (err != 0) {
		return err;
	}

	return bt_hci_cmd_op_wait(&req.cmd.op);
}

static const bt_addr_t addr_a = {{0x01, 0x02, 0x03, 0x04, 0x05, 0x06}};
static const bt_addr_t addr_b = {{0x11, 0x12, 0x13, 0x14, 0x15, 0x16}};

/* The commands of the sequences among what the controller has been given */
static bool is_seq_cmd(uint16_t opcode)
{
	return opcode == BT_HCI_OP_LE_SET_RANDOM_ADDRESS || opcode == BT_HCI_OP_LE_SET_SCAN_PARAM ||
	       opcode == BT_HCI_OP_LE_SET_SCAN_ENABLE || opcode == BT_HCI_OP_READ_BD_ADDR ||
	       opcode == TEST_OPCODE;
}

static void expect_sent(const uint16_t *opcodes, size_t count)
{
	size_t found = 0U;

	zassert_true(test_log_len <= ARRAY_SIZE(test_log), "More commands than the log holds");

	for (unsigned int i = 0U; i < test_log_len; i++) {
		if (!is_seq_cmd(test_log[i])) {
			continue;
		}

		zassert_true(found < count, "Command 0x%04x was not expected", test_log[i]);
		zassert_equal(test_log[i], opcodes[found], "Command %zu is 0x%04x, not 0x%04x",
			      found, test_log[i], opcodes[found]);
		found++;
	}

	zassert_equal(found, count, "%zu commands, not %zu", found, count);
}

#define EXPECT_SENT(...)                                                                           \
	do {                                                                                       \
		static const uint16_t expected[] = {__VA_ARGS__};                                  \
												   \
		expect_sent(expected, ARRAY_SIZE(expected));                                       \
	} while (false)

/* A request for the cases that look at how far it got */
static struct scan_req request;

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	test_driver_reset();

	if (!bt_is_ready()) {
		zassert_ok(bt_enable(NULL), "Bluetooth init failed");
	}

	(void)memset(&scanner, 0, sizeof(scanner));
	(void)memset(&request, 0, sizeof(request));
	disable_encode_block = false;
	k_sem_reset(&disable_encode_sem);
	k_sem_reset(&disable_encode_gate);
	test_log_len = 0U;
}

ZTEST_SUITE(cmd_seq, test_with_reset, NULL, before, NULL, NULL);

static bool scanning(void)
{
	return atomic_test_bit(scanner.flags, SCAN_FLAG_SCANNING);
}

/* The address that the controller has according to the record of it */
static bool ctlr_has(const bt_addr_t *addr)
{
	return atomic_test_bit(scanner.flags, SCAN_FLAG_ADDR_VALID) &&
	       bt_addr_eq(&scanner.ctlr_addr, addr);
}

static void recover(void)
{
	zassert_ok(bt_disable(), "Bluetooth disable failed");
	test_driver_reset();
	zassert_ok(bt_enable(NULL), "Bluetooth init failed");
}

/* The commands that are needed, in order, and one result at the end */
static ZTEST(cmd_seq, test_all_steps)
{
	zassert_ok(scan_start(&addr_a, 0x60, 0x30), "Scanning did not start");

	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
	zassert_true(scanning(), "Not scanning");
	zassert_true(ctlr_has(&addr_a), "The address is not on record");
}

/* Each step decides when it is its turn whether it has something to send:
 * for a scanner that runs, the same sequence stops it first and leaves the
 * address out, which the controller has.
 */
static ZTEST(cmd_seq, test_steps_by_state)
{
	zassert_ok(scan_start(&addr_a, 0x60, 0x30), "Scanning did not start");
	test_log_len = 0U;

	zassert_ok(scan_start(&addr_a, 0x40, 0x20), "Scanning did not start again");

	EXPECT_SENT(BT_HCI_OP_LE_SET_SCAN_ENABLE, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
	zassert_true(scanning(), "Not scanning");
}

/* A command that the controller refuses ends the sequence there. The caller
 * learns which one it was, and what the commands before it have changed
 * stays recorded: the scanner is off, and the controller has the new address.
 */
static ZTEST(cmd_seq, test_step_fails)
{
	zassert_ok(scan_start(&addr_a, 0x60, 0x30), "Scanning did not start");
	test_log_len = 0U;

	test_other_opcode = BT_HCI_OP_LE_SET_SCAN_PARAM;
	test_other.status = BT_HCI_ERR_CMD_DISALLOWED;

	zassert_ok(scan_start_async(&request, &addr_b, 0x60, 0x30), "Starting failed");
	zassert_ok(bt_future_wait(&request.cmd.fut, K_SECONDS(1)), "The sequence did not complete");

	EXPECT_SENT(BT_HCI_OP_LE_SET_SCAN_ENABLE, BT_HCI_OP_LE_SET_RANDOM_ADDRESS,
		    BT_HCI_OP_LE_SET_SCAN_PARAM);
	zassert_equal(request.cmd.fut.result, BT_HCI_ERR_CMD_DISALLOWED, "The result is %d",
		      request.cmd.fut.result);
	zassert_equal(bt_hci_cmd_op_wait(&request.cmd.op), -EACCES,
		      "A refused command was not reported");
	zassert_equal(bt_hci_cmd_op_step(&request.cmd.op), SCAN_STEP_PARAM,
		      "The sequence stopped at step %u", bt_hci_cmd_op_step(&request.cmd.op));
	zassert_false(scanning(), "Scanning although it was stopped");
	zassert_true(ctlr_has(&addr_b), "The address is not on record");
}

/* A command that the driver does not take ends the sequence as well, and
 * the next one is not held up by it.
 */
static ZTEST(cmd_seq, test_send_fails)
{
	test_other_opcode = BT_HCI_OP_LE_SET_SCAN_PARAM;
	test_other.send_err = -ENODEV;

	zassert_ok(scan_start_async(&request, &addr_a, 0x60, 0x30), "Starting failed");
	zassert_ok(bt_future_wait(&request.cmd.fut, K_SECONDS(1)), "The sequence did not complete");
	zassert_equal(request.cmd.fut.result, -EIO, "The result is %d", request.cmd.fut.result);
	zassert_equal(bt_hci_cmd_op_step(&request.cmd.op), SCAN_STEP_PARAM,
		      "The sequence stopped at step %u", bt_hci_cmd_op_step(&request.cmd.op));
	zassert_true(ctlr_has(&addr_a), "The address is not on record");
	zassert_false(scanning(), "Scanning without the command for it");

	test_other.send_err = 0;

	zassert_ok(scan_start(&addr_a, 0x60, 0x30), "Scanning did not start");
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
	zassert_true(scanning(), "Not scanning");
}

/* A command that gets no response ends the sequence when the command
 * deadline passes. What the commands before it have changed is on record,
 * and after a restart of the Host a sequence goes through again.
 */
static ZTEST(cmd_seq, test_not_answered)
{
	test_other_opcode = BT_HCI_OP_LE_SET_SCAN_PARAM;
	test_other.delay = K_FOREVER;

	zassert_ok(scan_start_async(&request, &addr_a, 0x60, 0x30), "Starting failed");
	zassert_ok(bt_future_wait(&request.cmd.fut, K_MSEC(DEADLINE_MS + MSEC_PER_SEC)),
		   "The sequence did not complete");
	zassert_equal(request.cmd.fut.result, -ETIMEDOUT, "The result is %d",
		      request.cmd.fut.result);
	zassert_equal(bt_hci_cmd_op_step(&request.cmd.op), SCAN_STEP_PARAM,
		      "The sequence stopped at step %u", bt_hci_cmd_op_step(&request.cmd.op));
	zassert_true(ctlr_has(&addr_a), "The address is not on record");

	/* The next sequence is not held up by what is left of this one */
	recover();
	(void)memset(&scanner, 0, sizeof(scanner));
	test_log_len = 0U;

	zassert_ok(scan_start(&addr_a, 0x60, 0x30), "Scanning did not start");
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
}

/* bt_disable() resets the controller with a command like any other, which
 * does not get in between either: a sequence that is on its way is finished
 * first.
 */
static ZTEST(cmd_seq, test_finished_before_disable)
{
	test_other_opcode = BT_HCI_OP_LE_SET_RANDOM_ADDRESS;
	test_other.delay = K_MSEC(50);

	zassert_ok(scan_start_async(&request, &addr_a, 0x60, 0x30), "Starting failed");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS);

	zassert_ok(bt_disable(), "Bluetooth disable failed");

	zassert_ok(bt_future_wait(&request.cmd.fut, K_NO_WAIT), "The sequence did not complete");
	zassert_equal(request.cmd.fut.result, 0, "The result is %d", request.cmd.fut.result);
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
	zassert_equal(test_log[3], BT_HCI_OP_RESET, "Command 0x%04x followed the sequence",
		      test_log[3]);
}

static K_SEM_DEFINE(competitor_done, 0, 1);
static int competitor_err;

static void competitor_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	competitor_err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_BD_ADDR, NULL, NULL);
	k_sem_give(&competitor_done);
}

static K_WORK_DEFINE(competitor_work, competitor_handler);

/* Nothing gets in between the commands of a sequence: a synchronous command
 * that was queued while the first one was outstanding is sent after the last
 * one. Its sender is on the system workqueue, which is where the TX processor
 * runs in one scenario of this suite and where the sender then sends what is
 * ahead of its own command by itself.
 */
static ZTEST(cmd_seq, test_not_interrupted)
{
	test_other_opcode = BT_HCI_OP_LE_SET_RANDOM_ADDRESS;
	test_other.delay = K_FOREVER;

	zassert_ok(scan_start_async(&request, &addr_a, 0x60, 0x30), "Starting failed");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS);

	k_sem_reset(&competitor_done);
	zassert_true(k_work_submit(&competitor_work) >= 0, "Starting the other sender failed");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS);

	test_cmd_respond(&test_other);

	zassert_ok(bt_future_wait(&request.cmd.fut, K_SECONDS(1)), "The sequence did not complete");
	zassert_equal(request.cmd.fut.result, 0, "The result is %d", request.cmd.fut.result);
	zassert_ok(k_sem_take(&competitor_done, K_SECONDS(1)), "The other sender did not finish");
	zassert_ok(competitor_err, "The other command failed");

	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE, BT_HCI_OP_READ_BD_ADDR);
}

/* An encoder runs without the lock of the command queue, so a command can be
 * queued while the first command of a sequence is being encoded. It is sent
 * after the sequence, as the queue was empty when the sequence got its turn.
 */
static ZTEST(cmd_seq, test_queued_while_encoding)
{
	zassert_ok(scan_start(&addr_a, 0x60, 0x30), "Scanning did not start");
	test_log_len = 0U;

	disable_encode_block = true;
	zassert_ok(scan_start_async(&request, &addr_a, 0x40, 0x20), "Starting failed");
	zassert_ok(k_sem_take(&disable_encode_sem, K_SECONDS(1)), "The encoder did not run");

	k_sem_reset(&competitor_done);
	zassert_true(k_work_submit(&competitor_work) >= 0, "Starting the other sender failed");
	k_sleep(K_MSEC(10));
	zassert_equal(test_log_len, 0U, "A command was sent under the encoder");

	k_sem_give(&disable_encode_gate);

	zassert_ok(bt_future_wait(&request.cmd.fut, K_SECONDS(1)), "The sequence did not complete");
	zassert_equal(request.cmd.fut.result, 0, "The result is %d", request.cmd.fut.result);
	zassert_ok(k_sem_take(&competitor_done, K_SECONDS(1)), "The other sender did not finish");
	zassert_ok(competitor_err, "The other command failed");

	EXPECT_SENT(BT_HCI_OP_LE_SET_SCAN_ENABLE, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE, BT_HCI_OP_READ_BD_ADDR);
}

/* A sequence that is given up lets go of what it has held back. The
 * controller keeps the buffer of the command that it has responded to, the
 * sender behind the sequence has a buffer for its command, and the test takes
 * what else the pool has: nothing is left for the next command of the
 * sequence, the command deadline gives the sequence up, and the other command
 * is sent.
 */
static ZTEST(cmd_seq, test_given_up_lets_go)
{
	/* More than the Host has command buffers in any scenario of this suite */
	struct net_buf *spare[32];
	size_t count = 0U;

	test_other_opcode = BT_HCI_OP_LE_SET_RANDOM_ADDRESS;
	test_other.delay = K_FOREVER;
	test_other.keep = true;

	zassert_ok(scan_start_async(&request, &addr_a, 0x60, 0x30), "Starting failed");
	k_sleep(K_MSEC(10));

	k_sem_reset(&competitor_done);
	zassert_true(k_work_submit(&competitor_work) >= 0, "Starting the other sender failed");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS);

	while (count < ARRAY_SIZE(spare)) {
		spare[count] = bt_hci_cmd_alloc(K_NO_WAIT);
		if (spare[count] == NULL) {
			break;
		}

		count++;
	}

	zassert_true(count < ARRAY_SIZE(spare), "The Host has more command buffers than expected");

	test_cmd_respond(&test_other);

	zassert_ok(bt_future_wait(&request.cmd.fut, K_SECONDS(11)),
		   "The sequence was not given up");
	zassert_equal(request.cmd.fut.result, -ENOBUFS, "The result is %d",
		      request.cmd.fut.result);
	zassert_ok(k_sem_take(&competitor_done, K_SECONDS(1)),
		   "The other sender is still held back");
	zassert_ok(competitor_err, "The other command failed");

	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_READ_BD_ADDR);

	while (count > 0U) {
		count--;
		net_buf_unref(spare[count]);
	}
}

/* Two callers at once: each has its request on its own stack, and the
 * sequences go one after the other. The second one finds the scanner as the
 * first one left it.
 */
static K_THREAD_STACK_ARRAY_DEFINE(caller_stacks, 2, 2048);
static struct k_thread caller_threads[2];
static int caller_err[2];

static void caller_entry(void *p1, void *p2, void *p3)
{
	uintptr_t idx = (uintptr_t)p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	caller_err[idx] = scan_start(&addr_a, (idx == 0U) ? 0x60 : 0x40, 0x20);
}

static ZTEST(cmd_seq, test_two_callers)
{
	test_other_opcode = BT_HCI_OP_LE_SET_RANDOM_ADDRESS;
	test_other.delay = K_FOREVER;

	for (uintptr_t i = 0U; i < ARRAY_SIZE(caller_threads); i++) {
		caller_err[i] = -EINPROGRESS;
		(void)k_thread_create(&caller_threads[i], caller_stacks[i],
				      K_THREAD_STACK_SIZEOF(caller_stacks[i]), caller_entry,
				      (void *)i, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
		k_sleep(K_MSEC(10));
	}

	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS);
	test_cmd_respond(&test_other);

	for (size_t i = 0U; i < ARRAY_SIZE(caller_threads); i++) {
		zassert_ok(k_thread_join(&caller_threads[i], K_SECONDS(1)),
			   "Caller %zu did not return", i);
		zassert_ok(caller_err[i], "Caller %zu failed", i);
	}

	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE, BT_HCI_OP_LE_SET_SCAN_ENABLE,
		    BT_HCI_OP_LE_SET_SCAN_PARAM, BT_HCI_OP_LE_SET_SCAN_ENABLE);
	zassert_true(scanning(), "Not scanning");
}

static int blocker_err;

static void blocker_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	blocker_err = bt_hci_cmd_send_sync(TEST_OPCODE, NULL, NULL);
}

/* The steps decide by what they find when the sequence starts, which is when
 * no command is ahead of it, not when it is queued: the address that the
 * controller gets while another command is outstanding is left out.
 */
static ZTEST(cmd_seq, test_starts_when_nothing_is_ahead)
{
	test_cmd.delay = K_FOREVER;

	blocker_err = -EINPROGRESS;
	(void)k_thread_create(&caller_threads[0], caller_stacks[0],
			      K_THREAD_STACK_SIZEOF(caller_stacks[0]), blocker_entry, NULL, NULL,
			      NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_sleep(K_MSEC(10));
	EXPECT_SENT(TEST_OPCODE);

	zassert_ok(scan_start_async(&request, &addr_a, 0x60, 0x30), "Starting failed");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(TEST_OPCODE);

	bt_addr_copy(&scanner.ctlr_addr, &addr_a);
	atomic_set_bit(scanner.flags, SCAN_FLAG_ADDR_VALID);
	test_cmd_respond(&test_cmd);

	zassert_ok(bt_future_wait(&request.cmd.fut, K_SECONDS(1)), "The sequence did not complete");
	zassert_equal(request.cmd.fut.result, 0, "The result is %d", request.cmd.fut.result);
	zassert_ok(k_thread_join(&caller_threads[0], K_SECONDS(1)), "The other sender is stuck");
	zassert_ok(blocker_err, "The other command failed");

	EXPECT_SENT(TEST_OPCODE, BT_HCI_OP_LE_SET_SCAN_PARAM, BT_HCI_OP_LE_SET_SCAN_ENABLE);
}

static K_SEM_DEFINE(workq_done, 0, 1);
static int workq_err;

static void workq_start_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	workq_err = scan_start(&addr_b, 0x60, 0x30);
	k_sem_give(&workq_done);
}

static K_WORK_DEFINE(workq_start_work, workq_start_handler);

/* The whole sequence can be waited for on the Bluetooth workqueue, as a
 * single synchronous command can: nothing between its commands needs that
 * queue. The controller takes its time over the parameters.
 */
static ZTEST(cmd_seq, test_waited_for_on_bluetooth_workqueue)
{
	test_other_opcode = BT_HCI_OP_LE_SET_SCAN_PARAM;
	test_other.delay = K_MSEC(20);

	k_sem_reset(&workq_done);
	zassert_true(bt_work_submit(&workq_start_work) >= 0, "Submitting work failed");
	zassert_ok(k_sem_take(&workq_done, K_SECONDS(1)), "The work item did not finish");
	zassert_ok(workq_err, "Scanning did not start");

	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
	zassert_true(scanning(), "Not scanning");
}

/* A sequence that reads and then writes: the handler of the first step keeps
 * what the controller responded with, which the encoder of the second finds.
 */
#define READ_RP_LEN 2U

struct read_obj {
	struct bt_hci_cmd_op_fut cmd;
	/* What the test controller gives back of the first parameter byte */
	uint8_t kept;
};

static struct read_obj read_write;

static int read_encode(struct net_buf *buf, struct bt_hci_cmd_op *op)
{
	ARG_UNUSED(op);

	net_buf_add_u8(buf, 0x77);

	return 0;
}

static int read_rsp(struct bt_hci_cmd_op *op, struct net_buf *rp)
{
	if (rp->len < 1U + READ_RP_LEN) {
		return -EMSGSIZE;
	}

	/* The first byte is the status */
	CONTAINER_OF(op, struct read_obj, cmd.op)->kept = rp->data[1];

	return 0;
}

static int write_encode(struct net_buf *buf, struct bt_hci_cmd_op *op)
{
	net_buf_add_u8(buf, CONTAINER_OF(op, struct read_obj, cmd.op)->kept + 1U);

	return 0;
}

static const struct bt_hci_cmd_step read_write_steps[] = {
	{TEST_OPCODE, read_encode, NULL, read_rsp},
	{BT_HCI_OP_LE_SET_SCAN_PARAM, write_encode},
};

static const struct bt_hci_cmd_seq read_write_seq = BT_HCI_CMD_SEQ_INIT(read_write_steps, NULL);

static ZTEST(cmd_seq, test_read_then_write)
{
	/* The test command gives back the first parameter byte that it got */
	test_cmd.rp_len = READ_RP_LEN;
	test_cmd.echo = true;
	test_other_opcode = BT_HCI_OP_LE_SET_SCAN_PARAM;

	(void)memset(&read_write, 0, sizeof(read_write));
	zassert_ok(bt_hci_cmd_send_seq_async(&read_write.cmd, &read_write_seq),
		   "Sending the sequence failed");
	zassert_ok(bt_future_wait(&read_write.cmd.fut, K_SECONDS(1)),
		   "The sequence did not complete");
	zassert_equal(read_write.cmd.fut.result, 0, "The result is %d", read_write.cmd.fut.result);

	EXPECT_SENT(TEST_OPCODE, BT_HCI_OP_LE_SET_SCAN_PARAM);
	zassert_equal(test_other_params[0], 0x78, "The second command was given 0x%02x",
		      test_other_params[0]);
}

/* A sequence whose steps all have nothing to send completes at once, without
 * a command: stopping a scanner that does not run.
 */
static ZTEST(cmd_seq, test_nothing_to_send)
{
	static const struct bt_hci_cmd_step steps[] = {
		{BT_HCI_OP_LE_SET_SCAN_ENABLE, scan_disable_encode, &scan_disabled},
	};
	static const struct bt_hci_cmd_seq seq = BT_HCI_CMD_SEQ_INIT(steps, NULL);

	zassert_ok(bt_hci_cmd_send_seq_async(&request.cmd, &seq), "Sending failed");
	zassert_ok(bt_future_wait(&request.cmd.fut, K_SECONDS(1)), "The sequence did not complete");
	zassert_equal(request.cmd.fut.result, 0, "The result is %d", request.cmd.fut.result);
	zassert_equal(test_log_len, 0U, "The controller got %u commands", test_log_len);
}

/* A cancel between two commands ends the sequence there: the command that it
 * was at is not sent, and the ones before it stay done. The controller gives
 * no credit with its response to the first command, which is what holds the
 * second one back.
 */
static ZTEST(cmd_seq, test_canceled_between_steps)
{
	test_other_opcode = BT_HCI_OP_LE_SET_RANDOM_ADDRESS;
	test_other.ncmd = 0U;

	zassert_ok(scan_start_async(&request, &addr_a, 0x60, 0x30), "Starting failed");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS);

	zassert_ok(bt_hci_cmd_op_cancel(&request.cmd.op), "The cancel was refused");
	zassert_ok(bt_future_wait(&request.cmd.fut, K_SECONDS(1)), "The future did not resolve");
	zassert_equal(request.cmd.fut.result, -ECANCELED, "The result is %d",
		      request.cmd.fut.result);
	zassert_equal(bt_hci_cmd_op_step(&request.cmd.op), SCAN_STEP_PARAM,
		      "The sequence stopped at step %u", bt_hci_cmd_op_step(&request.cmd.op));
	zassert_true(ctlr_has(&addr_a), "The address is not on record");
	zassert_false(scanning(), "Scanning after a cancel");

	test_credit_grant();
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS);
}

/* The completion of a sequence through a callback, which is told once */
static K_SEM_DEFINE(seq_cb_sem, 0, 10);
static int seq_cb_result;
static uint8_t seq_cb_step;

static void seq_cb(struct bt_hci_cmd_op *op, int result)
{
	seq_cb_result = result;
	seq_cb_step = bt_hci_cmd_op_step(op);
	k_sem_give(&seq_cb_sem);
}

static ZTEST(cmd_seq, test_callback_told_once)
{
	static const struct bt_hci_cmd_seq seq = BT_HCI_CMD_SEQ_INIT(scan_start_steps, seq_cb);

	k_sem_reset(&seq_cb_sem);
	bt_addr_copy(&request.addr, &addr_a);

	zassert_ok(bt_hci_cmd_send_seq_cb(&request.cmd.op, &seq), "Sending the sequence failed");
	zassert_ok(k_sem_take(&seq_cb_sem, K_SECONDS(1)), "The callback did not run");
	k_sleep(K_MSEC(10));

	zassert_equal(k_sem_count_get(&seq_cb_sem), 0U, "The callback ran again");
	zassert_equal(seq_cb_result, 0, "The result is %d", seq_cb_result);
	zassert_equal(seq_cb_step, ARRAY_SIZE(scan_start_steps), "Told at step %u", seq_cb_step);
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
	zassert_true(scanning(), "Not scanning");
}

/* What a command has changed is on record once the controller has responded
 * to it, also when the sequence is canceled before its next command gets its
 * turn: the controller has the address of the canceled request, so the next
 * request for the address that it had before sets that one again. The next
 * command is kept from its turn by the host lock, which it is encoded with.
 */
static ZTEST(cmd_seq, test_recorded_before_cancel)
{
	zassert_ok(scan_start(&addr_a, 0x60, 0x30), "Scanning did not start");
	test_log_len = 0U;

	test_other_opcode = BT_HCI_OP_LE_SET_RANDOM_ADDRESS;
	test_other.delay = K_FOREVER;

	zassert_ok(scan_start_async(&request, &addr_b, 0x60, 0x30), "Starting failed");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_SET_SCAN_ENABLE, BT_HCI_OP_LE_SET_RANDOM_ADDRESS);

	bt_dev_lock();
	test_cmd_respond(&test_other);
	zassert_ok(bt_hci_cmd_op_cancel(&request.cmd.op), "The cancel was refused");
	bt_dev_unlock();

	zassert_ok(bt_future_wait(&request.cmd.fut, K_SECONDS(1)), "The future did not resolve");
	zassert_equal(request.cmd.fut.result, -ECANCELED, "The result is %d",
		      request.cmd.fut.result);
	zassert_equal(bt_hci_cmd_op_step(&request.cmd.op), SCAN_STEP_PARAM,
		      "The sequence stopped at step %u", bt_hci_cmd_op_step(&request.cmd.op));
	zassert_true(ctlr_has(&addr_b), "The address is not on record");

	test_other.delay = K_NO_WAIT;

	zassert_ok(scan_start(&addr_a, 0x60, 0x30), "Scanning did not start again");
	EXPECT_SENT(BT_HCI_OP_LE_SET_SCAN_ENABLE, BT_HCI_OP_LE_SET_RANDOM_ADDRESS,
		    BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
	zassert_true(ctlr_has(&addr_a), "The address is not on record");
}

/* The handler of a step can end the sequence with a result of its own, for an
 * response that makes the rest pointless. The controller has done what the
 * command asked for all the same, which is on record.
 */
static int refuse_rsp(struct bt_hci_cmd_op *op, struct net_buf *rp)
{
	ARG_UNUSED(op);
	ARG_UNUSED(rp);

	return -ENODEV;
}

static ZTEST(cmd_seq, test_step_ends_sequence)
{
	static const struct bt_hci_cmd_step steps[] = {
		{TEST_OPCODE, NULL, &scan_enabled, refuse_rsp},
		{BT_HCI_OP_LE_SET_SCAN_PARAM, NULL},
	};
	static const struct bt_hci_cmd_seq seq = BT_HCI_CMD_SEQ_INIT(steps, NULL);
	static struct bt_hci_cmd_op_fut req;

	zassert_ok(bt_hci_cmd_send_seq_async(&req, &seq), "Sending the sequence failed");
	zassert_ok(bt_future_wait(&req.fut, K_SECONDS(1)), "The sequence did not complete");
	zassert_equal(req.fut.result, -ENODEV, "The result is %d", req.fut.result);
	zassert_equal(bt_hci_cmd_op_step(&req.op), 0U, "The sequence stopped at step %u",
		      bt_hci_cmd_op_step(&req.op));
	zassert_true(scanning(), "What the command did is not on record");
	EXPECT_SENT(TEST_OPCODE);
}

/* A sequence that has nothing to send gives its callback success, also when
 * the operation was used for a command that failed before.
 */
static ZTEST(cmd_seq, test_nothing_to_send_after_failure)
{
	static const struct bt_hci_cmd_seq failing = BT_HCI_CMD_SEQ_INIT(scan_start_steps, seq_cb);
	static const struct bt_hci_cmd_step stop_steps[] = {
		{BT_HCI_OP_LE_SET_SCAN_ENABLE, scan_disable_encode, &scan_disabled},
	};
	static const struct bt_hci_cmd_seq stop = BT_HCI_CMD_SEQ_INIT(stop_steps, seq_cb);

	k_sem_reset(&seq_cb_sem);
	bt_addr_copy(&request.addr, &addr_a);
	test_other_opcode = BT_HCI_OP_LE_SET_RANDOM_ADDRESS;
	test_other.status = BT_HCI_ERR_CMD_DISALLOWED;

	zassert_ok(bt_hci_cmd_send_seq_cb(&request.cmd.op, &failing),
		   "Sending the sequence failed");
	zassert_ok(k_sem_take(&seq_cb_sem, K_SECONDS(1)), "The callback did not run");
	zassert_equal(seq_cb_result, BT_HCI_ERR_CMD_DISALLOWED, "The result is %d", seq_cb_result);

	zassert_ok(bt_hci_cmd_send_seq_cb(&request.cmd.op, &stop), "Sending the sequence failed");
	zassert_ok(k_sem_take(&seq_cb_sem, K_SECONDS(1)), "The callback did not run");
	zassert_equal(seq_cb_result, 0, "The result is %d", seq_cb_result);
	zassert_equal(seq_cb_step, ARRAY_SIZE(stop_steps), "Told at step %u", seq_cb_step);
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS);
}

/* The callback of a sequence finds what the handler of its last step has kept
 * of the return parameters in the object of the operation.
 */
static uint8_t seq_cb_rp;

static void seq_rp_cb(struct bt_hci_cmd_op *op, int result)
{
	seq_cb_rp = CONTAINER_OF(op, struct read_obj, cmd.op)->kept;
	seq_cb(op, result);
}

static ZTEST(cmd_seq, test_callback_reads_answer)
{
	static const struct bt_hci_cmd_step steps[] = {
		{BT_HCI_OP_LE_SET_SCAN_PARAM, NULL},
		{TEST_OPCODE, read_encode, NULL, read_rsp},
	};
	static const struct bt_hci_cmd_seq seq = BT_HCI_CMD_SEQ_INIT(steps, seq_rp_cb);
	static struct read_obj read;

	/* The response to the first command is a Command Status event, which
	 * has no return parameters for a handler.
	 */
	test_other_opcode = BT_HCI_OP_LE_SET_SCAN_PARAM;
	test_cmd.rp_len = READ_RP_LEN;
	test_cmd.echo = true;
	k_sem_reset(&seq_cb_sem);

	(void)memset(&read, 0, sizeof(read));
	zassert_ok(bt_hci_cmd_send_seq_cb(&read.cmd.op, &seq), "Sending the sequence failed");
	zassert_ok(k_sem_take(&seq_cb_sem, K_SECONDS(1)), "The callback did not run");
	zassert_equal(seq_cb_result, 0, "The result is %d", seq_cb_result);
	zassert_equal(seq_cb_rp, 0x77, "The callback read 0x%02x", seq_cb_rp);
}

static K_SEM_DEFINE(syswq_done, 0, 1);
static int syswq_err;

static void syswq_start_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	syswq_err = scan_start(&addr_b, 0x60, 0x30);
	k_sem_give(&syswq_done);
}

static K_WORK_DEFINE(syswq_start_work, syswq_start_handler);

/* The whole sequence can be waited for on the system workqueue too, where
 * the TX processor runs in one scenario of this suite: the waiter then takes
 * the sequence from one command to the next by itself. The controller takes
 * its time over the parameters.
 */
static ZTEST(cmd_seq, test_waited_for_on_system_workqueue)
{
	test_other_opcode = BT_HCI_OP_LE_SET_SCAN_PARAM;
	test_other.delay = K_FOREVER;

	k_sem_reset(&syswq_done);
	zassert_true(k_work_submit(&syswq_start_work) >= 0, "Submitting work failed");
	k_sleep(K_MSEC(20));
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM);
	zassert_equal(k_sem_count_get(&syswq_done), 0U, "The waiter returned early");

	test_cmd_respond(&test_other);

	zassert_ok(k_sem_take(&syswq_done, K_SECONDS(1)), "The work item did not finish");
	zassert_ok(syswq_err, "Scanning did not start");
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
	zassert_true(scanning(), "Not scanning");
}

/* Without a reset of the controller bt_disable() sends no command, and what
 * is on its way is handed back when the HCI transport is closed: a sequence
 * ends at the command that the controller has not responded to.
 */
ZTEST_SUITE(cmd_seq_no_reset, test_without_reset, NULL, before, NULL, NULL);

static ZTEST(cmd_seq_no_reset, test_ended_by_disable)
{
	test_other_opcode = BT_HCI_OP_LE_SET_SCAN_PARAM;
	test_other.delay = K_FOREVER;

	zassert_ok(scan_start_async(&request, &addr_a, 0x60, 0x30), "Starting failed");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM);

	zassert_ok(bt_disable(), "Bluetooth disable failed");

	zassert_ok(bt_future_wait(&request.cmd.fut, K_NO_WAIT), "The sequence was not handed back");
	zassert_equal(request.cmd.fut.result, -EHOSTDOWN, "The result is %d",
		      request.cmd.fut.result);
	zassert_equal(bt_hci_cmd_op_step(&request.cmd.op), SCAN_STEP_PARAM,
		      "The sequence stopped at step %u", bt_hci_cmd_op_step(&request.cmd.op));
	zassert_true(ctlr_has(&addr_a), "The address is not on record");

	/* The next sequence is not held up by what is left of this one */
	test_driver_reset();
	zassert_ok(bt_enable(NULL), "Bluetooth init failed");
	(void)memset(&scanner, 0, sizeof(scanner));
	test_log_len = 0U;

	zassert_ok(scan_start(&addr_a, 0x60, 0x30), "Scanning did not start");
	EXPECT_SENT(BT_HCI_OP_LE_SET_RANDOM_ADDRESS, BT_HCI_OP_LE_SET_SCAN_PARAM,
		    BT_HCI_OP_LE_SET_SCAN_ENABLE);
}
