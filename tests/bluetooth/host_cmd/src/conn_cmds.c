/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

/* A connection sends the HCI commands that the Host starts on its own through
 * one command operation, one at a time. These cases cover what that means for
 * commands that are asked for while another one is outstanding: their order,
 * what a disconnection leaves of them, and a command that is asked for more
 * than once. A caller that needs the response of the controller sends the
 * same command from its stack instead, and gets that response.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "conn_internal.h"

#include "driver.h"

#define DEADLINE_MS (10 * MSEC_PER_SEC)

/* With these the Host of a peripheral reads the remote features and asks for
 * the 2M PHY once a connection is up, and updates the connection parameters
 * with a command rather than over L2CAP.
 */
static const uint8_t le_features[8] = {
	BIT(BT_LE_FEAT_BIT_CONN_PARAM_REQ) | BIT(BT_LE_FEAT_BIT_PER_INIT_FEAT_XCHG),
	BIT(BT_LE_FEAT_BIT_PHY_2M - 8),
};

static K_SEM_DEFINE(connected_sem, 0, 1);
static K_SEM_DEFINE(disconnected_sem, 0, 1);
static struct bt_conn *conn;
static bool running;

static void connected(struct bt_conn *new_conn, uint8_t err)
{
	ARG_UNUSED(err);

	/* A connection of another suite */
	if (!running) {
		return;
	}

	conn = bt_conn_ref(new_conn);
	k_sem_give(&connected_sem);
}

static void disconnected(struct bt_conn *old_conn, uint8_t reason)
{
	ARG_UNUSED(reason);

	if (!running || old_conn != conn) {
		return;
	}

	bt_conn_drop(&conn);
	k_sem_give(&disconnected_sem);
}

BT_CONN_CB_DEFINE(conn_cmds_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void connect(void)
{
	zassert_ok(bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, NULL, 0U, NULL, 0U),
		   "Advertising failed to start");

	/* What the controller is given from here on is for the connection */
	test_log_len = 0U;
	test_connect();
	zassert_ok(k_sem_take(&connected_sem, K_SECONDS(1)), "No connection");
}

static enum bt_conn_state conn_state(void)
{
	struct bt_conn_info info;

	zassert_ok(bt_conn_get_info(conn, &info), "No connection info");

	return info.state;
}

static void count_conn(struct bt_conn *counted, void *data)
{
	unsigned int *count = data;

	ARG_UNUSED(counted);

	(*count)++;
}

/* The connection objects that somebody holds a reference to */
static unsigned int conn_count(void)
{
	unsigned int count = 0U;

	bt_conn_foreach(BT_CONN_TYPE_LE, count_conn, &count);

	return count;
}

/* The commands of the connection among what the controller has been given */
static bool is_conn_cmd(uint16_t opcode)
{
	return opcode == BT_HCI_OP_DISCONNECT || opcode == BT_HCI_OP_LE_READ_REMOTE_FEATURES ||
	       opcode == BT_HCI_OP_LE_SET_PHY || opcode == BT_HCI_OP_LE_CONN_UPDATE;
}

static void expect_sent(const uint16_t *opcodes, size_t count)
{
	size_t found = 0U;

	zassert_true(test_log_len <= ARRAY_SIZE(test_log), "More commands than the log holds");

	for (unsigned int i = 0U; i < test_log_len; i++) {
		if (!is_conn_cmd(test_log[i])) {
			continue;
		}

		zassert_true(found < count, "Command 0x%04x was not expected", test_log[i]);
		zassert_equal(test_log[i], opcodes[found], "Command %zu is 0x%04x, not 0x%04x",
			      found, test_log[i], opcodes[found]);
		found++;
	}

	zassert_equal(found, count, "%zu commands of the connection, not %zu", found, count);
}

#define EXPECT_SENT(...)                                                                           \
	do {                                                                                       \
		static const uint16_t expected[] = {0U, __VA_ARGS__};                              \
												   \
		expect_sent(&expected[1], ARRAY_SIZE(expected) - 1U);                              \
	} while (false)

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	/* The controller is only asked for its features when it is enabled */
	if (bt_is_ready()) {
		zassert_ok(bt_disable(), "Bluetooth disable failed");
	}

	test_driver_reset();
	(void)memcpy(test_le_features, le_features, sizeof(test_le_features));
	k_sem_reset(&connected_sem);
	k_sem_reset(&disconnected_sem);
	running = true;

	zassert_ok(bt_enable(NULL), "Bluetooth init failed");
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);

	test_driver_reset();

	/* Whatever is left of the connection goes with the disable */
	(void)bt_disable();
	(void)memset(test_le_features, 0, sizeof(test_le_features));
	running = false;

	bt_conn_drop(&conn);
}

ZTEST_SUITE(conn_cmds, test_with_reset, NULL, before, after, NULL);

/* While a command of the connection is outstanding the others wait, and then
 * go out in their own order, not in the order they were asked for in: the
 * disconnect that came last is sent first, and it leaves nothing to send.
 */
static ZTEST(conn_cmds, test_one_at_a_time)
{
	test_other_opcode = BT_HCI_OP_LE_READ_REMOTE_FEATURES;
	test_other.delay = K_FOREVER;

	connect();
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES);

	zassert_ok(bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN),
		   "The request was refused");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES);
	zassert_equal(conn_state(), BT_CONN_STATE_DISCONNECTING, "Wrong state while requested");

	test_cmd_respond(&test_other);
	zassert_ok(k_sem_take(&disconnected_sem, K_SECONDS(1)), "No disconnection");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES, BT_HCI_OP_DISCONNECT);
}

/* A disconnect that the controller rejects leaves the connection up, and the
 * command that waited behind it is sent after all.
 */
static ZTEST(conn_cmds, test_rejected_disconnect)
{
	test_other_opcode = BT_HCI_OP_LE_READ_REMOTE_FEATURES;
	test_other.delay = K_FOREVER;
	test_disconnect.status = BT_HCI_ERR_CMD_DISALLOWED;

	connect();
	k_sleep(K_MSEC(10));

	zassert_ok(bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN),
		   "The request was refused");
	test_cmd_respond(&test_other);
	k_sleep(K_MSEC(10));

	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES, BT_HCI_OP_DISCONNECT, BT_HCI_OP_LE_SET_PHY);
	zassert_equal(conn_state(), BT_CONN_STATE_CONNECTED, "Wrong state after the rejection");
	zassert_equal(k_sem_count_get(&disconnected_sem), 0U, "The connection was reported gone");
}

/* The peer disconnects while one command is outstanding and another waits.
 * The one that waits is dropped. The outstanding one keeps the connection
 * object until its response comes, so that the response finds what it is
 * about.
 */
static ZTEST(conn_cmds, test_peer_disconnects)
{
	test_other_opcode = BT_HCI_OP_LE_READ_REMOTE_FEATURES;
	test_other.delay = K_FOREVER;

	connect();
	k_sleep(K_MSEC(10));

	test_disconnect_complete(BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	zassert_ok(k_sem_take(&disconnected_sem, K_SECONDS(1)), "No disconnection");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES);

	zassert_equal(conn_count(), 1U, "The connection object was released under its command");

	test_cmd_respond(&test_other);
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES);
	zassert_equal(conn_count(), 0U, "The connection object was not released");

	/* The next connection starts from nothing */
	test_other.delay = K_NO_WAIT;
	connect();
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES, BT_HCI_OP_LE_SET_PHY);
}

/* When the controller stops responding, the commands that wait behind the
 * outstanding one are dropped: the connection has nothing left to send, and
 * a command that it is asked for then is refused.
 */
static ZTEST(conn_cmds, test_dropped_with_controller)
{
	test_other_opcode = BT_HCI_OP_LE_READ_REMOTE_FEATURES;
	test_other.delay = K_FOREVER;

	connect();
	k_sleep(K_MSEC(10));

	zassert_ok(bt_conn_le_conn_update_nowait(conn, BT_LE_CONN_PARAM(30, 30, 0, 400)),
		   "The update was refused");
	zassert_true(atomic_test_bit(conn->flags, BT_CONN_CMD_AUTO_PHY), "No PHY update waits");
	zassert_true(atomic_test_bit(conn->flags, BT_CONN_CMD_CONN_UPDATE), "No update waits");

	k_sleep(K_MSEC(DEADLINE_MS + 100));

	zassert_false(atomic_test_bit(conn->flags, BT_CONN_CMD_AUTO_PHY), "A PHY update waits");
	zassert_false(atomic_test_bit(conn->flags, BT_CONN_CMD_CONN_UPDATE), "An update waits");
	zassert_false(atomic_test_bit(conn->flags, BT_CONN_CMD_BUSY), "A command is on its way");
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES);

	zassert_equal(bt_conn_le_conn_update_nowait(conn, BT_LE_CONN_PARAM(30, 30, 0, 400)),
		      -EHOSTDOWN, "The update was not refused");
}

/* The interval of the HCI LE Connection Update command that the controller
 * got last: Conn_Interval_Min follows the Connection_Handle.
 */
static uint16_t updated_interval(void)
{
	return sys_get_le16(&test_other_params[2]);
}

/* Commands that wait at the same time go out in the order that the connection
 * has them in, here the PHY update before the parameter update. No in-tree
 * caller asks for two of them in the other order, so that the order is not
 * that of the requests is not covered.
 */
static ZTEST(conn_cmds, test_waiting_in_order)
{
	test_other_opcode = BT_HCI_OP_LE_READ_REMOTE_FEATURES;
	test_other.delay = K_FOREVER;

	connect();
	k_sleep(K_MSEC(10));

	zassert_ok(bt_conn_le_conn_update_nowait(conn, BT_LE_CONN_PARAM(30, 30, 0, 400)),
		   "The update was refused");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES);

	test_cmd_respond(&test_other);
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES, BT_HCI_OP_LE_SET_PHY,
		    BT_HCI_OP_LE_CONN_UPDATE);
}

/* A command that is asked for again before it has been sent is sent once,
 * with the parameters that the connection has by then.
 */
static ZTEST(conn_cmds, test_asked_twice_sent_once)
{
	connect();
	k_sleep(K_MSEC(10));
	test_log_len = 0U;

	test_other_opcode = BT_HCI_OP_LE_CONN_UPDATE;
	test_other.delay = K_FOREVER;

	zassert_ok(bt_conn_le_conn_update_nowait(conn, BT_LE_CONN_PARAM(30, 30, 0, 400)),
		   "The first update was refused");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_CONN_UPDATE);

	zassert_ok(bt_conn_le_conn_update_nowait(conn, BT_LE_CONN_PARAM(40, 40, 0, 400)),
		   "The second update was refused");
	zassert_ok(bt_conn_le_conn_update_nowait(conn, BT_LE_CONN_PARAM(50, 50, 0, 400)),
		   "The third update was refused");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_CONN_UPDATE);

	test_cmd_respond(&test_other);
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_CONN_UPDATE, BT_HCI_OP_LE_CONN_UPDATE);
	zassert_equal(updated_interval(), 50U, "Updated to an interval of %u", updated_interval());

	test_cmd_respond(&test_other);
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_CONN_UPDATE, BT_HCI_OP_LE_CONN_UPDATE);
}

/* Nobody waits for an update that the Host starts on its own. When the
 * controller refuses it, the connection stays as it is and goes on with the
 * next command that it is asked for.
 */
static ZTEST(conn_cmds, test_own_update_refused)
{
	connect();
	k_sleep(K_MSEC(10));
	test_log_len = 0U;

	test_other_opcode = BT_HCI_OP_LE_CONN_UPDATE;
	test_other.status = BT_HCI_ERR_CMD_DISALLOWED;

	zassert_ok(bt_conn_le_conn_update_nowait(conn, BT_LE_CONN_PARAM(30, 30, 0, 400)),
		   "The update was refused");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_CONN_UPDATE);
	zassert_equal(conn_state(), BT_CONN_STATE_CONNECTED, "Wrong state after the refusal");

	test_other.status = BT_HCI_ERR_SUCCESS;

	zassert_ok(bt_conn_le_conn_update_nowait(conn, BT_LE_CONN_PARAM(40, 40, 0, 400)),
		   "The next update was refused");
	k_sleep(K_MSEC(10));
	EXPECT_SENT(BT_HCI_OP_LE_CONN_UPDATE, BT_HCI_OP_LE_CONN_UPDATE);
	zassert_equal(updated_interval(), 40U, "Updated to an interval of %u", updated_interval());
}

/* Bring a connection up to where bt_conn_le_param_update() sends a command:
 * the peer has the procedure too, and the time that a peripheral waits before
 * it asks for other parameters has passed.
 */
static void connect_for_param_update(void)
{
	connect();
	test_remote_features(le_features);
	k_sleep(K_MSEC(CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT + 100));

	test_log_len = 0U;
}

/* bt_conn_le_param_update() returns what the controller responded with,
 * which it waits for.
 */
static ZTEST(conn_cmds, test_update_returns_answer)
{
	int64_t start;
	int err;

	connect_for_param_update();

	test_other_opcode = BT_HCI_OP_LE_CONN_UPDATE;
	test_other.delay = K_MSEC(50);

	start = k_uptime_get();
	zassert_ok(bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(30, 30, 0, 400)),
		   "The update failed");
	zassert_true(k_uptime_get() - start >= 50, "Returned before the controller responded");
	zassert_equal(updated_interval(), 30U, "Updated to an interval of %u", updated_interval());

	test_other.status = BT_HCI_ERR_INVALID_PARAM;
	err = bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(40, 40, 0, 400));
	zassert_equal(err, -EINVAL, "Invalid parameters gave %d", err);

	test_other.status = BT_HCI_ERR_CMD_DISALLOWED;
	err = bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(50, 50, 0, 400));
	zassert_equal(err, -EACCES, "A disallowed command gave %d", err);

	EXPECT_SENT(BT_HCI_OP_LE_CONN_UPDATE, BT_HCI_OP_LE_CONN_UPDATE, BT_HCI_OP_LE_CONN_UPDATE);
	zassert_equal(conn_state(), BT_CONN_STATE_CONNECTED, "Wrong state after the refusals");
}

static K_SEM_DEFINE(syswq_update_done, 0, 1);
static int syswq_update_err;

static void syswq_update_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	syswq_update_err = bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(30, 30, 0, 400));
	k_sem_give(&syswq_update_done);
}

static K_WORK_DEFINE(syswq_update_work, syswq_update_handler);

/* The same from the system workqueue, where callers in the tree are and
 * where the TX processor runs in one scenario of this suite.
 */
static ZTEST(conn_cmds, test_update_from_system_workqueue)
{
	connect_for_param_update();

	test_other_opcode = BT_HCI_OP_LE_CONN_UPDATE;
	test_other.status = BT_HCI_ERR_INVALID_PARAM;

	k_sem_reset(&syswq_update_done);
	zassert_true(k_work_submit(&syswq_update_work) >= 0, "Starting the caller failed");
	zassert_ok(k_sem_take(&syswq_update_done, K_SECONDS(1)), "The caller did not return");
	zassert_equal(syswq_update_err, -EINVAL, "Invalid parameters gave %d", syswq_update_err);
	EXPECT_SENT(BT_HCI_OP_LE_CONN_UPDATE);
}

/* A caller that still has the connection object after the connection is
 * gone is told so, and the controller is not given a command for it.
 */
static ZTEST(conn_cmds, test_update_after_disconnection)
{
	struct bt_conn *gone;
	int err;

	connect_for_param_update();
	gone = bt_conn_ref(conn);

	test_disconnect_complete(BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	zassert_ok(k_sem_take(&disconnected_sem, K_SECONDS(1)), "No disconnection");

	err = bt_conn_le_param_update(gone, BT_LE_CONN_PARAM(30, 30, 0, 400));
	bt_conn_unref(gone);

	zassert_equal(err, -ENOTCONN, "The update gave %d", err);
	EXPECT_SENT();
}

/* A peripheral that is asked for other parameters right after the connection
 * came up keeps them until the time that it waits before an update has
 * passed. The Host then sends the command on its own.
 */
static ZTEST(conn_cmds, test_update_at_timeout)
{
	test_other_opcode = BT_HCI_OP_LE_CONN_UPDATE;

	connect();
	test_remote_features(le_features);
	zassert_ok(bt_conn_le_param_update(conn, BT_LE_CONN_PARAM(30, 30, 0, 400)),
		   "The update was refused");
	k_sleep(K_MSEC(100));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES, BT_HCI_OP_LE_SET_PHY);

	k_sleep(K_MSEC(CONFIG_BT_CONN_PARAM_UPDATE_TIMEOUT));
	EXPECT_SENT(BT_HCI_OP_LE_READ_REMOTE_FEATURES, BT_HCI_OP_LE_SET_PHY,
		    BT_HCI_OP_LE_CONN_UPDATE);
	zassert_equal(updated_interval(), 30U, "Updated to an interval of %u", updated_interval());
}
