/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/printk.h>

#include "babblekit/flags.h"
#include "babblekit/sync.h"
#include "babblekit/testcase.h"

#define ROUNDS 2

DEFINE_FLAG_STATIC(flag_connected);
DEFINE_FLAG_STATIC(flag_disconnected);

static struct bt_conn *g_conn;
static uint8_t g_disconnect_reason;

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0) {
		TEST_FAIL("Connection failed (err 0x%02x)", err);
		return;
	}

	g_conn = bt_conn_ref(conn);
	SET_FLAG(flag_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	if (conn != g_conn) {
		return;
	}

	bt_conn_unref(g_conn);
	g_conn = NULL;
	g_disconnect_reason = reason;
	SET_FLAG(flag_disconnected);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	struct bt_conn *conn;
	int err;

	if (type != BT_HCI_ADV_IND) {
		return;
	}

	err = bt_le_scan_stop();
	TEST_ASSERT(err == 0, "Stopping the scan failed (err %d)", err);

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &conn);
	TEST_ASSERT(err == 0, "Connection creation failed (err %d)", err);

	bt_conn_unref(conn);
}

void test_central_main(void)
{
	int err;

	err = bt_enable(NULL);
	TEST_ASSERT(err == 0, "Bluetooth init failed (err %d)", err);

	err = bk_sync_init();
	TEST_ASSERT(err == 0, "Backchannel init failed (err %d)", err);

	err = bt_conn_cb_register(&conn_callbacks);
	TEST_ASSERT(err == 0, "Callback registration failed (err %d)", err);

	for (int round = 0; round < ROUNDS; round++) {
		UNSET_FLAG(flag_connected);
		UNSET_FLAG(flag_disconnected);

		err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
		TEST_ASSERT(err == 0, "Scanning failed to start (err %d)", err);

		WAIT_FOR_FLAG(flag_connected);
		WAIT_FOR_FLAG(flag_disconnected);
		printk("Round %d disconnected, reason 0x%02x\n", round, g_disconnect_reason);
		TEST_ASSERT(g_disconnect_reason == BT_HCI_ERR_REMOTE_USER_TERM_CONN,
			    "Round %d disconnected with reason 0x%02x", round, g_disconnect_reason);
	}

	/* Let the peripheral observe its own disconnection before ending the simulation */
	bk_sync_wait();
	TEST_PASS_AND_EXIT("Central done");
}
