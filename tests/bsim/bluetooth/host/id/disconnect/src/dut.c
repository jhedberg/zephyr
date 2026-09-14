/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "babblekit/flags.h"
#include "babblekit/sync.h"
#include "babblekit/testcase.h"

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

static void connect_with_id(uint8_t id)
{
	const struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	};
	struct bt_le_adv_param param = *BT_LE_ADV_CONN_FAST_1;
	struct bt_conn_info info;
	int err;

	param.id = id;

	UNSET_FLAG(flag_connected);
	UNSET_FLAG(flag_disconnected);

	err = bt_le_adv_start(&param, ad, ARRAY_SIZE(ad), NULL, 0);
	TEST_ASSERT(err == 0, "Advertising with identity %u failed (err %d)", id, err);

	WAIT_FOR_FLAG(flag_connected);
	printk("Connected with identity %u\n", id);

	err = bt_conn_get_info(g_conn, &info);
	TEST_ASSERT(err == 0, "bt_conn_get_info failed (err %d)", err);
	TEST_ASSERT(info.id == id, "Connection uses identity %u, expected %u", info.id, id);
}

static void expect_local_disconnect(const char *what)
{
	WAIT_FOR_FLAG(flag_disconnected);
	printk("Disconnected after %s, reason 0x%02x\n", what, g_disconnect_reason);
	TEST_ASSERT(g_disconnect_reason == BT_HCI_ERR_LOCALHOST_TERM_CONN,
		    "%s disconnected with reason 0x%02x", what, g_disconnect_reason);
}

void test_peripheral_main(void)
{
	int id;
	int err;

	err = bt_enable(NULL);
	TEST_ASSERT(err == 0, "Bluetooth init failed (err %d)", err);

	err = bk_sync_init();
	TEST_ASSERT(err == 0, "Backchannel init failed (err %d)", err);

	err = bt_conn_cb_register(&conn_callbacks);
	TEST_ASSERT(err == 0, "Callback registration failed (err %d)", err);

	/* Deleting the identity of an unbonded connection disconnects it */
	id = bt_id_create(NULL, NULL);
	TEST_ASSERT(id > BT_ID_DEFAULT, "Identity creation failed (err %d)", id);

	connect_with_id((uint8_t)id);

	err = bt_id_delete((uint8_t)id);
	TEST_ASSERT(err == 0, "bt_id_delete failed (err %d)", err);
	expect_local_disconnect("bt_id_delete");

	/* Resetting the identity of an unbonded connection disconnects it */
	id = bt_id_create(NULL, NULL);
	TEST_ASSERT(id > BT_ID_DEFAULT, "Identity creation failed (err %d)", id);

	connect_with_id((uint8_t)id);

	err = bt_id_reset((uint8_t)id, NULL, NULL);
	TEST_ASSERT(err == id, "bt_id_reset returned %d, expected %d", err, id);
	expect_local_disconnect("bt_id_reset");

	/* The central ends the simulation once it has been told the peripheral is done */
	bk_sync_send();
	TEST_PASS("Peripheral done");
}
