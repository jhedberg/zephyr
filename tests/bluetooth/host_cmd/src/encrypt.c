/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/* The HCI LE Start Encryption command of a central that has a bond with its
 * peer. SMP asks for it without waiting when the peer sends a Security
 * Request, and bt_conn_set_security() waits for the response of the controller.
 * These cases cover what the command is given in both, and what a refusal of
 * the controller leads to.
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "hci_core.h"
#include "keys.h"

#include "driver.h"

#define SMP_CID              0x0006U
#define SMP_PAIRING_FAILED   0x05U
#define SMP_SECURITY_REQUEST 0x0bU
#define SMP_AUTH_BONDING     0x01U
#define SMP_ERR_UNSPECIFIED  0x08U

/* Shorter than the key, so that the command has to be given the rest as zero */
#define ENC_SIZE 7U

static const uint8_t ltk[16] = {0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
				0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf};
static const uint8_t ltk_rand[8] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
static const uint8_t ltk_ediv[2] = {0x20, 0x21};

static K_SEM_DEFINE(connected_sem, 0, 1);
static K_SEM_DEFINE(security_sem, 0, 1);
static struct bt_conn *conn;
static enum bt_security_err security_err;
static bool running;

static void connected(struct bt_conn *new_conn, uint8_t err)
{
	/* A connection of another suite */
	if (!running || err != 0U) {
		return;
	}

	k_sem_give(&connected_sem);
}

static void security_changed(struct bt_conn *changed, bt_security_t level, enum bt_security_err err)
{
	ARG_UNUSED(level);

	if (!running || changed != conn) {
		return;
	}

	security_err = err;
	k_sem_give(&security_sem);
}

BT_CONN_CB_DEFINE(encrypt_callbacks) = {
	.connected = connected,
	.security_changed = security_changed,
};

/* A bond with a legacy LTK, which has a random number and a diversifier */
static void bond(void)
{
	struct bt_keys *keys = bt_keys_get_addr(BT_ID_DEFAULT, &test_peer);

	zassert_not_null(keys, "No room for the keys");

	keys->keys = BT_KEYS_LTK;
	keys->enc_size = ENC_SIZE;
	(void)memcpy(keys->ltk.val, ltk, sizeof(keys->ltk.val));
	(void)memcpy(keys->ltk.rand, ltk_rand, sizeof(keys->ltk.rand));
	(void)memcpy(keys->ltk.ediv, ltk_ediv, sizeof(keys->ltk.ediv));
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	if (bt_is_ready()) {
		zassert_ok(bt_disable(), "Bluetooth disable failed");
	}

	test_driver_reset();
	k_sem_reset(&connected_sem);
	k_sem_reset(&security_sem);
	running = true;

	zassert_ok(bt_enable(NULL), "Bluetooth init failed");

	bond();

	zassert_ok(bt_conn_le_create(&test_peer, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
				     &conn),
		   "Creating the connection failed");
	test_connect_central();
	zassert_ok(k_sem_take(&connected_sem, K_SECONDS(1)), "No connection");

	/* What the Host sends when a connection comes up is of no interest */
	k_sleep(K_MSEC(10));
	test_log_len = 0U;
	test_acl_count = 0U;
	test_other_opcode = BT_HCI_OP_LE_START_ENCRYPTION;
}

static void after(void *fixture)
{
	ARG_UNUSED(fixture);

	test_driver_reset();

	bt_conn_drop(&conn);

	/* The connection and the bond go with the disable */
	(void)bt_disable();
	running = false;
}

ZTEST_SUITE(conn_encrypt, test_with_reset, NULL, before, after, NULL);

static unsigned int encrypt_count(void)
{
	unsigned int count = 0U;

	zassert_true(test_log_len <= ARRAY_SIZE(test_log), "More commands than the log holds");

	for (unsigned int i = 0U; i < test_log_len; i++) {
		if (test_log[i] == BT_HCI_OP_LE_START_ENCRYPTION) {
			count++;
		}
	}

	return count;
}

/* The command names the connection and has the key of the bond, cut to its
 * size, with the random number and the diversifier that belong to it.
 */
static void check_params(void)
{
	const struct bt_hci_cp_le_start_encryption *cp = (const void *)test_other_params;
	static const uint8_t zero[sizeof(ltk) - ENC_SIZE];

	zassert_equal(sys_le16_to_cpu(cp->handle), TEST_CONN_HANDLE, "Wrong connection");
	zassert_mem_equal(cp->rand, ltk_rand, sizeof(ltk_rand), "Wrong random number");
	zassert_mem_equal(cp->ediv, ltk_ediv, sizeof(ltk_ediv), "Wrong diversifier");
	zassert_mem_equal(cp->ltk, ltk, ENC_SIZE, "Wrong key");
	zassert_mem_equal(&cp->ltk[ENC_SIZE], zero, sizeof(zero), "The key was not cut");
}

static void security_request(void)
{
	static const uint8_t pdu[] = {SMP_SECURITY_REQUEST, SMP_AUTH_BONDING};

	test_acl_recv(SMP_CID, pdu, sizeof(pdu));
}

/* bt_conn_set_security() returns what the controller responded with, which
 * it waits for. After a refusal SMP is as before: the next call starts over.
 */
static ZTEST(conn_encrypt, test_set_security_returns_answer)
{
	int64_t start;
	int err;

	test_other.status = BT_HCI_ERR_CMD_DISALLOWED;

	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	zassert_equal(err, -EACCES, "A disallowed command gave %d", err);
	zassert_equal(encrypt_count(), 1U, "The controller got %u commands", encrypt_count());
	check_params();

	test_other.status = BT_HCI_ERR_SUCCESS;
	test_other.delay = K_MSEC(50);

	start = k_uptime_get();
	zassert_ok(bt_conn_set_security(conn, BT_SECURITY_L2), "Starting encryption failed");
	zassert_true(k_uptime_get() - start >= 50, "Returned before the controller responded");
	zassert_equal(encrypt_count(), 2U, "The controller got %u commands", encrypt_count());
	check_params();

	zassert_equal(k_sem_count_get(&security_sem), 0U, "Told before anything has changed");
	test_encrypt_change(BT_HCI_ERR_SUCCESS);
	zassert_ok(k_sem_take(&security_sem, K_SECONDS(1)), "No security change");
	zassert_equal(security_err, BT_SECURITY_ERR_SUCCESS, "Security failed with %d",
		      security_err);
}

static K_SEM_DEFINE(workq_free, 0, 1);

static void workq_free_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	k_sem_give(&workq_free);
}

static K_WORK_DEFINE(workq_free_work, workq_free_handler);

/* For a Security Request of the peer SMP starts encryption from the Bluetooth
 * workqueue, which it does not hold up while the controller has not
 * responded.
 */
static ZTEST(conn_encrypt, test_security_request_not_waited_for)
{
	test_other.delay = K_FOREVER;

	security_request();
	k_sleep(K_MSEC(10));
	zassert_equal(encrypt_count(), 1U, "The controller got %u commands", encrypt_count());
	check_params();

	k_sem_reset(&workq_free);
	zassert_true(bt_work_submit(&workq_free_work) >= 0, "Submitting work failed");
	zassert_ok(k_sem_take(&workq_free, K_MSEC(100)), "The Bluetooth workqueue is held up");

	test_cmd_respond(&test_other);
	test_encrypt_change(BT_HCI_ERR_SUCCESS);
	zassert_ok(k_sem_take(&security_sem, K_SECONDS(1)), "No security change");
	zassert_equal(security_err, BT_SECURITY_ERR_SUCCESS, "Security failed with %d",
		      security_err);
	zassert_equal(test_acl_count, 0U, "The peer was sent %u packets", test_acl_count);
}

/* When the controller refuses that command, SMP ends what it started
 * encryption for as it does for any other error: the peer is sent Pairing
 * Failed. SMP is as before afterwards, so encryption can be started again.
 */
static ZTEST(conn_encrypt, test_security_request_refused)
{
	test_other.status = BT_HCI_ERR_CMD_DISALLOWED;

	security_request();
	k_sleep(K_MSEC(10));
	zassert_equal(encrypt_count(), 1U, "The controller got %u commands", encrypt_count());

	zassert_equal(test_acl_count, 1U, "The peer was sent %u packets", test_acl_count);
	zassert_equal(sys_get_le16(&test_acl_data[2]), SMP_CID, "Not an SMP PDU");
	zassert_equal(test_acl_data[4], SMP_PAIRING_FAILED, "SMP code 0x%02x", test_acl_data[4]);
	zassert_equal(test_acl_data[5], SMP_ERR_UNSPECIFIED, "SMP reason 0x%02x", test_acl_data[5]);

	test_other.status = BT_HCI_ERR_SUCCESS;

	zassert_ok(bt_conn_set_security(conn, BT_SECURITY_L2), "Starting encryption failed");
	zassert_equal(encrypt_count(), 2U, "The controller got %u commands", encrypt_count());
}

/* A command that still waits when the connection goes is not sent. Here it
 * waits because the controller has not responded to the one before it.
 */
static ZTEST(conn_encrypt, test_dropped_at_disconnection)
{
	test_cmd.delay = K_FOREVER;
	zassert_ok(bt_hci_cmd_send(TEST_OPCODE, NULL), "Sending the other command failed");
	k_sleep(K_MSEC(10));

	security_request();
	k_sleep(K_MSEC(10));
	zassert_equal(encrypt_count(), 0U, "The controller got %u commands", encrypt_count());

	test_disconnect_complete(BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	k_sleep(K_MSEC(10));

	test_cmd_respond(&test_cmd);
	k_sleep(K_MSEC(10));
	zassert_equal(encrypt_count(), 0U, "The controller got %u commands", encrypt_count());
}
