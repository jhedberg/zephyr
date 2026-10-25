/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Silicon Laboratories Inc.
 * SPDX-License-Identifier: Apache-2.0
 */

/* A scriptable controller behind an HCI driver, for tests of the Host's HCI
 * command handling. It responds to the initialization commands as far as the
 * Host needs, and lets a test decide when and how it responds to two commands.
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/buf.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "driver.h"

#define DT_DRV_COMPAT zephyr_bt_hci_test

/* Large enough for the return parameters of initialization commands that are
 * irrelevant to these tests.
 */
#define GENERIC_RP_SIZE 64U

#define LE_ACL_MAX_LEN 27U
#define LE_ACL_MAX_NUM 3U

struct test_cmd_script test_cmd;
struct test_cmd_script test_disconnect;
struct test_cmd_script test_other;
uint16_t test_other_opcode;
uint8_t test_other_params[32];
uint16_t test_held_opcode;
uint8_t test_held_status;
unsigned int test_held_count;

/* Whether the controller holds back its response to test_held_opcode */
static atomic_t held;

uint8_t test_hci_version;
bool test_no_public_addr;
uint16_t test_log[16];
unsigned int test_log_len;
uint8_t test_le_features[8];
unsigned int test_acl_count;
uint8_t test_acl_data[8];
uint8_t test_cmd_plen[4];
unsigned int test_reset_count;
unsigned int test_close_count;

static const struct device *const hci_dev = DEVICE_DT_GET(DT_DRV_INST(0));

/* First parameter byte of the TEST_OPCODE command that the controller got last */
static uint8_t test_cmd_param;

static void *evt_create(struct net_buf **buf, uint8_t evt, uint8_t len)
{
	struct bt_hci_evt_hdr *hdr;

	*buf = bt_buf_get_evt(evt, false, K_FOREVER);
	hdr = net_buf_add(*buf, sizeof(*hdr));
	hdr->evt = evt;
	hdr->len = len;

	return net_buf_add(*buf, len);
}

static void *cmd_complete(struct net_buf **buf, uint16_t opcode, uint8_t ncmd, uint8_t plen)
{
	struct bt_hci_evt_cmd_complete *cc;

	cc = evt_create(buf, BT_HCI_EVT_CMD_COMPLETE, sizeof(*cc) + plen);
	cc->ncmd = ncmd;
	cc->opcode = sys_cpu_to_le16(opcode);
	(void)memset(&cc[1], 0, plen);

	return &cc[1];
}

void test_cmd_respond(const struct test_cmd_script *script)
{
	struct net_buf *evt;

	if (script == &test_disconnect || script == &test_other) {
		struct bt_hci_evt_cmd_status *status;

		status = evt_create(&evt, BT_HCI_EVT_CMD_STATUS, sizeof(*status));
		status->status = script->status;
		status->ncmd = script->ncmd;
		status->opcode = sys_cpu_to_le16((script == &test_other) ? test_other_opcode
									  : BT_HCI_OP_DISCONNECT);
		bt_hci_recv(hci_dev, evt);

		if (script == &test_disconnect && script->status == BT_HCI_ERR_SUCCESS) {
			test_disconnect_complete(BT_HCI_ERR_LOCALHOST_TERM_CONN);
		}
	} else {
		uint8_t *rp;

		rp = cmd_complete(&evt, TEST_OPCODE, script->ncmd, 1U + script->rp_len);
		rp[0] = script->status;
		for (uint8_t i = 0U; i < script->rp_len; i++) {
			rp[1U + i] = script->echo ? test_cmd_param : i;
		}

		bt_hci_recv(hci_dev, evt);
	}
}

void test_credit_grant(void)
{
	struct net_buf *evt;

	(void)cmd_complete(&evt, 0U, 1U, 0U);
	bt_hci_recv(hci_dev, evt);
}

bool test_with_reset(const void *state)
{
	ARG_UNUSED(state);

	return TEST_DRIVER_RESETS;
}

bool test_without_reset(const void *state)
{
	ARG_UNUSED(state);

	return !TEST_DRIVER_RESETS;
}

static void cmd_rsp_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	test_cmd_respond(&test_cmd);
}

static void disconnect_rsp_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	test_cmd_respond(&test_disconnect);
}

static void other_rsp_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	test_cmd_respond(&test_other);
}

static K_WORK_DELAYABLE_DEFINE(cmd_rsp_work, cmd_rsp_handler);
static K_WORK_DELAYABLE_DEFINE(disconnect_rsp_work, disconnect_rsp_handler);
static K_WORK_DELAYABLE_DEFINE(other_rsp_work, other_rsp_handler);

static void scripted(struct test_cmd_script *script, struct k_work_delayable *work)
{
	script->count++;

	if (K_TIMEOUT_EQ(script->delay, K_NO_WAIT)) {
		test_cmd_respond(script);
	} else if (!K_TIMEOUT_EQ(script->delay, K_FOREVER)) {
		(void)k_work_schedule(work, script->delay);
	}
}

const bt_addr_le_t test_peer = {
	.type = BT_ADDR_LE_RANDOM,
	.a.val = {0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0xc6U},
};

static void connect_as(uint8_t role)
{
	struct bt_hci_evt_le_conn_complete *cc;
	struct bt_hci_evt_le_meta_event *meta;
	struct net_buf *evt;

	meta = evt_create(&evt, BT_HCI_EVT_LE_META_EVENT, sizeof(*meta) + sizeof(*cc));
	meta->subevent = BT_HCI_EVT_LE_CONN_COMPLETE;
	cc = (void *)&meta[1];
	(void)memset(cc, 0, sizeof(*cc));
	cc->status = BT_HCI_ERR_SUCCESS;
	cc->handle = sys_cpu_to_le16(TEST_CONN_HANDLE);
	cc->role = role;
	bt_addr_le_copy(&cc->peer_addr, &test_peer);
	cc->interval = sys_cpu_to_le16(BT_GAP_INIT_CONN_INT_MIN);
	cc->supv_timeout = sys_cpu_to_le16(BT_GAP_MS_TO_CONN_TIMEOUT(4000U));

	bt_hci_recv(hci_dev, evt);
}

void test_connect(void)
{
	connect_as(BT_HCI_ROLE_PERIPHERAL);
}

void test_connect_central(void)
{
	connect_as(BT_HCI_ROLE_CENTRAL);
}

void test_encrypt_change(uint8_t status)
{
	struct bt_hci_evt_encrypt_change *change;
	struct net_buf *evt;

	change = evt_create(&evt, BT_HCI_EVT_ENCRYPT_CHANGE, sizeof(*change));
	change->status = status;
	change->handle = sys_cpu_to_le16(TEST_CONN_HANDLE);
	change->encrypt = (status == BT_HCI_ERR_SUCCESS) ? 0x01U : 0x00U;

	bt_hci_recv(hci_dev, evt);
}

void test_acl_recv(uint16_t cid, const uint8_t *data, uint8_t len)
{
	struct bt_hci_acl_hdr *acl;
	struct net_buf *buf;

	buf = bt_buf_get_rx(BT_BUF_ACL_IN, K_FOREVER);
	acl = net_buf_add(buf, sizeof(*acl));
	acl->handle = sys_cpu_to_le16(bt_acl_handle_pack(TEST_CONN_HANDLE, BT_ACL_START));
	acl->len = sys_cpu_to_le16(4U + len);

	/* The L2CAP header: length and channel */
	net_buf_add_le16(buf, len);
	net_buf_add_le16(buf, cid);
	net_buf_add_mem(buf, data, len);

	bt_hci_recv(hci_dev, buf);
}

void test_remote_features(const uint8_t features[8])
{
	struct bt_hci_evt_le_remote_feat_complete *complete;
	struct bt_hci_evt_le_meta_event *meta;
	struct net_buf *evt;

	meta = evt_create(&evt, BT_HCI_EVT_LE_META_EVENT, sizeof(*meta) + sizeof(*complete));
	meta->subevent = BT_HCI_EVT_LE_REMOTE_FEAT_COMPLETE;
	complete = (void *)&meta[1];
	complete->status = BT_HCI_ERR_SUCCESS;
	complete->handle = sys_cpu_to_le16(TEST_CONN_HANDLE);
	(void)memcpy(complete->features, features, sizeof(complete->features));

	bt_hci_recv(hci_dev, evt);
}

void test_disconnect_complete(uint8_t reason)
{
	struct bt_hci_evt_disconn_complete *complete;
	struct net_buf *evt;

	complete = evt_create(&evt, BT_HCI_EVT_DISCONN_COMPLETE, sizeof(*complete));
	complete->status = BT_HCI_ERR_SUCCESS;
	complete->handle = sys_cpu_to_le16(TEST_CONN_HANDLE);
	complete->reason = reason;

	bt_hci_recv(hci_dev, evt);
}

/* The command buffer that the controller keeps, see test_cmd_script.keep */
static struct net_buf *kept_cmd;

/* The response to a command that is not scripted: what the Host needs of the
 * commands that bt_enable() sends, zeroes otherwise
 */
static void respond(uint16_t opcode, uint8_t status)
{
	struct net_buf *evt;
	uint8_t *rp;

	rp = cmd_complete(&evt, opcode, 1U, GENERIC_RP_SIZE);
	rp[0] = status;

	switch (opcode) {
	case BT_HCI_OP_READ_LOCAL_FEATURES: {
		struct bt_hci_rp_read_local_features *features = (void *)rp;

		/* LE supported and BR/EDR not supported */
		features->features[4] = BIT(5) | BIT(6);
		break;
	}
	case BT_HCI_OP_READ_SUPPORTED_COMMANDS: {
		struct bt_hci_rp_read_supported_commands *commands = (void *)rp;

		(void)memset(commands->commands, 0xFF, sizeof(commands->commands));
		break;
	}
	case BT_HCI_OP_READ_LOCAL_VERSION_INFO: {
		struct bt_hci_rp_read_local_version_info *version = (void *)rp;

		version->hci_version = test_hci_version;
		break;
	}
	case BT_HCI_OP_READ_BD_ADDR: {
		struct bt_hci_rp_read_bd_addr *addr = (void *)rp;

		if (!test_no_public_addr) {
			(void)memset(&addr->bdaddr, 0x11, sizeof(addr->bdaddr));
		}
		break;
	}
	case BT_HCI_OP_LE_READ_BUFFER_SIZE: {
		struct bt_hci_rp_le_read_buffer_size *size = (void *)rp;

		size->le_max_len = sys_cpu_to_le16(LE_ACL_MAX_LEN);
		size->le_max_num = LE_ACL_MAX_NUM;
		break;
	}
	case BT_HCI_OP_LE_READ_LOCAL_FEATURES: {
		struct bt_hci_rp_le_read_local_features *features = (void *)rp;

		(void)memcpy(features->features, test_le_features, sizeof(features->features));
		break;
	}
	case BT_HCI_OP_LE_READ_SUPP_STATES: {
		struct bt_hci_rp_le_read_supp_states *states = (void *)rp;

		(void)memset(states->le_states, 0xFF, sizeof(states->le_states));
		break;
	}
	default:
		break;
	}

	bt_hci_recv(hci_dev, evt);
}

void test_held_respond(void)
{
	if (atomic_cas(&held, 1, 0)) {
		respond(test_held_opcode, test_held_status);
	}
}

static void cmd_handle(struct net_buf *cmd)
{
	struct bt_hci_cmd_hdr *chdr;
	uint16_t opcode;

	chdr = net_buf_pull_mem(cmd, sizeof(*chdr));
	opcode = sys_le16_to_cpu(chdr->opcode);

	if (test_log_len < ARRAY_SIZE(test_log)) {
		test_log[test_log_len] = opcode;
	}

	test_log_len++;

	if (opcode == test_other_opcode) {
		(void)memset(test_other_params, 0, sizeof(test_other_params));
		(void)memcpy(test_other_params, cmd->data,
			     MIN(cmd->len, sizeof(test_other_params)));
		if (test_other.keep && kept_cmd == NULL) {
			kept_cmd = net_buf_ref(cmd);
		}

		scripted(&test_other, &other_rsp_work);
		return;
	}

	switch (opcode) {
	case TEST_OPCODE:
		if (test_cmd.count < ARRAY_SIZE(test_cmd_plen)) {
			test_cmd_plen[test_cmd.count] = (uint8_t)cmd->len;
		}

		test_cmd_param = (cmd->len > 0U) ? cmd->data[0] : 0U;
		scripted(&test_cmd, &cmd_rsp_work);
		return;
	case BT_HCI_OP_DISCONNECT:
		scripted(&test_disconnect, &disconnect_rsp_work);
		return;
	case BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS:
		/* The one command that a controller does not respond to */
		return;
	case BT_HCI_OP_RESET:
		test_reset_count++;
		break;
	default:
		break;
	}

	if (opcode == test_held_opcode) {
		test_held_count++;
		atomic_set(&held, 1);
		return;
	}

	respond(opcode, BT_HCI_ERR_SUCCESS);
}

static int send_err(const struct net_buf *buf)
{
	uint16_t opcode = sys_get_le16(buf->data);

	if (opcode == TEST_OPCODE) {
		return test_cmd.send_err;
	}

	if (opcode == BT_HCI_OP_DISCONNECT) {
		return test_disconnect.send_err;
	}

	if (opcode == test_other_opcode) {
		return test_other.send_err;
	}

	return 0;
}

static int driver_open(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static int driver_close(const struct device *dev)
{
	ARG_UNUSED(dev);

	test_close_count++;

	return 0;
}

static int driver_send(const struct device *dev, struct net_buf *buf)
{
	ARG_UNUSED(dev);

	if (buf->data[0] == BT_HCI_H4_CMD) {
		int err;

		(void)net_buf_pull_u8(buf);

		err = send_err(buf);
		if (err != 0) {
			/* The buffer stays with the caller */
			return err;
		}

		cmd_handle(buf);
	} else if (buf->data[0] == BT_HCI_H4_ACL) {
		size_t hdr_len = 1U + sizeof(struct bt_hci_acl_hdr);

		test_acl_count++;
		(void)memset(test_acl_data, 0, sizeof(test_acl_data));
		if (buf->len > hdr_len) {
			(void)memcpy(test_acl_data, &buf->data[hdr_len],
				     MIN(buf->len - hdr_len, sizeof(test_acl_data)));
		}
	} else {
		/* Nothing else is looked at */
	}

	net_buf_unref(buf);

	return 0;
}

static DEVICE_API(bt_hci, driver_api) = {
	.open = driver_open,
	.close = driver_close,
	.send = driver_send,
};

void test_driver_reset(void)
{
	static const struct test_cmd_script respond_at_once = {
		.delay = K_NO_WAIT,
		.status = BT_HCI_ERR_SUCCESS,
		.ncmd = 1U,
	};

	(void)k_work_cancel_delayable(&cmd_rsp_work);
	(void)k_work_cancel_delayable(&disconnect_rsp_work);
	(void)k_work_cancel_delayable(&other_rsp_work);
	net_buf_drop(&kept_cmd);

	test_cmd = respond_at_once;
	test_disconnect = respond_at_once;
	test_other = respond_at_once;
	test_other_opcode = 0U;
	test_held_opcode = 0U;
	test_held_status = BT_HCI_ERR_SUCCESS;
	test_held_count = 0U;
	atomic_set(&held, 0);
	test_hci_version = 0U;
	test_no_public_addr = false;
	test_log_len = 0U;
	test_acl_count = 0U;
	test_reset_count = 0U;
	test_close_count = 0U;
}

#define TEST_DEVICE_INIT(inst)                                                                     \
	static struct bt_hci_driver_data driver_data_##inst = {0};                                 \
	static const struct bt_hci_driver_config driver_config_##inst =                            \
		BT_DT_HCI_DRIVER_CONFIG_INST_GET(inst);                                            \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &driver_data_##inst, &driver_config_##inst,        \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &driver_api)

DT_INST_FOREACH_STATUS_OKAY(TEST_DEVICE_INIT)
