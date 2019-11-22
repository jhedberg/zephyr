/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <soc.h>
#include <zephyr.h>
#include <sys/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <sys/byteorder.h>

#define BT_LE_SCAN_CUSTOM BT_LE_SCAN_PARAM(BT_HCI_LE_SCAN_ACTIVE, \
					   BT_HCI_LE_SCAN_FILTER_DUP_DISABLE, \
					   0x04, \
					   0x04)

static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

static int conn_update(struct bt_conn *conn)
{
	struct bt_le_conn_param param;
	static u16_t conn_interval;
	int err;

	param.interval_min = conn_interval + 0x06;
	param.interval_max = conn_interval + 0x06;
	param.latency = 0;
	param.timeout = 3000;

	err = bt_conn_le_param_update(conn, &param);
	if (err) {
		static u8_t flag = 0;

		printk("conn update failed (err %d).", err);
		conn_interval++;
		if (conn_interval > 0xC80) {
			conn_interval = 0;
		}

		if (!flag || conn_interval == 0x0010) {
			flag = 1;
			printk("bt_conn_security...\n");
			//err = bt_conn_security(conn, BT_SECURITY_FIPS);
			err = bt_conn_security(conn, BT_SECURITY_MEDIUM);
			if (err) {
				printk("bt_conn_security failed(%d)", err);
			} else {
				printk("bt_conn_security success.");
			}
		}
	} else {
		printk("conn update initiated.");
	}

	return err;
}

static u32_t write_count;
static u32_t write_len;
static u32_t write_rate;

static int write_cmd(struct bt_conn *conn);

static void write_cmd_cb(struct bt_conn *conn, void *user_data)
{
	static u32_t cycle_stamp;
	u32_t delta;
	u16_t len;

	len = bt_gatt_get_mtu(conn) - 3;

	delta = k_cycle_get_32() - cycle_stamp;
	delta = SYS_CLOCK_HW_CYCLES_TO_NS(delta);

	/* if last data rx-ed was greater than 1 second in the past,
	 * reset the metrics.
	 */
	if (delta > 1000000000) {
		printk("Write: count= %u, len= %u, rate= %u bps.\n",
		       write_count, write_len, write_rate);

		write_count = 0U;
		write_len = 0U;
		write_rate = 0U;
		cycle_stamp = k_cycle_get_32();
	} else {
		write_count++;
		write_len += len;
		write_rate = ((u64_t)write_len << 3) * 1000000000U / delta;
	}

	//write_cmd(conn);
	//write_cmd(conn);
}

static int write_cmd(struct bt_conn *conn)
{
	u8_t data[244] = {0, };
	int err;

	err = bt_gatt_write_without_response_cb(conn, 0x0001, data,
						bt_gatt_get_mtu(conn) - 3,
						false, write_cmd_cb, NULL);
	if (err) {
		printk("Write cmd failed (%d).\n", err);
	}

	return err;
}

static void mtu_exchange_cb(struct bt_conn *conn, u8_t err,
			    struct bt_gatt_exchange_params *params)
{
	printk("MTU exchange %s (%u)\n", err == 0U ? "successful" : "failed",
	       bt_gatt_get_mtu(conn));

	//write_cmd(conn);
}

static struct bt_gatt_exchange_params mtu_exchange_params;

static int mtu_exchange(struct bt_conn *conn)
{
	int err;

	printk("MTU: %u\n", bt_gatt_get_mtu(conn));

	mtu_exchange_params.func = mtu_exchange_cb;

	err = bt_gatt_exchange_mtu(conn, &mtu_exchange_params);
	if (err) {
		printk("MTU exchange failed (err %d)", err);
	} else {
		printk("Exchange pending...");
	}

	return err;
}

static u8_t notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, u16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	printk("[NOTIFICATION] data %p length %u\n", data, length);

	//conn_update(conn);

	return BT_GATT_ITER_CONTINUE;
}

static u8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;

	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printk("[ATTRIBUTE] handle %u\n", attr->handle);

	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_HRS)) {
		memcpy(&uuid, BT_UUID_HRS_MEASUREMENT, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_HRS_MEASUREMENT)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_params.notify = notify_func;
		subscribe_params.value = BT_GATT_CCC_NOTIFY;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED]\n");
		}

		return BT_GATT_ITER_STOP;
	}

	return BT_GATT_ITER_STOP;
}

static bool eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
	int i;

	printk("[AD]: %u data_len %u\n", data->type, data->data_len);

	switch (data->type) {
	case BT_DATA_UUID16_SOME:
	case BT_DATA_UUID16_ALL:
		if (data->data_len % sizeof(u16_t) != 0U) {
			printk("AD malformed\n");
			return true;
		}

		for (i = 0; i < data->data_len; i += sizeof(u16_t)) {
			struct bt_uuid *uuid;
			u16_t u16;
			int err;

			memcpy(&u16, &data->data[i], sizeof(u16));
			uuid = BT_UUID_DECLARE_16(sys_le16_to_cpu(u16));
			if (bt_uuid_cmp(uuid, BT_UUID_HRS)) {
				continue;
			}

			err = bt_le_scan_stop();
			if (err) {
				printk("Stop LE scan failed (err %d)\n", err);
				continue;
			}

			(void)bt_conn_create_le(addr, BT_LE_CONN_PARAM_DEFAULT);

			return false;
		}
	}

	return true;
}

static void device_found(const bt_addr_le_t *addr, s8_t rssi, u8_t type,
			 struct net_buf_simple *ad)
{
	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));

	printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
	       dev, type, ad->len, rssi);

	/* We're only interested in connectable events */
	if (type == BT_LE_ADV_IND || type == BT_LE_ADV_DIRECT_IND) {
		bt_data_parse(ad, eir_found, (void *)addr);
	}
}

static int scan_start(void)
{
	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type       = BT_HCI_LE_SCAN_ACTIVE,
		.filter_dup = BT_HCI_LE_SCAN_FILTER_DUP_DISABLE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	ARG_UNUSED(scan_param);

	return bt_le_scan_start(BT_LE_SCAN_CUSTOM, device_found);
}

static struct bt_conn *g_conn;

static void connected(struct bt_conn *conn, u8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr, conn_err);
		return;
	}

	printk("Connected: %s\n", addr);

	#if 0
	err = scan_start();
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
	}
	#endif

	g_conn = conn;

	mtu_exchange(conn);

	#if 0
	memcpy(&uuid, BT_UUID_HRS, sizeof(uuid));
	discover_params.uuid = &uuid.uuid;
	discover_params.func = discover_func;
	discover_params.start_handle = 0x0001;
	discover_params.end_handle = 0xffff;
	discover_params.type = BT_GATT_DISCOVER_PRIMARY;

	err = bt_gatt_discover(conn, &discover_params);
	if (err) {
		printk("Discover failed (err %d)\n", err);
	}
	#endif
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected: %s (reason %u)\n", addr, reason);

	g_conn = NULL;

	bt_conn_unref(conn);

	err = scan_start();
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
	}
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	printk("LE conn  param req: int (0x%04x, 0x%04x) lat %d to %d\n",
	       param->interval_min, param->interval_max, param->latency,
	       param->timeout);

	return true;
}

static void le_param_updated(struct bt_conn *conn, u16_t interval,
			     u16_t latency, u16_t timeout)
{
	printk("LE conn param updated: int 0x%04x lat %d to %d\n", interval,
	       latency, timeout);
}

static void security_changed(struct bt_conn *conn, bt_security_t level)
{
	printk("Security changed to level %u\n", level);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	.security_changed = security_changed,
};

void main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	printk("Bluetooth initialized\n");

	bt_conn_cb_register(&conn_callbacks);

	err = scan_start();
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");

	while (1) {
		//k_sleep(MSEC_PER_SEC);
		k_sleep(K_MSEC(3));
		//k_yield();

		if (g_conn) {
			write_cmd(g_conn);
		}
	}
}
