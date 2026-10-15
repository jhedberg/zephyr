/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TESTS_BLUETOOTH_HOST_CMD_SRC_DRIVER_H_
#define TESTS_BLUETOOTH_HOST_CMD_SRC_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/hci_types.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

/* A vendor command that only the tests send */
#define TEST_OPCODE BT_OP(BT_OGF_VS, 0x03f0)

/* The handle of the connection that test_connect() brings up */
#define TEST_CONN_HANDLE 0x0001U

/** What the test controller does with one command. It responds to a command
 *  without a script with success and one command credit from within send().
 */
struct test_cmd_script {
	/** When the response comes: K_NO_WAIT from within send(), K_FOREVER never */
	k_timeout_t delay;
	/** Status of the response */
	uint8_t status;
	/** Num_HCI_Command_Packets of the response */
	uint8_t ncmd;
	/** Length of the return parameters after the status (TEST_OPCODE only) */
	uint8_t rp_len;
	/** The return parameters repeat the first parameter byte of the command
	 *  instead of counting up (TEST_OPCODE only)
	 */
	bool echo;
	/** What send() returns instead of taking the command, if not 0 */
	int send_err;
	/** The controller keeps the buffer of the command, as a driver may,
	 *  until the next test_driver_reset() (test_other only)
	 */
	bool keep;
	/** How often the controller has been given the command */
	unsigned int count;
};

/** TEST_OPCODE, whose response is a Command Complete event */
extern struct test_cmd_script test_cmd;

/** Parameter length of the first few TEST_OPCODE commands the controller got */
extern uint8_t test_cmd_plen[4];

/** HCI Disconnect, whose response is a Command Status event, followed by a
 *  Disconnection Complete event when the status is success.
 */
extern struct test_cmd_script test_disconnect;

/** One more command, which a test picks by setting its opcode, whose response
 *  is a Command Status event. test_driver_reset() picks none.
 */
extern uint16_t test_other_opcode;
extern struct test_cmd_script test_other;

/** Parameters of the last command with test_other_opcode */
extern uint8_t test_other_params[32];

/** A command that the controller responds to as it responds to the commands
 *  that bt_enable() sends, with a Command Complete event, but only once the
 *  test calls test_held_respond(): for a test that looks at what happens in
 *  the meantime. test_driver_reset() picks none.
 */
extern uint16_t test_held_opcode;

/** Status of the response to test_held_opcode */
extern uint8_t test_held_status;

/** How often the controller has been given test_held_opcode */
extern unsigned int test_held_count;

/** Send the response to test_held_opcode now, if the controller holds it */
void test_held_respond(void);

/** HCI version that the controller reports, 0 after test_driver_reset() */
extern uint8_t test_hci_version;

/** The controller has no public address, false after test_driver_reset() */
extern bool test_no_public_addr;

/** The opcodes of the commands that the controller has been given, in order,
 *  since the last test_driver_reset() or since a test set the length to zero.
 */
extern uint16_t test_log[16];
extern unsigned int test_log_len;

/** LE features that the controller reports. Not touched by test_driver_reset() */
extern uint8_t test_le_features[8];

/** How many ACL packets the controller has been given since the last
 *  test_driver_reset(), and the first bytes of the last one behind its ACL
 *  header, which is where the L2CAP header starts.
 */
extern unsigned int test_acl_count;
extern uint8_t test_acl_data[8];

/** The peer of the connection that the test controller reports */
extern const bt_addr_le_t test_peer;

/** How often the controller has been reset and the transport closed */
extern unsigned int test_reset_count;
extern unsigned int test_close_count;

/** Back to a controller that responds at once, counters at zero */
void test_driver_reset(void);

/** Deliver the response of a scripted command now */
void test_cmd_respond(const struct test_cmd_script *script);

/** Deliver a Command Complete event for no command that grants a command credit */
void test_credit_grant(void);

/** Whether the Host resets the test controller, which it does unless the
 *  devicetree gives the controller the "no-reset" quirk.
 */
#define TEST_DRIVER_RESETS (!DT_NODE_HAS_PROP(DT_NODELABEL(bt_hci_test), bt_hci_quirks))

/** Suite predicates: most suites need the reset, one cannot do with it */
bool test_with_reset(const void *state);
bool test_without_reset(const void *state);

/** Deliver an LE Connection Complete event for TEST_CONN_HANDLE, peripheral role */
void test_connect(void);

/** The same in the central role, for a connection that the Host is creating */
void test_connect_central(void);

/** Deliver an Encryption Change event for TEST_CONN_HANDLE */
void test_encrypt_change(uint8_t status);

/** Deliver an L2CAP PDU from the peer of TEST_CONN_HANDLE */
void test_acl_recv(uint16_t cid, const uint8_t *data, uint8_t len);

/** Deliver a Disconnection Complete event for TEST_CONN_HANDLE */
void test_disconnect_complete(uint8_t reason);

/** Deliver an LE Read Remote Features Complete event for TEST_CONN_HANDLE */
void test_remote_features(const uint8_t features[8]);

#endif /* TESTS_BLUETOOTH_HOST_CMD_SRC_DRIVER_H_ */
