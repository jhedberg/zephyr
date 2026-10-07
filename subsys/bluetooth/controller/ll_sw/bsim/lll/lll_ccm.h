/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Bluetooth LE AES-CCM of data channel PDUs (Core Spec Vol 6, Part E),
 * done in software on whole PDUs.
 */

/* Encrypt the payload of pdu into out, appending the MIC. A PDU without
 * payload is copied as is.
 */
void lll_ccm_encrypt(const struct ccm *ccm, const struct pdu_data *pdu, struct pdu_data *out);

/* Decrypt the payload of pdu into out, without the MIC. Returns true if the
 * MIC is valid. A PDU without payload is copied as is and is valid.
 */
bool lll_ccm_decrypt(const struct ccm *ccm, const struct pdu_data *pdu, struct pdu_data *out);
