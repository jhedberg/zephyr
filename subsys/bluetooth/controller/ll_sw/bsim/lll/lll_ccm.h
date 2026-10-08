/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Bluetooth LE AES-CCM of ACL, CIS and BIS PDUs (Core Spec Vol 6, Part E),
 * done in software on whole PDUs: a 2 octet header and the payload.
 */

/* Header bits authenticated by the MIC, per PDU type: the bits that can
 * change on a retransmission are not.
 */
#define LLL_CCM_HDR_MASK_ACL 0xE3U /* Not NESN, SN and MD */
#define LLL_CCM_HDR_MASK_CIS 0xA3U /* Not NESN, SN, CIE and NPI */
#define LLL_CCM_HDR_MASK_BIS 0xC3U /* Not CSSN and CSTF */

/* Encrypt the payload of pdu into out, appending the MIC. A PDU without
 * payload is copied as is.
 */
void lll_ccm_encrypt(const struct ccm *ccm, uint8_t hdr_mask, const void *pdu, void *out);

/* Decrypt the payload of pdu into out, without the MIC. Returns true if the
 * MIC is valid. A PDU without payload is copied as is and is valid.
 */
bool lll_ccm_decrypt(const struct ccm *ccm, uint8_t hdr_mask, const void *pdu, void *out);
