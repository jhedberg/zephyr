/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* The radio model has no CCM, so the PDUs are encrypted and decrypted in
 * software, a whole PDU at a time (Core Spec Vol 6, Part E).
 */

/* The header bits that can differ between transmissions of the same payload
 * are not authenticated, NESN, SN and MD in ACL PDUs.
 */
#define LLL_CCM_HDR_MASK_ACL ((uint8_t)~(BIT(2) | BIT(3) | BIT(4)))

/* NESN, SN, CIE and NPI in CIS PDUs */
#define LLL_CCM_HDR_MASK_CIS ((uint8_t)~(BIT(2) | BIT(3) | BIT(4) | BIT(6)))

/* CSSN and CSTF in BIS PDUs */
#define LLL_CCM_HDR_MASK_BIS ((uint8_t)~(GENMASK(4, 2) | BIT(5)))

/* Encrypt the payload of pdu into out, appending the MIC. A PDU without
 * payload is copied as is.
 */
void lll_ccm_encrypt(const struct ccm *ccm, uint8_t hdr_mask, const void *pdu, void *out);

/* Decrypt the payload of pdu into out, without the MIC. Returns true if the
 * MIC is valid. A PDU without payload is copied as is and is valid.
 */
bool lll_ccm_decrypt(const struct ccm *ccm, uint8_t hdr_mask, const void *pdu, void *out);
