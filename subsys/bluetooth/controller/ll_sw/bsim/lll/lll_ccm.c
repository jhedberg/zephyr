/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "hal/ccm.h"
#include "hal/ecb.h"

#include "pdu_df.h"
#include "pdu_vendor.h"
#include "pdu.h"

#include "lll_ccm.h"

/* The header bits that change on a retransmission, NESN, SN and MD, are not
 * authenticated.
 */
#define CCM_HDR_MASK ((uint8_t)~(BIT(2) | BIT(3) | BIT(4)))

/* Flags of the B0 block: Adata, a 4 octet MIC (M' = 1) and a 2 octet length
 * (L' = 1), and of the A blocks: L' = 1 (RFC 3610).
 */
#define CCM_B0_FLAGS (BIT(6) | BIT(3) | BIT(0))
#define CCM_A_FLAGS  BIT(0)

static void ccm_nonce(const struct ccm *ccm, uint8_t nonce[13])
{
	sys_put_le32((uint32_t)ccm->counter, &nonce[0]);
	nonce[4] = ((ccm->counter >> 32) & BIT_MASK(7)) | (ccm->direction << 7);
	(void)memcpy(&nonce[5], ccm->iv, sizeof(ccm->iv));
}

static void block_xor(uint8_t *dst, const uint8_t *src, uint8_t len)
{
	for (uint8_t i = 0U; i < len; i++) {
		dst[i] ^= src[i];
	}
}

/* Apply the CTR key stream (blocks A1..An) to the payload in place */
static void ccm_ctr(const struct ccm *ccm, const uint8_t nonce[13], uint8_t *payload,
		    uint8_t len)
{
	uint8_t blk[16];
	uint8_t s[16];

	for (uint16_t off = 0U, i = 1U; off < len; off += 16U, i++) {
		blk[0] = CCM_A_FLAGS;
		(void)memcpy(&blk[1], nonce, 13U);
		sys_put_be16(i, &blk[14]);
		ecb_encrypt_be(ccm->key, blk, s);
		block_xor(&payload[off], s, MIN(16U, len - off));
	}
}

static void ccm_mic(const struct ccm *ccm, const uint8_t nonce[13], uint8_t hdr,
		    const uint8_t *payload, uint8_t len, uint8_t mic[PDU_MIC_SIZE])
{
	uint8_t blk[16];
	uint8_t x[16];

	/* B0: flags, nonce and payload length */
	blk[0] = CCM_B0_FLAGS;
	(void)memcpy(&blk[1], nonce, 13U);
	sys_put_be16(len, &blk[14]);
	ecb_encrypt_be(ccm->key, blk, x);

	/* B1: the masked header is the additional authenticated data */
	(void)memset(blk, 0, sizeof(blk));
	sys_put_be16(1U, &blk[0]);
	blk[2] = hdr & CCM_HDR_MASK;
	block_xor(x, blk, 16U);
	ecb_encrypt_be(ccm->key, x, x);

	for (uint16_t off = 0U; off < len; off += 16U) {
		block_xor(x, &payload[off], MIN(16U, len - off));
		ecb_encrypt_be(ccm->key, x, x);
	}

	/* A0 key stream encrypts the MIC */
	blk[0] = CCM_A_FLAGS;
	(void)memcpy(&blk[1], nonce, 13U);
	sys_put_be16(0U, &blk[14]);
	ecb_encrypt_be(ccm->key, blk, blk);
	for (uint8_t i = 0U; i < PDU_MIC_SIZE; i++) {
		mic[i] = x[i] ^ blk[i];
	}
}

void lll_ccm_encrypt(const struct ccm *ccm, const struct pdu_data *pdu, struct pdu_data *out)
{
	const uint8_t *in = (const uint8_t *)pdu;
	uint8_t *o = (uint8_t *)out;
	uint8_t len = in[1];
	uint8_t nonce[13];

	o[0] = in[0];
	o[1] = len;
	(void)memcpy(&o[2], &in[2], len);

	if (len == 0U) {
		return;
	}

	ccm_nonce(ccm, nonce);
	ccm_mic(ccm, nonce, in[0], &in[2], len, &o[2U + len]);
	ccm_ctr(ccm, nonce, &o[2], len);
	o[1] = len + PDU_MIC_SIZE;
}

bool lll_ccm_decrypt(const struct ccm *ccm, const struct pdu_data *pdu, struct pdu_data *out)
{
	const uint8_t *in = (const uint8_t *)pdu;
	uint8_t *o = (uint8_t *)out;
	uint8_t mic[PDU_MIC_SIZE];
	uint8_t nonce[13];
	uint8_t len;

	o[0] = in[0];

	if (in[1] == 0U) {
		o[1] = 0U;

		return true;
	}

	if (in[1] <= PDU_MIC_SIZE) {
		return false;
	}

	len = in[1] - PDU_MIC_SIZE;
	o[1] = len;
	(void)memcpy(&o[2], &in[2], len);

	ccm_nonce(ccm, nonce);
	ccm_ctr(ccm, nonce, &o[2], len);
	ccm_mic(ccm, nonce, in[0], &o[2], len, mic);

	return memcmp(mic, &in[2U + len], PDU_MIC_SIZE) == 0;
}
