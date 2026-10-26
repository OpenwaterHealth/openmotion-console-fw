/*
 * sha256.h - SHA-256 (FIPS 180-4), software implementation.
 *
 * Used by fpga_attest.c to hash the FPGA configuration pages read back over
 * I2C. The STM32H743 HASH peripheral is not enabled in this CubeMX
 * configuration (HAL_HASH_MODULE_ENABLED is off and the HAL driver is not in
 * the build), so a small first-party implementation is used instead. It is
 * recorded in scripts/sbom-manifest.json and verified against the NIST
 * FIPS 180-4 example vectors in host_tests/test_sha256.c.
 *
 * Streaming interface: init, any number of updates, final. No dynamic memory,
 * no dependency on the HAL, so it also builds in the host tests.
 */
#ifndef INC_SHA256_H_
#define INC_SHA256_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHA256_DIGEST_SIZE 32U
#define SHA256_BLOCK_SIZE  64U

typedef struct {
    uint32_t state[8];
    uint64_t bitlen;
    uint8_t  buf[SHA256_BLOCK_SIZE];
    size_t   buflen;
} sha256_ctx_t;

void sha256_init(sha256_ctx_t *ctx);
void sha256_update(sha256_ctx_t *ctx, const uint8_t *data, size_t len);
void sha256_final(sha256_ctx_t *ctx, uint8_t out[SHA256_DIGEST_SIZE]);

/* One-shot convenience wrapper. */
void sha256(const uint8_t *data, size_t len, uint8_t out[SHA256_DIGEST_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* INC_SHA256_H_ */
