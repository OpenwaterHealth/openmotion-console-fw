/*
 * sha256.c - SHA-256 (FIPS 180-4), software implementation. See sha256.h.
 *
 * Written for the console firmware from the FIPS 180-4 specification
 * (section 6.2). Byte order and padding follow the standard; the message
 * length is tracked in bits in a 64-bit counter.
 */
#include "sha256.h"

#include <string.h>

static const uint32_t K[64] = {
    0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
    0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
    0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
    0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
    0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
    0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
    0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
    0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u
};

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32U - (n))))
#define CH(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define BSIG0(x) (ROTR(x, 2) ^ ROTR(x, 13) ^ ROTR(x, 22))
#define BSIG1(x) (ROTR(x, 6) ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SSIG0(x) (ROTR(x, 7) ^ ROTR(x, 18) ^ ((x) >> 3))
#define SSIG1(x) (ROTR(x, 17) ^ ROTR(x, 19) ^ ((x) >> 10))

static void sha256_block(uint32_t state[8], const uint8_t block[SHA256_BLOCK_SIZE])
{
    uint32_t w[64];
    uint32_t a, b, c, d, e, f, g, h;
    unsigned int t;

    for (t = 0; t < 16U; t++) {
        w[t] = ((uint32_t)block[4U * t] << 24) | ((uint32_t)block[4U * t + 1U] << 16) |
               ((uint32_t)block[4U * t + 2U] << 8) | (uint32_t)block[4U * t + 3U];
    }
    for (t = 16U; t < 64U; t++) {
        w[t] = SSIG1(w[t - 2U]) + w[t - 7U] + SSIG0(w[t - 15U]) + w[t - 16U];
    }

    a = state[0]; b = state[1]; c = state[2]; d = state[3];
    e = state[4]; f = state[5]; g = state[6]; h = state[7];

    for (t = 0; t < 64U; t++) {
        uint32_t t1 = h + BSIG1(e) + CH(e, f, g) + K[t] + w[t];
        uint32_t t2 = BSIG0(a) + MAJ(a, b, c);
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }

    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
}

void sha256_init(sha256_ctx_t *ctx)
{
    ctx->state[0] = 0x6a09e667u; ctx->state[1] = 0xbb67ae85u;
    ctx->state[2] = 0x3c6ef372u; ctx->state[3] = 0xa54ff53au;
    ctx->state[4] = 0x510e527fu; ctx->state[5] = 0x9b05688cu;
    ctx->state[6] = 0x1f83d9abu; ctx->state[7] = 0x5be0cd19u;
    ctx->bitlen = 0U;
    ctx->buflen = 0U;
}

void sha256_update(sha256_ctx_t *ctx, const uint8_t *data, size_t len)
{
    while (len > 0U) {
        size_t take = SHA256_BLOCK_SIZE - ctx->buflen;
        if (take > len) {
            take = len;
        }
        memcpy(&ctx->buf[ctx->buflen], data, take);
        ctx->buflen += take;
        data += take;
        len -= take;
        ctx->bitlen += (uint64_t)take * 8U;
        if (ctx->buflen == SHA256_BLOCK_SIZE) {
            sha256_block(ctx->state, ctx->buf);
            ctx->buflen = 0U;
        }
    }
}

void sha256_final(sha256_ctx_t *ctx, uint8_t out[SHA256_DIGEST_SIZE])
{
    uint64_t bitlen = ctx->bitlen;
    unsigned int i;

    /* Padding: 0x80, zeros to 56 mod 64, then the 64-bit big-endian length. */
    ctx->buf[ctx->buflen++] = 0x80u;
    if (ctx->buflen > 56U) {
        memset(&ctx->buf[ctx->buflen], 0, SHA256_BLOCK_SIZE - ctx->buflen);
        sha256_block(ctx->state, ctx->buf);
        ctx->buflen = 0U;
    }
    memset(&ctx->buf[ctx->buflen], 0, 56U - ctx->buflen);
    for (i = 0; i < 8U; i++) {
        ctx->buf[56U + i] = (uint8_t)(bitlen >> (56U - 8U * i));
    }
    sha256_block(ctx->state, ctx->buf);

    for (i = 0; i < 8U; i++) {
        out[4U * i]      = (uint8_t)(ctx->state[i] >> 24);
        out[4U * i + 1U] = (uint8_t)(ctx->state[i] >> 16);
        out[4U * i + 2U] = (uint8_t)(ctx->state[i] >> 8);
        out[4U * i + 3U] = (uint8_t)(ctx->state[i]);
    }
    memset(ctx, 0, sizeof(*ctx));
}

void sha256(const uint8_t *data, size_t len, uint8_t out[SHA256_DIGEST_SIZE])
{
    sha256_ctx_t ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, data, len);
    sha256_final(&ctx, out);
}
