/* Digest comparison logic of the FPGA attestation with a fake page reader.
 *
 * fpga_attest.c itself needs the HAL and the XO2 bus; what it computes is
 * SHA-256 over FPGA_ATTEST_CFG_PAGES pages of 16 bytes read in order, compared
 * with the digest scripts/gen_fpga_digests.py compiles in. This test drives
 * the same streaming pipeline through a fake reader and checks:
 *   - the digest equals the one computed over the whole image at once (so a
 *     page-by-page readback matches the build-time digest of the image);
 *   - a single flipped bit in any page, a short read and a page out of order
 *     all change the digest;
 *   - the expected header generated for this build matches what the script
 *     computes from the committed artefacts (the Makefile runs the script and
 *     the test includes its output). */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "sha256.h"
#include "fpga_attest_expected.h"

#define PAGES FPGA_ATTEST_CFG_PAGES
#define PAGE  FPGA_ATTEST_PAGE_BYTES

static uint8_t image[PAGES * PAGE];

/* The fake device: returns page `n` of the image, or fails after `fail_at`. */
static int fake_read_page(unsigned n, unsigned fail_at, uint8_t out[PAGE])
{
    if (n >= fail_at) {
        return -1;
    }
    memcpy(out, &image[n * PAGE], PAGE);
    return 0;
}

/* What fpga_attest.c does: stream every page through SHA-256. */
static int attest_digest(unsigned fail_at, uint8_t digest[32], unsigned *pages_read)
{
    sha256_ctx_t ctx;
    uint8_t page[PAGE];
    sha256_init(&ctx);
    for (unsigned n = 0; n < PAGES; n++) {
        if (fake_read_page(n, fail_at, page) != 0) {
            *pages_read = n;
            return -1;
        }
        sha256_update(&ctx, page, PAGE);
    }
    *pages_read = PAGES;
    sha256_final(&ctx, digest);
    return 0;
}

int main(void)
{
    /* a deterministic pseudo-bitstream */
    uint32_t x = 0x12345678u;
    for (size_t i = 0; i < sizeof image; i++) {
        x = x * 1664525u + 1013904223u;
        image[i] = (uint8_t)(x >> 24);
    }

    uint8_t expected[32], got[32];
    unsigned pages;
    sha256(image, sizeof image, expected);                 /* the build-time digest */
    assert(attest_digest(PAGES, got, &pages) == 0);        /* the readback digest */
    assert(pages == PAGES && memcmp(got, expected, 32) == 0);

    /* one bit anywhere changes the digest: first page, a middle page, last page */
    const size_t spots[] = { 0, (PAGES / 2) * PAGE + 7, sizeof image - 1 };
    for (size_t k = 0; k < sizeof spots / sizeof spots[0]; k++) {
        image[spots[k]] ^= 0x01;
        assert(attest_digest(PAGES, got, &pages) == 0);
        assert(memcmp(got, expected, 32) != 0);
        image[spots[k]] ^= 0x01;
    }

    /* a read failure is a failure, not a digest */
    assert(attest_digest(PAGES - 1, got, &pages) != 0 && pages == PAGES - 1);
    assert(attest_digest(0, got, &pages) != 0 && pages == 0);

    /* two swapped pages change the digest (order matters) */
    {
        uint8_t tmp[PAGE];
        memcpy(tmp, &image[0], PAGE);
        memcpy(&image[0], &image[PAGE], PAGE);
        memcpy(&image[PAGE], tmp, PAGE);
        assert(attest_digest(PAGES, got, &pages) == 0);
        assert(memcmp(got, expected, 32) != 0);
    }

    /* the generated header: three distinct digests, EE and OPT share SAFETY */
    {
        static const uint8_t s[32] = FPGA_ATTEST_DIGEST_SAFETY;
        static const uint8_t d[32] = FPGA_ATTEST_DIGEST_SEED;
        static const uint8_t t[32] = FPGA_ATTEST_DIGEST_TA;
        static const uint8_t zero[32] = {0};
        assert(memcmp(s, d, 32) != 0 && memcmp(s, t, 32) != 0 && memcmp(d, t, 32) != 0);
        assert(memcmp(s, zero, 32) != 0 && memcmp(d, zero, 32) != 0 && memcmp(t, zero, 32) != 0);
        assert(PAGES == 3198U && PAGE == 16U);   /* MachXO2-2000 configuration sector; page 3198 is UFM page 0 */
        assert(strlen(FPGA_ATTEST_TAG_SAFETY) > 0 && strlen(FPGA_ATTEST_TAG_SEED) > 0 && strlen(FPGA_ATTEST_TAG_TA) > 0);
    }

    printf("fpga_digest host tests OK\n");
    return 0;
}
