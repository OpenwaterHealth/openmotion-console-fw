/* SHA-256 (Core/Src/sha256.c) against the FIPS 180-4 / NIST CAVP example
 * vectors, including the streaming interface across block boundaries. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "sha256.h"

static void hex(const uint8_t *d, char *out)
{
    for (int i = 0; i < 32; i++) {
        sprintf(out + 2 * i, "%02x", d[i]);
    }
    out[64] = '\0';
}

static void check(const char *msg, size_t len, const char *expect)
{
    uint8_t d[32];
    char h[65];
    sha256((const uint8_t *)msg, len, d);
    hex(d, h);
    if (strcmp(h, expect) != 0) {
        printf("FAIL len=%zu\n got %s\n exp %s\n", len, h, expect);
        assert(0);
    }
}

int main(void)
{
    /* FIPS 180-4 examples */
    check("", 0, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    check("abc", 3, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    check("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq", 56,
          "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    check("abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmnhijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu", 112,
          "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1");

    /* one million 'a', streamed in odd chunk sizes */
    {
        static const char expect[] = "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0";
        sha256_ctx_t ctx;
        uint8_t d[32];
        char h[65];
        uint8_t chunk[97];
        memset(chunk, 'a', sizeof chunk);
        sha256_init(&ctx);
        size_t left = 1000000U;
        while (left > 0U) {
            size_t take = left < sizeof chunk ? left : sizeof chunk;
            sha256_update(&ctx, chunk, take);
            left -= take;
        }
        sha256_final(&ctx, d);
        hex(d, h);
        assert(strcmp(h, expect) == 0);
    }

    /* streaming equals one-shot for every split of a 200-byte message */
    {
        uint8_t msg[200];
        for (int i = 0; i < 200; i++) {
            msg[i] = (uint8_t)(i * 7 + 3);
        }
        uint8_t ref[32], d[32];
        sha256(msg, sizeof msg, ref);
        for (size_t split = 0; split <= sizeof msg; split++) {
            sha256_ctx_t ctx;
            sha256_init(&ctx);
            sha256_update(&ctx, msg, split);
            sha256_update(&ctx, msg + split, sizeof msg - split);
            sha256_final(&ctx, d);
            assert(memcmp(ref, d, 32) == 0);
        }
    }

    printf("sha256 host tests OK\n");
    return 0;
}
