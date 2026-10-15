#include "json_strict.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* One unquoted value js[start..end): true, false, null or a finite number. */
static bool primitive_is_strict(const char *js, int start, int end)
{
    int len = end - start;
    char buf[32];

    if (len <= 0 || len >= (int)sizeof(buf)) {
        return false;
    }
    memcpy(buf, js + start, (size_t)len);
    buf[len] = '\0';

    if (strcmp(buf, "true") == 0 || strcmp(buf, "false") == 0 || strcmp(buf, "null") == 0) {
        return true;
    }
    /* JSON numbers start with '-' or a digit; strtod alone would also take
     * "inf", "nan" and hex. */
    if (!(buf[0] == '-' || (buf[0] >= '0' && buf[0] <= '9'))) {
        return false;
    }
    for (int i = 0; i < len; i++) {
        char c = buf[i];
        if (!((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E')) {
            return false;
        }
    }
    char *endp = NULL;
    double v = strtod(buf, &endp);
    return endp == buf + len && isfinite(v);
}

bool json_object_is_strict(const char *js, size_t len, jsmntok_t *tokens, unsigned int n)
{
    jsmn_parser parser;

    if (js == NULL || tokens == NULL || n == 0U) {
        return false;
    }
    jsmn_init(&parser, NULL);
    int r = jsmn_parse(&parser, js, len, tokens, n, NULL);
    if (r < 1 || tokens[0].type != JSMN_OBJECT) {
        return false;
    }
    for (int i = 1; i < r; i++) {
        if (tokens[i].type == JSMN_PRIMITIVE && !primitive_is_strict(js, tokens[i].start, tokens[i].end)) {
            return false;
        }
    }
    return true;
}
