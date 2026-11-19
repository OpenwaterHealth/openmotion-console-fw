#ifndef JSON_STRICT_H
#define JSON_STRICT_H

#include <stdbool.h>
#include <stddef.h>

#include "jsmn.h"

/*
 * Strict check of a JSON document before it is stored (CFW-15).
 *
 * jsmn in strict mode (JSMN_STRICT) rejects unquoted values whose first
 * character is not a digit, '-', 't', 'f' or 'n', but it accepts anything after
 * that first character: "-Infinity", "nan", "nope" or "1.2.3" all tokenize.
 * json_object_is_strict() parses js[0..len) with jsmn into tokens[0..n) and then
 * requires the root to be an object and every unquoted value to be exactly
 * true, false or null, or a finite number that strtod consumes completely.
 *
 * Returns true when the document passes. tokens is caller-provided scratch.
 */
bool json_object_is_strict(const char *js, size_t len, jsmntok_t *tokens, unsigned int n);

#endif /* JSON_STRICT_H */
