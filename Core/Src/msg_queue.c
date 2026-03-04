/*
 * msg_queue.c
 *
 * Simple IRQ-safe fixed-size JSON message queue implementation.
 */

#include "msg_queue.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h" // for __disable_irq/__enable_irq if needed

/* Internal storage */
static char mq_buf[MQ_MAX_MESSAGES][MQ_MAX_MSG_SIZE];
static uint16_t mq_len[MQ_MAX_MESSAGES];
static uint8_t mq_head = 0; // index for pop (oldest)
static uint8_t mq_tail = 0; // index for push (next free)
static uint8_t mq_count_var = 0;

void mq_init(void)
{
    __disable_irq();
    memset(mq_buf, 0, sizeof(mq_buf));
    memset(mq_len, 0, sizeof(mq_len));
    mq_head = 0;
    mq_tail = 0;
    mq_count_var = 0;
    __enable_irq();
}

bool mq_is_full(void)
{
    return (mq_count_var >= MQ_MAX_MESSAGES);
}

bool mq_is_empty(void)
{
    return (mq_count_var == 0);
}

size_t mq_count(void)
{
    return (size_t)mq_count_var;
}

bool mq_push(const char *msg, size_t len)
{
    if (msg == NULL) {
        return false;
    }
    if (len == 0 || len >= MQ_MAX_MSG_SIZE) {
        return false;
    }

    __disable_irq();
    if (mq_count_var >= MQ_MAX_MESSAGES) {
        __enable_irq();
        return false;
    }

    // Copy into tail slot, ensure NUL termination
    memcpy(mq_buf[mq_tail], msg, len);
    mq_buf[mq_tail][len] = '\0';
    mq_len[mq_tail] = (uint16_t)len;

    mq_tail = (uint8_t)((mq_tail + 1U) % MQ_MAX_MESSAGES);
    mq_count_var++;
    __enable_irq();
    return true;
}

static bool mq_peek_internal(char *out_buf, size_t bufsize, size_t *out_len)
{
    if (mq_count_var == 0) {
        return false;
    }

    uint8_t idx = mq_head;
    uint16_t len = mq_len[idx];
    if (bufsize < (size_t)(len + 1)) { // +1 for NUL
        return false;
    }

    memcpy(out_buf, mq_buf[idx], (size_t)len);
    out_buf[len] = '\0';
    if (out_len) *out_len = len;
    return true;
}

bool mq_peek(char *out_buf, size_t bufsize, size_t *out_len)
{
    if (out_buf == NULL) return false;
    __disable_irq();
    bool ok = mq_peek_internal(out_buf, bufsize, out_len);
    __enable_irq();
    return ok;
}

bool mq_pop(char *out_buf, size_t bufsize, size_t *out_len)
{
    if (out_buf == NULL) return false;
    __disable_irq();
    if (mq_count_var == 0) {
        __enable_irq();
        return false;
    }

    uint8_t idx = mq_head;
    uint16_t len = mq_len[idx];
    if (bufsize < (size_t)(len + 1)) {
        __enable_irq();
        return false;
    }

    memcpy(out_buf, mq_buf[idx], (size_t)len);
    out_buf[len] = '\0';
    if (out_len) *out_len = len;

    // clear slot (optional)
    mq_len[idx] = 0;
    mq_buf[idx][0] = '\0';

    mq_head = (uint8_t)((mq_head + 1U) % MQ_MAX_MESSAGES);
    mq_count_var--;
    __enable_irq();
    return true;
}
