/*
 * msg_queue.h
 *
 * Fixed-size JSON message queue
 */
#ifndef INC_MSG_QUEUE_H_
#define INC_MSG_QUEUE_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of messages the queue can hold
#define MQ_MAX_MESSAGES 64U

// Maximum size (in bytes) of each message (including terminating NUL)
#define MQ_MAX_MSG_SIZE 128U

// Initialize the message queue. Call once before use.
void mq_init(void);

// Push a JSON message into the queue.
// - msg: pointer to the message bytes (not required to be NUL-terminated)
// - len: number of bytes in msg (must be < MQ_MAX_MSG_SIZE)
// Returns true on success, false if queue full or message too large.
bool mq_push(const char *msg, size_t len);

// Pop (remove) the oldest message from the queue.
// - out_buf: buffer to receive the message. Must be at least MQ_MAX_MSG_SIZE.
// - bufsize: size of out_buf in bytes.
// - out_len: pointer that receives the number of bytes written (optional).
// Returns true on success, false if queue empty or bufsize too small.
bool mq_pop(char *out_buf, size_t bufsize, size_t *out_len);

// Peek at the oldest message without removing it.
// Same semantics as mq_pop but does not modify the queue. Returns false if empty.
bool mq_peek(char *out_buf, size_t bufsize, size_t *out_len);

// Return the number of messages currently queued.
size_t mq_count(void);

// Returns true if the queue is empty.
bool mq_is_empty(void);

// Returns true if the queue is full.
bool mq_is_full(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MSG_QUEUE_H_ */
