CC ?= gcc
CFLAGS ?= -Wall -Wextra -Wno-unused-parameter -O0 -g -I../Core/Inc
# jsmn is built in strict mode in the firmware (CMakeLists.txt); test it the same way.
CFLAGS += -DJSMN_STRICT

TESTS = test_pdc_buffer test_motion_cfg_tokens test_led_driver test_jsmn_strict

all: $(TESTS)

test_pdc_buffer: test_pdc_buffer.c ../Core/Src/pdc_buffer.c
	$(CC) $(CFLAGS) -o $@ $<

test_motion_cfg_tokens: test_motion_cfg_tokens.c ../Core/Src/jsmn.c
	$(CC) $(CFLAGS) -o $@ $<

test_led_driver: test_led_driver.c ../Core/Src/led_driver.c ../Core/Inc/led_driver.h
	$(CC) $(CFLAGS) -o $@ $<

test_jsmn_strict: test_jsmn_strict.c ../Core/Src/jsmn.c ../Core/Src/json_strict.c
	$(CC) $(CFLAGS) -o $@ $< -lm

run: $(TESTS)
	./test_pdc_buffer
	./test_motion_cfg_tokens
	./test_led_driver
	./test_jsmn_strict

clean:
	rm -f $(TESTS)
