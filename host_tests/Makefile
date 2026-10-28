CC ?= gcc
CFLAGS ?= -Wall -Wextra -Wno-unused-parameter -O0 -g -I../Core/Inc -Igenerated
# jsmn is built in strict mode in the firmware (CMakeLists.txt); test it the same way.
CFLAGS += -DJSMN_STRICT
PYTHON ?= python3

TESTS = test_pdc_buffer test_motion_cfg_tokens test_led_driver test_jsmn_strict \
        test_sha256 test_safety_ufm_record test_motion_cfg_keys test_fpga_digest

all: $(TESTS)

test_pdc_buffer: test_pdc_buffer.c ../Core/Src/pdc_buffer.c
	$(CC) $(CFLAGS) -o $@ $<

test_motion_cfg_tokens: test_motion_cfg_tokens.c ../Core/Src/jsmn.c
	$(CC) $(CFLAGS) -o $@ $<

test_led_driver: test_led_driver.c ../Core/Src/led_driver.c ../Core/Inc/led_driver.h
	$(CC) $(CFLAGS) -o $@ $<

test_jsmn_strict: test_jsmn_strict.c ../Core/Src/jsmn.c ../Core/Src/json_strict.c
	$(CC) $(CFLAGS) -o $@ $< -lm

test_sha256: test_sha256.c ../Core/Src/sha256.c ../Core/Inc/sha256.h
	$(CC) $(CFLAGS) -o $@ $< ../Core/Src/sha256.c

test_safety_ufm_record: test_safety_ufm_record.c ../Core/Src/safety_ufm_record.c ../Core/Src/safety_envelope.c \
                        ../Core/Src/fpga_register_map.c ../Core/Src/motion_cfg_keys.c ../Core/Src/jsmn.c
	$(CC) $(CFLAGS) -o $@ $< ../Core/Src/safety_ufm_record.c ../Core/Src/safety_envelope.c \
	    ../Core/Src/fpga_register_map.c ../Core/Src/motion_cfg_keys.c ../Core/Src/jsmn.c -lm
	$(CC) $(CFLAGS) -DOW_FACTORY_PROGRAMMING=1 -o $@_factory $< ../Core/Src/safety_ufm_record.c ../Core/Src/safety_envelope.c \
	    ../Core/Src/fpga_register_map.c ../Core/Src/motion_cfg_keys.c ../Core/Src/jsmn.c -lm

# Built twice: with the Release policy (safety keys refused) and with the
# Factory policy (OW_FACTORY_PROGRAMMING: safety keys accepted within bounds).
test_motion_cfg_keys: test_motion_cfg_keys.c ../Core/Src/motion_cfg_keys.c ../Core/Src/fpga_register_map.c ../Core/Src/jsmn.c
	$(CC) $(CFLAGS) -o $@ $< ../Core/Src/motion_cfg_keys.c ../Core/Src/fpga_register_map.c ../Core/Src/jsmn.c -lm
	$(CC) $(CFLAGS) -DOW_FACTORY_PROGRAMMING=1 -o $@_factory $< ../Core/Src/motion_cfg_keys.c ../Core/Src/fpga_register_map.c ../Core/Src/jsmn.c -lm

# The expected-digest header is generated exactly as the firmware build does.
generated/fpga_attest_expected.h: ../scripts/gen_fpga_digests.py ../scripts/sbom-manifest.json
	mkdir -p generated
	$(PYTHON) -I ../scripts/gen_fpga_digests.py --manifest ../scripts/sbom-manifest.json --artefacts ../fpga-artefacts -o $@

test_fpga_digest: test_fpga_digest.c ../Core/Src/sha256.c generated/fpga_attest_expected.h
	$(CC) $(CFLAGS) -o $@ $< ../Core/Src/sha256.c

run: $(TESTS)
	./test_pdc_buffer
	./test_motion_cfg_tokens
	./test_led_driver
	./test_jsmn_strict
	./test_sha256
	./test_safety_ufm_record
	./test_safety_ufm_record_factory
	./test_motion_cfg_keys
	./test_motion_cfg_keys_factory
	./test_fpga_digest

clean:
	rm -f $(TESTS) test_motion_cfg_keys_factory test_safety_ufm_record_factory
	rm -rf generated
