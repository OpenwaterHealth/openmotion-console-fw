/*
 * fpga_attest.c - FPGA configuration integrity check. See fpga_attest.h.
 *
 * Uses only the read-only entry points of the Lattice ECA API
 * (XO2ECAcmd_openCfgIF in TRANSPARENT_MODE, XO2ECAcmd_CfgResetAddr,
 * XO2ECAcmd_CfgReadPage, XO2ECAcmd_closeCfgIF, XO2ECAcmd_Bypass and
 * XO2ECA_apiReadBackUFM). The programming entry points are compiled out of
 * Release and Debug builds (OW_FACTORY_PROGRAMMING, Core/Src/LOCAL_CHANGES.md).
 */
#include "fpga_attest.h"
#include "fpga_attest_expected.h"

#include "main.h"
#include "XO2_cmds.h"
#include "XO2_api.h"
#include "msg_queue.h"
#include "sha256.h"
#include "trigger.h"

#include <stdio.h>
#include <string.h>

/* Only the two safety devices are attested (EE and OPT, both carrying the
 * safety design): they are the laser interlock and bound emission on their
 * own (ARCH-68). The seed and TA drivers are not checked, which keeps the
 * boot-time check to two devices; they cannot be reprogrammed from a Release
 * image either (no programming code). Their report entries say "not
 * checked" and carry no expected digest. Decision 2026-10-22 (bench review). */
#define FPGA_ATTEST_CHECKED(d) ((d) == (uint8_t)MUX_FPGA_SAFE_EE || (d) == (uint8_t)MUX_FPGA_SAFE_OPT)
#define FPGA_ATTEST_FIRST      ((uint8_t)MUX_FPGA_SAFE_EE)
#define FPGA_ATTEST_CHECKED_COUNT 2U
static const uint8_t s_expected_safety[SHA256_DIGEST_SIZE] = FPGA_ATTEST_DIGEST_SAFETY;
static const char *const s_dev_name[FPGA_ATTEST_DEV_COUNT] = { "seed", "TA", "EE", "OPT" };

typedef enum { STEP_OPEN = 0, STEP_PAGES, STEP_FINISH, STEP_UFM, STEP_NEXT } step_t;

static XO2Handle_t *s_h;
static fpga_attest_state_t s_state = FPGA_ATTEST_IDLE;
static fpga_attest_report_t s_report;
static step_t   s_step;
static uint8_t  s_dev;
static uint16_t s_page;
static uint32_t s_dev_start_ms;
static sha256_ctx_t s_sha;
static bool     s_passed_once;
static bool     s_reapply;
static safety_ufm_limits_t s_ufm[FPGA_ATTEST_DEV_COUNT];
static bool     s_ufm_valid[FPGA_ATTEST_DEV_COUNT];
static bool     s_prev_ufm_valid[FPGA_ATTEST_DEV_COUNT];
static safety_ufm_limits_t s_prev_ufm[FPGA_ATTEST_DEV_COUNT];

static void push_msg(const char *fmt, const char *a, unsigned b)
{
    char m[MQ_MAX_MSG_SIZE];
    int n = snprintf(m, sizeof m, fmt, a, b);
    if (n > 0 && (size_t)n < sizeof m) {
        (void)mq_push(m, (size_t)n);
    }
}

static void select_device(uint8_t dev)
{
    s_h->pI2CDrvrCalls->mux_idx = 0U;
    s_h->pI2CDrvrCalls->mux_ch = dev;
}

static void close_interface(void)
{
    (void)XO2ECAcmd_closeCfgIF(s_h);
    (void)XO2ECAcmd_Bypass(s_h);
}

static void dev_fail(uint8_t flag)
{
    s_report.dev[s_dev].status = (uint8_t)((s_report.dev[s_dev].status & ~FPGA_ATTEST_ST_PENDING) | flag);
}

void fpga_attest_init(XO2Handle_t *h)
{
    s_h = h;
    s_state = FPGA_ATTEST_IDLE;
    memset(&s_report, 0, sizeof s_report);
    s_report.struct_version = 1U;
    s_report.cfg_pages = (uint16_t)FPGA_ATTEST_CFG_PAGES;
    s_report.interlocked = 1U;
    for (uint8_t d = 0; d < FPGA_ATTEST_DEV_COUNT; d++) {
        s_report.dev[d].device = d;
        s_report.dev[d].status = FPGA_ATTEST_CHECKED(d) ? FPGA_ATTEST_ST_PENDING : FPGA_ATTEST_ST_NOT_CHECKED;
        s_report.dev[d].ufm_status = 0xFFU;
        if (FPGA_ATTEST_CHECKED(d)) {
            memcpy(s_report.dev[d].expected, s_expected_safety, SHA256_DIGEST_SIZE);
        }
    }
}

bool fpga_attest_start(void)
{
    if (s_h == NULL || s_state == FPGA_ATTEST_RUNNING || Trigger_IsRunning()) {
        return false;
    }
    for (uint8_t d = 0; d < FPGA_ATTEST_DEV_COUNT; d++) {
        s_report.dev[d].status = FPGA_ATTEST_CHECKED(d) ? FPGA_ATTEST_ST_PENDING : FPGA_ATTEST_ST_NOT_CHECKED;
        s_report.dev[d].ufm_status = 0xFFU;
        s_report.dev[d].pages_read = 0U;
        s_report.dev[d].elapsed_ms = 0U;
        memset(s_report.dev[d].digest, 0, SHA256_DIGEST_SIZE);
        s_ufm_valid[d] = false;
    }
    s_report.progress_pct = 0U;
    s_state = FPGA_ATTEST_RUNNING;
    s_report.state = (uint8_t)s_state;
    s_report.interlocked = 1U;
    s_dev = FPGA_ATTEST_FIRST;
    s_step = STEP_OPEN;
    printf("FPGA attestation: started\r\n");
    return true;
}

/* Open the configuration interface of the current device, transparent mode.
 * The device must answer its ID read and must not report BUSY first: the
 * vendored XO2ECAcmd_openCfgIF() waits for BUSY to clear with a long poll
 * loop that an absent or wedged device would stretch past the 250 ms
 * watchdog window. */
static void step_open(void)
{
    unsigned int devid = 0U, sr = 0U;

    select_device(s_dev);
    s_dev_start_ms = HAL_GetTick();
    s_page = 0U;
    if (XO2ECAcmd_readDevID(s_h, &devid) != OK ||
        XO2ECAcmd_readStatusReg(s_h, &sr) != OK || (sr & 0x00001000u) != 0U) {
        s_report.dev[s_dev].devid = devid;
        dev_fail(FPGA_ATTEST_ST_READ_FAIL);
        s_step = STEP_UFM;
        return;
    }
    s_report.dev[s_dev].devid = devid;
    if (XO2ECAcmd_openCfgIF(s_h, TRANSPARENT_MODE) != OK ||
        XO2ECAcmd_CfgResetAddr(s_h) != OK) {
        close_interface();
        dev_fail(FPGA_ATTEST_ST_READ_FAIL);
        s_step = STEP_UFM;
        return;
    }
    sha256_init(&s_sha);
    s_step = STEP_PAGES;
}

/* Read pages until the budget is spent or the sector is done. */
static void step_pages(uint32_t start, uint32_t budget_ms)
{
    uint8_t page[XO2_FLASH_PAGE_SIZE];

    while (s_page < FPGA_ATTEST_CFG_PAGES) {
        if (XO2ECAcmd_CfgReadPage(s_h, page) != OK) {
            close_interface();
            dev_fail(FPGA_ATTEST_ST_READ_FAIL);
            s_report.dev[s_dev].pages_read = s_page;
            s_step = STEP_UFM;
            return;
        }
        sha256_update(&s_sha, page, sizeof page);
        s_page++;
        if ((HAL_GetTick() - start) >= budget_ms) {
            break;
        }
    }
    s_report.dev[s_dev].pages_read = s_page;
    if (s_page >= FPGA_ATTEST_CFG_PAGES) {
        s_step = STEP_FINISH;
    }
}

static void step_finish(void)
{
    close_interface();
    sha256_final(&s_sha, s_report.dev[s_dev].digest);
    uint32_t elapsed = HAL_GetTick() - s_dev_start_ms;
    s_report.dev[s_dev].elapsed_ms = (elapsed > 0xFFFFU) ? 0xFFFFU : (uint16_t)elapsed;
    if (memcmp(s_report.dev[s_dev].digest, s_expected_safety, SHA256_DIGEST_SIZE) != 0) {
        dev_fail(FPGA_ATTEST_ST_DIGEST);
    } else {
        s_report.dev[s_dev].status &= (uint8_t)~FPGA_ATTEST_ST_PENDING;
    }
    s_step = STEP_UFM;
}

/* Safety devices: read and validate the per-unit UFM record. A read failure
 * is a read fault; a blank UFM is reported but is not a fault. */
static void step_ufm(void)
{
    if (s_dev == (uint8_t)MUX_FPGA_SAFE_EE || s_dev == (uint8_t)MUX_FPGA_SAFE_OPT) {
        uint8_t raw[SAFETY_UFM_RECORD_PAGES * XO2_FLASH_PAGE_SIZE];
        select_device(s_dev);
        if (XO2ECA_apiReadBackUFM(s_h, SAFETY_UFM_RECORD_PAGE, SAFETY_UFM_RECORD_PAGES, raw) != OK) {
            close_interface();
            dev_fail(FPGA_ATTEST_ST_READ_FAIL);
        } else {
            safety_ufm_status_t st = safety_ufm_record_parse(raw, sizeof raw, s_dev, &s_ufm[s_dev]);
            s_report.dev[s_dev].ufm_status = (uint8_t)st;
            if (st == SAFETY_UFM_VALID) {
                s_ufm_valid[s_dev] = true;
                s_report.dev[s_dev].status |= FPGA_ATTEST_ST_UFM_VALID;
            } else if (st == SAFETY_UFM_BLANK) {
                s_report.dev[s_dev].status |= FPGA_ATTEST_ST_UFM_BLANK;
            } else {
                dev_fail(FPGA_ATTEST_ST_UFM_INVALID);
            }
        }
    }
    s_report.dev[s_dev].status &= (uint8_t)~FPGA_ATTEST_ST_PENDING;
    s_step = STEP_NEXT;
}

static bool ufm_changed(void)
{
    for (uint8_t d = 0; d < FPGA_ATTEST_DEV_COUNT; d++) {
        if (s_ufm_valid[d] != s_prev_ufm_valid[d]) {
            return true;
        }
        if (s_ufm_valid[d] && memcmp(&s_ufm[d], &s_prev_ufm[d], sizeof s_ufm[d]) != 0) {
            return true;
        }
    }
    return false;
}

static void run_finished(void)
{
    bool fault = false;
    for (uint8_t d = 0; d < FPGA_ATTEST_DEV_COUNT; d++) {
        if (!FPGA_ATTEST_CHECKED(d)) {
            continue;
        }
        const fpga_attest_dev_report_t *r = &s_report.dev[d];
        printf("FPGA attestation: %s %s (id 0x%08lX, %u pages, %u ms, UFM %s)\r\n", s_dev_name[d],
               (r->status & FPGA_ATTEST_ST_FAULT_MASK) ? "FAIL" : "ok",
               (unsigned long)r->devid, (unsigned)r->pages_read, (unsigned)r->elapsed_ms,
               r->ufm_status == 0xFFU ? "n/a" : safety_ufm_status_name((safety_ufm_status_t)r->ufm_status));
        if (r->status & FPGA_ATTEST_ST_FAULT_MASK) {
            fault = true;
            push_msg("{\"type\":\"system\",\"state\":\"error\",\"msg\":\"FPGA attestation failed\",\"device\":\"%s\",\"status\":%u}",
                     s_dev_name[d], (unsigned)r->status);
        }
    }
    s_report.run_count++;
    s_report.progress_pct = 100U;
    if (fault) {
        s_state = FPGA_ATTEST_FAIL;
        s_passed_once = false;
        s_report.interlocked = 1U;
        (void)Trigger_Stop();
        printf("ERROR: FPGA attestation FAILED; trigger refused\r\n");
    } else {
        s_state = FPGA_ATTEST_PASS;
        s_passed_once = true;
        s_report.interlocked = 0U;
        printf("FPGA attestation: PASS\r\n");
    }
    s_report.state = (uint8_t)s_state;
    /* Re-apply the safety loads when the per-unit limits are new or changed,
     * and always after the first completed run (the boot apply ran before
     * the UFM records were known). */
    if (ufm_changed() || s_report.run_count == 1U) {
        s_reapply = true;
        memcpy(s_prev_ufm, s_ufm, sizeof s_ufm);
        memcpy(s_prev_ufm_valid, s_ufm_valid, sizeof s_ufm_valid);
    }
}

void fpga_attest_service(uint32_t budget_ms)
{
    if (s_state != FPGA_ATTEST_RUNNING) {
        return;
    }
    uint32_t start = HAL_GetTick();
    switch (s_step) {
    case STEP_OPEN:   step_open(); break;
    case STEP_PAGES:  step_pages(start, budget_ms); break;
    case STEP_FINISH: step_finish(); break;
    case STEP_UFM:    step_ufm(); break;
    case STEP_NEXT:
    default:
        do {
            s_dev++;
        } while (s_dev < FPGA_ATTEST_DEV_COUNT && !FPGA_ATTEST_CHECKED(s_dev));
        if (s_dev >= FPGA_ATTEST_DEV_COUNT) {
            run_finished();
        } else {
            s_step = STEP_OPEN;
        }
        break;
    }
    if (s_state == FPGA_ATTEST_RUNNING) {
        uint32_t done = (uint32_t)(s_dev - FPGA_ATTEST_FIRST) * FPGA_ATTEST_CFG_PAGES + s_page;
        s_report.progress_pct = (uint8_t)((done * 100U) / (FPGA_ATTEST_CHECKED_COUNT * FPGA_ATTEST_CFG_PAGES));
    }
    watchdog_refresh();
}

fpga_attest_state_t fpga_attest_state(void)
{
    return s_state;
}

bool fpga_attest_running(void)
{
    return s_state == FPGA_ATTEST_RUNNING;
}

bool fpga_attest_interlocked(void)
{
    return !s_passed_once || s_state == FPGA_ATTEST_FAIL;
}

bool fpga_attest_failed(void)
{
    return s_state == FPGA_ATTEST_FAIL;
}

const fpga_attest_report_t *fpga_attest_report(void)
{
    s_report.state = (uint8_t)s_state;
    s_report.interlocked = fpga_attest_interlocked() ? 1U : 0U;
    return &s_report;
}

const safety_ufm_limits_t *fpga_attest_ufm_limits(uint8_t device)
{
    if (device >= FPGA_ATTEST_DEV_COUNT || s_state == FPGA_ATTEST_RUNNING || !s_prev_ufm_valid[device]) {
        return NULL;
    }
    return &s_prev_ufm[device];
}

bool fpga_attest_take_reapply_request(void)
{
    bool r = s_reapply;
    s_reapply = false;
    return r;
}
