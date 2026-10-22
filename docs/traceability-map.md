Here’s a **concise traceability map (HL → SR → FR)** that you can directly load into a requirements tool or expand into a full RTM.

---

### Traceability Structure
- HL = High-Level Requirement  
- SR = System Requirement (matrix-ready)  
- FR = Baseline Functional Requirement (from your spec)

---

### 1. Architecture & System

- HL-AR-001 → SR-002 → FR-001, FR-007  
- HL-AR-002 → SR-003 → FR-001–FR-008  
- HL-AR-003 → SR-001 → FR-001  
- HL-AR-004 → SR-004 → FR-090  

---

### 2. Communication

- HL-COM-001 → SR-COM-001, SR-COM-002 → SI-007, SI-009  
- HL-COM-002 → SR-COM-003, SR-COM-004 → SI-001, SI-002, SI-003  
- HL-COM-003 → SR-COM-006 → SI-004  
- HL-COM-004 → SR-COM-005 → SI-005, SI-006  
- HL-COM-005 → SR-COM-007, SR-COM-008 → SI-008  

---

### 3. Command Processing

- HL-CTRL-001 → SR-CMD-001 → FR-020–FR-029, FR-040–FR-054, FR-070–FR-077  
- HL-CTRL-002 → SR-CMD-002 → FR-020–FR-024  
- HL-CTRL-003 → SR-CMD-003 → FR-026–FR-027  
- HL-CTRL-004 → SR-CMD-004 → FR-028–FR-029  
- HL-CTRL-005 → SR-CMD-005 → SI-006, ER-003  

---

### 4. Hardware Interfaces

- HL-HW-001 → SR-HW-001 → FR-040, FR-044, FR-045  
- HL-HW-002 → SR-HW-002, SR-HW-003 → FR-044, FR-045  
- HL-HW-003 → SR-HW-008, SR-HW-009 → FR-049, FR-052  
- HL-HW-004 → SR-HW-006, SR-HW-007 → FR-042, FR-043  
- HL-HW-005 → SR-HW-010 → FR-070–FR-077  
- HL-HW-006 → SR-HW-005 → FR-041  

---

### 5. FPGA Programming

- HL-CTRL-006 → SR-FPGA-001 → FR-070, FR-074  
- HL-CTRL-006 → SR-FPGA-002 → FR-071  
- HL-CTRL-006 → SR-FPGA-003 → FR-072, FR-073  
- HL-CTRL-006 → SR-FPGA-004 → FR-075  
- HL-CTRL-006 → SR-FPGA-005 → FR-076  
- HL-CTRL-006 → SR-FPGA-006 → FR-077  

---

### 5a. FPGA attestation and firmware-owned safety parameters (PTR-2026-1-5)

- HL-SAFE-006 → SR-ATTEST-001 → FR-140, FR-141
- HL-SAFE-006 → SR-ATTEST-002 → FR-142, FR-143, FR-145, FR-146
- HL-SAFE-006 → SR-ATTEST-003 → FR-144
- HL-SAFE-007 → SR-SAFEP-001 → FR-150, FR-151, FR-152
- HL-SAFE-007 → SR-SAFEP-002 → FR-153, FR-154, FR-155, FR-156

---

### 6. Telemetry

- HL-TLM-001 → SR-TLM-001 → FR-090, FR-091  
- HL-TLM-002 → SR-TLM-004, SR-TLM-005 → FR-092  
- HL-TLM-003 → SR-TLM-006 → FR-050, FR-054  
- HL-TLM-004 → SR-TLM-007 → FR-093  

---

### 7. Safety & Interlocks

- HL-SAFE-001 → SR-SAFE-001 → FR-094  
- HL-SAFE-002 → SR-SAFE-002 → FR-094  
- HL-SAFE-003 → SR-SAFE-003, SR-SAFE-004 → FR-113, FR-114  
- HL-SAFE-004 → SR-SAFE-005 → FR-094  
- HL-SAFE-005 → SR-SAFE-006 → FR-095  

---

### 8. Trigger Control

- HL-CTRL-005 → SR-TRIG-001 → FR-046  
- HL-CTRL-005 → SR-TRIG-002 → FR-110  
- HL-CTRL-005 → SR-TRIG-003 → FR-047  
- HL-CTRL-005 → SR-TRIG-004 → FR-112  
- HL-CTRL-005 → SR-TRIG-005 → FR-048  
- HL-CTRL-005 → SR-TRIG-006 → FR-111  

---

### 9. Configuration & Persistence

- HL-CFG-001 → SR-CFG-001 → FR-130  
- HL-CFG-002 → SR-CFG-002 → FR-130  
- HL-CFG-003 → SR-CFG-003 → FR-131  
- HL-CFG-004 → SR-CFG-004 → FR-132  
- HL-CFG-005 → SR-CFG-005, SR-CFG-006 → FR-133, FR-027  
- HL-CFG-006 → SR-CFG-008 → FR-135, FR-136  
- HL-CFG-007 → SR-CFG-007 → FR-134  

---

### 10. Error Handling

- HL-ERR-001 → SR-ERR-001, SR-ERR-002 → ER-001  
- HL-ERR-002 → SR-ERR-003 → ER-002  
- HL-ERR-003 → SR-ERR-002 → ER-003  
- HL-ERR-004 → SR-ERR-004, SR-ERR-005 → ER-004  

---

### 11. Performance

- HL-PERF-001 → SR-PERF-003 → PR-003  
- HL-PERF-002 → SR-PERF-001 → PR-001  
- HL-PERF-002 → SR-PERF-002 → PR-002  

---

### 12. Initialization

- HL-INIT-001 → SR-INIT-001 → FR-001  
- HL-INIT-002 → SR-INIT-004 → FR-008  
- HL-INIT-003 → SR-INIT-002 → SI-007  
- HL-INIT-004 → SR-INIT-003 → FR-004, FR-005  

---

### How to Use This

You can import this into a matrix tool with columns like:

- Requirement ID (HL / SR / FR)
- Parent Requirement ID
- Description
- Verification Method
- Status

---

If you want, I can convert this into a **ready-to-import CSV (Jama/DOORS format)** or add **test cases mapped to each SR**.