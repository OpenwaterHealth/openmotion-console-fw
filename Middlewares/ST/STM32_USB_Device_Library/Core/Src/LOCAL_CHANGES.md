# Local changes to the STM32 USB Device Library (SOUP)

Upstream: STMicroelectronics `stm32_mw_usb_device`, Core files identified as v2.11.3
(Cybersecurity Vulnerability Assessment, 2026-09-28, §4.3 O2). Record every local
change here and in the SOUP register; do not patch silently.

| Date | File | Change | Reason |
|------|------|--------|--------|
| 2026-10-08 | `usbd_ctlreq.c` `USBD_StdEPReq` | Reject endpoint requests whose endpoint number (`wIndex & 0x7F`) is 16 or more before it is used to index `ep_in[16]` / `ep_out[16]`. Marked `LOCAL CHANGE`. | CVA O2 / R7. Without it, GET_STATUS in the configured state from a malicious host wrote a 16-bit status at `ep_in[ep_addr & 0x7F]`, up to index 127 of a 16-entry array, and read the reply back from there. Equivalent to the fix in upstream v2.11.6. |
| 2026-10-08 | `usbd_ctlreq.c` `USBD_GetString` | Bound the UTF-16 copy loop by the clamped `*len` (`USBD_MAX_STR_DESC_SIZ`). Marked `LOCAL CHANGE`. | CVA O2 / R7. The length was clamped but the loop copied the whole source string into `unicode[]`. Equivalent to the v2.11.6 bound. |

Verification: `host_tests` do not cover the USB stack; bench check is a USB control
request `GET_STATUS` with `wIndex = 0x81`..`0xFF` and `0x10`..`0x7F` (e.g. with pyusb
`ctrl_transfer(0x82, 0x00, 0, wIndex, 2)`): the device must STALL (pipe error on the
host) and keep enumerating; a `GET_DESCRIPTOR` for each string index must return at most
64 bytes.

Preferred long-term action: move all four firmware images to upstream v2.11.6 or later
in one change set (console firmware, sensor firmware `USB/Core`, both bootloaders), then
delete this file's entries that upstream covers.
