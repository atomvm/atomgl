<!-- SPDX-License-Identifier: Apache-2.0 -->
<!-- SPDX-FileCopyrightText: 2026 Peter M <petermm@gmail.com> -->

# E-paper Low Power Spec Notes

Internal working notes for mapping panel data sheets and vendor drivers onto
AtomGL `standby`, `sleep`, `deep_sleep`, and `wake` behavior.

## Summary Table

| AtomGL panel key | Panel / controller | Source | Power-off / standby | Sleep | Deep sleep | Wake requirement | Partial after wake |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `waveshare,epd2in9_V2` | 2.9 inch SSD1680 | SSD1680 command table / Waveshare command use | Not exposed separately | `0x10,0x01` | `0x10,0x03` | Reset + init | Fast/partial allowed only when the bytecode reseeds controller previous RAM |
| `dke,depg0290bns800f6` | 2.9 inch SSD1680 | DEPG0290BNS800F6 controller sheet | Not exposed separately | `0x10,0x01` | `0x10,0x03` | Reset + init | No differential mode in AtomGL descriptor |
| `waveshare,epd2in13_V4` | 2.13 inch SSD1680-style | Waveshare command use | Not exposed separately | `0x10,0x01` | `0x10,0x03` | Reset + init | Fast/partial allowed only when the bytecode reseeds controller previous RAM |
| `waveshare,epd2in9`, `waveshare,epd2in13` | UC8151-era mono panels | Existing AtomGL command table | Not exposed separately | `0x10,0x01` | Not exposed by default | Reset + init | Partial promoted unless the selected program reseeds previous RAM |
| `waveshare,epd4in2_V2` | UC8276 4.2 inch | Existing AtomGL command table | Not exposed separately | `0x10,0x01` | Not exposed by default | Reset + init | Fast/partial allowed only when the bytecode reseeds controller previous RAM |
| `heltec,lcmen2r13efc1` | JD79656 2.13 inch | Existing AtomGL command table | `0x02`, wait BUSY high | Not exposed by default | Not exposed by default | Reset + init | No differential mode in AtomGL descriptor |
| `waveshare,5in65-acep-7c` | 5.65 inch ACeP 7-color | Waveshare command use / existing AtomGL refresh flow | `0x02`, wait BUSY low | Not exposed by default | `0x02`, wait, `0x07,0xA5` | Hardware reset + init | No partial mode in AtomGL descriptor |
| `good-display/gdep073e01` | 7.3 inch Spectra 6, 800x480, GDEP073E01-style | `Waveshare_7.3inch-e-Paper-(E)-user-manual.pdf` | `0x02,0x00`; BUSY low while active, high when idle | Not documented | `0x02,0x00`, wait, `0x07,0xA5` | Hardware reset + init | No partial mode in AtomGL descriptor |

## `good-display/gdep073e01`

Local spec sheet:

- `/Users/petermm/Downloads/Waveshare_7.3inch-e-Paper-(E)-user-manual.pdf`

Extracted from the command table:

| Function | Command | Data | Notes |
| --- | --- | --- | --- |
| Power OFF | `0x02` | `0x00` | Driver enters power-off/standby state. |
| Power ON | `0x04` | none | Used before refresh. |
| Deep Sleep | `0x07` | `0xA5` | Check code; exiting deep sleep requires HWRESET. |
| Data Start Transmission | `0x10` | frame bytes | Writes display RAM. |
| Data Refresh | `0x12` | `0x01` in spec | Local Waveshare sample code sends `0x00`; AtomGL currently follows sample code. |

Power/current table notes:

- The sheet includes rows for module standby current and module deep sleep current,
  but the extracted table does not provide numeric values.
- The sheet advertises low-current deep sleep and on-chip display RAM, but does not
  describe a separate RAM-retaining sleep mode analogous to SSD1680 `0x10,0x01`.

Cross-checks:

- Waveshare Arduino/STM32/C drivers for `EPD_7IN3E_Sleep()` send:
  `0x02, 0x00`, wait busy, then `0x07, 0xA5`.
- Waveshare Python `epd7in3e.py` sleep sends only `0x07, 0xA5`.
- `bb_epaper` sends data refresh as `0x12, 0x00` for UC81xx 4-gray,
  4-color, and 7-color panels, including Spectra 6.
- `bb_epaper` currently treats `BBEP_7COLOR` sleep as power-off only and does not
  send the `0x07,0xA5` deep-sleep command in its 7-color path.
- AtomGL currently does power-off after each ACEP7/Spectra 6 refresh:
  `0x04`, `0x12`, optional refresh data, `0x02`.

Implementation implication:

- For this panel family, `standby` means the already-supported power-off state
  (`0x02,0x00`) when exposed as an explicit command.
- `deep_sleep` should mean `0x02,0x00`, wait for idle, then `0x07,0xA5`.
- `wake` from deep sleep must use hardware reset and full init.
- There is no basis in this sheet for preserving controller RAM across
  `deep_sleep`.
