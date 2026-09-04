### Configuration

- **Type:** 18650 Lithium‑ion cells
- **Arrangement:** 3S1P (3 cells in series, 1 parallel string)
- **Nominal Voltage per Cell:** 3.7 V
- **Capacity per Cell:** ~3000 mAh (3.0 Ah)

### Pack Specifications

- **Nominal Voltage:**
		Vtotal=3×3.7 V=11.1 V

- **Maximum Voltage (Fully Charged):**
		Vmax=3×4.2 V=12.6 V

- **Total Capacity:**
		Ctotal=3.0 Ah

- **Energy Stored:**
		E=11.1 V×3.0 Ah≈33.3 Wh

### Current Output

- **Continuous Current:** Depends on the discharge rating of the cells.
    - Standard 18650 cells: ~5–10 A continuous.
    - High‑drain 18650 cells (used in power tools, e‑bikes, robotics): 15–20 A continuous, sometimes higher.
- **Peak Current:** Short bursts can exceed 20 A depending on cell quality.
- **Pack Limitation:** Since this is a 3S1P pack, the current capability is limited to what a single cell can provide (no parallel strings to share load).

### Practical Notes

- Runtime per pack: ~45 minutes under typical robot load.
- Multiple packs are rotated to ensure a **fully charged set** is always available.
- Inline **5 A fuse** in combination with the **Buck Converter** is used for protection of the board, but the cells themselves can deliver much higher current.