# Hardware design

The recovered Altium sources are tracked under [`hardware/altium/`](../hardware/altium/):

- `Lab_8_v3.PrjPcb` — Altium project
- `Lab_8_v3.SchDoc` — schematic source
- `Lab_8_v3.PcbDoc` — PCB layout source

![Rendered custom motor-control PCB](assets/motor-controller-pcb-render.jpg)

Open the `.PrjPcb` file in Altium Designer to regenerate manufacturing outputs and documentation. The archive did not include a BOM, Gerbers, pick-and-place output, 3D model, or rendered schematic. Do not infer those artifacts from this repository; regenerate and review them from the native project before fabrication.

## Documented design details

The final report provides the following implementation record for the recovered PCB:

| Area | Documented implementation |
| --- | --- |
| Motor drive | Two isolated paths: VNH5019ATR-E H-bridge with current sense, and IRF3205PBF low-side MOSFET driven by MAX4427CPA+ |
| Current feedback | H-bridge CS output filtered by a 10 kOhm / 33 nF RC network and routed to OpenMV |
| Logic power | LM2940CT-5.0 5 V rail for OpenMV and HC-06, with 0.47 uF and 22 uF support capacitors |
| Servo power | LM1086CT-ADJ configured for a 6 V rail, with R9 = 470 Ohm, R10 = 121 Ohm, and 10 uF capacitors |
| Ground strategy | GNDA, GND_SERVO, and GND_LOGIC pours joined at one star point near the battery connector |
| Layout and fabrication | Two layers, board size below 100 mm x 100 mm, 10 A high-current trace sizing, DRC reported as passed before Gerber export |

The report identifies JLCPCB as the fabrication service. It also records that the VNH5019 was assembled by pick-and-place and the remaining components were hand-soldered. Treat these details as a design record, not as current fabrication instructions: regenerate all manufacturing outputs from the native Altium project and complete an electrical review before building another board.

## Recovered interface map

The pin mapping below is taken from the latest archived firmware and is consolidated in `firmware/openmv/config.py`.

| Function | OpenMV pin | Notes |
| --- | --- | --- |
| Servo PWM | P2 | 100 Hz, calibrated around 1.5 ms |
| Motor PWM | P8 | 1.6 kHz |
| Motor direction A | P10 | Driven high in the final forward-only configuration |
| Motor direction B | P9 | Driven low in the final forward-only configuration |
| IR speed input | P5 | Falling-edge interrupt with pull-up |
| IR sensor ground | P6 | Driven low |

## Publication checklist

Before publishing fabrication-ready hardware, export and review:

1. PDF schematic and high-resolution PCB renders.
2. BOM with manufacturer part numbers and quantity.
3. Gerber, drill, board-outline, and pick-and-place files.
4. Power-tree and motor-current calculations.
5. Revision, date, and electrical-rule-check status.
