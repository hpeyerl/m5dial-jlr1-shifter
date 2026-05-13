# JLR G1 CAN Shifter — M5Stack Dial

A clean, minimal firmware that turns your M5Stack Dial into a CAN bus gear
selector that talks to the Zombieverter VCU using the **JLR Gen1 shifter
protocol** (same as `JLR_G1.cpp` in the Stm32-vcu project).

---

## Hardware Required

| Item | Notes |
|------|-------|
| M5Stack Dial | ESP32-S3 based |
| M5Stack CAN Unit (or SN65HVD230 breakout) | Connects via Grove port |
| CAN bus wiring to Zombieverter | CAN-H / CAN-L + common GND |
| 120 Ω termination resistor | One at each end of the CAN bus |

### Wiring

```
M5Dial Grove Port (4-pin)
  Pin 1  Yellow  GPIO2  → CAN Unit TX
  Pin 2  White   GPIO1  → CAN Unit RX
  Pin 3  Red     5V     → CAN Unit VCC
  Pin 4  Black   GND    → CAN Unit GND

CAN Unit
  CAN H  ────────────── Zombieverter CAN H
  CAN L  ────────────── Zombieverter CAN L
  GND    ────────────── Common GND
```

## Zombieverter configuration

Set GearLvr to 2 (JLR-G1) and set VehicleCan to whichever CAN the M5 is connected to.
---

## CAN Protocol

### Sending (M5Dial → Zombieverter)

| Field       | Value |
|-------------|-------|
| CAN ID      | `0x312` |
| DLC         | 8 bytes |
| Rate        | Every 20 ms |
| Gear in     | Byte encoding per JLR G1 spec |

Gear byte map (matches `JLR_G1.cpp`):

| Gear    | bytes[1] | bytes[6] |
|---------|----------|----------|
| Park    | `0x66`   | `0x00`   |
| Reverse | `0x24`   | `0x01`   |
| Neutral | `0x25`   | `0x02`   |
| Drive   | `0x24`   | `0x04`   |

---

## Display Behaviour

```
        P
   D  [center]  R
        N
```

- **Rotate dial** → highlight cycles P→R→N→D (wraps)
- **Press button** → commits selection, begins sending that gear on CAN
- **Blue letter** → currently selected / pending VCU acknowledgement
- **Red letter**  → VCU has confirmed the gear via `0x312` echo

---

## Zombieverter Configuration

In the Zombieverter web interface, enable the **JLR Gen1 shifter** module.
No extra CAN mapping table entries needed — the VCU's JLR code handles
`0x3F3` natively and sends `0x312` back.

---

## Build & Flash

```bash
# Open project in VS Code with PlatformIO
# Select env:m5stack-dial
pio run -t upload
pio device monitor   # 115200 baud for debug output
```

---

## Serial Debug Output

```
CAN started at 500kbps  TX=GPIO2  RX=GPIO1
Button pressed → sending gear: D
VCU ack gear: D
```
