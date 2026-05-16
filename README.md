# FIFO (First-In First-Out Buffer) — SystemVerilog Implementation

## Table of Contents

1. [Overview](#overview)
2. [Architecture Diagram](#architecture-diagram)
3. [Port Reference](#port-reference)
4. [Internal Signals & Storage](#internal-signals--storage)
5. [Functional Description](#functional-description)
   - [Push / Pop Gating](#push--pop-gating)
   - [Write Pointer (`waddr`)](#write-pointer-waddr)
   - [Memory Array & Shift Behaviour](#memory-array--shift-behaviour)
   - [Empty Flag](#empty-flag)
   - [Full Flag](#full-flag)
   - [Overrun Detection](#overrun-detection)
   - [Underrun Detection](#underrun-detection)
   - [Threshold / THRE Trigger](#threshold--thre-trigger)
   - [Read Port](#read-port)
6. [Operation Truth Table](#operation-truth-table)
7. [Timing Diagrams](#timing-diagrams)
   - [Normal Push Sequence](#normal-push-sequence)
   - [Normal Pop Sequence](#normal-pop-sequence)
   - [Simultaneous Push & Pop](#simultaneous-push--pop)
   - [Overrun Condition](#overrun-condition)
   - [Underrun Condition](#underrun-condition)
8. [State / Flag Transition Summary](#state--flag-transition-summary)
9. [Known Issues & Notes](#known-issues--notes)
10. [Integration with UART](#integration-with-uart)

---

## Overview

This module implements a **16-entry × 8-bit synchronous FIFO** in SystemVerilog, designed for use as the TX and RX data buffers inside a 16550-compatible UART.

Key characteristics:

- **Depth:** 16 bytes (indices 0–15)
- **Width:** 8 bits
- **Read order:** Always from `mem[0]` — shift-on-pop model (queue-style, not ring-buffer)
- **Clock domain:** Single clock, synchronous read/write, asynchronous reset
- **Flow control:** Hardware `full` / `empty` flags gate push/pop automatically
- **Error flags:** `overrun` (write to full FIFO) and `underrun` (read from empty FIFO)
- **Threshold:** Programmable 4-bit level that asserts `thre_trigger` when occupancy reaches it
- **Enable:** Global `en` signal — disables the FIFO, forcing `full` and `empty` asserted

---

## Architecture Diagram
<img width="2816" height="1536" alt="fifo" src="https://github.com/user-attachments/assets/a986a055-1bae-4ad1-95a7-68bc37f73cf8" />

---

## Port Reference

| Port            | Dir    | Width | Description                                               |
|-----------------|--------|-------|-----------------------------------------------------------|
| `clk`           | Input  | 1     | System clock — all FFs are rising-edge triggered          |
| `rst`           | Input  | 1     | Asynchronous reset, active high                           |
| `en`            | Input  | 1     | FIFO enable — when low, forces `full` and `empty` high    |
| `push_in`       | Input  | 1     | Push request from upstream logic (e.g. host write / RX engine) |
| `pop_in`        | Input  | 1     | Pop request from downstream logic (e.g. TX engine / host read) |
| `din[7:0]`      | Input  | 8     | Data byte to write into FIFO                              |
| `dout[7:0]`     | Output | 8     | Always presents the oldest (head) byte in the FIFO        |
| `empty`         | Output | 1     | Asserted when FIFO holds no valid data                    |
| `full`          | Output | 1     | Asserted when FIFO has no free slots                      |
| `overrun`       | Output | 1     | Pulse: push attempted while `full` was asserted           |
| `underrun`      | Output | 1     | Pulse: pop attempted while `empty` was asserted           |
| `threshold[3:0]`| Input  | 4     | Occupancy level at which `thre_trigger` asserts           |
| `thre_trigger`  | Output | 1     | Asserts when `waddr >= threshold`                         |

---

## Internal Signals & Storage

| Signal       | Type            | Width  | Description                                         |
|--------------|-----------------|--------|-----------------------------------------------------|
| `mem`        | `reg [7:0]`     | 16 × 8 | The physical storage array                          |
| `waddr`      | `reg [3:0]`     | 4      | Write pointer = current occupancy count (0–15)      |
| `push`       | `logic`         | 1      | Gated push: `push_in & ~full_t`                     |
| `pop`        | `logic`         | 1      | Gated pop: `pop_in & ~empty_t`                      |
| `empty_t`    | `reg`           | 1      | Internal empty flag register                        |
| `full_t`     | `reg`           | 1      | Internal full flag register                         |
| `underrun_t` | `reg`           | 1      | Internal underrun flag register                     |
| `overrun_t`  | `reg`           | 1      | Internal overrun flag register                      |
| `thre_t`     | `reg`           | 1      | Internal threshold trigger register                 |

---

## Functional Description

### Push / Pop Gating

Raw requests from external logic are gated before reaching the memory and pointer logic:

```systemverilog
assign push = push_in & ~full_t;   // Block writes when FIFO is full
assign pop  = pop_in  & ~empty_t;  // Block reads when FIFO is empty
```

This means the internal `push` and `pop` signals are **never simultaneously active during overflow or underflow** conditions. The raw `push_in` / `pop_in` signals are still monitored separately to raise `overrun` / `underrun` flags.

---

### Write Pointer (`waddr`)

`waddr` acts as both the **write address** and the **occupancy counter**:

- `waddr == 0` → FIFO empty (no valid entries)
- `waddr == 15` → FIFO full (all 16 slots occupied, since slot 0–14 hold data and slot 15 is the next target)

```
Occupancy = waddr
Free slots = 16 - waddr
```

**Update rules (synchronous, on `clk` rising edge):**

| `{push, pop}` | Condition              | Action                        |
|---------------|------------------------|-------------------------------|
| `2'b10`       | `waddr < 15 & ~full`   | `waddr <= waddr + 1`          |
| `2'b10`       | `waddr == 15 or full`  | `waddr` unchanged             |
| `2'b01`       | `waddr > 0 & ~empty`   | `waddr <= waddr - 1`          |
| `2'b01`       | `waddr == 0 or empty`  | `waddr` unchanged             |
| `2'b11`       | push + pop together    | `waddr` unchanged (net zero)  |
| `2'b00`       | idle                   | `waddr` unchanged             |

---

### Memory Array & Shift Behaviour

This FIFO uses a **shift-register (queue) model** rather than a ring-buffer. On every pop, all entries physically shift down by one position:

```
Before pop:   mem[0]=A  mem[1]=B  mem[2]=C  mem[3]=D  ...
After pop:    mem[0]=B  mem[1]=C  mem[2]=D  mem[3]=0  ...
```

This is implemented with a `for` loop:

```systemverilog
for(int i = 0; i < 14; i++)
    mem[i] <= mem[i+1];
mem[15] <= 8'h00;  // clear tail slot
```

**All four combinations of `{push, pop}`:**

| `{push, pop}` | Memory Action                                                          |
|---------------|------------------------------------------------------------------------|
| `2'b00`       | No change                                                              |
| `2'b01`       | Shift all entries down: `mem[i] <= mem[i+1]`, clear `mem[15]`         |
| `2'b10`       | Write: `mem[waddr] <= din`                                             |
| `2'b11`       | Shift down first, then write new data at `mem[waddr-1]` (net: replace head with tail shift + new tail write) |

> **Note on `2'b11`:** When pushing and popping simultaneously, the shift happens first and the new byte is written at `mem[waddr - 1]`, keeping occupancy constant. This is valid but requires `waddr` to be non-zero — guaranteed since `pop` is already gated by `~empty_t`.

---

### Empty Flag

```systemverilog
case({push, pop})
  2'b01: empty_t <= (~|(waddr) | ~en);  // After pop: empty if waddr reaches 0, or FIFO disabled
  2'b10: empty_t <= 1'b0;               // After push: definitely not empty
  default: ;                             // No change on idle or simultaneous push+pop
endcase
```

- `~|(waddr)` is a NOR reduction — true when `waddr == 0` (all bits zero)
- When `en == 0`, the FIFO is treated as always empty (no valid data)
- **Reset value:** `empty_t = 0` (note: see [Known Issues](#known-issues--notes))

---

### Full Flag

```systemverilog
case({push, pop})
  2'b10: full_t <= (&(waddr) | ~en);   // After push: full if waddr reaches 15 (all bits 1), or disabled
  2'b01: full_t <= 1'b0;               // After pop: definitely not full
  default: ;
endcase
```

- `&(waddr)` is an AND reduction — true when `waddr == 15` (all bits one)
- When `en == 0`, the FIFO is treated as always full (no writes accepted)
- **Reset value:** `full_t = 0`

---

### Overrun Detection

Asserts for **one clock cycle** when a write is attempted on an already-full FIFO:

```systemverilog
else if(push_in == 1'b1 && full_t == 1'b1)
    overrun_t <= 1'b1;
else
    overrun_t <= 1'b0;
```

This is a **pulse output** — it deasserts the very next cycle unless the condition persists. In the UART context this feeds `lsr.oe` (Overrun Error bit in the Line Status Register).

---

### Underrun Detection

Asserts for **one clock cycle** when a read is attempted on an already-empty FIFO:

```systemverilog
else if(pop_in == 1'b1 && empty_t == 1'b1)
    underrun_t <= 1'b1;
else
    underrun_t <= 1'b0;
```

In the UART TX path, the TX engine uses `thre` (derived from `empty`) to avoid popping from an empty FIFO, so underrun should never occur during normal operation.

---

### Threshold / THRE Trigger

```systemverilog
else if(push ^ pop)  // Only update on net change in occupancy
    thre_t <= (waddr >= threshold) ? 1'b1 : 1'b0;
```

- `thre_trigger` asserts when `waddr >= threshold`
- Only re-evaluated when occupancy **changes** (push XOR pop is true)
- In 16550 UART mode, the RX threshold maps to `fcr.rx_trigger`:

| `rx_trigger` | Threshold Value | Trigger Level |
|:---:|:---:|---|
| `2'b00` | 1  | 1 byte  |
| `2'b01` | 4  | 4 bytes |
| `2'b10` | 8  | 8 bytes |
| `2'b11` | 14 | 14 bytes|

---

### Read Port

```systemverilog
assign dout = mem[0];
```

`dout` is **combinatorially driven** from `mem[0]` at all times. There is no registered read stage — the head of the queue is always visible on `dout` without needing a separate read strobe. This is standard for a transparent FIFO used in UART applications where the host reads data via a bus cycle that also issues the `pop_in` strobe.

---

## Operation Truth Table

| `push_in` | `pop_in` | `full` | `empty` | `push` | `pop` | Effect                                 |
|:---------:|:--------:|:------:|:-------:|:------:|:-----:|----------------------------------------|
| 0         | 0        | X      | X       | 0      | 0     | Idle — no change                       |
| 1         | 0        | 0      | X       | 1      | 0     | Write `din` to `mem[waddr]`, increment `waddr` |
| 1         | 0        | 1      | X       | 0      | 0     | Blocked — `overrun` asserted           |
| 0         | 1        | X      | 0       | 0      | 1     | Shift memory down, decrement `waddr`   |
| 0         | 1        | X      | 1       | 0      | 0     | Blocked — `underrun` asserted          |
| 1         | 1        | 0      | 0       | 1      | 1     | Simultaneous: shift + write new tail   |
| 1         | 1        | 1      | 0       | 0      | 1     | Only pop executes; write blocked       |
| 1         | 1        | 0      | 1       | 1      | 0     | Only push executes; pop blocked        |

---

## Timing Diagrams

### Normal Push Sequence

```
clk       : __|‾|__|‾|__|‾|__|‾|__|‾|__
push_in   : ___|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|____
din       : ---[A ][B ][C ][D ]--------
waddr     :  0   1   2   3   4
full      : ___________________________  (stays low, only 4 entries)
empty     : |‾‾‾|________________________ (clears after first push)
dout      : ---[A ][A ][A ][A ]--------  (head always mem[0])
```

### Normal Pop Sequence

```
clk       : __|‾|__|‾|__|‾|__|‾|__
pop_in    : ___|‾‾‾‾‾‾‾‾‾‾‾‾‾|____
             (FIFO holds A, B, C, D)
waddr     :  4   3   2   1   0
dout      :  A   B   C   D   (invalid — empty asserts)
empty     : ________________|‾‾‾|
```

### Simultaneous Push & Pop

```
clk       : __|‾|__|‾|__|‾|__
push_in   : ___|‾‾‾‾‾‾‾|____
pop_in    : ___|‾‾‾‾‾‾‾|____
din       : ---[X ][Y ]------
             (FIFO holds A, B, C → 3 entries)
waddr     :  3   3   3        (no net change)
dout      :  A   B   C        (head advances each cycle)
```

### Overrun Condition

```
clk       : __|‾|__|‾|__
push_in   : ___|‾‾‾‾‾|__
full      : ‾‾‾‾‾‾‾‾‾‾‾‾  (FIFO already full, waddr=15)
push      : ______________ (blocked by gate)
overrun   : ___|‾‾‾‾‾|___  (asserts for duration of push_in & full)
waddr     :  15  15  15   (unchanged)
```

### Underrun Condition

```
clk       : __|‾|__|‾|__
pop_in    : ___|‾‾‾‾‾|__
empty     : ‾‾‾‾‾‾‾‾‾‾‾‾  (FIFO already empty, waddr=0)
pop       : ______________ (blocked by gate)
underrun  : ___|‾‾‾‾‾|___  (asserts for duration of pop_in & empty)
waddr     :  0   0   0    (unchanged)
```

---

## State / Flag Transition Summary

```
                      PUSH (not full)
         ┌──────────────────────────────────────────────┐
         │                                              │
         ▼                                              │
   ┌───────────┐   POP (last entry)    ┌───────────────────┐
   │  EMPTY    │◄──────────────────────┤    HAS DATA       │
   │ empty=1   │                       │ empty=0, full=0   │
   │ full=0    │──────────────────────►│                   │
   └───────────┘   PUSH (first entry)  └────────┬──────────┘
                                                 │  PUSH (waddr=14→15)
                                                 ▼
                                        ┌───────────────┐
                                        │     FULL      │
                                        │ full=1        │
                                        │ empty=0       │
                                        └───────────────┘
                                                 │  POP (any)
                                                 └──────────────► HAS DATA
```

**Error flags** are orthogonal to the above — they pulse for one cycle and do not affect state:

| Condition                   | Flag asserted  |
|-----------------------------|----------------|
| `push_in=1` while `full=1`  | `overrun`      |
| `pop_in=1` while `empty=1`  | `underrun`     |

---

## Known Issues & Notes

### 1. Empty Flag Reset Value is Wrong

```systemverilog
// Current:
always@(posedge clk, posedge rst)
  if(rst) empty_t <= 1'b0;   // BUG: FIFO is actually empty after reset
```

After reset, `waddr == 0` meaning the FIFO has no valid data. The `empty_t` flag should reset to `1'b1`, not `1'b0`. As written, the FIFO will appear non-empty immediately after reset, allowing a spurious pop to occur before the first push.

**Fix:**
```systemverilog
if(rst) empty_t <= 1'b1;
```

### 2. `push ^ pop` Threshold Condition Misses Simultaneous Case

```systemverilog
else if(push ^ pop)   // XOR — true only when exactly one of push/pop is active
    thre_t <= (waddr >= threshold) ? 1'b1 : 1'b0;
```

When `push` and `pop` are both asserted simultaneously (`2'b11`), the XOR is 0, so `thre_t` is not updated. Since waddr doesn't change on a simultaneous push+pop this is benign in most cases, but the comment in the source says `/// 1 1` suggesting simultaneous operation was intended to also trigger a threshold re-evaluation.

### 3. Shift Loop Upper Bound is 14, Not 15

```systemverilog
for(int i = 0; i < 14; i++)
    mem[i] <= mem[i+1];
mem[15] <= 8'h00;
```

The loop copies `mem[1]→mem[0]` through `mem[14]→mem[13]`. `mem[14]` is then explicitly cleared by `mem[15] <= 8'h00`... but that clears slot **15**, not slot **14**. After a pop, `mem[14]` retains stale data. Since `waddr` tracks occupancy and reads are bounded by `waddr`, this stale data is never visible via `dout` under normal use, but it is a latent correctness issue.

**Fix:** Change loop bound to `i < 15` and remove the separate `mem[15] <= 8'h00` line, or keep the loop at `i < 14` and also clear `mem[14]`.

### 4. Simultaneous Push+Pop Memory Write Uses `waddr - 1`

```systemverilog
2'b11: begin
    for(int i = 0; i < 14; i++)
        mem[i] <= mem[i+1];
    mem[15] <= 8'h00;
    mem[waddr - 1] <= din;   // waddr not yet updated in this cycle
end
```

Since all assignments in an `always` block are non-blocking, `waddr` still holds its pre-cycle value here. After the shift, the correct write slot for the incoming byte is `mem[waddr - 1]` (because one slot was freed by the shift). This logic is correct — but only works when `waddr >= 1`. If `waddr == 0` (empty FIFO), this underflows to `mem[15]`, which would be incorrect. However, `push` is gated by `~full_t` and `pop` by `~empty_t`, so `{push,pop} == 2'b11` is only possible when the FIFO is neither full nor empty, meaning `1 <= waddr <= 14`. The edge case is therefore safely unreachable.

### 5. `en` Affects Flags But Not Memory

When `en` is deasserted:
- `full_t` is forced high (via `| ~en` in full flag logic)
- `empty_t` is forced high (via `| ~en` in empty flag logic)

This prevents new pushes and pops via the gates. However, `waddr` and `mem[]` contents are **not cleared** when `en` goes low. Re-enabling the FIFO without a reset may expose stale data. For safe re-enable, assert `rst` first.

---

## Integration with UART

In the 16550 UART design this FIFO is instantiated twice:

### TX FIFO

| FIFO Signal   | Connected to          | Description                                  |
|---------------|-----------------------|----------------------------------------------|
| `push_in`     | `tx_push_o` (regs)    | Host write to THR address                    |
| `pop_in`      | `pop` (uart_tx_top)   | TX engine requests next byte                 |
| `din`         | `din_i` (host bus)    | Byte written by host                         |
| `dout`        | `din` (uart_tx_top)   | Byte read by TX shift register               |
| `empty`       | `thre` (LSR bit 5)    | Signals host that TX buffer is free          |
| `full`        | (tie-off or IRQ)      | Optional: interrupt when TX FIFO fills       |
| `threshold`   | Fixed (e.g. 4'h0)     | TX threshold not used in basic implementation|
| `overrun`     | Ignored (TX)          | Host overwrite — software should check THRE  |

### RX FIFO

| FIFO Signal   | Connected to          | Description                                   |
|---------------|-----------------------|-----------------------------------------------|
| `push_in`     | `push` (uart_rx_top)  | RX engine deposits received byte              |
| `pop_in`      | `rx_pop_o` (regs)     | Host read from RHR address                    |
| `din`         | `dout` (uart_rx_top)  | Byte from RX shift register                   |
| `dout`        | `rx_fifo_in` (regs)   | Byte visible to host on bus read              |
| `empty`       | `~lsr.dr`             | Data Ready flag (inverted empty)              |
| `full`        | (overrun source)      | If full when RX engine pushes → `lsr.oe`      |
| `threshold`   | `rx_fifo_threshold`   | From `fcr.rx_trigger` — drives RX interrupt   |
| `thre_trigger`| RX FIFO interrupt     | Fires when RX occupancy reaches trigger level |
| `overrun`     | `lsr.oe`              | RX overrun error reported to host             |

---

*This README was generated from the provided `fifo_top` SystemVerilog source. All signal names, behaviour descriptions, and identified issues reflect the code as written.*
