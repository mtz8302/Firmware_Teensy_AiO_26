# Memory Usage Baseline - Before Optimization

**Date:** 2025-10-11
**Branch:** main (commit: 9768c29)
**Firmware Version:** 1.0.76-beta

## Memory Statistics

### RAM1 (FlexRAM - ITCM + DTCM)
```
Total RAM1:       524,288 bytes (512 KB)
Variables:        140,832 bytes (137.5 KB)
Code (ITCM):      337,508 bytes (329.6 KB)
Padding:           22,940 bytes ( 22.4 KB)
--------------------------------
Used:             501,280 bytes (489.5 KB)
Free (stack):      23,008 bytes ( 22.5 KB)
Utilization:       95.5% ⚠️ CRITICAL
```

### Build Output
```
Processing teensy41 (platform: teensy; board: teensy41; framework: arduino)
teensy_size:    RAM1: variables:140832, code:337508, padding:22940   free for local variables:23008
========================= [SUCCESS] Took 3.17 seconds =========================
```

## Key Memory Consumers

### Network (QNEthernet/lwIP)
- PBUF_POOL: ~24 KB
- TCP/UDP PCBs: ~2 KB
- Ethernet DMA buffers: ~15 KB (in RAM2)
- **Total Network: ~41 KB**

### CAN Buffers (FlexCAN_T4)
- CAN1 (V-Bus): RX=256, TX=16 = ~7.6 KB
- CAN2 (K-Bus): RX=256, TX=16 = ~7.6 KB
- CAN3 (ISO-Bus): RX=256, TX=256 = ~14.3 KB
- **Total CAN: ~29.5 KB**

### Other
- System tables: ~10 KB
- Library objects: ~5 KB
- Application data: ~30 KB
- Remaining variables: ~25 KB

## Problem Statement

With only **22.5 KB** free for stack and local variables, the system is at **95.5% RAM utilization**. This leaves minimal headroom for:
- Deep function call stacks
- Large local arrays
- Future features (sensor fusion, VWAS, etc.)
- Safety margin for stack growth

## Optimization Goals

### Target After Optimization
```
Total RAM1:       524,288 bytes (512 KB)
Used:            ~400,000 bytes (390 KB)
Free (stack):    ~124,000 bytes (121 KB)
Utilization:      ~76% ✓ HEALTHY
```

### Expected Savings
- NativeEthernet/FNET migration: **~80 KB**
- CAN buffer optimization: **~21 KB**
- **Total savings: ~101 KB**

## Notes

- Current configuration uses QNEthernet with lwIP stack
- CAN buffers sized for maximum theoretical traffic
- No PSRAM currently utilized
- FlexRAM dynamically partitions between ITCM/DTCM (11 banks ITCM, 5 banks DTCM)


## Update 2026-10-03 - the ITCM bank cliff (this is what breaks builds)

RAM1 is the FlexRAM: **ITCM and DTCM share the same 512 KB**, and the ITCM share is
allocated in 32 KB banks. `teensy_size`'s `padding` is the unused remainder of that
bank allocation, so it is **quantized** - it does not grow smoothly:

```
11 banks = 360 448 B of ITCM
  before:  code 360260 -> padding = 360448 - 360260 =    188 B   OK
  after:   code 361412 -> 12 banks needed
           padding = 393216 - 361412 = 31 804 B
           -> 32 KB are taken away from DTCM
           -> "free for local variables" becomes NEGATIVE (-27 424)
           -> [size] Error 4294967295 / "Error program exceeds memory space"
```

Measured on 2026-10-03: adding ~1.1 KB of ITCM code (a new web endpoint) crossed
`360 448` and the build failed, even though only ~1 KB had been added - the 32 KB bank
did the damage. `variables` and `code` are both counted against the 512 KB.

**Rule: ITCM code must stay below 360 448 bytes. Every byte above costs 32 KB of DTCM.**

The lever is `FLASHMEM`: it moves a function - and, importantly, its **string literals**,
which otherwise land in DTCM as `.rodata` - to the 8 MB flash. Applied on 2026-10-03 to
`HeadingBridge::init()`, `HeadingBridge::setPhysicalBaselineM()` and
`SimpleWebManager::handleBridgeBaseline()` to get back under the cliff. Never apply it
to ISRs or timing-critical code.

State after the fix:

```
teensy_size:   FLASH: code:381464, data:326932, headers:8400   free for files:7409668
teensy_size:    RAM1: variables:158496, code:359572, padding:876   free for local variables:5344
teensy_size:    RAM2: variables:156000  free for malloc/new:368288
```

Headroom is now **876 bytes of ITCM code** and **5344 bytes of stack** - both thin. The
next feature costing more than 876 bytes of ITCM code will take 32 KB of DTCM with it and
the build will fail again the same way. The ~101 KB of savings sketched at the top of this
file are still the real fix; `FLASHMEM` only buys time.
