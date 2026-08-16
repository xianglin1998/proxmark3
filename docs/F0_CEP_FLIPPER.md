# Proxmark5 — Flipper Zero F0 CEP (experimental)

Branch of [xianglin1998/proxmark3](https://github.com/xianglin1998/proxmark3) (`proxmark5`) that **re-enables / isolates** the Flipper Zero Type-C CEP handshake and SPI NG transport so the [FAP](https://github.com/limerx/Proxmark5_FlipperZero_FAP) can leave “Connecting…”.

## Credits

| Who | Contribution |
|-----|----------------|
| **DXL** ([xianglin1998](https://github.com/xianglin1998)) | Proxmark5 platform, original CEP experiments, HAL |
| **RfidResearchGroup** / **Iceman** / PM3 contributors | Proxmark3 armsrc, protocols, tooling |
| **limerx** | Extracted/enabled `pm5_f0_cep` path, docs, companion FAP |

Keep GPL-3.0 (`LICENSE.txt`) and upstream copyright headers.

## What changed (this branch)

- New `armsrc/pm5_f0_cep.c` / `pm5_f0_cep.h` — UART handshake (`iamf0rupm5`) + SPI NG
- Wired from `appmain.c` (non-blocking poll; USB PC client still works)
- `cmd.c` CEP-aware reply packing / framing tweaks
- Small Classic nested stack-safety tweak in `mifarecmd.c`
- `CMakeLists.txt` builds `pm5_f0_cep.c`
- Unit-test file cleaned of inlined CEP stubs moved to the new module

## Build / flash (PM5 only)

```bash
# Example — follow current DXL/RRG PM5 build docs for your tree
# PLATFORM must target Proxmark5, never Flipper firmware images
```

Before flashing, confirm the USB device is the **Proxmark5**, not the Flipper (`ID_MODEL` / serial). Flashing the wrong target can brick or confuse devices.

## Companion FAP

https://github.com/limerx/Proxmark5_FlipperZero_FAP

## Status

Experimental. Upstream may disable CEP paths intentionally until HAL work lands. Coordinate with DXL / RRG before assuming this should merge as-is.

## Legal

GPL-3.0. Research / authorized testing only.
