# GPSRx — Software-Defined GPS Receiver

A from-scratch GPS **L1 C/A** software receiver written in **C++Builder (RAD Studio 13)**.
It ingests down-converted, digitized GPS IF samples (signed 8-bit integers) and runs the
full receiver chain to a position fix:

```
IF samples ──▶ Acquisition ──▶ Tracking ──▶ Nav decode ──▶ PVT ──▶ lat / lon / alt
```

The DSP is pure standard C++ (no VCL); a VCL GUI (`MainUnit`) drives it with
**Acquisition**, **Tracking**, and **Position** tabs, each running its heavy work on a
background thread.

## Modules

| File | Role |
|------|------|
| `Fft.*` | Radix-2 FFT + Bluestein FFT (arbitrary length, e.g. 38192 samples/ms) |
| `CACode.*` | C/A Gold-code generator (PRN 1–32) and sampler |
| `Acquisition.*` | FFT parallel-code-phase sky search + fine-Doppler refinement |
| `Tracking.*` | Costas PLL + DLL carrier/code tracking, per-ms correlator epochs, NWPR C/N0 |
| `NavMessage.*` | Bit sync, LNAV TLM/HOW frame sync + parity (IS-GPS-200 Table 20-XIV), ephemeris decode |
| `Pvt.*` | Satellite ECEF position (Kepler), pseudoranges, least-squares PVT with Sagnac correction, WGS-84 geodetic |
| `MainUnit.*` | VCL form: tabbed GUI + threaded acquisition / tracking / position workers |

`*Console.cpp` are standalone command-line harnesses (built with `bcc64x`) that exercise and
verify each stage against a real capture, independent of the GUI.

## Building

Open `GPSRx.cbproj` in RAD Studio 13 and build (target **Win64x**), or from a command line:

```bat
_build.bat Release      :: calls rsvars.bat then msbuild GPSRx.cbproj /p:Platform=Win64x
```

The console harnesses build directly with the C++Builder Clang compiler, e.g.:

```bat
bcc64x -O2 -std=c++17 PvtConsole.cpp Pvt.cpp NavMessage.cpp Tracking.cpp ^
       Acquisition.cpp CACode.cpp Fft.cpp -o PvtConsole.exe
```

## Verified result

End-to-end on a 38.192 MHz / 9.55 MHz IF capture: **8 satellites** acquired and tracked
(C/N0 40–50 dB-Hz), all ephemerides decoded, and a position fix at

> **40.008074° N, −105.262681° W, 1637 m** (Boulder, CO) — GDOP 2.24, post-fit residual RMS 2.4 m.

## Sample data

The GPS IF captures (`*.bin`) are **not** included — they are gigabytes each and exceed
GitHub's file-size limits. The receiver reads `fs` and IF from the file name
(`GPSdata-DiscreteComponents-fs38_192-if9_55.bin` → fs = 38.192 MHz, IF = 9.55 MHz; `_` is the
decimal point), so any compatible int8 IF capture can be loaded via **File ▸ Open**.
