//---------------------------------------------------------------------------
// GpuAcqConsole.cpp - validate the GPU acquisition path against the CPU
// reference on a real capture, and time both. NO CUDA toolchain is needed to
// build this - GpuAcquire.dll is loaded at runtime by the shim. If the DLL or a
// GPU is absent, it just reports that and runs the CPU path only.
//
// Build (C++Builder Clang compiler):
//   bcc64x -O2 -std=c++17 GpuAcqConsole.cpp AcquisitionGpu.cpp ^
//          ..\Acquisition.cpp ..\CACode.cpp ..\Fft.cpp -o GpuAcqConsole.exe
//
// Usage:
//   GpuAcqConsole [file] [numMs] [dopplerStepHz] [threshold]
//---------------------------------------------------------------------------
#include "../Acquisition.h"
#include "AcquisitionGpu.h"

#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <chrono>

int main(int argc, char** argv)
{
    std::string file = (argc > 1) ? argv[1]
                                  : "GPSdata-DiscreteComponents-fs38_192-if9_55.bin";
    gps::AcqConfig cfg;                       // defaults: fs=38.192M, IF=9.55M
    cfg.numMs       = (argc > 2) ? std::atoi(argv[2]) : 2;
    cfg.dopplerStep = (argc > 3) ? std::atof(argv[3]) : 500.0;
    cfg.threshold   = (argc > 4) ? std::atof(argv[4]) : 2.5;

    const int n = (int)std::lround(cfg.fs * 1.0e-3);
    const std::size_t need = (std::size_t)n * cfg.numMs;

    std::vector<std::int8_t> sig(need);
    {
        std::ifstream f(file, std::ios::binary);
        if (!f) { std::printf("ERROR: cannot open %s\n", file.c_str()); return 2; }
        f.read(reinterpret_cast<char*>(sig.data()), (std::streamsize)need);
        if ((std::size_t)f.gcount() < need) {
            std::printf("ERROR: short read (need %zu samples; is the file big enough?)\n", need);
            return 3;
        }
    }
    std::printf("File: %s\n", file.c_str());
    std::printf("Config: fs=%.4f MHz  IF=%.4f MHz  step %.0f Hz  %d ms  thr=%.2f\n\n",
                cfg.fs / 1e6, cfg.ifFreq / 1e6, cfg.dopplerStep, cfg.numMs, cfg.threshold);

    // --- CPU reference ---
    auto t0 = std::chrono::steady_clock::now();
    std::vector<gps::AcqResult> cpu = gps::acquireAll(sig.data(), sig.size(), cfg);
    auto t1 = std::chrono::steady_clock::now();
    const double cpuSecs = std::chrono::duration<double>(t1 - t0).count();

    // --- GPU path ---
    std::printf("GPU available: %s\n", gps::gpuAcqIsAvailable() ? "yes" : "no (CPU-only run)");
    if (gps::gpuAcqIsAvailable())
        std::printf("Selected GPU : %s\n", gps::gpuAcqDeviceName().c_str());
    std::vector<gps::AcqResult> gpu;
    double gpuSecs = 0.0;
    bool ranGpu = false;
    if (gps::gpuAcqIsAvailable()) {
        auto g0 = std::chrono::steady_clock::now();
        ranGpu = gps::acquireAllGpu(sig.data(), sig.size(), cfg, gpu);
        auto g1 = std::chrono::steady_clock::now();
        gpuSecs = std::chrono::duration<double>(g1 - g0).count();
    }

    if (!ranGpu) {
        std::printf("\nCPU-only results (%.2f s):\n", cpuSecs);
        std::printf("PRN  found  Doppler   phase(samp)  ratio\n");
        for (const auto& r : cpu)
            if (r.found)
                std::printf("%3d   YES   %+7.0f   %9d   %6.2f\n",
                            r.prn, r.doppler, r.codePhaseSamp, r.peakRatio);
        return 0;
    }

    // --- Side-by-side comparison ---
    std::printf("\n%-4s | %-22s | %-22s | %s\n", "PRN", "CPU (found/dop/ph/ratio)",
                "GPU (found/dop/ph/ratio)", "match");
    std::printf("-----+------------------------+------------------------+------\n");
    int foundCpu = 0, foundGpu = 0, foundMismatch = 0, phaseMismatch = 0;
    double maxRatioRelDiff = 0.0;
    const int phaseTol = 2;   // samples (FP32 vs FP64 can nudge the argmax by 1)

    for (int i = 0; i < 32; ++i) {
        const gps::AcqResult& a = cpu[i];
        const gps::AcqResult& b = gpu[i];
        if (a.found) ++foundCpu;
        if (b.found) ++foundGpu;

        bool fMatch = (a.found == b.found);
        if (!fMatch) ++foundMismatch;

        int dphase = std::abs(a.codePhaseSamp - b.codePhaseSamp);
        if (dphase > n / 2) dphase = n - dphase;               // circular
        bool pMatch = (!a.found && !b.found) || (dphase <= phaseTol);
        if (a.found && b.found && !pMatch) ++phaseMismatch;

        if (a.found && b.found && a.peakRatio > 0) {
            double rel = std::fabs(a.peakRatio - b.peakRatio) / a.peakRatio;
            if (rel > maxRatioRelDiff) maxRatioRelDiff = rel;
        }

        // Only print rows where either side found something (keep it short).
        if (a.found || b.found) {
            char cbuf[40], gbuf[40];
            std::snprintf(cbuf, sizeof(cbuf), "%3s %+6.0f %7d %5.2f",
                          a.found ? "YES" : " . ", a.doppler, a.codePhaseSamp, a.peakRatio);
            std::snprintf(gbuf, sizeof(gbuf), "%3s %+6.0f %7d %5.2f",
                          b.found ? "YES" : " . ", b.doppler, b.codePhaseSamp, b.peakRatio);
            std::printf("%-4d | %-22s | %-22s | %s\n",
                        i + 1, cbuf, gbuf,
                        (fMatch && pMatch) ? "ok" : "DIFF");
        }
    }

    std::printf("\nCPU: %d found in %.2f s    GPU: %d found in %.3f s    speedup x%.1f\n",
                foundCpu, cpuSecs, foundGpu, gpuSecs, gpuSecs > 0 ? cpuSecs / gpuSecs : 0.0);
    std::printf("found-set mismatches: %d   code-phase mismatches (>%d samp): %d   "
                "max peak-ratio rel.diff: %.1f%%\n",
                foundMismatch, phaseTol, phaseMismatch, maxRatioRelDiff * 100.0);

    const bool agree = (foundMismatch == 0) && (phaseMismatch == 0);
    std::printf("\nVERDICT: %s\n", agree
        ? "GPU matches CPU (same satellites, same code phase)."
        : "GPU differs from CPU - investigate (see notes in README.md).");
    return agree ? 0 : 1;
}
