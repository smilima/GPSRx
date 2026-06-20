//---------------------------------------------------------------------------
// TrackConsole.cpp - Standalone harness to validate carrier+code tracking on
// the real sample file. Acquires the strongest satellite, then tracks it and
// checks for lock (carrier convergence, I-vs-Q power, C/N0, nav-bit edges).
//
// Build:
//   bcc64x -O2 -std=c++17 TrackConsole.cpp Tracking.cpp Acquisition.cpp \
//          CACode.cpp Fft.cpp -o TrackConsole.exe
//---------------------------------------------------------------------------
#include "Acquisition.h"
#include "Tracking.h"
#include "NavMessage.h"

#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <chrono>

int main(int argc, char** argv)
{
    std::string file = (argc > 1) ? argv[1]
                                  : "GPSdata-DiscreteComponents-fs38_192-if9_55.bin";
    int numMs = (argc > 2) ? std::atoi(argv[2]) : 1000;   // tracking duration (ms)

    gps::AcqConfig acfg;            // fs=38.192 MHz, IF=9.55 MHz
    const int n = (int)std::lround(acfg.fs * 1.0e-3);     // samples per 1 ms

    std::ifstream f(file, std::ios::binary);
    if (!f) { std::printf("ERROR: cannot open %s\n", file.c_str()); return 2; }

    // --- Acquire (first 2 ms) to seed tracking ---------------------------------
    std::vector<std::int8_t> acqBuf((std::size_t)n * acfg.numMs);
    f.read(reinterpret_cast<char*>(acqBuf.data()), (std::streamsize)acqBuf.size());
    auto results = gps::acquireAll(acqBuf.data(), acqBuf.size(), acfg);

    gps::AcqResult best; best.peakRatio = -1;
    for (const auto& r : results)
        if (r.found && r.peakRatio > best.peakRatio) best = r;
    if (best.prn == 0) { std::printf("No satellite acquired; cannot track.\n"); return 3; }

    std::printf("=== Tracking PRN %d ===\n", best.prn);
    std::printf("Seed (from acquisition): Doppler %+.0f Hz, code phase %d samples (%.1f chips), ratio %.1f\n\n",
                best.doppler, best.codePhaseSamp, best.codePhaseChips, best.peakRatio);

    // --- Read a contiguous block starting at the acquired code phase -----------
    const std::size_t need = (std::size_t)(numMs + 2) * n;
    std::vector<std::int8_t> sig(need);
    f.clear();
    f.seekg((std::streamoff)best.codePhaseSamp, std::ios::beg);
    f.read(reinterpret_cast<char*>(sig.data()), (std::streamsize)need);
    std::size_t got = (std::size_t)f.gcount();
    if (got < (std::size_t)n * 50) { std::printf("ERROR: not enough data to track.\n"); return 4; }

    // --- Track -----------------------------------------------------------------
    gps::TrackConfig tcfg;          // fs/IF default to 38.192/9.55
    gps::TrackChannel ch(best.prn, best.doppler, tcfg);

    auto t0 = std::chrono::steady_clock::now();
    std::vector<gps::TrackEpoch> ep = ch.run(sig.data(), got, numMs);
    auto t1 = std::chrono::steady_clock::now();
    double secs = std::chrono::duration<double>(t1 - t0).count();

    const int N = (int)ep.size();
    if (N < 100) { std::printf("Only %d ms tracked; aborting.\n", N); return 5; }

    // --- Sampled time series ---------------------------------------------------
    std::printf("   ms   Doppler(Hz)      I_P        Q_P    C/N0(dB-Hz)\n");
    std::printf("--------------------------------------------------------\n");
    for (int i = 0; i < N; i += 100)
        std::printf("%5d   %+8.1f   %9.0f  %9.0f      %5.1f\n",
                    i, ep[i].doppler, ep[i].iP, ep[i].qP, ep[i].cn0);

    // --- Lock metrics (skip first 100 ms for loop settling) --------------------
    const int s = 100;
    double sumAbsIP = 0, sumAbsQP = 0, sumCn0 = 0, dMin = 1e9, dMax = -1e9, dSum = 0;
    int cn0Cnt = 0;
    for (int i = s; i < N; ++i) {
        sumAbsIP += std::fabs(ep[i].iP);
        sumAbsQP += std::fabs(ep[i].qP);
        dSum += ep[i].doppler;
        dMin = std::min(dMin, ep[i].doppler);
        dMax = std::max(dMax, ep[i].doppler);
        if (ep[i].cn0 > 0) { sumCn0 += ep[i].cn0; ++cn0Cnt; }
    }
    const int m = N - s;
    const double ipOverQp = sumAbsIP / (sumAbsQP > 0 ? sumAbsQP : 1);
    const double cn0Avg = cn0Cnt ? sumCn0 / cn0Cnt : 0;

    // --- Nav-bit edges: sign(I_P) per ms; runs of ~20 ms = 50 bps data ---------
    std::printf("\nsign(I_P) for ms %d..%d (each char = 1 ms; runs of ~20 = nav bits):\n", s, s + 200);
    std::string signs;
    for (int i = s; i < s + 200 && i < N; ++i) signs += (ep[i].iP >= 0 ? '+' : '-');
    std::printf("%s\n", signs.c_str());

    int edges = 0;
    for (int i = s + 1; i < N; ++i)
        if ((ep[i].iP >= 0) != (ep[i - 1].iP >= 0)) ++edges;

    std::printf("\n--- Lock assessment (ms %d..%d) ---\n", s, N);
    std::printf("  Carrier Doppler:  %+.1f Hz  (range %.1f Hz over the run)\n",
                dSum / m, dMax - dMin);
    std::printf("  mean|I_P|/mean|Q_P|: %.2f   (>>1 => carrier locked, energy in I)\n", ipOverQp);
    std::printf("  C/N0 (avg):       %.1f dB-Hz\n", cn0Avg);
    std::printf("  I_P sign edges:   %d over %d ms (~1 per 20 ms bit boundary expected)\n",
                edges, N - s);

    bool locked = (ipOverQp > 3.0) && (cn0Avg > 33.0) && ((dMax - dMin) < 200.0);
    std::printf("\n  VERDICT: %s\n", locked ? "LOCKED (carrier + code tracking)" : "not locked");
    std::printf("  Tracked %d ms in %.2f s.\n", N, secs);

    // --- Fine-Doppler refinement + bit-aligned C/N0, all found sats ---
    std::printf("\n=== Fine-Doppler refine + bit-aligned C/N0 (all found sats) ===\n");
    std::printf("PRN  coarseDop  fineDop  |IP|/|QP|  C/N0(dB-Hz)  bitSync  LOCKED\n");
    gps::TrackConfig tc2;
    const std::size_t need2 = (std::size_t)(1000 + 2) * n;
    std::vector<std::int8_t> sig2(need2);
    int lockCount = 0;
    for (const auto& a : results) {
        if (!a.found) continue;
        f.clear();
        f.seekg((std::streamoff)a.codePhaseSamp, std::ios::beg);
        f.read(reinterpret_cast<char*>(sig2.data()), (std::streamsize)need2);
        std::size_t got2 = (std::size_t)f.gcount();

        const double fd = gps::refineDoppler(sig2.data(), got2, a.prn, a.doppler, acfg);
        gps::TrackChannel c2(a.prn, fd, tc2);
        auto e2 = c2.run(sig2.data(), got2, 1000);
        const int N2 = (int)e2.size(), s2 = (N2 > 100) ? 100 : 0;
        double si = 0, sq = 0, dmn = 1e9, dmx = -1e9;
        for (int i = s2; i < N2; ++i) {
            si += std::fabs(e2[i].iP); sq += std::fabs(e2[i].qP);
            if (e2[i].doppler < dmn) dmn = e2[i].doppler;
            if (e2[i].doppler > dmx) dmx = e2[i].doppler;
        }
        const double ratio2 = si / (sq > 0 ? sq : 1);
        gps::BitSync bs2 = gps::findBitSync(e2);
        const double newCn0 = gps::estimateCN0(e2, bs2.valid ? bs2.offset : 0);
        const bool lk = (ratio2 > 3.0) && ((dmx - dmn) < 200.0) && bs2.valid;
        if (lk) ++lockCount;
        std::printf("%3d   %+8.0f  %+7.0f  %8.1f    %7.1f     %-5s    %s\n",
                    a.prn, a.doppler, fd, ratio2, newCn0,
                    bs2.valid ? "ok" : "weak", lk ? "YES" : " . ");
    }
    std::printf("\n%d satellites lock with fine-Doppler refinement.\n", lockCount);
    return locked ? 0 : 1;
}
