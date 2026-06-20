//---------------------------------------------------------------------------
// AcqConsole.cpp - Standalone console harness to validate the GPS acquisition
// DSP core against the real sample file. NOT part of the VCL project; built
// directly with bcc64x. The DSP sources it links (Fft, CACode, Acquisition)
// are the exact same files the VCL application will use.
//
// Usage:
//   AcqConsole [file] [numMs] [dopplerStepHz] [threshold]
//---------------------------------------------------------------------------
#include "Fft.h"
#include "CACode.h"
#include "Acquisition.h"

#include <cstdio>
#include <cstdint>
#include <vector>
#include <complex>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <chrono>

using dsp::cd;

//---------------------------------------------------------------------------
// Self-tests
//---------------------------------------------------------------------------
static bool testCACode()
{
    // PRN 1, first 10 chips, must equal octal 1440 (IS-GPS-200 reference).
    std::vector<int8_t> c = gps::generateCACode(1);
    int val = 0;
    for (int i = 0; i < 10; ++i) {
        int bit = (c[i] == -1) ? 1 : 0;   // +1 -> logical 0, -1 -> logical 1
        val = (val << 1) | bit;
    }
    std::printf("  [CA ] PRN1 first 10 chips = octal %o (decimal %d), expected octal 1440\n",
                val, val);
    return val == 0x320; // 0o1440 == 800 == 0x320
}

static bool testFft()
{
    // Compare Bluestein forward against a brute-force DFT for a small odd N,
    // then check a forward/inverse round trip at the acquisition length.
    const int N = 13;
    std::vector<cd> x(N), X;
    for (int i = 0; i < N; ++i) x[i] = cd(std::sin(0.7 * i) + 0.3 * i, std::cos(0.2 * i));

    dsp::BluesteinFft plan(N);
    plan.forward(x, X);

    double maxErr = 0.0;
    const double TWO_PI = 6.283185307179586;
    for (int k = 0; k < N; ++k) {
        cd acc(0, 0);
        for (int n = 0; n < N; ++n) {
            double a = -TWO_PI * k * n / N;
            acc += x[n] * cd(std::cos(a), std::sin(a));
        }
        maxErr = std::max(maxErr, std::abs(acc - X[k]));
    }
    std::printf("  [FFT] Bluestein vs direct DFT (N=13): max error = %.3e\n", maxErr);

    // Round trip at acquisition length.
    const int M = 38192;
    std::vector<cd> y(M), Y, z;
    for (int i = 0; i < M; ++i) y[i] = cd((i * 7 % 11) - 5, (i * 3 % 5) - 2);
    dsp::BluesteinFft plan2(M);
    plan2.forward(y, Y);
    plan2.inverse(Y, z);
    double rt = 0.0;
    for (int i = 0; i < M; ++i) rt = std::max(rt, std::abs(z[i] - y[i]));
    std::printf("  [FFT] forward/inverse round trip (N=38192): max error = %.3e\n", rt);

    return maxErr < 1e-9 && rt < 1e-7;
}

//---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    std::string file = (argc > 1) ? argv[1]
                                  : "GPSdata-DiscreteComponents-fs38_192-if9_55.bin";
    gps::AcqConfig cfg;                // defaults: fs=38.192M, IF=9.55M
    cfg.numMs       = (argc > 2) ? std::atoi(argv[2]) : 2;
    cfg.dopplerStep = (argc > 3) ? std::atof(argv[3]) : 500.0;
    cfg.threshold   = (argc > 4) ? std::atof(argv[4]) : 2.5;

    std::printf("=== GPS acquisition self-test ===\n");
    bool okCA  = testCACode();
    bool okFft = testFft();
    std::printf("  CA generator: %s   FFT: %s\n\n",
                okCA ? "PASS" : "FAIL", okFft ? "PASS" : "FAIL");
    if (!okCA || !okFft) {
        std::printf("Self-tests failed; aborting before acquisition.\n");
        return 1;
    }

    // Read the first numMs milliseconds of samples.
    const int n = (int)std::lround(cfg.fs * 1.0e-3);
    const std::size_t need = (std::size_t)n * cfg.numMs;
    std::vector<int8_t> sig(need);
    {
        std::ifstream f(file, std::ios::binary);
        if (!f) { std::printf("ERROR: cannot open %s\n", file.c_str()); return 2; }
        f.read(reinterpret_cast<char*>(sig.data()), (std::streamsize)need);
        if ((std::size_t)f.gcount() < need) {
            std::printf("ERROR: short read (%lld of %zu bytes)\n",
                        (long long)f.gcount(), need);
            return 3;
        }
    }
    std::printf("File: %s\n", file.c_str());
    std::printf("Config: fs=%.4f MHz  IF=%.4f MHz  Doppler=[%.0f..%.0f] step %.0f Hz  %d ms  thr=%.2f\n\n",
                cfg.fs / 1e6, cfg.ifFreq / 1e6, cfg.dopplerMin, cfg.dopplerMax,
                cfg.dopplerStep, cfg.numMs, cfg.threshold);

    std::printf("Searching PRN 1..32 ...\n");
    std::printf("PRN   found   Doppler(Hz)   codePhase(samp)   codePhase(chip)   peak/2nd\n");
    std::printf("----------------------------------------------------------------------------\n");

    auto t0 = std::chrono::steady_clock::now();
    auto results = gps::acquireAll(sig.data(), sig.size(), cfg,
        [](const gps::AcqResult& r) {
            std::printf("%3d   %-5s   %+8.0f      %8d          %8.2f         %7.2f\n",
                        r.prn, r.found ? "YES" : " . ", r.doppler,
                        r.codePhaseSamp, r.codePhaseChips, r.peakRatio);
            std::fflush(stdout);
        });
    auto t1 = std::chrono::steady_clock::now();
    double secs = std::chrono::duration<double>(t1 - t0).count();

    // Summary, sorted by descending peak ratio.
    std::sort(results.begin(), results.end(),
              [](const gps::AcqResult& a, const gps::AcqResult& b) {
                  return a.peakRatio > b.peakRatio;
              });
    int found = 0;
    std::printf("\n=== Acquired satellites (ratio >= %.2f) ===\n", cfg.threshold);
    for (const auto& r : results) {
        if (!r.found) continue;
        ++found;
        std::printf("  PRN %2d : Doppler %+6.0f Hz, code phase %.1f chips, ratio %.2f\n",
                    r.prn, r.doppler, r.codePhaseChips, r.peakRatio);
    }
    std::printf("\n%d satellites acquired in %.1f s.\n", found, secs);
    return 0;
}
