//---------------------------------------------------------------------------
// Acquisition.cpp - see Acquisition.h
//---------------------------------------------------------------------------
#include "Acquisition.h"
#include "CACode.h"
#include "Fft.h"

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace gps {

using dsp::cd;
static const double TWO_PI = 6.283185307179586476925286766559;

namespace {

// Number of samples in one 1 ms C/A code period at fs.
int samplesPerCode(double fs)
{
    return (int)std::lround(fs * 1.0e-3);
}

// Split the int8 input into 'numMs' real-valued, mean-removed 1 ms blocks.
std::vector<std::vector<double>> makeBlocks(const int8_t* sig, std::size_t len,
                                            int n, int numMs)
{
    std::vector<std::vector<double>> blocks;
    // Overall mean across everything we will use (removes the small DC bias).
    std::size_t need = (std::size_t)n * numMs;
    if (need > len) throw std::runtime_error("acquire: not enough samples in buffer");

    double mean = 0.0;
    for (std::size_t i = 0; i < need; ++i) mean += sig[i];
    mean /= (double)need;

    blocks.resize(numMs);
    for (int m = 0; m < numMs; ++m) {
        blocks[m].resize(n);
        const int8_t* p = sig + (std::size_t)m * n;
        for (int i = 0; i < n; ++i)
            blocks[m][i] = (double)p[i] - mean;
    }
    return blocks;
}

// Precompute the complex carrier for each Doppler bin: exp(-j2pi(IF+fd)n/fs).
std::vector<std::vector<cd>> makeCarriers(const AcqConfig& cfg, int n, int nBins)
{
    std::vector<std::vector<cd>> carriers(nBins);
    for (int b = 0; b < nBins; ++b) {
        double fd = cfg.dopplerMin + b * cfg.dopplerStep;
        double w  = TWO_PI * (cfg.ifFreq + fd) / cfg.fs;
        carriers[b].resize(n);
        // Incremental phasor rotation avoids n sin/cos calls per bin.
        cd rot(std::cos(-w), std::sin(-w));
        cd cur(1.0, 0.0);
        for (int i = 0; i < n; ++i) {
            carriers[b][i] = cur;
            cur *= rot;
        }
    }
    return carriers;
}

// Forward-transform the carrier-wiped baseband for every (Doppler bin, ms
// block). This is independent of PRN, so it is computed once and reused for
// all 32 satellites.
std::vector<std::vector<cd>> makeBasebandSpectra(
        const std::vector<std::vector<double>>& blocks,
        const std::vector<std::vector<cd>>& carriers,
        const dsp::BluesteinFft& plan, int n, int numMs, int nBins)
{
    std::vector<std::vector<cd>> spectra((std::size_t)nBins * numMs);
    std::vector<cd> bb(n), B;
    for (int b = 0; b < nBins; ++b) {
        for (int m = 0; m < numMs; ++m) {
            for (int i = 0; i < n; ++i)
                bb[i] = carriers[b][i] * blocks[m][i];
            plan.forward(bb, B);
            spectra[(std::size_t)b * numMs + m] = B;
        }
    }
    return spectra;
}

AcqResult acquirePrnImpl(int prn,
                         const std::vector<std::vector<cd>>& basebandSpectra,
                         const dsp::BluesteinFft& plan, int n, int numMs,
                         int nBins, const AcqConfig& cfg)
{
    // Local replica spectrum, conjugated for correlation.
    std::vector<int8_t> code = generateCACode(prn);
    std::vector<double> codeSamp = sampleCACode(code, cfg.fs, n);
    std::vector<cd> codeC(n), C;
    for (int i = 0; i < n; ++i) codeC[i] = cd(codeSamp[i], 0.0);
    plan.forward(codeC, C);
    for (int i = 0; i < n; ++i) C[i] = std::conj(C[i]);

    std::vector<cd> prod(n), corr;
    std::vector<double> acc(n), bestAcc;
    double bestPeak = -1.0;
    int    bestBin = 0, bestPhase = 0;

    for (int b = 0; b < nBins; ++b) {
        std::fill(acc.begin(), acc.end(), 0.0);
        for (int m = 0; m < numMs; ++m) {
            const std::vector<cd>& B = basebandSpectra[(std::size_t)b * numMs + m];
            for (int i = 0; i < n; ++i) prod[i] = B[i] * C[i];
            plan.inverse(prod, corr);
            for (int i = 0; i < n; ++i)
                acc[i] += std::norm(corr[i]); // |corr|^2
        }
        // Strongest code phase in this Doppler bin.
        int idx = (int)(std::max_element(acc.begin(), acc.end()) - acc.begin());
        if (acc[idx] > bestPeak) {
            bestPeak  = acc[idx];
            bestBin   = b;
            bestPhase = idx;
            bestAcc   = acc;
        }
    }

    // Second-highest peak in the winning Doppler bin, excluding a +/-1 chip
    // guard band around the main peak (circular).
    const double sampPerChip = cfg.fs / CA_CHIP_RATE;
    const int guard = (int)std::ceil(sampPerChip);
    double second = 0.0;
    for (int i = 0; i < n; ++i) {
        int d = std::abs(i - bestPhase);
        if (d > n / 2) d = n - d;          // circular distance
        if (d <= guard) continue;
        if (bestAcc[i] > second) second = bestAcc[i];
    }

    AcqResult r;
    r.prn            = prn;
    r.doppler        = cfg.dopplerMin + bestBin * cfg.dopplerStep;
    r.codePhaseSamp  = bestPhase;
    r.codePhaseChips = bestPhase / sampPerChip;
    r.peakRatio      = (second > 0.0) ? (bestPeak / second) : 0.0;
    r.found          = r.peakRatio >= cfg.threshold;
    return r;
}

} // namespace

AcqResult acquireOne(int prn, const int8_t* signal, std::size_t signalLen,
                     const AcqConfig& cfg)
{
    const int n = samplesPerCode(cfg.fs);
    const int nBins = (int)std::lround((cfg.dopplerMax - cfg.dopplerMin) / cfg.dopplerStep) + 1;

    dsp::BluesteinFft plan(n);
    auto blocks   = makeBlocks(signal, signalLen, n, cfg.numMs);
    auto carriers = makeCarriers(cfg, n, nBins);
    auto spectra  = makeBasebandSpectra(blocks, carriers, plan, n, cfg.numMs, nBins);
    return acquirePrnImpl(prn, spectra, plan, n, cfg.numMs, nBins, cfg);
}

AcqSurface acquireSurface(int prn, const int8_t* signal, std::size_t signalLen,
                          const AcqConfig& cfg, int nCols)
{
    const int n = samplesPerCode(cfg.fs);
    const int nBins = (int)std::lround((cfg.dopplerMax - cfg.dopplerMin) / cfg.dopplerStep) + 1;
    if (nCols < 1)  nCols = 1;
    if (nCols > n)  nCols = n;

    dsp::BluesteinFft plan(n);
    auto blocks   = makeBlocks(signal, signalLen, n, cfg.numMs);
    auto carriers = makeCarriers(cfg, n, nBins);
    auto spectra  = makeBasebandSpectra(blocks, carriers, plan, n, cfg.numMs, nBins);

    // Conjugated replica spectrum for the chosen PRN.
    std::vector<int8_t> code = generateCACode(prn);
    std::vector<double> codeSamp = sampleCACode(code, cfg.fs, n);
    std::vector<cd> codeC(n), C;
    for (int i = 0; i < n; ++i) codeC[i] = cd(codeSamp[i], 0.0);
    plan.forward(codeC, C);
    for (int i = 0; i < n; ++i) C[i] = std::conj(C[i]);

    const double sampPerChip = cfg.fs / CA_CHIP_RATE;

    AcqSurface s;
    s.prn = prn; s.nBins = nBins; s.nCols = nCols;
    s.dopplerHz.resize(nBins);
    s.codeChips.resize(nCols);
    s.mag.assign((std::size_t)nBins * nCols, 0.0f);
    for (int c = 0; c < nCols; ++c)
        s.codeChips[c] = ((double)c / nCols) * ((double)n / sampPerChip); // 0..1023

    std::vector<cd> prod(n), corr;
    std::vector<double> acc(n);
    std::vector<int>    binPeakIdx(nBins, 0);
    double globalPeak = -1.0;
    int    peakBin = 0, peakPhase = 0;

    for (int b = 0; b < nBins; ++b) {
        s.dopplerHz[b] = cfg.dopplerMin + b * cfg.dopplerStep;
        std::fill(acc.begin(), acc.end(), 0.0);
        for (int m = 0; m < cfg.numMs; ++m) {
            const std::vector<cd>& B = spectra[(std::size_t)b * cfg.numMs + m];
            for (int i = 0; i < n; ++i) prod[i] = B[i] * C[i];
            plan.inverse(prod, corr);
            for (int i = 0; i < n; ++i) acc[i] += std::norm(corr[i]);
        }
        int idxPeak = 0; double valPeak = -1.0;
        float* row = &s.mag[(std::size_t)b * nCols];
        for (int i = 0; i < n; ++i) {
            int c = (int)((long long)i * nCols / n);
            if (c >= nCols) c = nCols - 1;
            if ((float)acc[i] > row[c]) row[c] = (float)acc[i];   // max-pool into column
            if (acc[i] > valPeak) { valPeak = acc[i]; idxPeak = i; }
        }
        binPeakIdx[b] = idxPeak;
        if (valPeak > globalPeak) { globalPeak = valPeak; peakBin = b; peakPhase = idxPeak; }
    }

    s.peakBin       = peakBin;
    s.peakCol       = (int)((long long)peakPhase * nCols / n);
    s.peakDopplerHz = cfg.dopplerMin + peakBin * cfg.dopplerStep;
    s.peakCodeChip  = peakPhase / sampPerChip;

    // Peak/second-peak ratio along the winning Doppler row (code domain), with a
    // small guard either side of the peak column.
    {
        const float* row = &s.mag[(std::size_t)peakBin * nCols];
        const int guard = std::max(1, (int)std::ceil(sampPerChip * nCols / n));
        double second = 0.0;
        for (int c = 0; c < nCols; ++c) {
            if (std::abs(c - s.peakCol) <= guard) continue;
            if (row[c] > second) second = row[c];
        }
        s.peakRatio = (second > 0.0) ? (globalPeak / second) : 0.0;
    }

    // Normalise the surface and pull the two slices through the peak.
    const float inv = (globalPeak > 0.0) ? (float)(1.0 / globalPeak) : 1.0f;
    for (std::size_t i = 0; i < s.mag.size(); ++i) s.mag[i] *= inv;
    s.codeSlice.resize(nCols);
    for (int c = 0; c < nCols; ++c) s.codeSlice[c] = s.mag[(std::size_t)peakBin * nCols + c];
    s.dopplerSlice.resize(nBins);
    for (int b = 0; b < nBins; ++b) s.dopplerSlice[b] = s.mag[(std::size_t)b * nCols + s.peakCol];
    return s;
}

double refineDoppler(const std::int8_t* sig, std::size_t len, int prn,
                     double coarseDoppler, const AcqConfig& cfg,
                     double rangeHz, double stepHz, int numMs)
{
    const int n = (int)std::lround(cfg.fs * 1.0e-3);
    std::vector<int8_t>  ca   = generateCACode(prn);
    std::vector<double>  code = sampleCACode(ca, cfg.fs, n);   // +/-1, code-aligned at sig[0]

    if ((std::size_t)numMs * n > len) numMs = (int)(len / n);
    if (numMs < 2) return coarseDoppler;

    double bestPow = -1.0, bestF = coarseDoppler;
    for (double f = coarseDoppler - rangeHz; f <= coarseDoppler + rangeHz + 1e-6; f += stepHz) {
        const double w = TWO_PI * (cfg.ifFreq + f) / cfg.fs;
        const double rc = std::cos(-w), rs = std::sin(-w);
        double pow = 0.0;
        for (int k = 0; k < numMs; ++k) {              // non-coherent sum over 1 ms blocks
            const int8_t* p = sig + (std::size_t)k * n;
            double cs = 1.0, sn = 0.0, I = 0.0, Q = 0.0;
            for (int i = 0; i < n; ++i) {
                const double r = (double)p[i] * code[i];
                I += cs * r; Q += sn * r;
                const double ncs = cs * rc - sn * rs;
                sn = sn * rc + cs * rs;
                cs = ncs;
            }
            pow += I * I + Q * Q;
        }
        if (pow > bestPow) { bestPow = pow; bestF = f; }
    }
    return bestF;
}

std::vector<AcqResult> acquireAll(const int8_t* signal, std::size_t signalLen,
                                  const AcqConfig& cfg,
                                  std::function<void(const AcqResult&)> progress)
{
    const int n = samplesPerCode(cfg.fs);
    const int nBins = (int)std::lround((cfg.dopplerMax - cfg.dopplerMin) / cfg.dopplerStep) + 1;

    dsp::BluesteinFft plan(n);
    auto blocks   = makeBlocks(signal, signalLen, n, cfg.numMs);
    auto carriers = makeCarriers(cfg, n, nBins);
    auto spectra  = makeBasebandSpectra(blocks, carriers, plan, n, cfg.numMs, nBins);

    std::vector<AcqResult> results;
    results.reserve(32);
    for (int prn = 1; prn <= 32; ++prn) {
        AcqResult r = acquirePrnImpl(prn, spectra, plan, n, cfg.numMs, nBins, cfg);
        results.push_back(r);
        if (progress) progress(r);
    }
    return results;
}

} // namespace gps
