//---------------------------------------------------------------------------
// GpuAcquire.cu - CUDA + cuFFT implementation of GPS parallel-code-phase
// acquisition, exposed as the C-ABI DLL declared in GpuAcquire.h.
//
// It is a faithful GPU port of gps::acquireAll (../Acquisition.cpp). The mapping
// is 1:1:
//
//   CPU (Acquisition.cpp)                 GPU (this file)
//   --------------------------------------------------------------------------
//   makeBlocks (DC removal)               host mean + subtract in kBaseband
//   makeCarriers (per Doppler bin)        d_carr, precomputed once in buildCtx
//   makeBasebandSpectra (carrier*block,   kBaseband + one batched forward FFT
//     forward FFT)                          over nBins*numMs transforms (once)
//   per-PRN conj(FFT(code))               d_code, precomputed once in buildCtx
//   prod = B * Cconj                      kMul
//   inverse FFT -> corr                   batched inverse FFT (cuFFT, unscaled)
//   acc += |corr|^2 over ms               kAccum
//   argmax + second-peak/guard ratio      host peak pick (acc is only ~3 MB/PRN)
//
// GPU selection on a hybrid (Optimus) laptop: CUDA only ever enumerates NVIDIA
// GPUs, so the Intel/AMD integrated GPU is never a candidate. pickBestDevice()
// additionally prefers a *discrete* NVIDIA GPU with the highest capability and
// binds to it with cudaSetDevice, and gpuAcqDeviceName() reports the choice.
//
// Two notes on numerical equivalence:
//   * cuFFT's inverse transform is UNSCALED (no 1/N), whereas the CPU divides by
//     N. That makes every GPU correlation N x the CPU value, so |corr|^2 is N^2 x
//     and the accumulator is N^2 x - but the peak/second-peak RATIO and the
//     argmax are unchanged, so found/doppler/codePhase are identical. We skip the
//     scaling deliberately (one less kernel).
//   * This prototype uses single precision (cufftComplex). The detection metric
//     is a ratio of peaks over int8 data, so FP32 is ample; switch the buffers to
//     cufftDoubleComplex / Z2Z if you want to bit-match the double-precision CPU.
//---------------------------------------------------------------------------
#define GPUACQ_BUILD_DLL 1
#include "GpuAcquire.h"
#include "../CACode.h"          // gps::generateCACode, gps::sampleCACode, CA_CHIP_RATE

#include <cuda_runtime.h>
#include <cufft.h>

#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstdint>

static std::string g_err     = "ok";
static std::string g_devName = "";     // name of the selected NVIDIA GPU
extern "C" GPUACQ_API const char* gpuAcqLastError(void) { return g_err.c_str(); }

// Hybrid-graphics hint: ask the NVIDIA (and AMD) driver to prefer the
// high-performance discrete GPU for this module. This flag mainly governs
// OpenGL/display routing and is most effective when exported from the main .exe;
// it is harmless here. What actually pins our compute to the NVIDIA GPU is CUDA
// device selection (pickBestDevice + cudaSetDevice) - CUDA never enumerates the
// Intel/AMD integrated GPU at all.
extern "C" {
    __declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
    __declspec(dllexport) int           AmdPowerXpressRequestHighPerformance = 1;
}

#define CUDA_CHECK(call) do { cudaError_t _e = (call); if (_e != cudaSuccess) { \
    g_err = std::string("CUDA: ") + cudaGetErrorString(_e); return false; } } while (0)
#define CUFFT_CHECK(call) do { cufftResult _r = (call); if (_r != CUFFT_SUCCESS) { \
    char _b[64]; std::snprintf(_b, sizeof(_b), "cuFFT error %d", (int)_r); g_err = _b; return false; } } while (0)

static const double TWO_PI = 6.283185307179586476925286766559;
static const int    TPB    = 256;   // threads per block

//=========================================================================
// Kernels
//=========================================================================

// Baseband: bb[(bin,m),i] = carrier[bin,i] * (sig[m,i] - mean).
// One thread per output element; total = nBins*numMs*n.
__global__ void kBaseband(const signed char* sig, const cufftComplex* carr,
                          cufftComplex* bb, int n, int numMs, long long total, float mean)
{
    long long g = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (g >= total) return;
    const int i  = (int)(g % n);
    const int bm = (int)(g / n);
    const int bin = bm / numMs;
    const int m   = bm % numMs;
    const float s = (float)sig[(long long)m * n + i] - mean;
    const cufftComplex c = carr[(long long)bin * n + i];
    bb[g].x = c.x * s;
    bb[g].y = c.y * s;
}

// In-place complex conjugate (used to turn FFT(code) into conj(FFT(code))).
__global__ void kConj(cufftComplex* a, long long total)
{
    long long g = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (g >= total) return;
    a[g].y = -a[g].y;
}

// prod[(bin,m),i] = B[(bin,m),i] * code[i]  (code spectrum already conjugated).
__global__ void kMul(const cufftComplex* B, const cufftComplex* code,
                     cufftComplex* prod, int n, long long total)
{
    long long g = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (g >= total) return;
    const int i = (int)(g % n);
    const cufftComplex b = B[g], c = code[i];
    prod[g].x = b.x * c.x - b.y * c.y;
    prod[g].y = b.x * c.y + b.y * c.x;
}

// acc[bin,i] = sum over the numMs blocks of |prod[(bin,m),i]|^2.
// One thread per (bin,i); total = nBins*n.
__global__ void kAccum(const cufftComplex* prod, float* acc,
                       int n, int numMs, long long total)
{
    long long g = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (g >= total) return;
    const int i   = (int)(g % n);
    const int bin = (int)(g / n);
    float s = 0.0f;
    for (int m = 0; m < numMs; ++m) {
        const cufftComplex c = prod[((long long)bin * numMs + m) * n + i];
        s += c.x * c.x + c.y * c.y;
    }
    acc[g] = s;
}

static inline int gridFor(long long total) { return (int)((total + TPB - 1) / TPB); }

//=========================================================================
// Device selection (prefer the discrete NVIDIA GPU on a hybrid laptop)
//=========================================================================
// CUDA only ever enumerates NVIDIA GPUs, so a laptop's Intel/AMD integrated GPU
// is never a candidate here. Among NVIDIA GPUs we prefer a discrete one
// (integrated == 0), then the highest compute capability, then the most SMs,
// then the most memory. Returns the chosen device index (or -1) and its name.
static int pickBestDevice(std::string& nameOut)
{
    int cnt = 0;
    if (cudaGetDeviceCount(&cnt) != cudaSuccess || cnt < 1) return -1;

    int    bestDev = -1, bestCap = -1, bestSm = -1;
    bool   bestDiscrete = false;
    size_t bestMem = 0;

    for (int d = 0; d < cnt; ++d) {
        cudaDeviceProp p;
        if (cudaGetDeviceProperties(&p, d) != cudaSuccess) continue;
        if (p.computeMode == cudaComputeModeProhibited)    continue;   // unusable
        if (p.major < 1)                                   continue;   // no real CUDA

        const bool   discrete = (p.integrated == 0);
        const int    cap = p.major * 10 + p.minor;
        const int    sm  = p.multiProcessorCount;
        const size_t mem = p.totalGlobalMem;

        // Lexicographic preference: discrete > capability > SM count > memory.
        bool better;
        if      (bestDev < 0)               better = true;
        else if (discrete != bestDiscrete)  better = discrete;          // prefer discrete
        else if (cap != bestCap)            better = (cap > bestCap);
        else if (sm  != bestSm)             better = (sm  > bestSm);
        else                                better = (mem > bestMem);

        if (better) {
            bestDev = d; bestDiscrete = discrete; bestCap = cap; bestSm = sm; bestMem = mem;
            nameOut = p.name;
        }
    }
    return bestDev;
}

//=========================================================================
// Context
//=========================================================================
struct GpuAcqCtx {
    GpuAcqConfig cfg;
    int dev = 0;                         // selected CUDA (NVIDIA) device index
    int n = 0, nBins = 0, numMs = 0;
    long long batch = 0;                 // nBins * numMs

    signed char*  d_sig  = nullptr;      // numMs * n
    cufftComplex* d_carr = nullptr;      // nBins * n
    cufftComplex* d_bb   = nullptr;      // batch * n  (forward spectra B)
    cufftComplex* d_prod = nullptr;      // batch * n
    cufftComplex* d_code = nullptr;      // 32 * n     (conjugated code spectra)
    float*        d_acc  = nullptr;      // nBins * n

    cufftHandle   plan = 0;              // length n, batch = nBins*numMs, C2C
    std::vector<float> h_acc;
};

static void freeCtx(GpuAcqCtx* c)
{
    if (!c) return;
    if (c->plan) cufftDestroy(c->plan);
    cudaFree(c->d_sig);  cudaFree(c->d_carr); cudaFree(c->d_bb);
    cudaFree(c->d_prod); cudaFree(c->d_code); cudaFree(c->d_acc);
    delete c;
}

// Allocate buffers, precompute carriers and the 32 conjugated code spectra, and
// build the batched FFT plan. Everything here depends only on cfg.
static bool buildCtx(GpuAcqCtx* c)
{
    const GpuAcqConfig& cfg = c->cfg;
    const int n     = (int)std::lround(cfg.fs * 1.0e-3);
    const int nBins = (int)std::lround((cfg.dopplerMax - cfg.dopplerMin) / cfg.dopplerStep) + 1;
    const int numMs = cfg.numMs;
    if (n < 1 || nBins < 1 || numMs < 1) { g_err = "bad config"; return false; }
    c->n = n; c->nBins = nBins; c->numMs = numMs;
    c->batch = (long long)nBins * numMs;

    CUDA_CHECK(cudaMalloc(&c->d_sig,  (size_t)numMs * n * sizeof(signed char)));
    CUDA_CHECK(cudaMalloc(&c->d_carr, (size_t)nBins * n * sizeof(cufftComplex)));
    CUDA_CHECK(cudaMalloc(&c->d_bb,   (size_t)c->batch * n * sizeof(cufftComplex)));
    CUDA_CHECK(cudaMalloc(&c->d_prod, (size_t)c->batch * n * sizeof(cufftComplex)));
    CUDA_CHECK(cudaMalloc(&c->d_code, (size_t)32 * n * sizeof(cufftComplex)));
    CUDA_CHECK(cudaMalloc(&c->d_acc,  (size_t)nBins * n * sizeof(float)));
    c->h_acc.resize((size_t)nBins * n);

    // Carriers exp(-j 2pi (IF+fd) i / fs) by incremental rotation (matches the
    // CPU's makeCarriers). Phasor math in double, stored as float.
    std::vector<cufftComplex> hc((size_t)nBins * n);
    for (int b = 0; b < nBins; ++b) {
        const double fd = cfg.dopplerMin + b * cfg.dopplerStep;
        const double w  = TWO_PI * (cfg.ifFreq + fd) / cfg.fs;
        const double cr = std::cos(-w), ci = std::sin(-w);
        double xr = 1.0, xi = 0.0;
        for (int i = 0; i < n; ++i) {
            hc[(size_t)b * n + i].x = (float)xr;
            hc[(size_t)b * n + i].y = (float)xi;
            const double nr = xr * cr - xi * ci;
            xi = xr * ci + xi * cr;
            xr = nr;
        }
    }
    CUDA_CHECK(cudaMemcpy(c->d_carr, hc.data(),
               (size_t)nBins * n * sizeof(cufftComplex), cudaMemcpyHostToDevice));

    // 32 sampled C/A replicas -> upload -> forward FFT -> conjugate, in place.
    std::vector<cufftComplex> code((size_t)32 * n);
    for (int p = 1; p <= 32; ++p) {
        std::vector<std::int8_t> ca = gps::generateCACode(p);
        std::vector<double>      cs = gps::sampleCACode(ca, cfg.fs, n);
        for (int i = 0; i < n; ++i) {
            code[(size_t)(p - 1) * n + i].x = (float)cs[i];
            code[(size_t)(p - 1) * n + i].y = 0.0f;
        }
    }
    CUDA_CHECK(cudaMemcpy(c->d_code, code.data(),
               (size_t)32 * n * sizeof(cufftComplex), cudaMemcpyHostToDevice));
    {
        cufftHandle planCode;
        CUFFT_CHECK(cufftPlan1d(&planCode, n, CUFFT_C2C, 32));
        const cufftResult r = cufftExecC2C(planCode, c->d_code, c->d_code, CUFFT_FORWARD);
        cufftDestroy(planCode);
        if (r != CUFFT_SUCCESS) { g_err = "cuFFT code forward failed"; return false; }
        const long long tot = (long long)32 * n;
        kConj<<<gridFor(tot), TPB>>>(c->d_code, tot);
        CUDA_CHECK(cudaGetLastError());
    }

    // One batched plan reused for the forward baseband transform and every
    // per-PRN inverse transform (same length, same batch count).
    CUFFT_CHECK(cufftPlan1d(&c->plan, n, CUFFT_C2C, (int)c->batch));
    return true;
}

// The per-run search. Heavy work on the GPU; the small peak pick on the host.
static bool runImpl(GpuAcqCtx* c, const signed char* sig, size_t len,
                    GpuAcqResult* out, int maxResults)
{
    CUDA_CHECK(cudaSetDevice(c->dev));   // pin this thread to the chosen NVIDIA GPU

    const int n = c->n, nBins = c->nBins, numMs = c->numMs;
    const size_t need = (size_t)numMs * n;
    if (len < need)      { g_err = "signal too short"; return false; }
    if (maxResults < 32) { g_err = "out buffer < 32"; return false; }

    // DC mean over exactly the samples we use (matches makeBlocks).
    double mean = 0.0;
    for (size_t i = 0; i < need; ++i) mean += sig[i];
    mean /= (double)need;

    CUDA_CHECK(cudaMemcpy(c->d_sig, sig, need * sizeof(signed char), cudaMemcpyHostToDevice));

    const long long tBB  = c->batch * n;          // baseband / product elements
    const long long tAcc = (long long)nBins * n;

    // Baseband (carrier wipe-off) + one batched forward FFT -> B. Done ONCE and
    // reused for all 32 PRNs (the CPU does the same).
    kBaseband<<<gridFor(tBB), TPB>>>(c->d_sig, c->d_carr, c->d_bb, n, numMs, tBB, (float)mean);
    CUDA_CHECK(cudaGetLastError());
    if (cufftExecC2C(c->plan, c->d_bb, c->d_bb, CUFFT_FORWARD) != CUFFT_SUCCESS)
        { g_err = "cuFFT baseband forward failed"; return false; }

    const double sampPerChip = c->cfg.fs / gps::CA_CHIP_RATE;
    const int    guard       = (int)std::ceil(sampPerChip);

    for (int p = 0; p < 32; ++p) {
        kMul<<<gridFor(tBB), TPB>>>(c->d_bb, c->d_code + (size_t)p * n, c->d_prod, n, tBB);
        CUDA_CHECK(cudaGetLastError());
        if (cufftExecC2C(c->plan, c->d_prod, c->d_prod, CUFFT_INVERSE) != CUFFT_SUCCESS)
            { g_err = "cuFFT inverse failed"; return false; }   // unscaled - cancels in the ratio
        kAccum<<<gridFor(tAcc), TPB>>>(c->d_prod, c->d_acc, n, numMs, tAcc);
        CUDA_CHECK(cudaGetLastError());
        CUDA_CHECK(cudaMemcpy(c->h_acc.data(), c->d_acc,
                   (size_t)nBins * n * sizeof(float), cudaMemcpyDeviceToHost));

        // Peak pick - identical logic to acquirePrnImpl (Acquisition.cpp).
        const float* acc = c->h_acc.data();
        double bestPeak = -1.0; int bestBin = 0, bestPhase = 0;
        for (int b = 0; b < nBins; ++b) {
            const float* row = acc + (size_t)b * n;
            int idx = 0; float mx = row[0];
            for (int i = 1; i < n; ++i) if (row[i] > mx) { mx = row[i]; idx = i; }
            if (mx > bestPeak) { bestPeak = mx; bestBin = b; bestPhase = idx; }
        }
        const float* bestRow = acc + (size_t)bestBin * n;
        double second = 0.0;
        for (int i = 0; i < n; ++i) {
            int d = std::abs(i - bestPhase);
            if (d > n / 2) d = n - d;             // circular distance
            if (d <= guard) continue;             // +/-1 chip guard band
            if (bestRow[i] > second) second = bestRow[i];
        }

        GpuAcqResult& r = out[p];
        r.prn            = p + 1;
        r.doppler        = c->cfg.dopplerMin + bestBin * c->cfg.dopplerStep;
        r.codePhaseSamp  = bestPhase;
        r.codePhaseChips = bestPhase / sampPerChip;
        r.peakRatio      = (second > 0.0) ? (bestPeak / second) : 0.0;
        r.found          = (r.peakRatio >= c->cfg.threshold) ? 1 : 0;
    }
    return true;
}

//=========================================================================
// C ABI
//=========================================================================
extern "C" GPUACQ_API int gpuAcqAvailable(void)
{
    std::string name;
    if (pickBestDevice(name) < 0) return 0;
    g_devName = name;
    return 1;
}

extern "C" GPUACQ_API const char* gpuAcqDeviceName(void)
{
    if (g_devName.empty()) { std::string n; if (pickBestDevice(n) >= 0) g_devName = n; }
    return g_devName.c_str();
}

extern "C" GPUACQ_API void* gpuAcqCreate(const GpuAcqConfig* cfg)
{
    if (!cfg) { g_err = "null config"; return nullptr; }
    std::string name;
    const int dev = pickBestDevice(name);
    if (dev < 0) { g_err = "no usable CUDA (NVIDIA) device"; return nullptr; }
    if (cudaSetDevice(dev) != cudaSuccess) { g_err = "cudaSetDevice failed"; return nullptr; }
    g_devName = name;

    GpuAcqCtx* c = new GpuAcqCtx();
    c->cfg = *cfg;
    c->dev = dev;
    if (!buildCtx(c)) { freeCtx(c); return nullptr; }
    return c;
}

extern "C" GPUACQ_API int gpuAcqRun(void* ctx, const signed char* signal, size_t signalLen,
                                    GpuAcqResult* out, int maxResults)
{
    GpuAcqCtx* c = (GpuAcqCtx*)ctx;
    if (!c || !signal || !out) { g_err = "null arg"; return -1; }
    if (!runImpl(c, signal, signalLen, out, maxResults)) return -1;
    return 32;
}

extern "C" GPUACQ_API void gpuAcqDestroy(void* ctx) { freeCtx((GpuAcqCtx*)ctx); }
