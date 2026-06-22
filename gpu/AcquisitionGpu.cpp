//---------------------------------------------------------------------------
// AcquisitionGpu.cpp - see AcquisitionGpu.h.
//
// Loads GpuAcquire.dll at runtime (once, cached) and bridges gps::AcqConfig /
// gps::AcqResult <-> the DLL's C structs. Pure Win32 + standard C++ so it builds
// with bcc64x in the C++Builder project (no CUDA headers required here).
//---------------------------------------------------------------------------
#include "AcquisitionGpu.h"
#include "GpuAcquire.h"

#include <windows.h>

namespace {

// Function-pointer types matching the DLL's C ABI.
typedef int          (*FnAvail)  (void);
typedef const char*  (*FnName)   (void);
typedef void*        (*FnCreate) (const GpuAcqConfig*);
typedef int          (*FnRun)    (void*, const signed char*, size_t, GpuAcqResult*, int);
typedef void         (*FnDestroy)(void*);

// Lazy, cached DLL binding. Constructed on first use; never unloaded (process
// lifetime). Loading is cheap and failure is silent (we just fall back to CPU).
struct GpuDll {
    HMODULE   h       = nullptr;
    FnAvail   avail   = nullptr;
    FnName    name    = nullptr;
    FnCreate  create  = nullptr;
    FnRun     run     = nullptr;
    FnDestroy destroy = nullptr;
    bool      ok      = false;

    GpuDll() {
        h = ::LoadLibraryA("GpuAcquire.dll");
        if (!h) return;
        avail   = reinterpret_cast<FnAvail>  (::GetProcAddress(h, "gpuAcqAvailable"));
        name    = reinterpret_cast<FnName>   (::GetProcAddress(h, "gpuAcqDeviceName"));
        create  = reinterpret_cast<FnCreate> (::GetProcAddress(h, "gpuAcqCreate"));
        run     = reinterpret_cast<FnRun>    (::GetProcAddress(h, "gpuAcqRun"));
        destroy = reinterpret_cast<FnDestroy>(::GetProcAddress(h, "gpuAcqDestroy"));
        ok = avail && create && run && destroy;   // 'name' is optional
    }
};

const GpuDll& dll() { static GpuDll d; return d; }

} // namespace

namespace gps {

bool gpuAcqIsAvailable()
{
    const GpuDll& d = dll();
    return d.ok && d.avail() != 0;
}

std::string gpuAcqDeviceName()
{
    const GpuDll& d = dll();
    if (!d.ok || !d.name) return std::string();
    const char* s = d.name();
    return s ? std::string(s) : std::string();
}

bool acquireAllGpu(const std::int8_t* signal, std::size_t signalLen,
                   const AcqConfig& cfg, std::vector<AcqResult>& out)
{
    const GpuDll& d = dll();
    if (!d.ok || d.avail() == 0) return false;

    GpuAcqConfig gc;
    gc.fs          = cfg.fs;
    gc.ifFreq      = cfg.ifFreq;
    gc.dopplerMin  = cfg.dopplerMin;
    gc.dopplerMax  = cfg.dopplerMax;
    gc.dopplerStep = cfg.dopplerStep;
    gc.numMs       = cfg.numMs;
    gc.threshold   = cfg.threshold;

    void* ctx = d.create(&gc);
    if (!ctx) return false;

    GpuAcqResult res[32];
    const int got = d.run(ctx, reinterpret_cast<const signed char*>(signal),
                          signalLen, res, 32);
    d.destroy(ctx);
    if (got <= 0) return false;

    out.clear();
    out.reserve((std::size_t)got);
    for (int i = 0; i < got; ++i) {
        AcqResult r;
        r.prn            = res[i].prn;
        r.found          = (res[i].found != 0);
        r.doppler        = res[i].doppler;
        r.codePhaseSamp  = res[i].codePhaseSamp;
        r.codePhaseChips = res[i].codePhaseChips;
        r.peakRatio      = res[i].peakRatio;
        out.push_back(r);
    }
    return true;
}

std::vector<AcqResult> acquireAllAuto(const std::int8_t* signal, std::size_t signalLen,
                   const AcqConfig& cfg, std::function<void(const AcqResult&)> progress)
{
    std::vector<AcqResult> out;
    if (acquireAllGpu(signal, signalLen, cfg, out)) {
        if (progress)
            for (std::size_t i = 0; i < out.size(); ++i) progress(out[i]);
        return out;
    }
    // No GPU: identical behaviour to a direct acquireAll call.
    return acquireAll(signal, signalLen, cfg, progress);
}

} // namespace gps
