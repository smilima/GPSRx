//---------------------------------------------------------------------------
// GpuAcquire.h - C-ABI surface of the GPU acquisition DLL (GpuAcquire.dll).
//
// This header is deliberately pure C (no C++, no VCL, no CUDA types) so it can
// be included by BOTH the CUDA implementation (compiled with nvcc/MSVC) and the
// C++Builder application (compiled with bcc64x) without dragging either
// toolchain into the other. The application talks to the DLL only through these
// functions and two plain structs.
//
// The structs mirror the relevant fields of gps::AcqConfig / gps::AcqResult so
// the shim (AcquisitionGpu.cpp) can translate 1:1.
//---------------------------------------------------------------------------
#ifndef GPU_ACQUIRE_H
#define GPU_ACQUIRE_H

#include <stddef.h>   // size_t

#ifdef __cplusplus
extern "C" {
#endif

#ifdef GPUACQ_BUILD_DLL
#  define GPUACQ_API __declspec(dllexport)
#else
#  define GPUACQ_API __declspec(dllimport)
#endif

// Search parameters (subset of gps::AcqConfig that the GPU search needs).
typedef struct GpuAcqConfig {
    double fs;           // sample rate (Hz)
    double ifFreq;       // intermediate frequency (Hz)
    double dopplerMin;   // Doppler search range (Hz)
    double dopplerMax;
    double dopplerStep;  // Doppler bin width (Hz)
    int    numMs;        // 1 ms records summed non-coherently
    double threshold;    // peak/second-peak ratio to declare a lock
} GpuAcqConfig;

// One PRN's outcome (mirrors gps::AcqResult; 'found' is 0/1 for C).
typedef struct GpuAcqResult {
    int    prn;
    int    found;
    double doppler;
    int    codePhaseSamp;
    double codePhaseChips;
    double peakRatio;
} GpuAcqResult;

// 1 if a usable CUDA (NVIDIA) device is present, else 0. Safe to call before
// create. (CUDA never enumerates an integrated Intel/AMD GPU, so a "yes" here
// always means an NVIDIA GPU.)
GPUACQ_API int   gpuAcqAvailable(void);

// Name of the NVIDIA GPU the search will use (the best discrete CUDA device),
// e.g. "NVIDIA GeForce RTX 4070 Laptop GPU". Empty string if none. Lets the
// caller display / verify which of a hybrid laptop's GPUs was chosen.
GPUACQ_API const char* gpuAcqDeviceName(void);

// Create a reusable acquisition context: selects the best NVIDIA device, binds
// to it (cudaSetDevice), allocates device buffers and FFT plans, and precomputes
// the per-bin carriers + all 32 conjugated code spectra (these depend only on
// 'cfg', not on the samples). Returns NULL on failure (see gpuAcqLastError).
GPUACQ_API void* gpuAcqCreate(const GpuAcqConfig* cfg);

// Run the PRN 1..32 search on 'signal' (>= numMs*round(fs/1000) int8 samples).
// Writes up to 'maxResults' (must be >= 32) entries to 'out'. Returns the
// number written (32) or a negative value on error.
GPUACQ_API int   gpuAcqRun(void* ctx, const signed char* signal, size_t signalLen,
                           GpuAcqResult* out, int maxResults);

// Release a context created by gpuAcqCreate.
GPUACQ_API void  gpuAcqDestroy(void* ctx);

// Human-readable description of the most recent failure (never NULL).
GPUACQ_API const char* gpuAcqLastError(void);

#ifdef __cplusplus
}
#endif

#endif // GPU_ACQUIRE_H
