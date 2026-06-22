//---------------------------------------------------------------------------
// AcquisitionGpu.h - thin C++ shim that lets the receiver use the GPU
// acquisition DLL when it is present, and transparently falls back to the CPU
// path (gps::acquireAll) when it is not.
//
// The DLL is loaded lazily with LoadLibrary/GetProcAddress, so the application
// links against NOTHING CUDA-related: if GpuAcquire.dll (or a GPU/driver) is
// missing, acquireAllGpu simply returns false and acquireAllAuto runs on the
// CPU. This keeps the C++Builder build free of the CUDA toolchain.
//
// Drop-in usage: replace a call to
//     gps::acquireAll(sig, len, cfg, progress)
// with
//     gps::acquireAllAuto(sig, len, cfg, progress)
// and ship GpuAcquire.dll (+ cudart/cufft DLLs) next to the executable.
//---------------------------------------------------------------------------
#ifndef AcquisitionGpuH
#define AcquisitionGpuH

#include <vector>
#include <functional>
#include <string>
#include <cstdint>
#include <cstddef>
#include "../Acquisition.h"   // gps::AcqConfig, gps::AcqResult, gps::acquireAll

namespace gps {

// Run the PRN 1..32 search on the GPU. Returns true and fills 'out' on success;
// returns false (leaving 'out' untouched) if no GPU/DLL is available or the run
// fails - the caller should then use the CPU path.
bool acquireAllGpu(const std::int8_t* signal, std::size_t signalLen,
                   const AcqConfig& cfg, std::vector<AcqResult>& out);

// GPU if available, otherwise the CPU acquireAll. Always returns a result.
// 'progress' (optional) is invoked once per PRN result, matching acquireAll.
std::vector<AcqResult> acquireAllAuto(const std::int8_t* signal, std::size_t signalLen,
                   const AcqConfig& cfg,
                   std::function<void(const AcqResult&)> progress = nullptr);

// True if GpuAcquire.dll loaded and reports a usable NVIDIA device.
bool gpuAcqIsAvailable();

// Name of the NVIDIA GPU the GPU path will use (empty if none) - for display so
// you can confirm the discrete GPU was chosen on a hybrid laptop.
std::string gpuAcqDeviceName();

} // namespace gps

#endif
