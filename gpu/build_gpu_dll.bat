@echo off
REM ---------------------------------------------------------------------------
REM Build GpuAcquire.dll (CUDA + cuFFT acquisition).
REM
REM Prerequisites:
REM   * NVIDIA GPU + recent driver
REM   * CUDA Toolkit installed (provides nvcc + cuFFT), on PATH
REM   * Microsoft Visual C++ Build Tools - nvcc uses cl.exe as its host compiler.
REM     The simplest way to satisfy this is to run THIS script from a
REM     "x64 Native Tools Command Prompt for VS" so cl.exe is on PATH.
REM
REM Adjust -arch to your GPU's compute capability:
REM   sm_75 = Turing (RTX 20xx, GTX 16xx)   sm_86 = Ampere (RTX 30xx)
REM   sm_89 = Ada    (RTX 40xx)             sm_90 = Hopper (H100)
REM ---------------------------------------------------------------------------
setlocal
cd /d "%~dp0"

nvcc -O3 -std=c++17 -arch=sm_75 --shared ^
     GpuAcquire.cu ..\CACode.cpp ^
     -o GpuAcquire.dll -lcufft
if errorlevel 1 (
    echo.
    echo BUILD FAILED. Check that nvcc and cl.exe are both on PATH.
    exit /b 1
)

echo.
echo Built GpuAcquire.dll
echo Ship it next to GPSRx.exe along with the matching CUDA runtime DLLs
echo (cudart64_*.dll and cufft64_*.dll from the CUDA Toolkit 'bin' folder).
endlocal
