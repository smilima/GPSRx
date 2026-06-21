//---------------------------------------------------------------------------
// Acquisition.h - FFT-based parallel-code-phase GPS acquisition.
//
// For each PRN, the search:
//   1. wipes off the IF carrier (at IF + trial Doppler),
//   2. circularly correlates the result against the local C/A replica using
//      the FFT (one transform tests all 38192 code phases at once),
//   3. sweeps Doppler, and
//   4. accumulates several 1 ms records non-coherently for sensitivity.
//
// A satellite is declared "found" when the ratio of the strongest correlation
// peak to the next strongest peak (>= 1 chip away) exceeds a threshold.
//
// Pure standard C++ (no VCL).
//---------------------------------------------------------------------------
#ifndef AcquisitionH
#define AcquisitionH

#include <vector>
#include <cstdint>
#include <cstddef>
#include <functional>

namespace gps {

struct AcqConfig {
    double fs          = 38.192e6; // sample rate (Hz)
    double ifFreq      = 9.55e6;   // intermediate frequency (Hz)
    double dopplerMin  = -5000.0;  // Doppler search range (Hz)
    double dopplerMax  =  5000.0;
    double dopplerStep =  500.0;   // Doppler bin width (Hz)
    int    numMs       = 2;        // 1 ms records summed non-coherently
    double threshold   = 2.5;      // peak/second-peak ratio to declare a lock
};

struct AcqResult {
    int    prn            = 0;
    bool   found          = false;
    double doppler        = 0.0; // Hz (IF offset; +ve = satellite approaching)
    int    codePhaseSamp  = 0;   // peak position, samples into the code period
    double codePhaseChips = 0.0; // same, expressed in C/A chips (0..1023)
    double peakRatio      = 0.0; // metric compared against AcqConfig.threshold
};

// Acquire a single PRN from 'signal' (>= numMs * round(fs/1000) int8 samples).
AcqResult acquireOne(int prn, const int8_t* signal, std::size_t signalLen,
                     const AcqConfig& cfg);

// Full 2-D acquisition search surface for ONE PRN: the non-coherent correlation
// magnitude over every Doppler bin x code phase. This is the data behind the
// iconic "thumbtack" peak. The code-phase axis is down-sampled to 'nCols'
// columns (max within each column) so a present satellite's 1-chip-wide spike
// always survives. All magnitudes are normalised to the surface peak (0..1).
struct AcqSurface {
    int    prn   = 0;
    int    nBins = 0;                 // Doppler rows
    int    nCols = 0;                 // code-phase columns
    std::vector<double> dopplerHz;    // nBins  : Doppler of each row (Hz)
    std::vector<double> codeChips;    // nCols  : code phase of each column (chips)
    std::vector<float>  mag;          // nBins*nCols, row-major, normalised 0..1
    std::vector<float>  codeSlice;    // nCols  : the row through the peak Doppler
    std::vector<float>  dopplerSlice; // nBins  : the column through the peak code phase
    int    peakBin = 0, peakCol = 0;
    double peakDopplerHz = 0.0;       // Doppler at the peak (Hz)
    double peakCodeChip  = 0.0;       // code phase at the peak (chips)
    double peakRatio     = 0.0;       // peak / strongest competing column (same row)
};

// Compute the search surface for 'prn'. 'signal' must hold >= numMs 1 ms records
// starting at the capture origin (same buffer acquireAll uses).
AcqSurface acquireSurface(int prn, const int8_t* signal, std::size_t signalLen,
                          const AcqConfig& cfg, int nCols = 512);

// Acquire PRNs 1..32. 'progress' (optional) is called as each PRN completes.
std::vector<AcqResult> acquireAll(const int8_t* signal, std::size_t signalLen,
                                  const AcqConfig& cfg,
                                  std::function<void(const AcqResult&)> progress = nullptr);

// Refine the coarse acquisition Doppler with a fine 1-D search (non-coherent over
// numMs 1 ms blocks) so the result is within the carrier-loop pull-in range. 'sig'
// must start at the satellite's code phase (codePhaseSamp); returns the refined
// Doppler (Hz). This lets the Costas PLL lock satellites whose true Doppler sits
// far from a coarse 500 Hz bin centre.
double refineDoppler(const std::int8_t* sig, std::size_t len, int prn,
                     double coarseDoppler, const AcqConfig& cfg,
                     double rangeHz = 300.0, double stepHz = 25.0, int numMs = 10);

} // namespace gps

#endif
