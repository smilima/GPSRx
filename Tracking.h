//---------------------------------------------------------------------------
// Tracking.h - GPS L1 C/A carrier + code tracking (one channel per PRN).
//
// Seeded by acquisition (code phase + Doppler), each channel runs two coupled
// feedback loops, once per 1 ms integration period:
//   * Carrier loop - Costas PLL (atan(Q/I) discriminator, data-bit insensitive)
//     that follows carrier phase / Doppler.
//   * Code loop    - Delay-Locked Loop using Early/Prompt/Late correlators
//     (normalized power discriminator) that keeps the local C/A code aligned.
//
// When locked, the Prompt-I correlator output carries the 50 bps navigation
// data (its sign over each 20 ms = one data bit).
//
// Pure standard C++ (no VCL) - same source for the console harness and the app.
//---------------------------------------------------------------------------
#ifndef TrackingH
#define TrackingH

#include <vector>
#include <cstdint>
#include <cstddef>

namespace gps {

struct TrackConfig {
    double fs            = 38.192e6; // sample rate (Hz)
    double ifFreq        = 9.55e6;   // intermediate frequency (Hz)
    double pllBW         = 25.0;     // carrier (Costas) loop noise bandwidth (Hz)
    double pllZeta       = 0.7;      // carrier loop damping
    double dllBW         = 2.0;      // code (DLL) loop noise bandwidth (Hz)
    double dllZeta       = 0.7;      // code loop damping
    double dllCorrSpacing = 0.5;     // early-late spacing (chips, total)
    double pdi           = 1.0e-3;   // integration / predetection time (s)
};

// One integration period (1 ms) of tracking output.
struct TrackEpoch {
    double iP = 0, qP = 0;           // Prompt I/Q (nav bit = sign(iP) when locked)
    double iE = 0, qE = 0;           // Early  I/Q
    double iL = 0, qL = 0;           // Late   I/Q
    double doppler   = 0;            // carrier Doppler (Hz, = carrierFreq - IF)
    double codeFreq  = 0;            // code NCO frequency (chips/s)
    double remCodePhase = 0;        // residual code phase (chips)
    double pllDisc   = 0;            // carrier discriminator (cycles)
    double dllDisc   = 0;            // code discriminator (chips)
    double cn0       = 0;            // running C/N0 estimate (dB-Hz)
    std::size_t sampleIndex = 0;    // sample offset (within the run buffer) at the start of this 1 ms
};

class TrackChannel {
public:
    TrackChannel(int prn, double dopplerHz, const TrackConfig& cfg);

    // Track 'numMs' integration periods from a contiguous int8 stream that the
    // caller has aligned to the satellite's code phase (i.e. sig[0] is the start
    // of a code period). Returns one TrackEpoch per period actually processed
    // (fewer than numMs if the stream runs out).
    std::vector<TrackEpoch> run(const std::int8_t* sig, std::size_t numSamples, int numMs);

    int prn() const { return prn_; }

private:
    int    prn_;
    TrackConfig cfg_;

    std::vector<double> code_;       // 1023-chip C/A as +/-1, padded to 1025 (guard chips)

    // Carrier loop state
    double carrFreq_;                // current local carrier freq (Hz)
    double carrFreqBasis_;           // acquired carrier freq (IF + Doppler)
    double remCarrPhase_;            // residual carrier phase (rad)
    double oldCarrNco_, oldCarrError_;
    double tau1carr_, tau2carr_;

    // Code loop state
    double codeFreq_;                // current code NCO freq (chips/s)
    double remCodePhase_;            // residual code phase (chips)
    double oldCodeNco_, oldCodeError_;
    double tau1code_, tau2code_;

    // C/N0 (narrowband/wideband power ratio) accumulators
    double nbpI_, nbpQ_, wbp_;       // within a 20 ms window
    int    cn0Count_;
    double cn0_;
};

} // namespace gps

#endif
