//---------------------------------------------------------------------------
// NavMessage.h - GPS L1 C/A navigation-message decode.
//
// Consumes the per-millisecond prompt-I correlator outputs from tracking and:
//   1. bit-syncs (each 50 bps data bit = 20 C/A periods = 20 ms),
//   2. demodulates the bit stream,
//   3. (next) finds the TLM preamble + checks LNAV parity,
//   4. (next) parses subframes 1/2/3 into ephemeris + clock parameters.
//
// Pure standard C++ (no VCL).
//---------------------------------------------------------------------------
#ifndef NavMessageH
#define NavMessageH

#include <vector>
#include "Tracking.h"   // gps::TrackEpoch (prompt-I per ms)

namespace gps {

// Result of the 20 ms histogram bit-sync search.
struct BitSync {
    int  offset      = -1;   // ms index (0..19) of the first sample of a data bit
    int  peakCount   = 0;    // sign transitions at the dominant phase
    int  totalTrans  = 0;    // total sign transitions counted
    bool valid       = false;// peak clearly dominates -> sync trustworthy
};

// Decoded ephemeris + SV clock (filled in once the verified bit-field spec is in).
struct Ephemeris {
    int    prn      = 0;
    bool   valid    = false;
    // clock (subframe 1)
    int    weekNumber = 0;
    double tgd = 0, toc = 0, af0 = 0, af1 = 0, af2 = 0;
    int    iodc = 0, health = 0, ura = 0;
    // ephemeris (subframes 2, 3)
    int    iode = 0;
    double crs = 0, deltaN = 0, m0 = 0, cuc = 0, ecc = 0, cus = 0, sqrtA = 0, toe = 0;
    double cic = 0, omega0 = 0, cis = 0, i0 = 0, crc = 0, omega = 0, omegaDot = 0, idot = 0;
};

// --- Bit synchronisation & demodulation (independent of the LNAV bit-field spec) ---

// Find the 20 ms bit boundary from the prompt-I sign-transition histogram.
BitSync findBitSync(const std::vector<TrackEpoch>& epochs);

// Demodulate to hard bits (0/1) at 50 bps by summing 20 prompt-I per bit, starting
// at sync.offset. The BPSK 180-degree ambiguity is left unresolved here (parity /
// preamble polarity resolves it). Returns one bit per 20 ms.
std::vector<int> demodulateBits(const std::vector<TrackEpoch>& epochs, int offset);

// C/N0 (dB-Hz) via the narrowband/wideband power ratio over 20 ms windows that are
// ALIGNED to the data-bit boundary (bitOffset from findBitSync), so a window never
// straddles a nav-bit edge. Returns 0 if it cannot be estimated.
double estimateCN0(const std::vector<TrackEpoch>& epochs, int bitOffset);

// --- Frame sync + LNAV parity + ephemeris decode (per IS-GPS-200 Tables 20-XIV / 20-III) ---

// One parity-passing subframe boundary, for pseudorange timing.
struct SubframeRef {
    int bitIndex = 0;   // bit-stream index of the subframe's first bit (TLM)
    int towCount = 0;   // 17-bit HOW TOW count (time of NEXT subframe leading edge, 6 s units)
    int id       = 0;   // subframe ID (1..5)
};

struct NavDecode {
    int  subframesOk   = 0;    // parity-passing subframes found
    int  parityFails   = 0;    // preamble candidates that failed parity
    int  polarity      = -1;   // 0 = data upright, 1 = inverted (resolved via parity)
    bool gotSf[4]      = {false,false,false,false}; // indices 1,2,3 used
    std::vector<SubframeRef> subframes;             // all parity-passing subframes (for timing)
    Ephemeris eph;
};

// Search the demodulated bit stream for the TLM preamble, check LNAV parity on
// every word (resolving the 180-degree polarity), and parse subframes 1/2/3 into
// ephemeris + clock parameters.
NavDecode decodeNav(const std::vector<int>& bits, int prn);

} // namespace gps

#endif
