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

// --- Detailed inspection of the LNAV frame (for the Advanced "Nav message" view) ---

// One 30-bit LNAV word, fully exposed for teaching/inspection.
struct NavWord {
    std::uint32_t raw = 0;      // 30 received bits D1..D30 (polarity-corrected), MSB first
    std::uint32_t data = 0;     // 24 recovered source data bits d1..d24
    bool          parityOk = false;
};

// One parity-passing subframe, with its 10 words and decoded TLM/HOW fields.
struct NavSubframe {
    int  bitIndex = 0;          // start bit in the demodulated stream
    int  id       = 0;          // subframe ID (1..5)
    int  towCount = 0;          // HOW 17-bit TOW count (next subframe leading edge)
    int  page     = 0;          // SF4/5 SV-ID / page id (word-3 bits 63..68), 0 for SF1-3
    bool alert    = false;      // HOW alert flag
    bool antiSpoof= false;      // HOW anti-spoof flag
    NavWord words[10];
    int  recovered[300];        // recovered source bits (1-based field extraction)
};

struct NavDetail {
    int polarity = -1;          // 0 upright, 1 inverted
    int totalParityFails = 0;   // preamble candidates rejected on parity
    std::vector<NavSubframe> subframes;
};

// Full bit/word/parity breakdown of every parity-passing subframe in the stream.
NavDetail inspectNav(const std::vector<int>& bits, int prn);

// --- Almanac (subframes 4 & 5) - reduced-precision, long-validity orbit data ---
// NOTE: the full almanac (all PRNs + iono/UTC) spans the 25-page cycle = 12.5 min,
// so a short capture yields only the few pages it happens to contain.

struct Almanac {
    int    prn   = 0;
    bool   valid = false;
    double ecc=0;          // eccentricity (dimensionless)
    double toa=0;          // almanac reference time (s)
    double deltaI=0;       // inclination offset from 0.30 semicircles (rad)
    double i0=0;           // inclination (rad) = (0.30 semicircles + deltaI)
    double omegaDot=0;     // rate of right ascension (rad/s)
    double sqrtA=0;        // sqrt semi-major axis (sqrt m)
    double omega0=0;       // longitude of ascending node (rad)
    double omega=0;        // argument of perigee (rad)
    double m0=0;           // mean anomaly (rad)
    double af0=0, af1=0;   // SV clock (s, s/s)
    int    health=0;       // 8-bit SV health
};

struct AlmanacSet {
    int    refWeek = -1;   // WNa almanac reference week (from page 25), -1 if not captured
    double refToa  = -1;   // toa from page 25 (s), -1 if not captured
    Almanac alm[33];       // per-PRN (index = PRN 1..32)
    int    svHealth[33];   // health from the page-25 health pages (-1 = unknown)
    // Subframe 4 page 18: ionosphere (Klobuchar) + UTC parameters.
    bool   haveIono = false;
    double alpha[4] = {0,0,0,0};   // Klobuchar alpha (s, s/semicircle, ...)
    double beta[4]  = {0,0,0,0};   // Klobuchar beta
    bool   haveUtc = false;
    double utcA0=0, utcA1=0;       // UTC offset polynomial
    int    utcTot=0, utcWNt=0, utcDtLS=0;
    std::vector<int> pagesSeen;    // SV-ID/page ids actually captured (for teaching the 12.5 min cycle)
};

// Decode whatever subframe 4/5 almanac/health/iono/UTC pages are present in the stream.
AlmanacSet decodeAlmanac(const std::vector<int>& bits);

} // namespace gps

#endif
