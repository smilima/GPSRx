//---------------------------------------------------------------------------
// CACode.h - GPS L1 C/A Gold-code generator (PRN 1..32).
//
// Generates the 1023-chip coarse/acquisition (C/A) code for any GPS satellite
// per IS-GPS-200: two 10-stage LFSRs (G1, G2) whose outputs are XORed, with a
// per-PRN phase selection on G2.
//
// Pure standard C++ (no VCL).
//---------------------------------------------------------------------------
#ifndef CACodeH
#define CACodeH

#include <vector>
#include <cstdint>

namespace gps {

const int CA_CODE_LENGTH = 1023;   // chips per C/A code period
const double CA_CHIP_RATE = 1.023e6; // chips per second

// Generate the 1023-chip C/A code for the given PRN (1..32).
// Output is bipolar: each chip is +1 or -1 (logical 0 -> +1, logical 1 -> -1).
std::vector<int8_t> generateCACode(int prn);

// Resample a 1023-chip code to 'numSamples' samples covering exactly one code
// period at sample rate fs (nearest-chip sampling). Output values are +/-1.0.
// Used to build the local replica for FFT correlation.
std::vector<double> sampleCACode(const std::vector<int8_t>& code,
                                 double fs, int numSamples);

} // namespace gps

#endif
