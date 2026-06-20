//---------------------------------------------------------------------------
// CACode.cpp - see CACode.h
//---------------------------------------------------------------------------
#include "CACode.h"
#include <stdexcept>
#include <cmath>

namespace gps {

// Per-PRN G2 phase-selector taps (1-indexed register stages), IS-GPS-200.
// The G2 output for a PRN is G2[tapA] XOR G2[tapB].
struct G2Taps { int a; int b; };

static const G2Taps kG2Taps[32] = {
    {2,6},  {3,7},  {4,8},  {5,9},  {1,9},  {2,10}, {1,8},  {2,9},
    {3,10}, {2,3},  {3,4},  {5,6},  {6,7},  {7,8},  {8,9},  {9,10},
    {1,4},  {2,5},  {3,6},  {4,7},  {5,8},  {6,9},  {1,3},  {4,6},
    {5,7},  {6,8},  {7,9},  {8,10}, {1,6},  {2,7},  {3,8},  {4,9}
};

std::vector<int8_t> generateCACode(int prn)
{
    if (prn < 1 || prn > 32)
        throw std::invalid_argument("generateCACode: PRN must be 1..32");

    const G2Taps taps = kG2Taps[prn - 1];

    // Registers G1[1..10], G2[1..10]; index 0 unused. All stages start at 1.
    int g1[11], g2[11];
    for (int i = 1; i <= 10; ++i) { g1[i] = 1; g2[i] = 1; }

    std::vector<int8_t> code(CA_CODE_LENGTH);

    for (int n = 0; n < CA_CODE_LENGTH; ++n) {
        int g1out = g1[10];
        int g2out = g2[taps.a] ^ g2[taps.b];
        int chip  = g1out ^ g2out;               // logical 0/1
        code[n]   = (int8_t)(1 - 2 * chip);       // 0 -> +1, 1 -> -1

        // Feedback taps:  G1 = 1 + x^3 + x^10
        int fb1 = g1[3] ^ g1[10];
        // G2 = 1 + x^2 + x^3 + x^6 + x^8 + x^9 + x^10
        int fb2 = g2[2] ^ g2[3] ^ g2[6] ^ g2[8] ^ g2[9] ^ g2[10];

        for (int i = 10; i >= 2; --i) { g1[i] = g1[i - 1]; g2[i] = g2[i - 1]; }
        g1[1] = fb1;
        g2[1] = fb2;
    }

    return code;
}

std::vector<double> sampleCACode(const std::vector<int8_t>& code,
                                 double fs, int numSamples)
{
    std::vector<double> out(numSamples);
    const double tc = 1.0 / CA_CHIP_RATE;   // chip duration (s)
    const double ts = 1.0 / fs;             // sample duration (s)
    for (int n = 0; n < numSamples; ++n) {
        int idx = (int)std::floor((n * ts) / tc);
        if (idx >= CA_CODE_LENGTH) idx %= CA_CODE_LENGTH; // wrap last partial
        out[n] = (double)code[idx];
    }
    return out;
}

} // namespace gps
