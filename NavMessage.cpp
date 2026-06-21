//---------------------------------------------------------------------------
// NavMessage.cpp - see NavMessage.h
//
// Bit synchronisation + demodulation. Frame sync, parity and ephemeris parsing
// are added once the LNAV bit-field spec is cross-verified.
//---------------------------------------------------------------------------
#include "NavMessage.h"
#include <cstdint>
#include <cstring>
#include <cmath>
#include <initializer_list>

namespace gps {

static const int    BIT_MS = 20;   // one 50 bps data bit spans 20 C/A periods (20 ms)
static const double NAV_PI = 3.1415926535897932;

BitSync findBitSync(const std::vector<TrackEpoch>& ep)
{
    BitSync r;
    const int N = (int)ep.size();
    if (N < 2 * BIT_MS) return r;

    // A data-bit transition shows up as a prompt-I sign change. Genuine bit edges
    // all land at the same (n mod 20) phase; noise transitions are spread evenly.
    int hist[BIT_MS] = {0};
    int total = 0;
    for (int n = 1; n < N; ++n) {
        const bool s0 = ep[n - 1].iP >= 0.0;
        const bool s1 = ep[n].iP     >= 0.0;
        if (s0 != s1) { hist[n % BIT_MS]++; ++total; }
    }

    int best = 0, second = 0;
    for (int k = 1; k < BIT_MS; ++k) if (hist[k] > hist[best]) best = k;
    for (int k = 0; k < BIT_MS; ++k) if (k != best && hist[k] > second) second = hist[k];

    r.offset     = best;        // bits start at indices n with (n % 20) == best
    r.peakCount  = hist[best];
    r.totalTrans = total;
    // Trust the sync when the peak clearly dominates the runner-up.
    r.valid = (hist[best] >= 5) && (hist[best] >= 3 * second);
    return r;
}

std::vector<int> demodulateBits(const std::vector<TrackEpoch>& ep, int offset)
{
    std::vector<int> bits;
    const int N = (int)ep.size();
    if (offset < 0 || offset >= BIT_MS) return bits;

    for (int n = offset; n + BIT_MS <= N; n += BIT_MS) {
        double s = 0.0;
        for (int j = 0; j < BIT_MS; ++j) s += ep[n + j].iP;
        bits.push_back(s >= 0.0 ? 1 : 0);
    }
    return bits;
}

double estimateCN0(const std::vector<TrackEpoch>& ep, int bitOffset)
{
    const int N = (int)ep.size();
    if (bitOffset < 0) bitOffset = 0;

    // Average the NWPR ratio over bit-aligned 20 ms windows, then convert once.
    double muSum = 0.0;
    int    cnt = 0;
    for (int n = bitOffset; n + BIT_MS <= N; n += BIT_MS) {
        double si = 0, sq = 0, wb = 0;
        for (int j = 0; j < BIT_MS; ++j) {
            si += ep[n + j].iP;
            sq += ep[n + j].qP;
            wb += ep[n + j].iP * ep[n + j].iP + ep[n + j].qP * ep[n + j].qP;
        }
        if (wb > 0.0) { muSum += (si * si + sq * sq) / wb; ++cnt; }
    }
    if (cnt == 0) return 0.0;

    double mu = muSum / cnt;               // averaged ratio in [1, 20]
    if (mu > 19.8) mu = 19.8;              // clamp toward the estimator ceiling (~50 dB-Hz)
    if (mu <= 1.0) return 0.0;
    return 10.0 * std::log10((mu - 1.0) / (20.0 - mu) / 1.0e-3);
}

//---------------------------------------------------------------------------
// LNAV parity - IS-GPS-200 Table 20-XIV
//---------------------------------------------------------------------------
static int xorOf(const int* d /*1-based d[1..24]*/, std::initializer_list<int> idx)
{
    int p = 0;
    for (int k : idx) p ^= d[k];
    return p;
}

// w[0..29] = received word bits D1..D30; D29s/D30s = bits 29/30 of the PREVIOUS
// word. Recovers data d_i = D_i XOR D30s, recomputes the six parity bits, and
// compares them to the received parity (D25..D30). Returns true iff all match;
// out24 receives the recovered data bits.
static bool checkParity(const int* w, int D29s, int D30s, int* out24)
{
    int d[25];
    for (int i = 1; i <= 24; ++i) d[i] = w[i - 1] ^ D30s;

    const int p25 = D29s ^ xorOf(d, {1,2,3,5,6,10,11,12,13,14,17,18,20,23});
    const int p26 = D30s ^ xorOf(d, {2,3,4,6,7,11,12,13,14,15,18,19,21,24});
    const int p27 = D29s ^ xorOf(d, {1,3,4,5,7,8,12,13,14,15,16,19,20,22});
    const int p28 = D30s ^ xorOf(d, {2,4,5,6,8,9,13,14,15,16,17,20,21,23});
    const int p29 = D30s ^ xorOf(d, {1,3,5,6,7,9,10,14,15,16,17,18,21,22,24});
    const int p30 = D29s ^ xorOf(d, {3,5,6,8,9,10,11,13,15,19,22,23,24});

    if (p25==w[24] && p26==w[25] && p27==w[26] && p28==w[27] && p29==w[28] && p30==w[29]) {
        for (int i = 1; i <= 24; ++i) out24[i - 1] = d[i];
        return true;
    }
    return false;
}

//---------------------------------------------------------------------------
// Bit-field extraction from a recovered 300-bit subframe (1-based, MSB first).
//---------------------------------------------------------------------------
static std::uint32_t getU(const int* sf, int a, int b)
{
    std::uint32_t v = 0;
    for (int i = a; i <= b; ++i) v = (v << 1) | (std::uint32_t)(sf[i - 1] & 1);
    return v;
}
// split field: MSBs in [a1,b1], LSBs in [a2,b2]
static std::uint32_t getU2(const int* sf, int a1, int b1, int a2, int b2)
{
    const std::uint32_t hi = getU(sf, a1, b1);
    const std::uint32_t lo = getU(sf, a2, b2);
    return (hi << (b2 - a2 + 1)) | lo;
}
static double sScale(std::uint32_t v, int nbits, double scale)  // two's complement
{
    std::int64_t s = (std::int64_t)v;
    if (v & (std::uint32_t)(1u << (nbits - 1))) s -= ((std::int64_t)1 << nbits);
    return (double)s * scale;
}
static double uScale(std::uint32_t v, double scale) { return (double)v * scale; }

static const double P2_5  = 1.0/32.0;                 // 2^-5
static const double P2_19 = 1.0/524288.0;             // 2^-19
static const double P2_29 = 1.0/536870912.0;          // 2^-29
static const double P2_31 = 1.0/2147483648.0;         // 2^-31
static const double P2_33 = P2_31/4.0;                // 2^-33
static const double P2_43 = 1.0/8796093022208.0;      // 2^-43
static const double P2_55 = P2_43/4096.0;             // 2^-55
static const double P2_4  = 16.0;                     // 2^+4
// almanac / iono / UTC scale factors
static const double P2_11 = 1.0/2048.0;               // 2^-11
static const double P2_20 = 1.0/1048576.0;            // 2^-20
static const double P2_21 = 1.0/2097152.0;            // 2^-21
static const double P2_23 = 1.0/8388608.0;            // 2^-23
static const double P2_24 = 1.0/16777216.0;           // 2^-24
static const double P2_27 = 1.0/134217728.0;          // 2^-27
static const double P2_30 = 1.0/1073741824.0;         // 2^-30
static const double P2_38 = P2_30/256.0;              // 2^-38
static const double P2_50 = 1.0/1125899906842624.0;   // 2^-50
static const double P2_P12 = 4096.0;                  // 2^+12

//---------------------------------------------------------------------------
// Frame sync + ephemeris decode
//---------------------------------------------------------------------------
NavDecode decodeNav(const std::vector<int>& bits, int prn)
{
    NavDecode r;
    r.eph.prn = prn;
    const int N = (int)bits.size();
    const int PRE[8] = {1,0,0,0,1,0,1,1};   // TLM preamble 0x8B

    int  sf[4][300];                         // recovered subframes 1,2,3

    for (int i = 2; i + 300 <= N; ) {
        // Preamble match at i, in either polarity.
        bool norm = true, inv = true;
        for (int k = 0; k < 8; ++k) {
            if (bits[i + k] != PRE[k])       norm = false;
            if (bits[i + k] != (1 - PRE[k])) inv  = false;
        }
        if (!norm && !inv) { ++i; continue; }
        const int pol = norm ? 0 : 1;        // XOR mask to upright the data

        // Parity-check all 10 words (need the previous word's bits 29/30).
        int recovered[300];
        bool ok = true;
        for (int wi = 0; wi < 10 && ok; ++wi) {
            const int base = i + wi * 30;
            int word[30];
            for (int k = 0; k < 30; ++k) word[k] = bits[base + k] ^ pol;
            const int D29s = (wi == 0) ? (bits[i - 2] ^ pol) : (bits[base - 30 + 28] ^ pol);
            const int D30s = (wi == 0) ? (bits[i - 1] ^ pol) : (bits[base - 30 + 29] ^ pol);
            int data[24];
            if (!checkParity(word, D29s, D30s, data)) { ok = false; break; }
            for (int k = 0; k < 24; ++k) recovered[wi * 30 + k]      = data[k];
            for (int k = 24; k < 30; ++k) recovered[wi * 30 + k]     = word[k];
        }
        if (!ok) { ++r.parityFails; ++i; continue; }

        ++r.subframesOk;
        if (r.polarity < 0) r.polarity = pol;

        const int sfid = (int)getU(recovered, 50, 52);   // HOW subframe ID
        const int tow  = (int)getU(recovered, 31, 47);   // HOW TOW count (next subframe leading edge)
        { SubframeRef sr; sr.bitIndex = i; sr.towCount = tow; sr.id = sfid; r.subframes.push_back(sr); }
        if (sfid >= 1 && sfid <= 3 && !r.gotSf[sfid]) {
            r.gotSf[sfid] = true;
            std::memcpy(sf[sfid], recovered, sizeof(recovered));
        }
        i += 300;                                        // next subframe
    }

    Ephemeris& e = r.eph;
    // Subframe 1 - SV clock.
    if (r.gotSf[1]) {
        const int* s = sf[1];
        e.weekNumber = (int)getU(s, 61, 70);
        e.iodc       = (int)getU2(s, 83, 84, 211, 218);
        e.tgd        = sScale(getU(s, 197, 204), 8,  P2_31);
        e.toc        = uScale(getU(s, 219, 234),     P2_4);
        e.af2        = sScale(getU(s, 241, 248), 8,  P2_55);
        e.af1        = sScale(getU(s, 249, 264), 16, P2_43);
        e.af0        = sScale(getU(s, 271, 292), 22, P2_31);
    }
    // Subframe 2 - ephemeris part 1.
    if (r.gotSf[2]) {
        const int* s = sf[2];
        e.iode   = (int)getU(s, 61, 68);
        e.crs    = sScale(getU(s, 69, 84), 16, P2_5);
        e.deltaN = sScale(getU(s, 91, 106), 16, P2_43) * NAV_PI;   // semicircles/s -> rad/s
        e.m0     = sScale(getU2(s, 107, 114, 121, 144), 32, P2_31) * NAV_PI;
        e.cuc    = sScale(getU(s, 151, 166), 16, P2_29);
        e.ecc    = uScale(getU2(s, 167, 174, 181, 204), P2_33);    // unsigned
        e.cus    = sScale(getU(s, 211, 226), 16, P2_29);
        e.sqrtA  = uScale(getU2(s, 227, 234, 241, 264), P2_19);    // unsigned
        e.toe    = uScale(getU(s, 271, 286), P2_4);
    }
    // Subframe 3 - ephemeris part 2.
    if (r.gotSf[3]) {
        const int* s = sf[3];
        e.cic      = sScale(getU(s, 61, 76), 16, P2_29);
        e.omega0   = sScale(getU2(s, 77, 84, 91, 114), 32, P2_31) * NAV_PI;
        e.cis      = sScale(getU(s, 121, 136), 16, P2_29);
        e.i0       = sScale(getU2(s, 137, 144, 151, 174), 32, P2_31) * NAV_PI;
        e.crc      = sScale(getU(s, 181, 196), 16, P2_5);
        e.omega    = sScale(getU2(s, 197, 204, 211, 234), 32, P2_31) * NAV_PI;
        e.omegaDot = sScale(getU(s, 241, 264), 24, P2_43) * NAV_PI;
        e.idot     = sScale(getU(s, 279, 292), 14, P2_43) * NAV_PI;
    }
    e.valid = r.gotSf[1] && r.gotSf[2] && r.gotSf[3];
    return r;
}

//---------------------------------------------------------------------------
// Detailed per-word / per-subframe inspection (Advanced "Nav message" view).
//---------------------------------------------------------------------------
NavDetail inspectNav(const std::vector<int>& bits, int prn)
{
    (void)prn;
    NavDetail d;
    const int N = (int)bits.size();
    const int PRE[8] = {1,0,0,0,1,0,1,1};   // TLM preamble 0x8B

    for (int i = 2; i + 300 <= N; ) {
        bool norm = true, inv = true;
        for (int k = 0; k < 8; ++k) {
            if (bits[i + k] != PRE[k])       norm = false;
            if (bits[i + k] != (1 - PRE[k])) inv  = false;
        }
        if (!norm && !inv) { ++i; continue; }
        const int pol = norm ? 0 : 1;

        NavSubframe sf;
        sf.bitIndex = i;
        int recovered[300];
        bool ok = true;
        for (int wi = 0; wi < 10; ++wi) {
            const int base = i + wi * 30;
            int word[30];
            for (int k = 0; k < 30; ++k) word[k] = bits[base + k] ^ pol;
            const int D29s = (wi == 0) ? (bits[i - 2] ^ pol) : (bits[base - 30 + 28] ^ pol);
            const int D30s = (wi == 0) ? (bits[i - 1] ^ pol) : (bits[base - 30 + 29] ^ pol);
            int data[24];
            const bool wok = checkParity(word, D29s, D30s, data);

            std::uint32_t raw = 0; for (int k = 0; k < 30; ++k) raw = (raw << 1) | (std::uint32_t)(word[k] & 1);
            std::uint32_t dv  = 0; for (int k = 0; k < 24; ++k) dv  = (dv  << 1) | (std::uint32_t)(data[k] & 1);
            sf.words[wi].raw = raw; sf.words[wi].data = dv; sf.words[wi].parityOk = wok;

            if (!wok) ok = false;
            for (int k = 0; k < 24; ++k) recovered[wi * 30 + k]  = wok ? data[k] : (word[k]);
            for (int k = 24; k < 30; ++k) recovered[wi * 30 + k]  = word[k];
        }
        if (!ok) { ++d.totalParityFails; ++i; continue; }

        std::memcpy(sf.recovered, recovered, sizeof(recovered));
        sf.id        = (int)getU(recovered, 50, 52);
        sf.towCount  = (int)getU(recovered, 31, 47);
        sf.alert     = getU(recovered, 48, 48) != 0;
        sf.antiSpoof = getU(recovered, 49, 49) != 0;
        if (sf.id == 4 || sf.id == 5) sf.page = (int)getU(recovered, 63, 68);  // SV-ID / page
        if (d.polarity < 0) d.polarity = pol;
        d.subframes.push_back(sf);
        i += 300;
    }
    return d;
}

//---------------------------------------------------------------------------
// Almanac (subframes 4 & 5) decode - per IS-GPS-200 Figure 20-1 / Tables 20-VI..IX.
//---------------------------------------------------------------------------
static void decodeAlmanacPage(const int* s, int svid, AlmanacSet& out)
{
    if (svid >= 1 && svid <= 32) {                 // per-SV almanac page
        Almanac a;
        a.prn      = svid;
        a.ecc      = uScale(getU(s, 69, 84), P2_21);
        a.toa      = uScale(getU(s, 91, 98), P2_P12);
        a.deltaI   = sScale(getU(s, 99, 114), 16, P2_19) * NAV_PI;        // semicircles -> rad
        a.i0       = 0.30 * NAV_PI + a.deltaI;                            // ref inclination 0.30 semicircles
        a.omegaDot = sScale(getU(s, 121, 136), 16, P2_38) * NAV_PI;
        a.health   = (int)getU(s, 137, 144);
        a.sqrtA    = uScale(getU(s, 151, 174), P2_11);
        a.omega0   = sScale(getU(s, 181, 204), 24, P2_23) * NAV_PI;
        a.omega    = sScale(getU(s, 211, 234), 24, P2_23) * NAV_PI;
        a.m0       = sScale(getU(s, 241, 264), 24, P2_23) * NAV_PI;
        const std::uint32_t af0bits = (getU(s, 271, 278) << 3) | getU(s, 290, 292);  // 8 MSB + 3 LSB
        a.af0      = sScale(af0bits, 11, P2_20);
        a.af1      = sScale(getU(s, 279, 289), 11, P2_38);
        a.valid    = (a.sqrtA > 4000.0 && a.sqrtA < 6000.0);             // sane GPS orbit
        out.alm[svid] = a;
        if (a.valid) out.svHealth[svid] = a.health;
    }
    else if (svid == 51) {                          // SF5 page 25: toa + WNa + SV1-24 health
        out.refToa  = uScale(getU(s, 69, 76), P2_P12);
        out.refWeek = (int)getU(s, 77, 84);
        // SV health 1-24: 6 bits each, starting bit 91 (word 4), 4 per word.
        int bit = 91;
        for (int prn = 1; prn <= 24 && bit + 5 <= 276; ++prn) {
            out.svHealth[prn] = (int)getU(s, bit, bit + 5);
            bit += 6;
        }
    }
    else if (svid == 56) {                          // SF4 page 18: ionosphere (Klobuchar) + UTC
        out.haveIono = true;
        out.alpha[0] = sScale(getU(s, 69, 76),  8, P2_30);
        out.alpha[1] = sScale(getU(s, 77, 84),  8, P2_27);
        out.alpha[2] = sScale(getU(s, 91, 98),  8, P2_24);
        out.alpha[3] = sScale(getU(s, 99, 106), 8, P2_24);
        out.beta[0]  = sScale(getU(s, 107, 114), 8, 2048.0);             // 2^11
        out.beta[1]  = sScale(getU(s, 115, 122), 8, 16384.0);           // 2^14
        out.beta[2]  = sScale(getU(s, 123, 130), 8, 65536.0);           // 2^16
        out.beta[3]  = sScale(getU(s, 131, 138), 8, 65536.0);           // 2^16
        out.haveUtc  = true;
        out.utcA1    = sScale(getU(s, 151, 174), 24, P2_50);
        out.utcA0    = sScale(getU2(s, 181, 188, 191, 206), 32, P2_30);  // 32-bit A0 (8 MSB + 24 LSB)
        out.utcTot   = (int)getU(s, 219, 226);
        out.utcWNt   = (int)getU(s, 227, 234);
        out.utcDtLS  = (int)sScale(getU(s, 241, 248), 8, 1.0);
    }
}

AlmanacSet decodeAlmanac(const std::vector<int>& bits)
{
    AlmanacSet out;
    for (int i = 0; i <= 32; ++i) out.svHealth[i] = -1;

    NavDetail d = inspectNav(bits, 0);
    for (std::size_t k = 0; k < d.subframes.size(); ++k) {
        const NavSubframe& sf = d.subframes[k];
        if (sf.id != 4 && sf.id != 5) continue;
        out.pagesSeen.push_back(sf.page);
        decodeAlmanacPage(sf.recovered, sf.page, out);
    }
    return out;
}

} // namespace gps
