//---------------------------------------------------------------------------
// PvtConsole.cpp - End-to-end position fix from the raw sample file.
// Acquires + tracks all satellites, decodes each ephemeris, forms pseudoranges
// at a common subframe boundary (SoftGNSS calculatePseudoranges convention),
// and solves least-squares PVT -> lat/lon/alt.
//
// Build:
//   bcc64x -O2 -std=c++17 PvtConsole.cpp Pvt.cpp NavMessage.cpp Tracking.cpp \
//          Acquisition.cpp CACode.cpp Fft.cpp -o PvtConsole.exe
//---------------------------------------------------------------------------
#include "Acquisition.h"
#include "Tracking.h"
#include "NavMessage.h"
#include "Pvt.h"

#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <algorithm>
#include <cmath>

struct Channel {
    int prn = 0;
    long long codePhaseSamp = 0;
    int bitOffset = 0;
    gps::Ephemeris eph;
    std::vector<gps::TrackEpoch> ep;
    std::vector<gps::SubframeRef> subs;
};

int main(int argc, char** argv)
{
    std::string file = (argc > 1) ? argv[1]
                                  : "GPSdata-DiscreteComponents-fs38_192-if9_55.bin";
    int trackMs = (argc > 2) ? std::atoi(argv[2]) : 36000;

    gps::AcqConfig acfg;
    const int n = (int)std::lround(acfg.fs * 1.0e-3);
    const double C = 299792458.0;

    std::ifstream f(file, std::ios::binary);
    if (!f) { std::printf("ERROR: cannot open %s\n", file.c_str()); return 2; }

    // --- Acquire ---
    std::vector<std::int8_t> acqBuf((std::size_t)n * acfg.numMs);
    f.read(reinterpret_cast<char*>(acqBuf.data()), (std::streamsize)acqBuf.size());
    auto results = gps::acquireAll(acqBuf.data(), acqBuf.size(), acfg);
    int nf = 0; for (auto& r : results) if (r.found) ++nf;
    std::printf("Acquired %d satellites. Tracking each %d ms + decoding ephemeris...\n\n",
                nf, trackMs);

    // --- Track + decode every found satellite ---
    gps::TrackConfig tcfg;
    gps::AcqConfig   rcfg; rcfg.fs = tcfg.fs; rcfg.ifFreq = tcfg.ifFreq;
    const std::size_t need = (std::size_t)(trackMs + 2) * n;
    std::vector<std::int8_t> sig(need);
    std::vector<Channel> chans;

    for (const auto& a : results) {
        if (!a.found) continue;
        f.clear();
        f.seekg((std::streamoff)a.codePhaseSamp, std::ios::beg);
        f.read(reinterpret_cast<char*>(sig.data()), (std::streamsize)need);
        const std::size_t got = (std::size_t)f.gcount();

        const double fd = gps::refineDoppler(sig.data(), got, a.prn, a.doppler, rcfg);
        gps::TrackChannel ch(a.prn, fd, tcfg);
        std::vector<gps::TrackEpoch> ep = ch.run(sig.data(), got, trackMs);

        gps::BitSync bs = gps::findBitSync(ep);
        if (!bs.valid) { std::printf("  PRN %2d: no bit sync\n", a.prn); continue; }
        std::vector<int> bits = gps::demodulateBits(ep, bs.offset);
        gps::NavDecode nd = gps::decodeNav(bits, a.prn);

        std::printf("  PRN %2d: %2d subframes, ephemeris %s\n",
                    a.prn, (int)nd.subframes.size(), nd.eph.valid ? "OK" : "incomplete");
        if (!nd.eph.valid) continue;

        Channel c;
        c.prn = a.prn; c.codePhaseSamp = a.codePhaseSamp; c.bitOffset = bs.offset;
        c.eph = nd.eph; c.ep = std::move(ep); c.subs = nd.subframes;
        chans.push_back(std::move(c));
    }

    std::printf("\n%d channels with a full ephemeris.\n", (int)chans.size());
    if (chans.size() < 4) { std::printf("Need >= 4 for a fix.\n"); return 3; }

    // --- Pick a subframe boundary (TOW) common to the most channels ---
    std::map<int, std::vector<std::pair<int,int>>> byTow;   // tow -> [(chIdx, bitIndex)]
    for (int ci = 0; ci < (int)chans.size(); ++ci)
        for (const auto& s : chans[ci].subs)
            byTow[s.towCount].push_back({ci, s.bitIndex});

    int bestTow = -1; std::size_t bestCnt = 0;
    for (auto& kv : byTow) if (kv.second.size() > bestCnt) { bestCnt = kv.second.size(); bestTow = kv.first; }
    std::printf("Common subframe TOW count = %d (in %d channels).\n", bestTow, (int)bestCnt);

    // --- Form pseudoranges at that common instant ---
    const double samplesPerCode = std::round(acfg.fs / 1000.0);   // 38192 samples / ms
    const double START_OFFSET = 68.802;                            // ms, nominal one-way travel time
    std::vector<double>       ttms;
    std::vector<gps::SatState> sats;
    std::vector<int>          usedPrn;

    const double transmitTime = bestTow * 6.0 - 6.0;               // this subframe's leading-edge GPS time
    for (const auto& cb : byTow[bestTow]) {
        const int ci = cb.first, bi = cb.second;
        const std::size_t ms = (std::size_t)(chans[ci].bitOffset + bi * 20);
        if (ms >= chans[ci].ep.size()) continue;
        const long long absSample = chans[ci].codePhaseSamp + (long long)chans[ci].ep[ms].sampleIndex;
        ttms.push_back((double)absSample / samplesPerCode);
        sats.push_back(gps::satPosition(chans[ci].eph, transmitTime));
        usedPrn.push_back(chans[ci].prn);
    }
    if (ttms.size() < 4) { std::printf("Fewer than 4 aligned channels.\n"); return 4; }

    const double mn = std::floor(*std::min_element(ttms.begin(), ttms.end()));
    std::vector<double> pr(ttms.size());
    std::printf("\nPRN   pseudorange(km)   SV ECEF X,Y,Z (km)        SVclk(us)\n");
    for (std::size_t i = 0; i < ttms.size(); ++i) {
        ttms[i] = ttms[i] - mn + START_OFFSET;             // ms
        pr[i]   = ttms[i] * (C / 1000.0);                  // meters
        std::printf("%3d   %12.3f   (%.0f, %.0f, %.0f)   %+8.3f\n",
                    usedPrn[i], pr[i] / 1000.0,
                    sats[i].x/1e3, sats[i].y/1e3, sats[i].z/1e3, sats[i].clockBias*1e6);
    }

    // --- Solve ---
    gps::PvtSolution sol = gps::solvePvt(pr, sats);
    const double rad = std::sqrt(sol.x*sol.x + sol.y*sol.y + sol.z*sol.z);

    std::printf("\n=== POSITION FIX (%d satellites) ===\n", (int)pr.size());
    std::printf("  ECEF      = (%.1f, %.1f, %.1f) m   (radius %.1f km)\n",
                sol.x, sol.y, sol.z, rad/1e3);
    std::printf("  Lat/Lon   = %.6f deg, %.6f deg\n", sol.lat, sol.lon);
    std::printf("  Altitude  = %.1f m\n", sol.alt);
    std::printf("  Rx clock  = %.3f m (%.3f us)\n", sol.clockBias, sol.clockBias/C*1e6);
    std::printf("  GDOP=%.2f   iters=%d   post-fit residual RMS=%.1f m\n",
                sol.gdop, sol.iterations, sol.residRms);

    const bool sane = (rad > 6.30e6 && rad < 6.42e6)          // near the WGS-84 surface
                   && (sol.alt > -2000.0 && sol.alt < 12000.0)
                   && (sol.residRms < 1.0e4);
    std::printf("\n  SANITY: %s\n", sane ?
        "PASS - receiver on/near Earth's surface, residuals consistent" :
        "FAIL - off-surface or large residuals");
    std::printf("\n%s\n", sane ? "POSITION FIX OK." : "Fix out of range.");
    return sane ? 0 : 1;
}
