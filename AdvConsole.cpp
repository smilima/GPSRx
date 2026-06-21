//---------------------------------------------------------------------------
// AdvConsole.cpp - verify the Advanced-view DSP additions (inspectNav +
// decodeAlmanac) against the real capture.
//
// Build:
//   bcc64x -O2 -std=c++17 AdvConsole.cpp NavMessage.cpp Tracking.cpp \
//          Acquisition.cpp CACode.cpp Fft.cpp -o AdvConsole.exe
//---------------------------------------------------------------------------
#include "Acquisition.h"
#include "Tracking.h"
#include "NavMessage.h"
#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>

int main(int argc, char** argv)
{
    std::string file = (argc > 1) ? argv[1]
                                  : "GPSdata-DiscreteComponents-fs38_192-if9_55.bin";
    int trackMs = (argc > 2) ? std::atoi(argv[2]) : 36000;

    gps::AcqConfig acfg;
    const int n = (int)std::lround(acfg.fs * 1.0e-3);

    std::ifstream f(file, std::ios::binary);
    if (!f) { std::printf("ERROR: cannot open %s\n", file.c_str()); return 2; }

    std::vector<std::int8_t> acq((std::size_t)n * acfg.numMs);
    f.read(reinterpret_cast<char*>(acq.data()), (std::streamsize)acq.size());
    auto results = gps::acquireAll(acq.data(), acq.size(), acfg);

    gps::TrackConfig tcfg;
    gps::AcqConfig   rcfg; rcfg.fs = tcfg.fs; rcfg.ifFreq = tcfg.ifFreq;
    const std::size_t need = (std::size_t)(trackMs + 2) * n;
    std::vector<std::int8_t> sig(need);

    std::vector<int> allBits;   // accumulate bits from the strongest sat for almanac

    int shown = 0;
    for (const auto& a : results) {
        if (!a.found) continue;
        f.clear(); f.seekg((std::streamoff)a.codePhaseSamp, std::ios::beg);
        f.read(reinterpret_cast<char*>(sig.data()), (std::streamsize)need);
        const std::size_t got = (std::size_t)f.gcount();

        const double fd = gps::refineDoppler(sig.data(), got, a.prn, a.doppler, rcfg);
        gps::TrackChannel ch(a.prn, fd, tcfg);
        auto ep = ch.run(sig.data(), got, trackMs);
        gps::BitSync bs = gps::findBitSync(ep);
        if (!bs.valid) continue;
        auto bits = gps::demodulateBits(ep, bs.offset);

        gps::NavDetail nd = gps::inspectNav(bits, a.prn);
        if (nd.subframes.empty()) continue;

        if (shown < 2) {
            std::printf("=== PRN %d: %d subframes, polarity %d, parityFails %d ===\n",
                        a.prn, (int)nd.subframes.size(), nd.polarity, nd.totalParityFails);
            for (const auto& s : nd.subframes) {
                int okWords = 0; for (int w = 0; w < 10; ++w) if (s.words[w].parityOk) ++okWords;
                std::printf("  SF%d  TOW=%d  page=%d  alert=%d AS=%d  words OK=%d/10  TLM=0x%06X HOW=0x%06X\n",
                            s.id, s.towCount, s.page, s.alert, s.antiSpoof, okWords,
                            s.words[0].raw, s.words[1].raw);
            }
            ++shown;
        }
        for (int b : bits) allBits.push_back(b);
    }

    // Almanac from the strongest sat's bit stream.
    if (!allBits.empty()) {
        gps::AlmanacSet alm = gps::decodeAlmanac(allBits);
        std::printf("\n=== ALMANAC (pages seen:");
        for (int p : alm.pagesSeen) std::printf(" %d", p);
        std::printf(") refWeek=%d refToa=%.0f iono=%d utc=%d ===\n",
                    alm.refWeek, alm.refToa, alm.haveIono, alm.haveUtc);
        for (int prn = 1; prn <= 32; ++prn) {
            if (!alm.alm[prn].valid) continue;
            const gps::Almanac& a = alm.alm[prn];
            std::printf("  PRN %2d alm: sqrtA=%.3f (a=%.1f km) e=%.6f i0=%.4f rad (%.2f deg) "
                        "OmegaDot=%.3e health=%d toa=%.0f\n",
                        prn, a.sqrtA, a.sqrtA*a.sqrtA/1e3, a.ecc, a.i0, a.i0*180.0/3.14159265,
                        a.omegaDot, a.health, a.toa);
        }
        if (alm.haveIono)
            std::printf("  IONO alpha0=%.3e beta0=%.0f\n", alm.alpha[0], alm.beta[0]);
    }

    std::printf("\nDONE.\n");
    return 0;
}
