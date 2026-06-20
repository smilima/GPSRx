//---------------------------------------------------------------------------
// NavConsole.cpp - Validate bit synchronisation on the real sample file.
// Acquires the strongest satellite, tracks it for a few seconds, then runs the
// 20 ms histogram bit-sync and prints the demodulated bit stream.
//
// Build:
//   bcc64x -O2 -std=c++17 NavConsole.cpp NavMessage.cpp Tracking.cpp \
//          Acquisition.cpp CACode.cpp Fft.cpp -o NavConsole.exe
//---------------------------------------------------------------------------
#include "Acquisition.h"
#include "Tracking.h"
#include "NavMessage.h"
#include "Pvt.h"

#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>

int main(int argc, char** argv)
{
    std::string file = (argc > 1) ? argv[1]
                                  : "GPSdata-DiscreteComponents-fs38_192-if9_55.bin";
    int trackMs = (argc > 2) ? std::atoi(argv[2]) : 36000;   // tracking duration (ms; 36 s >= a full 30 s frame)

    gps::AcqConfig acfg;
    const int n = (int)std::lround(acfg.fs * 1.0e-3);

    std::ifstream f(file, std::ios::binary);
    if (!f) { std::printf("ERROR: cannot open %s\n", file.c_str()); return 2; }

    // Acquire (first 2 ms) -> strongest satellite.
    std::vector<std::int8_t> acqBuf((std::size_t)n * acfg.numMs);
    f.read(reinterpret_cast<char*>(acqBuf.data()), (std::streamsize)acqBuf.size());
    auto results = gps::acquireAll(acqBuf.data(), acqBuf.size(), acfg);
    gps::AcqResult best; best.peakRatio = -1;
    for (const auto& r : results) if (r.found && r.peakRatio > best.peakRatio) best = r;
    if (best.prn == 0) { std::printf("No satellite acquired.\n"); return 3; }

    std::printf("=== Bit sync on PRN %d ===\n", best.prn);
    std::printf("Seed: Doppler %+.0f Hz, code phase %d samples, ratio %.1f\n",
                best.doppler, best.codePhaseSamp, best.peakRatio);
    std::printf("Tracking %d ms (%.1f s)...\n\n", trackMs, trackMs / 1000.0);

    // Track from the acquired code phase.
    const std::size_t need = (std::size_t)(trackMs + 2) * n;
    std::vector<std::int8_t> sig(need);
    f.clear();
    f.seekg((std::streamoff)best.codePhaseSamp, std::ios::beg);
    f.read(reinterpret_cast<char*>(sig.data()), (std::streamsize)need);
    if ((std::size_t)f.gcount() < (std::size_t)n * 100) {
        std::printf("ERROR: not enough data.\n"); return 4;
    }

    gps::TrackConfig tcfg;
    gps::TrackChannel ch(best.prn, best.doppler, tcfg);
    std::vector<gps::TrackEpoch> ep = ch.run(sig.data(), (std::size_t)f.gcount(), trackMs);
    std::printf("Tracked %d epochs.\n", (int)ep.size());

    // Bit synchronisation.
    gps::BitSync bs = gps::findBitSync(ep);
    std::printf("\nbit-sync transition histogram (phase : count):\n");
    {
        int hist[20] = {0};
        for (std::size_t i = 1; i < ep.size(); ++i)
            if ((ep[i].iP >= 0) != (ep[i-1].iP >= 0)) hist[i % 20]++;
        for (int k = 0; k < 20; ++k) {
            std::printf("%3d : %4d %s\n", k, hist[k],
                        (k == bs.offset) ? "  <-- bit boundary" : "");
        }
    }
    std::printf("\noffset = %d   peak = %d / %d transitions   %s\n",
                bs.offset, bs.peakCount, bs.totalTrans,
                bs.valid ? "VALID (sharp peak)" : "weak");

    // Demodulate to a 50 bps bit stream.
    std::vector<int> bits = gps::demodulateBits(ep, bs.offset < 0 ? 0 : bs.offset);
    std::printf("\n%d data bits (%.1f s @ 50 bps).\n", (int)bits.size(), bits.size() / 50.0);

    // Frame sync + parity + ephemeris decode.
    gps::NavDecode nd = gps::decodeNav(bits, best.prn);
    std::printf("\nFrame decode: %d subframes passed parity, %d preamble candidates failed parity, polarity = %s\n",
                nd.subframesOk, nd.parityFails, nd.polarity == 1 ? "inverted" : "upright");
    std::printf("Subframes captured: SF1=%s  SF2=%s  SF3=%s\n",
                nd.gotSf[1] ? "yes" : "no", nd.gotSf[2] ? "yes" : "no", nd.gotSf[3] ? "yes" : "no");

    if (!nd.eph.valid) {
        std::printf("\nDid not capture SF1+SF2+SF3 (need a full frame). Track longer.\n");
        return 1;
    }

    const gps::Ephemeris& e = nd.eph;
    const double R2D = 180.0 / 3.14159265358979;
    std::printf("\n=== Decoded ephemeris, PRN %d ===\n", e.prn);
    std::printf("  WN(mod1024)=%d   IODC=%d   IODE=%d\n", e.weekNumber, e.iodc, e.iode);
    std::printf("  toe=%.0f s   toc=%.0f s\n", e.toe, e.toc);
    std::printf("  sqrtA=%.5f sqrt(m)   -> a=%.1f km\n", e.sqrtA, e.sqrtA * e.sqrtA / 1000.0);
    std::printf("  e=%.10f\n", e.ecc);
    std::printf("  i0=%.7f rad (%.3f deg)\n", e.i0, e.i0 * R2D);
    std::printf("  M0=%.7f   omega=%.7f   OMEGA0=%.7f rad\n", e.m0, e.omega, e.omega0);
    std::printf("  deltaN=%.4e   OMEGADOT=%.4e   IDOT=%.4e rad/s\n", e.deltaN, e.omegaDot, e.idot);
    std::printf("  af0=%.4e s  af1=%.4e s/s  af2=%.4e s/s^2  TGD=%.4e s\n", e.af0, e.af1, e.af2, e.tgd);
    std::printf("  Crs=%.2f Crc=%.2f m   Cuc=%.3e Cus=%.3e Cic=%.3e Cis=%.3e rad\n",
                e.crs, e.crc, e.cuc, e.cus, e.cic, e.cis);

    const bool sane = (e.ecc > 0.0 && e.ecc < 0.03)
                   && (e.sqrtA > 5000.0 && e.sqrtA < 5300.0)
                   && (e.i0 > 0.9 && e.i0 < 1.1)
                   && (e.toe >= 0.0 && e.toe <= 604800.0)
                   && (e.iode == (e.iodc & 0xFF));   // IODE must equal 8 LSBs of IODC
    std::printf("\n  SANITY: %s\n", sane ?
        "PASS - e in (0,0.03), sqrtA~5153, i0~0.97 rad, toe in week, IODE==IODC LSBs" :
        "FAIL - value(s) out of physical range");

    // Satellite ECEF position from the decoded ephemeris.
    gps::SatState ss = gps::satPosition(e, e.toe);
    const double rad = std::sqrt(ss.x*ss.x + ss.y*ss.y + ss.z*ss.z);
    std::printf("\n=== SV ECEF position at toe ===\n");
    std::printf("  X=%.1f  Y=%.1f  Z=%.1f km\n", ss.x/1e3, ss.y/1e3, ss.z/1e3);
    std::printf("  radius=%.1f km (GPS orbit ~26560 km)   SV clock bias=%.4e s\n",
                rad/1e3, ss.clockBias);
    const bool radOk = (rad > 25.0e6 && rad < 28.0e6);
    std::printf("  SV-POSITION SANITY: %s\n", radOk ? "PASS (on the GPS orbit sphere)" : "FAIL");

    std::printf("\n%s\n", (sane && radOk) ? "Nav decode + SV position OK." : "Out of range.");
    return (sane && radOk) ? 0 : 1;
}
