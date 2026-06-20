//---------------------------------------------------------------------------
// Pvt.h - GPS position/velocity/time from ephemeris + pseudoranges.
//
//   * satPosition()  - broadcast ephemeris -> satellite ECEF position + SV clock
//                      correction (IS-GPS-200 user algorithm, Table 20-IV).
//   * solvePvt()     - least-squares receiver position + clock bias from
//                      pseudoranges + satellite positions (with Sagnac correction).
//   * ecefToGeodetic - ECEF -> WGS-84 latitude/longitude/altitude.
//
// Pure standard C++ (no VCL).
//---------------------------------------------------------------------------
#ifndef PvtH
#define PvtH

#include <vector>
#include "NavMessage.h"   // gps::Ephemeris

namespace gps {

struct SatState {
    double x = 0, y = 0, z = 0;   // ECEF position at transmit time (m)
    double clockBias = 0;         // SV clock correction dt_sv (s): af0/1/2 + relativistic - TGD
    double ek = 0;                // eccentric anomaly (diagnostic)
};

// Satellite ECEF position + clock correction at GPS system time 'transmitTime'
// (seconds of week).
SatState satPosition(const Ephemeris& eph, double transmitTime);

struct PvtSolution {
    bool   ok = false;
    double x = 0, y = 0, z = 0;   // receiver ECEF (m)
    double clockBias = 0;         // receiver clock bias (m)
    double lat = 0, lon = 0, alt = 0;  // WGS-84 (deg, deg, m)
    double gdop = 0;
    int    iterations = 0;
    double residRms = 0;          // RMS of post-fit residuals (m)
};

// Least-squares fix from >= 4 (pseudorange, satellite) pairs. satClock is the SV
// clock correction (s) already applied to each pseudorange's satellite.
PvtSolution solvePvt(const std::vector<double>& pseudoranges,
                     const std::vector<SatState>& sats);

void ecefToGeodetic(double x, double y, double z,
                    double& latDeg, double& lonDeg, double& altM);

} // namespace gps

#endif
