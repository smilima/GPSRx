//---------------------------------------------------------------------------
// Pvt.cpp - see Pvt.h
//---------------------------------------------------------------------------
#include "Pvt.h"
#include <cmath>

namespace gps {

static const double MU       = 3.986005e14;        // WGS-84 earth gravitational constant (m^3/s^2)
static const double OMEGA_E  = 7.2921151467e-5;    // earth rotation rate (rad/s)
static const double F_REL    = -4.442807633e-10;   // relativistic clock constant
static const double C_LIGHT  = 299792458.0;        // m/s
static const double PVT_PI   = 3.14159265358979323846;

static double wrapWeek(double t)   // wrap a time difference to +/- half a week
{
    if (t >  302400.0) t -= 604800.0;
    else if (t < -302400.0) t += 604800.0;
    return t;
}

//---------------------------------------------------------------------------
SatState satPosition(const Ephemeris& e, double t)
{
    SatState s;
    const double A   = e.sqrtA * e.sqrtA;          // semi-major axis
    const double n0  = std::sqrt(MU / (A * A * A));
    const double tk  = wrapWeek(t - e.toe);
    const double n   = n0 + e.deltaN;
    const double Mk  = e.m0 + n * tk;

    // Kepler's equation (fixed-point; converges fast for GPS e ~ 0.01).
    double Ek = Mk;
    for (int i = 0; i < 12; ++i) Ek = Mk + e.ecc * std::sin(Ek);

    const double vk   = std::atan2(std::sqrt(1.0 - e.ecc * e.ecc) * std::sin(Ek),
                                   std::cos(Ek) - e.ecc);
    const double Phik = vk + e.omega;
    const double s2 = std::sin(2.0 * Phik), c2 = std::cos(2.0 * Phik);

    const double uk = Phik + e.cus * s2 + e.cuc * c2;                 // arg of latitude
    const double rk = A * (1.0 - e.ecc * std::cos(Ek)) + e.crs * s2 + e.crc * c2;  // radius
    const double ik = e.i0 + e.idot * tk + e.cis * s2 + e.cic * c2;   // inclination

    const double xkp = rk * std::cos(uk);          // in-orbital-plane
    const double ykp = rk * std::sin(uk);
    const double Omegak = e.omega0 + (e.omegaDot - OMEGA_E) * tk - OMEGA_E * e.toe;

    s.x = xkp * std::cos(Omegak) - ykp * std::cos(ik) * std::sin(Omegak);
    s.y = xkp * std::sin(Omegak) + ykp * std::cos(ik) * std::cos(Omegak);
    s.z = ykp * std::sin(ik);

    const double dtc = wrapWeek(t - e.toc);
    const double dtr = F_REL * e.ecc * e.sqrtA * std::sin(Ek);        // relativistic
    s.clockBias = e.af0 + e.af1 * dtc + e.af2 * dtc * dtc + dtr - e.tgd;
    s.ek = Ek;
    return s;
}

//---------------------------------------------------------------------------
// Solve A x = b for a 4x4 system (Gaussian elimination, partial pivot).
static bool solve4(double A[4][4], double b[4], double x[4])
{
    for (int col = 0; col < 4; ++col) {
        int piv = col;
        for (int r = col + 1; r < 4; ++r)
            if (std::fabs(A[r][col]) > std::fabs(A[piv][col])) piv = r;
        if (std::fabs(A[piv][col]) < 1e-12) return false;
        if (piv != col) {
            for (int c = 0; c < 4; ++c) std::swap(A[piv][c], A[col][c]);
            std::swap(b[piv], b[col]);
        }
        for (int r = 0; r < 4; ++r) {
            if (r == col) continue;
            const double f = A[r][col] / A[col][col];
            for (int c = col; c < 4; ++c) A[r][c] -= f * A[col][c];
            b[r] -= f * b[col];
        }
    }
    for (int i = 0; i < 4; ++i) x[i] = b[i] / A[i][i];
    return true;
}

//---------------------------------------------------------------------------
PvtSolution solvePvt(const std::vector<double>& pr, const std::vector<SatState>& sats)
{
    PvtSolution sol;
    const int m = (int)pr.size();
    if (m < 4 || (int)sats.size() != m) return sol;

    double x = 0, y = 0, z = 0, b = 0;             // start at earth centre
    double lastAtA[4][4] = {{0}};

    for (int iter = 0; iter < 12; ++iter) {
        double AtA[4][4] = {{0}}, Atb[4] = {0}, rss = 0;

        for (int i = 0; i < m; ++i) {
            // First-pass range to get the signal travel time, then rotate the
            // satellite ECEF by the earth rotation during that travel (Sagnac).
            const double dx0 = sats[i].x - x, dy0 = sats[i].y - y, dz0 = sats[i].z - z;
            const double rho0 = std::sqrt(dx0*dx0 + dy0*dy0 + dz0*dz0);
            const double th = OMEGA_E * (rho0 / C_LIGHT);
            const double ct = std::cos(th), st = std::sin(th);
            const double xs =  sats[i].x*ct + sats[i].y*st;
            const double ys = -sats[i].x*st + sats[i].y*ct;
            const double zs =  sats[i].z;

            const double dx = xs - x, dy = ys - y, dz = zs - z;
            const double rho = std::sqrt(dx*dx + dy*dy + dz*dz);

            // Corrected observation: raw pseudorange + SV clock advance.
            const double obs = pr[i] + C_LIGHT * sats[i].clockBias;
            const double res = obs - (rho + b);

            const double row[4] = { -dx/rho, -dy/rho, -dz/rho, 1.0 };
            for (int r = 0; r < 4; ++r) {
                Atb[r] += row[r] * res;
                for (int c = 0; c < 4; ++c) AtA[r][c] += row[r] * row[c];
            }
            rss += res * res;
        }

        for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) lastAtA[r][c] = AtA[r][c];

        double A2[4][4], b2[4], dxv[4];
        for (int r = 0; r < 4; ++r) { b2[r] = Atb[r]; for (int c = 0; c < 4; ++c) A2[r][c] = AtA[r][c]; }
        if (!solve4(A2, b2, dxv)) break;

        x += dxv[0]; y += dxv[1]; z += dxv[2]; b += dxv[3];
        sol.iterations = iter + 1;
        sol.residRms = std::sqrt(rss / m);
        if (std::fabs(dxv[0]) + std::fabs(dxv[1]) + std::fabs(dxv[2]) + std::fabs(dxv[3]) < 1e-4)
            break;
    }

    // GDOP = sqrt(trace((A^T A)^-1)).
    double inv[4][4], col[4], e[4];
    bool dopOk = true;
    double trace = 0;
    for (int j = 0; j < 4 && dopOk; ++j) {
        double A2[4][4];
        for (int r = 0; r < 4; ++r) { e[r] = (r == j) ? 1.0 : 0.0; for (int c = 0; c < 4; ++c) A2[r][c] = lastAtA[r][c]; }
        if (!solve4(A2, e, col)) { dopOk = false; break; }
        for (int r = 0; r < 4; ++r) inv[r][j] = col[r];
    }
    if (dopOk) { for (int i = 0; i < 4; ++i) trace += inv[i][i]; sol.gdop = std::sqrt(trace > 0 ? trace : 0); }

    sol.ok = true;
    sol.x = x; sol.y = y; sol.z = z; sol.clockBias = b;
    ecefToGeodetic(x, y, z, sol.lat, sol.lon, sol.alt);
    return sol;
}

//---------------------------------------------------------------------------
void ecefToGeodetic(double x, double y, double z, double& latDeg, double& lonDeg, double& altM)
{
    const double a  = 6378137.0;                   // WGS-84 semi-major axis
    const double f  = 1.0 / 298.257223563;
    const double e2 = f * (2.0 - f);

    const double p = std::sqrt(x * x + y * y);
    double lat = std::atan2(z, p * (1.0 - e2));    // initial
    double alt = 0.0;
    for (int i = 0; i < 8; ++i) {
        const double sinl = std::sin(lat);
        const double N = a / std::sqrt(1.0 - e2 * sinl * sinl);
        alt = p / std::cos(lat) - N;
        lat = std::atan2(z, p * (1.0 - e2 * N / (N + alt)));
    }
    latDeg = lat * 180.0 / PVT_PI;
    lonDeg = std::atan2(y, x) * 180.0 / PVT_PI;
    altM   = alt;
}

} // namespace gps
