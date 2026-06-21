//---------------------------------------------------------------------------
// Tracking.cpp - see Tracking.h
//---------------------------------------------------------------------------
#include "Tracking.h"
#include "CACode.h"

#include <cmath>
#include <algorithm>

namespace gps {

static const double TWO_PI = 6.283185307179586476925286766559;

// 2nd-order loop-filter coefficients from noise bandwidth / damping / gain.
static void calcLoopCoef(double LBW, double zeta, double k,
                         double& tau1, double& tau2)
{
    double Wn = LBW * 8.0 * zeta / (4.0 * zeta * zeta + 1.0);
    tau1 = k / (Wn * Wn);
    tau2 = 2.0 * zeta / Wn;
}

TrackChannel::TrackChannel(int prn, double dopplerHz, const TrackConfig& cfg)
    : prn_(prn), cfg_(cfg)
{
    // Local C/A replica as +/-1 doubles (1023 chips; floor+mod handles wrap).
    std::vector<std::int8_t> ca = generateCACode(prn);
    code_.resize(CA_CODE_LENGTH);
    for (int i = 0; i < CA_CODE_LENGTH; ++i) code_[i] = (double)ca[i];

    carrFreqBasis_ = cfg.ifFreq + dopplerHz;
    carrFreq_      = carrFreqBasis_;
    remCarrPhase_  = 0.0;
    oldCarrNco_    = 0.0;
    oldCarrError_  = 0.0;

    codeFreq_      = CA_CHIP_RATE;     // 1.023 MHz basis
    remCodePhase_  = 0.0;
    oldCodeNco_    = 0.0;
    oldCodeError_  = 0.0;

    calcLoopCoef(cfg.pllBW, cfg.pllZeta, 0.25, tau1carr_, tau2carr_);
    calcLoopCoef(cfg.dllBW, cfg.dllZeta, 1.00, tau1code_, tau2code_);

    nbpI_ = nbpQ_ = wbp_ = 0.0;
    cn0Count_ = 0;
    cn0_ = 0.0;
}

std::vector<TrackEpoch> TrackChannel::run(const std::int8_t* sig,
                                          std::size_t numSamples, int numMs,
                                          const std::atomic<bool>* abort)
{
    std::vector<TrackEpoch> out;
    out.reserve((std::size_t)numMs);

    const double half = cfg_.dllCorrSpacing * 0.5;   // early/late offset (chips)
    std::size_t idx = 0;

    for (int ms = 0; ms < numMs; ++ms) {
        if (abort && abort->load(std::memory_order_relaxed)) break;   // responsive shutdown
        const double codePhaseStep = codeFreq_ / cfg_.fs;       // chips per sample
        const int blksize =
            (int)std::ceil((CA_CODE_LENGTH - remCodePhase_) / codePhaseStep);
        if (idx + (std::size_t)blksize > numSamples) break;

        // Local carrier by incremental phasor rotation (no per-sample trig).
        const double carrStep = TWO_PI * carrFreq_ / cfg_.fs;
        double cs = std::cos(remCarrPhase_), sn = std::sin(remCarrPhase_);
        const double rc = std::cos(carrStep), rs = std::sin(carrStep);

        double I_E = 0, Q_E = 0, I_P = 0, Q_P = 0, I_L = 0, Q_L = 0;

        for (int i = 0; i < blksize; ++i) {
            const double raw = (double)sig[idx + i];
            const double ib = sn * raw;   // carrSin -> in-phase   (SoftGNSS convention)
            const double qb = cs * raw;   // carrCos -> quadrature

            const double cp = remCodePhase_ + i * codePhaseStep;
            int ip = (int)std::floor(cp);
            int ie = (int)std::floor(cp - half);
            int il = (int)std::floor(cp + half);
            ip %= CA_CODE_LENGTH; if (ip < 0) ip += CA_CODE_LENGTH;
            ie %= CA_CODE_LENGTH; if (ie < 0) ie += CA_CODE_LENGTH;
            il %= CA_CODE_LENGTH; if (il < 0) il += CA_CODE_LENGTH;
            const double cE = code_[ie], cP = code_[ip], cL = code_[il];

            I_E += ib * cE; Q_E += qb * cE;
            I_P += ib * cP; Q_P += qb * cP;
            I_L += ib * cL; Q_L += qb * cL;

            const double ncs = cs * rc - sn * rs;   // advance phasor
            sn = sn * rc + cs * rs;
            cs = ncs;
        }

        remCarrPhase_ = std::fmod(remCarrPhase_ + carrStep * blksize, TWO_PI);
        remCodePhase_ = remCodePhase_ + blksize * codePhaseStep - CA_CODE_LENGTH;

        // Carrier loop: Costas discriminator + 2nd-order filter.
        const double carrError = (I_P != 0.0 ? std::atan(Q_P / I_P) : 0.0) / TWO_PI;
        const double carrNco = oldCarrNco_
            + (tau2carr_ / tau1carr_) * (carrError - oldCarrError_)
            + carrError * (cfg_.pdi / tau1carr_);
        oldCarrNco_ = carrNco;
        oldCarrError_ = carrError;
        carrFreq_ = carrFreqBasis_ + carrNco;

        // Code loop: normalized early-minus-late power discriminator + filter.
        const double E = std::sqrt(I_E * I_E + Q_E * Q_E);
        const double L = std::sqrt(I_L * I_L + Q_L * Q_L);
        const double codeError = ((E + L) > 0.0) ? (E - L) / (E + L) : 0.0;
        const double codeNco = oldCodeNco_
            + (tau2code_ / tau1code_) * (codeError - oldCodeError_)
            + codeError * (cfg_.pdi / tau1code_);
        oldCodeNco_ = codeNco;
        oldCodeError_ = codeError;
        codeFreq_ = CA_CHIP_RATE - codeNco;

        // C/N0 via narrowband/wideband power ratio over a 20 ms window.
        nbpI_ += I_P; nbpQ_ += Q_P;
        wbp_  += I_P * I_P + Q_P * Q_P;
        if (++cn0Count_ >= 20) {
            const double nbp = nbpI_ * nbpI_ + nbpQ_ * nbpQ_;
            double mu = (wbp_ > 0.0) ? nbp / wbp_ : 0.0;          // 1..20
            // Cap the NWPR ratio: for a very clean lock mu -> 20 and the
            // estimator blows up (denominator -> 0). Clamp to ~50 dB-Hz.
            mu = std::min(mu, 19.8);
            if (mu > 1.0)
                cn0_ = 10.0 * std::log10((mu - 1.0) / (20.0 - mu) / cfg_.pdi);
            nbpI_ = nbpQ_ = wbp_ = 0.0;
            cn0Count_ = 0;
        }

        TrackEpoch ep;
        ep.iP = I_P; ep.qP = Q_P;
        ep.iE = I_E; ep.qE = Q_E;
        ep.iL = I_L; ep.qL = Q_L;
        ep.doppler      = carrFreq_ - cfg_.ifFreq;
        ep.codeFreq     = codeFreq_;
        ep.remCodePhase = remCodePhase_;
        ep.pllDisc      = carrError;
        ep.dllDisc      = codeError;
        ep.cn0          = cn0_;
        ep.sampleIndex  = idx;          // buffer offset at the start of this 1 ms
        out.push_back(ep);

        idx += (std::size_t)blksize;
    }

    return out;
}

} // namespace gps
