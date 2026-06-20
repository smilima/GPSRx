//---------------------------------------------------------------------------
// Fft.cpp - see Fft.h
//---------------------------------------------------------------------------
#include "Fft.h"
#include <cmath>
#include <stdexcept>

namespace dsp {

static const double TWO_PI = 6.283185307179586476925286766559;
static const double PI     = 3.141592653589793238462643383279;

int Fft::nextPow2(int n)
{
    int p = 1;
    while (p < n) p <<= 1;
    return p;
}

void Fft::transform(std::vector<cd>& a, bool inverse)
{
    const int n = (int)a.size();
    if (n <= 1) return;
    if (n & (n - 1))
        throw std::invalid_argument("Fft::transform: size must be a power of two");

    // Bit-reversal permutation.
    for (int i = 1, j = 0; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }

    // Danielson-Lanczos butterflies.
    for (int len = 2; len <= n; len <<= 1) {
        double ang = (inverse ? TWO_PI : -TWO_PI) / len;
        cd wlen(std::cos(ang), std::sin(ang));
        for (int i = 0; i < n; i += len) {
            cd w(1.0, 0.0);
            for (int k = 0; k < len / 2; ++k) {
                cd u = a[i + k];
                cd v = a[i + k + len / 2] * w;
                a[i + k]           = u + v;
                a[i + k + len / 2] = u - v;
                w *= wlen;
            }
        }
    }

    if (inverse) {
        for (int i = 0; i < n; ++i)
            a[i] /= (double)n;
    }
}

//---------------------------------------------------------------------------
// Bluestein
//---------------------------------------------------------------------------
BluesteinFft::BluesteinFft(int n)
    : n_(n)
{
    if (n_ < 1) throw std::invalid_argument("BluesteinFft: n must be >= 1");

    m_ = Fft::nextPow2(2 * n_ - 1);

    // Chirp a[k] = exp(-i*pi*k^2/N). Reduce k^2 mod 2N before forming the
    // angle so the argument stays small and accurate for large k.
    chirp_.resize(n_);
    for (int k = 0; k < n_; ++k) {
        long long kk = ((long long)k * (long long)k) % (2LL * n_);
        double ang = -PI * (double)kk / (double)n_;
        chirp_[k] = cd(std::cos(ang), std::sin(ang));
    }

    // Kernel v: linear-convolution image of conj(chirp), zero padded to M with
    // the negative-lag half wrapped to the top of the buffer.
    std::vector<cd> v(m_, cd(0.0, 0.0));
    v[0] = std::conj(chirp_[0]);
    for (int k = 1; k < n_; ++k) {
        cd g = std::conj(chirp_[k]);
        v[k]      = g;
        v[m_ - k] = g;
    }
    Fft::transform(v, false);
    vb_ = std::move(v);
}

void BluesteinFft::run(const std::vector<cd>& in, std::vector<cd>& out, int sign) const
{
    // sign = -1 forward, +1 inverse (caller applies the 1/N scaling).
    // For the inverse direction we use IDFT(x) = conj(DFT(conj(x)))/N, so this
    // routine itself always evaluates the forward chirp transform; 'sign' just
    // selects whether the caller pre/post-conjugates.
    std::vector<cd> u(m_, cd(0.0, 0.0));
    for (int k = 0; k < n_; ++k)
        u[k] = in[k] * chirp_[k];

    Fft::transform(u, false);
    for (int i = 0; i < m_; ++i)
        u[i] *= vb_[i];
    Fft::transform(u, true); // inverse radix-2 (includes 1/M)

    out.resize(n_);
    for (int k = 0; k < n_; ++k)
        out[k] = u[k] * chirp_[k];

    (void)sign;
}

void BluesteinFft::forward(const std::vector<cd>& in, std::vector<cd>& out) const
{
    run(in, out, -1);
}

void BluesteinFft::inverse(const std::vector<cd>& in, std::vector<cd>& out) const
{
    // IDFT(X) = conj( DFT( conj(X) ) ) / N
    std::vector<cd> cin(n_);
    for (int k = 0; k < n_; ++k)
        cin[k] = std::conj(in[k]);

    run(cin, out, +1);

    const double inv = 1.0 / (double)n_;
    for (int k = 0; k < n_; ++k)
        out[k] = std::conj(out[k]) * inv;
}

} // namespace dsp
