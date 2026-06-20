//---------------------------------------------------------------------------
// Fft.h - Minimal FFT support for the GPS software receiver.
//
// Provides:
//   * Fft::transform()  - in-place iterative radix-2 FFT (length must be 2^k)
//   * BluesteinFft      - DFT/IDFT of ARBITRARY length N, built on the radix-2
//                         core via Bluestein's chirp-z algorithm.
//
// We need arbitrary-length transforms because one 1 ms C/A code period at
// fs = 38.192 MHz is 38192 samples, which is not a power of two
// (38192 = 16 * 7 * 11 * 31). Bluestein lets us do exact 38192-point circular
// correlation for the parallel-code-phase acquisition search.
//
// Pure standard C++ (no VCL) so the same source compiles in the console test
// harness and in the C++Builder VCL application.
//---------------------------------------------------------------------------
#ifndef FftH
#define FftH

#include <vector>
#include <complex>

namespace dsp {

typedef std::complex<double> cd;

struct Fft {
    // In-place radix-2 FFT. a.size() MUST be a power of two.
    // inverse == false : forward DFT  X[k] = sum x[n] exp(-2pi i nk/N)
    // inverse == true  : inverse DFT  x[n] = (1/N) sum X[k] exp(+2pi i nk/N)
    static void transform(std::vector<cd>& a, bool inverse);

    // Smallest power of two >= n.
    static int nextPow2(int n);
};

// DFT of arbitrary length N via Bluestein's algorithm.
// Construct once for a given N (precomputes the chirp and its transform),
// then call forward()/inverse() as many times as needed.
class BluesteinFft {
public:
    explicit BluesteinFft(int n);

    int size() const { return n_; }

    // Forward DFT:  out[k] = sum_n in[n] exp(-2pi i nk/N)
    void forward(const std::vector<cd>& in, std::vector<cd>& out) const;

    // Inverse DFT:  out[n] = (1/N) sum_k in[k] exp(+2pi i nk/N)
    void inverse(const std::vector<cd>& in, std::vector<cd>& out) const;

private:
    void run(const std::vector<cd>& in, std::vector<cd>& out, int sign) const;

    int n_;                 // logical transform length
    int m_;                 // padded power-of-two convolution length (>= 2N-1)
    std::vector<cd> chirp_; // a[k] = exp(-i*pi*k^2/N), k = 0..N-1
    std::vector<cd> vb_;    // FFT of the zero-padded conjugate-chirp kernel
};

} // namespace dsp

#endif
