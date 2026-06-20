//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "MainUnit.h"
#include <fstream>
#include <vector>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <string>
#include <algorithm>
#include <exception>
#include <map>
#include <utility>
#include "Acquisition.h"
#include "Tracking.h"
#include "NavMessage.h"
#include "CACode.h"
#include "Fft.h"
#include "Pvt.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TMainForm *MainForm;

//---------------------------------------------------------------------------
// Acquisition config from the file name.
//
// Our captures encode their sample rate / IF in the file name, e.g.
//   GPSdata-DiscreteComponents-fs38_192-if9_55.bin  ->  fs=38.192 MHz, IF=9.55 MHz
// ('_' is the decimal point, value in MHz). When the tags are absent (e.g. a
// generic .sim) we fall back to the AcqConfig defaults (38.192 / 9.55 MHz).
//---------------------------------------------------------------------------
static bool extractMhz(const String& name, const String& tag, double& outHz)
{
    int p = name.Pos(tag);                 // 1-based; 0 if not found
    if (p == 0) return false;
    p += tag.Length();
    String num;
    while (p <= name.Length())
    {
        wchar_t c = name[p];
        if (c >= L'0' && c <= L'9') num += c;
        else if (c == L'_')         num += L'.';   // '_' is the decimal point
        else break;
        ++p;
    }
    if (num.IsEmpty()) return false;
    try { outHz = std::stod(std::string(AnsiString(num).c_str())) * 1.0e6; }
    catch (...) { return false; }
    return true;
}

static gps::AcqConfig configForFile(const String& path)
{
    gps::AcqConfig cfg;                     // defaults: fs=38.192 MHz, IF=9.55 MHz
    const String name = ExtractFileName(path);
    double v;
    if (extractMhz(name, L"fs", v)) cfg.fs     = v;
    if (extractMhz(name, L"if", v)) cfg.ifFreq = v;
    return cfg;
}

//---------------------------------------------------------------------------
// TAcqThread - runs the full PRN 1..32 sky search OFF the UI thread.
//
// The acquisition is CPU-heavy (seconds, longer in a Debug build), so running
// it on the main thread freezes the window. This worker posts status lines to
// the memo and the final 32-PRN result set to the chart via Synchronize(),
// keeping the UI responsive. It frees itself (FreeOnTerminate) and re-enables
// the button when finished.
//---------------------------------------------------------------------------
class TAcqThread : public TThread
{
public:
    __fastcall TAcqThread(TMainForm* AForm, const String& AFile, const gps::AcqConfig& ACfg)
        : TThread(true), fForm(AForm), fFile(AFile), fCfg(ACfg)
    {
        FreeOnTerminate = true;
        // NB: caller calls Start() AFTER construction - calling it here trips
        // "Cannot call Start on a running or suspended thread".
    }

protected:
    void __fastcall Execute();

private:
    TMainForm*                  fForm;
    String                      fFile;
    gps::AcqConfig              fCfg;
    String                      fPending;
    std::vector<gps::AcqResult> fAll;

    void status(const String& s) { fPending = s; Synchronize(doStatus); }
    void __fastcall doStatus()   { fForm->Status(fPending); }
    void __fastcall doPublish()  { fForm->ApplyResults(fAll); }
    void __fastcall reenable()
    {
        fForm->btnAcquire->Caption = L"Acquire";
        fForm->btnAcquire->Enabled = true;
    }
};

void __fastcall TAcqThread::Execute()
{
    status(L"Acquiring " + ExtractFileName(fFile) + L" ...");
    {
        char hbuf[160];
        std::snprintf(hbuf, sizeof(hbuf),
            "  fs=%.3f MHz  IF=%.3f MHz  Doppler +/-%.0f Hz  %d ms",
            fCfg.fs / 1e6, fCfg.ifFreq / 1e6, fCfg.dopplerMax, fCfg.numMs);
        status(String(hbuf));
    }

    const int n = (int)std::lround(fCfg.fs * 1.0e-3);   // samples per 1 ms
    const std::size_t need = (std::size_t)n * fCfg.numMs;

    // Read the first numMs milliseconds of int8 samples.
    std::vector<std::int8_t> sig(need);
    {
        std::ifstream f(AnsiString(fFile).c_str(), std::ios::binary);
        if (!f)
        {
            status(L"ERROR: cannot open file.");
            Synchronize(reenable);
            return;
        }
        f.read(reinterpret_cast<char*>(sig.data()), (std::streamsize)need);
        if ((std::size_t)f.gcount() < need)
        {
            status(L"ERROR: short read (file too small?).");
            Synchronize(reenable);
            return;
        }
    }

    try
    {
        // Stream a status line as each satellite is found; the full 32-PRN set
        // goes to the chart at the end.
        fAll = gps::acquireAll(sig.data(), sig.size(), fCfg,
            [&](const gps::AcqResult& r)
            {
                if (Terminated) return;
                if (r.found)
                {
                    char buf[160];
                    std::snprintf(buf, sizeof(buf),
                        "  PRN %2d acquired:  Doppler %+6.0f Hz,  %.1f chips,  ratio %.1f",
                        r.prn, r.doppler, r.codePhaseChips, r.peakRatio);
                    status(String(buf));
                }
            });

        int nfound = 0;
        for (std::size_t i = 0; i < fAll.size(); ++i)
            if (fAll[i].found) ++nfound;

        Synchronize(doPublish);   // fill the chart + cache results (main thread)

        char sbuf[80];
        std::snprintf(sbuf, sizeof(sbuf),
                      "Acquisition complete: %d satellites acquired.", nfound);
        status(String(sbuf));
    }
    catch (const std::exception& e)
    {
        status(String(L"ERROR: ") + String(e.what()));
    }

    Synchronize(reenable);
}

//---------------------------------------------------------------------------
__fastcall TMainForm::TMainForm(TComponent* Owner)
    : TForm(Owner)
{
    FHaveResults = false;
    FTrackChartsBuilt = false;

    OpenDialog1->Filter =
        L"GPS data (*.bin;*.sim)|*.bin;*.sim|All files (*.*)|*.*";
    OpenDialog1->Options = OpenDialog1->Options << ofFileMustExist;

    // Bar chart of all 32 PRNs; hover wired in code so no .dfm change is needed.
    Series1->Marks->Visible = false;
    Chart1->OnMouseMove = Chart1MouseMove;
    Chart1->ShowHint = true;
    Chart1->Hint = L"Hover a bar for PRN details";   // arm the hint so dynamic text can show
    Application->HintHidePause = 30000;              // keep the PRN tooltip up ~30 s (default 2.5 s)

    Status(L"Ready. Open a .bin or .sim capture from File > Open, then click Acquire.");
}
//---------------------------------------------------------------------------
void TMainForm::Status(const String& s)
{
    memoResults->Lines->Add(s);
}
//---------------------------------------------------------------------------
void TMainForm::ApplyResults(const std::vector<gps::AcqResult>& results)
{
    for (int i = 0; i <= 32; ++i) FResults[i] = gps::AcqResult();

    Series1->Clear();
    for (std::size_t i = 0; i < results.size(); ++i)
    {
        const gps::AcqResult& r = results[i];
        if (r.prn >= 1 && r.prn <= 32) FResults[r.prn] = r;
        TColor c = r.found ? (TColor)clGreen : (TColor)clSilver;
        Series1->Add(r.peakRatio, IntToStr(r.prn), c);
    }
    FHaveResults = true;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::Chart1MouseMove(TObject *Sender, TShiftState Shift,
                                           int X, int Y)
{
    if (!FHaveResults) return;

    int idx = Series1->Clicked(X, Y);   // bar under the cursor, -1 if none
    if (idx >= 0)
    {
        int prn = idx + 1;              // bars are added in PRN order 1..32
        if (prn >= 1 && prn <= 32)
        {
            const gps::AcqResult& r = FResults[prn];
            char buf[200];
            std::snprintf(buf, sizeof(buf),
                "PRN %d: %s   Doppler %+.0f Hz   phase %.1f chips   peak/2nd %.2f",
                prn, r.found ? "ACQUIRED" : "not found",
                r.doppler, r.codePhaseChips, r.peakRatio);
            String h = String(buf);
            if (Chart1->Hint != h)
            {
                Chart1->Hint = h;
                Application->CancelHint();   // force the tooltip to refresh
            }
            Chart1->ShowHint = true;
            return;
        }
    }
    Chart1->ShowHint = false;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::File2Click(TObject *Sender)
{
    if (!OpenDialog1->Execute()) return;

    FFilePath = OpenDialog1->FileName;
    editFile->Text = ExtractFileName(FFilePath);   // name only; full path kept in FFilePath

    FHaveResults = false;
    Series1->Clear();
    Status(L"Opened " + ExtractFileName(FFilePath));
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::btnAcquireClick(TObject *Sender)
{
    if (FFilePath.IsEmpty())
    {
        Status(L"No file loaded. Use File > Open to select a .bin or .sim capture.");
        return;
    }

    btnAcquire->Enabled = false;
    btnAcquire->Caption = L"Working...";

    gps::AcqConfig cfg = configForFile(FFilePath);  // fs/IF from the file name
    cfg.numMs       = 2;        // 1 ms records summed non-coherently
    cfg.dopplerStep = 500.0;    // Hz
    cfg.threshold   = 2.5;      // peak/2nd-peak ratio to declare a lock

    TAcqThread* worker = new TAcqThread(this, FFilePath, cfg);
    worker->Start();            // start AFTER construction
}

//---------------------------------------------------------------------------
// Tracking tab
//---------------------------------------------------------------------------
// Worker: track every acquired satellite (off the UI thread), pushing each
// channel's results to the form via Synchronize as it finishes.
class TTrackThread : public TThread
{
public:
    __fastcall TTrackThread(TMainForm* AForm, const String& AFile,
                            const std::vector<gps::AcqResult>& ASats, int ANumMs)
        : TThread(true), fForm(AForm), fFile(AFile), fSats(ASats), fNumMs(ANumMs)
    {
        FreeOnTerminate = true;
    }

protected:
    void __fastcall Execute();

private:
    TMainForm*                  fForm;
    String                      fFile;
    std::vector<gps::AcqResult> fSats;
    int                         fNumMs;
    String                      fPending;
    GuiTrackedChannel           fCur;

    void status(const String& s) { fPending = s; Synchronize(doStatus); }
    void __fastcall doStatus()   { fForm->Status(fPending); }
    void __fastcall doAdd()      { fForm->addTrackedChannel(fCur); }
    void __fastcall reenable()
    {
        fForm->btnTrack->Caption = L"Track Acquired";
        fForm->btnTrack->Enabled = true;
    }
};

void __fastcall TTrackThread::Execute()
{
    std::ifstream f(AnsiString(fFile).c_str(), std::ios::binary);
    if (!f) { status(L"ERROR: cannot open file for tracking."); Synchronize(reenable); return; }

    gps::TrackConfig tcfg;
    gps::AcqConfig   rcfg;                 // fine-Doppler refinement uses the same fs/IF
    rcfg.fs = tcfg.fs; rcfg.ifFreq = tcfg.ifFreq;
    const int n = (int)std::lround(tcfg.fs * 1.0e-3);
    const std::size_t need = (std::size_t)(fNumMs + 2) * n;
    std::vector<std::int8_t> sig(need);

    for (std::size_t k = 0; k < fSats.size(); ++k) {
        if (Terminated) break;
        const gps::AcqResult& a = fSats[k];

        char b[80];
        std::snprintf(b, sizeof(b), "  tracking PRN %d ...", a.prn);
        status(String(b));

        f.clear();
        f.seekg((std::streamoff)a.codePhaseSamp, std::ios::beg);
        f.read(reinterpret_cast<char*>(sig.data()), (std::streamsize)need);
        const std::size_t got = (std::size_t)f.gcount();

        const double fd = gps::refineDoppler(sig.data(), got, a.prn, a.doppler, rcfg);
        gps::TrackChannel ch(a.prn, fd, tcfg);
        std::vector<gps::TrackEpoch> eps = ch.run(sig.data(), got, fNumMs);

        GuiTrackedChannel gc;
        gc.prn = a.prn;
        gc.epochs = std::move(eps);

        const int N = (int)gc.epochs.size();
        const int s = (N > 100) ? 100 : 0;
        double sumAbsIP = 0, sumAbsQP = 0, dMin = 1e9, dMax = -1e9, dSum = 0;
        for (int i = s; i < N; ++i) {
            sumAbsIP += std::fabs(gc.epochs[i].iP);
            sumAbsQP += std::fabs(gc.epochs[i].qP);
            dSum += gc.epochs[i].doppler;
            if (gc.epochs[i].doppler < dMin) dMin = gc.epochs[i].doppler;
            if (gc.epochs[i].doppler > dMax) dMax = gc.epochs[i].doppler;
        }
        const int m = N - s;
        gc.finalDoppler = (m > 0) ? dSum / m : 0;

        // Bit-edge-robust C/N0 + lock: bit-sync, then align the C/N0 windows to the
        // nav-bit boundary (no straddling), and gate lock on the I/Q power ratio +
        // a stable Doppler + a valid bit sync (not on the C/N0 number).
        gps::BitSync bsync = gps::findBitSync(gc.epochs);
        gc.avgCn0 = gps::estimateCN0(gc.epochs, bsync.valid ? bsync.offset : 0);
        const double ratio = sumAbsIP / (sumAbsQP > 0 ? sumAbsQP : 1);
        gc.locked = (m > 0) && (ratio > 3.0) && ((dMax - dMin) < 200.0) && bsync.valid;

        fCur = std::move(gc);
        Synchronize(doAdd);
    }
    status(L"Tracking complete.");
    Synchronize(reenable);
}

//---------------------------------------------------------------------------
void TMainForm::buildTrackingCharts()
{
    if (FTrackChartsBuilt) return;

    // Container filling the area to the right of the channel grid. Its children
    // are laid out with VCL alignment, so the whole block fills the tab and
    // resizes with the window.
    TPanel* pnl = new TPanel(this);
    pnl->Parent = tsTracking;
    pnl->Caption = L"";
    pnl->BevelOuter = bvNone;
    const int gx = sgChannels->Left + sgChannels->Width + 8;
    pnl->SetBounds(gx, sgChannels->Top,
                   tsTracking->ClientWidth  - gx - 6,
                   tsTracking->ClientHeight - sgChannels->Top - 6);
    pnl->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;

    // Top row (upper half): square I/Q constellation on the left, wide Prompt-I
    // time plot filling the rest.
    TPanel* pnlTop = new TPanel(this);
    pnlTop->Parent = pnl;
    pnlTop->Caption = L"";
    pnlTop->BevelOuter = bvNone;
    pnlTop->Height = pnl->ClientHeight / 2;
    pnlTop->Align = alTop;

    // I/Q constellation - square (width = row height), two clusters when locked.
    FchIQ = new TChart(this);
    FchIQ->Parent = pnlTop;
    FchIQ->View3D = false;
    FchIQ->Legend->Visible = false;
    FchIQ->Title->Text->Text = L"I/Q constellation";
    FchIQ->Width = pnlTop->ClientHeight;
    FchIQ->Align = alLeft;
    FiqSeries = new TPointSeries(FchIQ);
    FchIQ->AddSeries(FiqSeries);

    // Prompt-I over time - the 50 bps nav-bit transitions (wide, fills the row).
    FchPromptI = new TChart(this);
    FchPromptI->Parent = pnlTop;
    FchPromptI->View3D = false;
    FchPromptI->Legend->Visible = false;
    FchPromptI->Title->Text->Text = L"Prompt I (nav bits)";
    FchPromptI->Align = alClient;
    FpromptSeries = new TFastLineSeries(FchPromptI);
    FchPromptI->AddSeries(FpromptSeries);

    // Bottom row (lower half): Doppler (left axis) + C/N0 (right axis) trends,
    // full width; legend BELOW the plot so it does not cover it.
    FchTrend = new TChart(this);
    FchTrend->Parent = pnl;
    FchTrend->View3D = false;
    FchTrend->Legend->Visible = true;
    FchTrend->Legend->Alignment = laBottom;
    FchTrend->Title->Text->Text = L"Doppler (Hz) / C/N0 (right)";
    FchTrend->Align = alClient;
    FdopSeries = new TFastLineSeries(FchTrend);
    FchTrend->AddSeries(FdopSeries);
    FdopSeries->Title = L"Doppler";
    Fcn0Series = new TFastLineSeries(FchTrend);
    FchTrend->AddSeries(Fcn0Series);
    Fcn0Series->Title = L"C/N0";
    Fcn0Series->VertAxis = aRightAxis;

    FTrackChartsBuilt = true;
}

//---------------------------------------------------------------------------
void TMainForm::plotChannel(int idx)
{
    if (!FTrackChartsBuilt || idx < 0 || idx >= (int)FTracked.size()) return;
    const std::vector<gps::TrackEpoch>& ep = FTracked[idx].epochs;
    const int N = (int)ep.size();

    FiqSeries->Clear();
    FpromptSeries->Clear();
    FdopSeries->Clear();
    Fcn0Series->Clear();

    const int s = (N > 50) ? 50 : 0;               // skip loop-settling for the scatter
    const int stepIQ = std::max(1, (N - s) / 800);
    for (int i = s; i < N; i += stepIQ)
        FiqSeries->AddXY(ep[i].iP, ep[i].qP, L"", clTeeColor);

    for (int i = 100; i < N && i < 400; ++i)       // 300 ms window of prompt-I
        FpromptSeries->AddXY((double)i, ep[i].iP, L"", clTeeColor);

    const int stepT = std::max(1, N / 600);
    for (int i = 0; i < N; i += stepT) {
        FdopSeries->AddXY((double)i, ep[i].doppler, L"", clTeeColor);
        Fcn0Series->AddXY((double)i, ep[i].cn0, L"", clTeeColor);
    }

    FchIQ->Title->Text->Text = String(L"PRN ") + IntToStr(FTracked[idx].prn) + L" I/Q";
}

//---------------------------------------------------------------------------
void TMainForm::addTrackedChannel(const GuiTrackedChannel& gc)
{
    FTracked.push_back(gc);
    const int row = (int)FTracked.size();          // row 0 is the header
    if (sgChannels->RowCount < row + 1) sgChannels->RowCount = row + 1;

    char b[40];
    sgChannels->Cells[0][row] = IntToStr(gc.prn);
    std::snprintf(b, sizeof(b), "%+.0f", gc.finalDoppler);
    sgChannels->Cells[1][row] = String(b);
    std::snprintf(b, sizeof(b), "%.1f", gc.avgCn0);
    sgChannels->Cells[2][row] = String(b);
    sgChannels->Cells[3][row] = gc.locked ? L"LOCKED" : L"--";

    char sb[120];
    std::snprintf(sb, sizeof(sb), "  PRN %d: %s  Doppler %+.0f Hz  C/N0 %.1f dB-Hz",
                  gc.prn, gc.locked ? "LOCKED" : "no lock", gc.finalDoppler, gc.avgCn0);
    Status(String(sb));

    if (FTracked.size() == 1) plotChannel(0);      // show the first one
}

//---------------------------------------------------------------------------
void __fastcall TMainForm::btnTrackClick(TObject *Sender)
{
    if (!FHaveResults) { Status(L"Acquire satellites first (Acquisition tab)."); return; }
    if (FFilePath.IsEmpty()) { Status(L"No file loaded."); return; }

    std::vector<gps::AcqResult> found;
    for (int prn = 1; prn <= 32; ++prn)
        if (FResults[prn].found) found.push_back(FResults[prn]);
    if (found.empty()) { Status(L"No acquired satellites to track."); return; }

    buildTrackingCharts();
    FTracked.clear();
    sgChannels->RowCount = 2;
    for (int r = 1; r < sgChannels->RowCount; ++r)
        for (int c = 0; c < sgChannels->ColCount; ++c) sgChannels->Cells[c][r] = L"";
    sgChannels->Cells[0][0] = L"PRN";
    sgChannels->Cells[1][0] = L"Doppler";
    sgChannels->Cells[2][0] = L"C/N0";
    sgChannels->Cells[3][0] = L"Status";

    btnTrack->Enabled = false;
    btnTrack->Caption = L"Tracking...";

    char b[80];
    std::snprintf(b, sizeof(b), "Tracking %d satellites (1000 ms each)...", (int)found.size());
    Status(String(b));

    TTrackThread* w = new TTrackThread(this, FFilePath, found, 1000);
    w->Start();
}

//---------------------------------------------------------------------------
void __fastcall TMainForm::sgChannelsSelectCell(TObject *Sender, System::LongInt ACol,
                                                System::LongInt ARow, bool &CanSelect)
{
    CanSelect = true;
    const int idx = (int)ARow - 1;                 // row 0 = header
    if (idx >= 0 && idx < (int)FTracked.size()) plotChannel(idx);
}

//---------------------------------------------------------------------------
// Position tab
//---------------------------------------------------------------------------
// One satellite carried through the fix: ephemeris + tracking epochs + the
// parity-passing subframe boundaries used to time the pseudorange.
struct PvtChan {
    int                           prn = 0;
    long long                     codePhaseSamp = 0;
    int                           bitOffset = 0;
    gps::Ephemeris                eph;
    std::vector<gps::TrackEpoch>  ep;
    std::vector<gps::SubframeRef> subs;
};

// Worker: for each acquired satellite track ~36 s, decode the ephemeris, then
// form pseudoranges at the subframe boundary common to the most channels and
// solve least-squares PVT -> lat/lon/alt. Mirrors PvtConsole.cpp. Runs off the
// UI thread (heavy: ~1.3 GB buffer, tens of seconds) and posts the fix back via
// Synchronize.
class TPvtThread : public TThread
{
public:
    __fastcall TPvtThread(TMainForm* AForm, const String& AFile,
                          const std::vector<gps::AcqResult>& ASats,
                          const gps::AcqConfig& ACfg, int ATrackMs)
        : TThread(true), fForm(AForm), fFile(AFile), fSats(ASats),
          fCfg(ACfg), fTrackMs(ATrackMs)
    {
        FreeOnTerminate = true;
    }

protected:
    void __fastcall Execute();

private:
    TMainForm*                  fForm;
    String                      fFile;
    std::vector<gps::AcqResult> fSats;
    gps::AcqConfig              fCfg;
    int                         fTrackMs;
    String                      fPending;
    String                      fMsg;
    GuiFix                      fFix;

    void status(const String& s) { fPending = s; Synchronize(doStatus); }
    void __fastcall doStatus()   { fForm->Status(fPending); }
    void __fastcall doApply()    { fForm->applyFix(fFix); }
    void __fastcall doFail()     { fForm->Status(fMsg); }
    void __fastcall reenable()
    {
        fForm->btnFix->Caption = L"Compute Fix";
        fForm->btnFix->Enabled = true;
    }
};

void __fastcall TPvtThread::Execute()
{
    const double C = 299792458.0;

    std::ifstream f(AnsiString(fFile).c_str(), std::ios::binary);
    if (!f) { fMsg = L"ERROR: cannot open file for the position fix.";
              Synchronize(doFail); Synchronize(reenable); return; }

    gps::TrackConfig tcfg;
    tcfg.fs = fCfg.fs; tcfg.ifFreq = fCfg.ifFreq;
    gps::AcqConfig   rcfg; rcfg.fs = tcfg.fs; rcfg.ifFreq = tcfg.ifFreq;

    const int n = (int)std::lround(tcfg.fs * 1.0e-3);
    const std::size_t need = (std::size_t)(fTrackMs + 2) * n;
    std::vector<std::int8_t> sig;
    try { sig.resize(need); }
    catch (...) { fMsg = L"ERROR: out of memory for the tracking buffer.";
                  Synchronize(doFail); Synchronize(reenable); return; }

    std::vector<PvtChan> chans;
    for (std::size_t k = 0; k < fSats.size() && !Terminated; ++k) {
        const gps::AcqResult& a = fSats[k];
        { char b[96]; std::snprintf(b, sizeof(b),
              "  PRN %d: tracking %d ms + decoding ephemeris...", a.prn, fTrackMs);
          status(String(b)); }

        f.clear();
        f.seekg((std::streamoff)a.codePhaseSamp, std::ios::beg);
        f.read(reinterpret_cast<char*>(sig.data()), (std::streamsize)need);
        const std::size_t got = (std::size_t)f.gcount();

        const double fd = gps::refineDoppler(sig.data(), got, a.prn, a.doppler, rcfg);
        gps::TrackChannel ch(a.prn, fd, tcfg);
        std::vector<gps::TrackEpoch> ep = ch.run(sig.data(), got, fTrackMs);

        gps::BitSync bs = gps::findBitSync(ep);
        if (!bs.valid) { status(String(L"    PRN ") + IntToStr(a.prn) + L": no bit sync"); continue; }
        std::vector<int> bits = gps::demodulateBits(ep, bs.offset);
        gps::NavDecode nd = gps::decodeNav(bits, a.prn);
        { char b[96]; std::snprintf(b, sizeof(b), "    PRN %d: %d subframes, ephemeris %s",
              a.prn, (int)nd.subframes.size(), nd.eph.valid ? "OK" : "incomplete");
          status(String(b)); }
        if (!nd.eph.valid) continue;

        PvtChan c;
        c.prn = a.prn; c.codePhaseSamp = a.codePhaseSamp; c.bitOffset = bs.offset;
        c.eph = nd.eph; c.ep = std::move(ep); c.subs = nd.subframes;
        chans.push_back(std::move(c));
    }

    if (chans.size() < 4) {
        fMsg = L"Position: fewer than 4 satellites yielded a full ephemeris (need >= 4).";
        Synchronize(doFail); Synchronize(reenable); return;
    }

    // Subframe boundary (TOW count) common to the most channels.
    std::map<int, std::vector<std::pair<int,int> > > byTow;   // tow -> [(chIdx, bitIndex)]
    for (int ci = 0; ci < (int)chans.size(); ++ci)
        for (std::size_t j = 0; j < chans[ci].subs.size(); ++j)
            byTow[chans[ci].subs[j].towCount].push_back(
                std::make_pair(ci, chans[ci].subs[j].bitIndex));

    int bestTow = -1; std::size_t bestCnt = 0;
    for (std::map<int, std::vector<std::pair<int,int> > >::iterator it = byTow.begin();
         it != byTow.end(); ++it)
        if (it->second.size() > bestCnt) { bestCnt = it->second.size(); bestTow = it->first; }

    // Form pseudoranges at that common instant (SoftGNSS convention).
    const double samplesPerCode = std::round(tcfg.fs / 1000.0);
    const double START_OFFSET   = 68.802;                      // ms, nominal travel time
    std::vector<double>        ttms;
    std::vector<gps::SatState> sats;
    std::vector<int>           usedPrn;
    const double transmitTime = bestTow * 6.0 - 6.0;           // subframe leading-edge GPS time

    const std::vector<std::pair<int,int> >& list = byTow[bestTow];
    for (std::size_t q = 0; q < list.size(); ++q) {
        const int ci = list[q].first, bi = list[q].second;
        const std::size_t ms = (std::size_t)(chans[ci].bitOffset + bi * 20);
        if (ms >= chans[ci].ep.size()) continue;
        const long long absSample =
            chans[ci].codePhaseSamp + (long long)chans[ci].ep[ms].sampleIndex;
        ttms.push_back((double)absSample / samplesPerCode);
        sats.push_back(gps::satPosition(chans[ci].eph, transmitTime));
        usedPrn.push_back(chans[ci].prn);
    }
    if (ttms.size() < 4) {
        fMsg = L"Position: fewer than 4 channels aligned to a common subframe.";
        Synchronize(doFail); Synchronize(reenable); return;
    }

    const double mn = std::floor(*std::min_element(ttms.begin(), ttms.end()));
    std::vector<double> pr(ttms.size());
    fFix = GuiFix();
    fFix.bestTow = bestTow;
    for (std::size_t i = 0; i < ttms.size(); ++i) {
        ttms[i] = ttms[i] - mn + START_OFFSET;                 // ms
        pr[i]   = ttms[i] * (C / 1000.0);                      // meters
        GuiSatRow row;
        row.prn  = usedPrn[i];
        row.prKm = pr[i] / 1000.0;
        row.x = sats[i].x / 1e3; row.y = sats[i].y / 1e3; row.z = sats[i].z / 1e3;
        row.svClkUs = sats[i].clockBias * 1e6;
        fFix.rows.push_back(row);
    }

    gps::PvtSolution sol = gps::solvePvt(pr, sats);
    fFix.ok         = sol.ok;
    fFix.nSats      = (int)pr.size();
    fFix.x = sol.x; fFix.y = sol.y; fFix.z = sol.z;
    fFix.lat = sol.lat; fFix.lon = sol.lon; fFix.alt = sol.alt;
    fFix.clockBiasM = sol.clockBias;
    fFix.gdop       = sol.gdop;
    fFix.iterations = sol.iterations;
    fFix.residRms   = sol.residRms;
    fFix.radiusKm   = std::sqrt(sol.x*sol.x + sol.y*sol.y + sol.z*sol.z) / 1e3;

    Synchronize(doApply);
    status(L"Position fix complete.");
    Synchronize(reenable);
}

//---------------------------------------------------------------------------
void TMainForm::applyFix(const GuiFix& fix)
{
    // Satellite table (PRN, pseudorange, SV ECEF position).
    sgSats->RowCount = (int)fix.rows.size() + 1;
    sgSats->Cells[0][0] = L"PRN";
    sgSats->Cells[1][0] = L"Pseudorange km";
    sgSats->Cells[2][0] = L"SV X (km)";
    sgSats->Cells[3][0] = L"SV Y (km)";
    sgSats->Cells[4][0] = L"SV Z (km)";
    for (std::size_t i = 0; i < fix.rows.size(); ++i) {
        const GuiSatRow& r = fix.rows[i];
        const int row = (int)i + 1;
        char b[40];
        sgSats->Cells[0][row] = IntToStr(r.prn);
        std::snprintf(b, sizeof(b), "%.3f", r.prKm); sgSats->Cells[1][row] = String(b);
        std::snprintf(b, sizeof(b), "%.0f", r.x);    sgSats->Cells[2][row] = String(b);
        std::snprintf(b, sizeof(b), "%.0f", r.y);    sgSats->Cells[3][row] = String(b);
        std::snprintf(b, sizeof(b), "%.0f", r.z);    sgSats->Cells[4][row] = String(b);
    }

    const bool sane = (fix.radiusKm > 6300.0 && fix.radiusKm < 6420.0)
                   && (fix.alt > -2000.0 && fix.alt < 12000.0)
                   && (fix.residRms < 1.0e4);

    char b[120];
    memoFix->Lines->BeginUpdate();
    memoFix->Clear();
    memoFix->Lines->Add(L"=== POSITION FIX ===");
    std::snprintf(b, sizeof(b), "Satellites used : %d", fix.nSats);          memoFix->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "Common TOW      : %d  (x6 s)", fix.bestTow);memoFix->Lines->Add(String(b));
    memoFix->Lines->Add(L"");
    std::snprintf(b, sizeof(b), "Latitude   : %12.6f deg", fix.lat);         memoFix->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "Longitude  : %12.6f deg", fix.lon);         memoFix->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "Altitude   : %12.1f m",   fix.alt);         memoFix->Lines->Add(String(b));
    memoFix->Lines->Add(L"");
    std::snprintf(b, sizeof(b), "ECEF X     : %14.1f m", fix.x);             memoFix->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "ECEF Y     : %14.1f m", fix.y);             memoFix->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "ECEF Z     : %14.1f m", fix.z);             memoFix->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "Radius     : %14.1f km", fix.radiusKm);     memoFix->Lines->Add(String(b));
    memoFix->Lines->Add(L"");
    std::snprintf(b, sizeof(b), "Rx clock   : %.1f m  (%.3f us)",
                  fix.clockBiasM, fix.clockBiasM / 299792458.0 * 1e6);       memoFix->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "GDOP       : %.2f", fix.gdop);              memoFix->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "Iterations : %d", fix.iterations);          memoFix->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "Resid RMS  : %.1f m", fix.residRms);        memoFix->Lines->Add(String(b));
    memoFix->Lines->Add(L"");
    memoFix->Lines->Add(sane ? L"SANITY: PASS - on/near Earth's surface"
                             : L"SANITY: FAIL - off-surface / large residuals");
    memoFix->Lines->EndUpdate();

    char sb[140];
    std::snprintf(sb, sizeof(sb),
        "Position fix: Lat %.6f, Lon %.6f, Alt %.1f m  (%d sats, GDOP %.2f, RMS %.1f m)",
        fix.lat, fix.lon, fix.alt, fix.nSats, fix.gdop, fix.residRms);
    Status(String(sb));
}

//---------------------------------------------------------------------------
void __fastcall TMainForm::btnFixClick(TObject *Sender)
{
    if (!FHaveResults)        { Status(L"Acquire satellites first (Acquisition tab)."); return; }
    if (FFilePath.IsEmpty())  { Status(L"No file loaded."); return; }

    std::vector<gps::AcqResult> found;
    for (int prn = 1; prn <= 32; ++prn)
        if (FResults[prn].found) found.push_back(FResults[prn]);
    if (found.size() < 4) { Status(L"Need >= 4 acquired satellites for a position fix."); return; }

    // Reset the satellite table header.
    sgSats->RowCount = 2;
    for (int r = 1; r < sgSats->RowCount; ++r)
        for (int c = 0; c < sgSats->ColCount; ++c) sgSats->Cells[c][r] = L"";
    sgSats->Cells[0][0] = L"PRN";
    sgSats->Cells[1][0] = L"Pseudorange km";
    sgSats->Cells[2][0] = L"SV X (km)";
    sgSats->Cells[3][0] = L"SV Y (km)";
    sgSats->Cells[4][0] = L"SV Z (km)";
    memoFix->Clear();

    btnFix->Enabled = false;
    btnFix->Caption = L"Computing...";

    gps::AcqConfig cfg = configForFile(FFilePath);   // fs/IF from the file name

    char b[100];
    std::snprintf(b, sizeof(b),
        "Position: tracking %d satellites ~36 s each + decoding ephemerides (please wait)...",
        (int)found.size());
    Status(String(b));

    TPvtThread* w = new TPvtThread(this, FFilePath, found, cfg, 36000);
    w->Start();
}
//---------------------------------------------------------------------------
