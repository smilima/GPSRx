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
#include <thread>
#include <atomic>
#include <mutex>
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
// the memo and fills the bar chart in real time (one PRN at a time) via
// Synchronize(), keeping the UI responsive. It frees itself (FreeOnTerminate)
// and re-enables the button when finished.
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
    gps::AcqResult              fBar;     // PRN result currently being added to the chart

    void status(const String& s) { fPending = s; Synchronize(doStatus); }
    void __fastcall doStatus()   { fForm->Status(fPending); }
    void __fastcall doBegin()    { fForm->beginAcquisition(); }
    void __fastcall doAddBar()   { fForm->addAcqResult(fBar); }
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
        // Clear the chart, then fill it in real time: acquireAll calls back once
        // per PRN (1..32, in order), so each bar is added as that PRN is searched
        // and a status line is streamed for the ones that lock.
        Synchronize(doBegin);

        fAll = gps::acquireAll(sig.data(), sig.size(), fCfg,
            [&](const gps::AcqResult& r)
            {
                if (Terminated) return;
                fBar = r;                 // add this PRN's bar (green if found, else grey)
                Synchronize(doAddBar);
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
    FchSky = NULL;
    FPosCount = 0;
    for (int i = 0; i <= 32; ++i) FPosRow[i] = 0;

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
// Start a fresh sky search: empty the chart and the cached results. Hover stays
// disabled (FHaveResults = false) until the first bar arrives.
void TMainForm::beginAcquisition()
{
    for (int i = 0; i <= 32; ++i) FResults[i] = gps::AcqResult();
    Series1->Clear();
    FHaveResults = false;
}
//---------------------------------------------------------------------------
// Add one PRN's bar as acquisition reaches it (called per PRN, in order 1..32),
// caching the result so the hover tooltip can show its details immediately.
void TMainForm::addAcqResult(const gps::AcqResult& r)
{
    if (r.prn >= 1 && r.prn <= 32) FResults[r.prn] = r;
    TColor c = r.found ? (TColor)clGreen : (TColor)clSilver;
    Series1->Add(r.peakRatio, IntToStr(r.prn), c);
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
    FchIQ->BottomAxis->Title->Caption = L"In-phase  I";
    FchIQ->LeftAxis->Title->Caption   = L"Quadrature  Q";
    FchIQ->Width = pnlTop->ClientHeight;
    FchIQ->Align = alLeft;
    FiqSeries = new TPointSeries(FchIQ);
    FchIQ->AddSeries(FiqSeries);

    // Draggable divider between the constellation and the prompt-I plot.
    // (Left set before Align so VCL docks it to the RIGHT of FchIQ.)
    TSplitter* spIQ = new TSplitter(this);
    spIQ->Parent  = pnlTop;
    spIQ->Left    = FchIQ->Left + FchIQ->Width;
    spIQ->Width   = 6;
    spIQ->Align   = alLeft;
    spIQ->MinSize = 80;

    // Prompt-I over time - the 50 bps nav-bit transitions (wide, fills the row).
    FchPromptI = new TChart(this);
    FchPromptI->Parent = pnlTop;
    FchPromptI->View3D = false;
    FchPromptI->Legend->Visible = false;
    FchPromptI->Title->Text->Text = L"Prompt I (nav bits)";
    FchPromptI->BottomAxis->Title->Caption = L"epoch (ms)";
    FchPromptI->LeftAxis->Title->Caption   = L"prompt I correlation";
    FchPromptI->Align = alClient;
    FpromptSeries = new TFastLineSeries(FchPromptI);
    FchPromptI->AddSeries(FpromptSeries);

    // Draggable divider between the top row and the trend plot.
    // (Top set before Align so VCL docks it BELOW the top row.)
    TSplitter* spRow = new TSplitter(this);
    spRow->Parent  = pnl;
    spRow->Top     = pnlTop->Top + pnlTop->Height;
    spRow->Height  = 6;
    spRow->Align   = alTop;
    spRow->MinSize = 80;

    // Bottom row (lower half): Doppler (left axis) + C/N0 (right axis) trends,
    // full width; legend BELOW the plot so it does not cover it.
    FchTrend = new TChart(this);
    FchTrend->Parent = pnl;
    FchTrend->View3D = false;
    FchTrend->Legend->Visible = true;
    FchTrend->Legend->Alignment = laBottom;
    FchTrend->Title->Text->Text = L"Doppler (Hz) / C/N0 (right)";
    FchTrend->BottomAxis->Title->Caption = L"epoch (ms)";
    FchTrend->LeftAxis->Title->Caption   = L"Doppler (Hz)";
    FchTrend->RightAxis->Title->Caption  = L"C/N0 (dB-Hz)";
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
// One-time: create the sky plot on the Position tab, to the right of the fix-
// summary memo (the memo is fixed to a left-anchored column to free the space).
// Built the first time a fix is computed, when the tab sizes are final.
void TMainForm::buildPositionSky()
{
    if (FchSky) return;

    // Container below the button holding the three panes laid out with VCL
    // alignment, so draggable vertical splitters can sit between them.
    TPanel* pnlPos = new TPanel(this);
    pnlPos->Parent = tsPosition;
    pnlPos->Caption = L"";
    pnlPos->BevelOuter = bvNone;
    pnlPos->SetBounds(8, sgSats->Top, tsPosition->ClientWidth - 16,
                      tsPosition->ClientHeight - sgSats->Top - 8);
    pnlPos->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;

    // Satellite table (left). Re-parented into the container and given the
    // live-update column widths.
    sgSats->Parent = pnlPos;
    sgSats->Top = 0; sgSats->Left = 0;
    sgSats->Align = alLeft;
    sgSats->DefaultColWidth = 80;
    sgSats->ColWidths[0] = 40;
    sgSats->ColWidths[1] = 64;
    sgSats->ColWidths[2] = 76;
    sgSats->ColWidths[3] = 60;
    sgSats->ColWidths[4] = 130;

    // Divider between the table and the summary memo (Left before Align).
    TSplitter* spP1 = new TSplitter(this);
    spP1->Parent  = pnlPos;
    spP1->Left    = sgSats->Width + 1;
    spP1->Width   = 6;
    spP1->Align   = alLeft;
    spP1->MinSize = 120;

    // Fix-summary memo (middle).
    memoFix->Parent = pnlPos;
    memoFix->Top    = 0;
    memoFix->Left   = spP1->Left + spP1->Width + 1;
    memoFix->Width  = 256;
    memoFix->Align  = alLeft;

    // Divider between the memo and the sky plot.
    TSplitter* spP2 = new TSplitter(this);
    spP2->Parent  = pnlPos;
    spP2->Left    = memoFix->Left + memoFix->Width + 1;
    spP2->Width   = 6;
    spP2->Align   = alLeft;
    spP2->MinSize = 120;

    // Sky plot (fills the rest on the right; custom-drawn in FchSkyAfterDraw).
    FchSky = new TChart(this);
    FchSky->Parent = pnlPos;
    FchSky->View3D = false;
    FchSky->Legend->Visible = false;
    FchSky->Title->Text->Text = L"Sky plot (N up)";
    FchSky->LeftAxis->Visible   = false;
    FchSky->BottomAxis->Visible = false;
    FchSky->RightAxis->Visible  = false;
    FchSky->TopAxis->Visible    = false;
    FchSky->OnAfterDraw = FchSkyAfterDraw;
    FchSky->Align = alClient;
}

//---------------------------------------------------------------------------
// Custom-drawn sky plot: polar az/el grid (N up, outer ring = horizon) with one
// dot + PRN label per satellite that has a look angle from the most recent fix.
// Always round (sized from the chart rect, not the axes), so no polar package is
// needed. Draws the empty grid until a fix populates FSkyRows.
void __fastcall TMainForm::FchSkyAfterDraw(TObject *Sender)
{
    TChart* ch = static_cast<TChart*>(Sender);
    const TRect rc = ch->ChartRect;
    const int w = rc.Width(), h = rc.Height();
    if (w < 24 || h < 24) return;

    const int cx = (rc.Left + rc.Right) / 2;
    const int cy = (rc.Top  + rc.Bottom) / 2;
    const int R  = (int)(0.46 * (double)std::min(w, h));
    const double DEG = 3.14159265358979323846 / 180.0;

    // Grid: elevation rings (0 deg horizon, 30, 60) + N-S / E-W cross.
    ch->Canvas->Brush->Style = bsClear;
    ch->Canvas->Pen->Style   = psSolid;
    ch->Canvas->Pen->Width    = 1;
    ch->Canvas->Pen->Color   = clSilver;
    for (int el = 0; el <= 60; el += 30) {
        const int rr = (int)(R * (90 - el) / 90.0);
        ch->Canvas->Ellipse(cx - rr, cy - rr, cx + rr, cy + rr);
    }
    ch->Canvas->MoveTo(cx - R, cy); ch->Canvas->LineTo(cx + R, cy);
    ch->Canvas->MoveTo(cx, cy - R); ch->Canvas->LineTo(cx, cy + R);

    // Cardinal labels (String args disambiguate TextWidth/TextHeight overloads).
    const String sN(L"N"), sS(L"S"), sE(L"E"), sW(L"W");
    const int th = ch->Canvas->TextHeight(sN);
    ch->Canvas->Font->Color = clGray;
    ch->Canvas->TextOut(cx - ch->Canvas->TextWidth(sN) / 2, cy - R - th,    sN);
    ch->Canvas->TextOut(cx - ch->Canvas->TextWidth(sS) / 2, cy + R,         sS);
    ch->Canvas->TextOut(cx + R + 1,                         cy - th / 2,    sE);
    ch->Canvas->TextOut(cx - R - 1 - ch->Canvas->TextWidth(sW), cy - th / 2, sW);

    // Satellites (dot + PRN, North up, clockwise azimuth, radius by 90-elevation).
    for (std::size_t i = 0; i < FSkyRows.size(); ++i) {
        const GuiSatRow& s = FSkyRows[i];
        if (!s.elValid) continue;
        const double rr = R * (90.0 - s.elDeg) / 90.0;
        const double a  = s.azDeg * DEG;
        const int x = cx + (int)(rr * std::sin(a));
        const int y = cy - (int)(rr * std::cos(a));
        const int d = 7;
        ch->Canvas->Brush->Style = bsSolid;
        ch->Canvas->Brush->Color = (TColor)clGreen;
        ch->Canvas->Pen->Color   = (TColor)clBlack;
        ch->Canvas->Ellipse(x - d, y - d, x + d, y + d);
        ch->Canvas->Brush->Style = bsClear;
        ch->Canvas->Font->Color  = clBlack;
        ch->Canvas->TextOut(x + d + 1, y - th / 2, IntToStr(s.prn));
    }
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

// Per-PRN result slot filled by a parallel tracking worker (one slot each, so
// no locking on the slots themselves).
struct PerSat {
    bool    valid = false;   // ephemeris decoded OK
    PvtChan chan;
};

// Worker: reads the ~36 s window once into a shared buffer, tracks every
// acquired satellite IN PARALLEL (one ~1.3 GB read-only buffer shared by a
// bounded std::thread pool), decodes each ephemeris, then forms pseudoranges at
// the subframe boundary common to the most channels and solves least-squares
// PVT -> lat/lon/alt. Mirrors PvtConsole.cpp. Runs off the UI thread and posts
// progress + the fix back via Synchronize (only this thread touches the GUI).
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
    PosSatProgress              fProgRow;

    void status(const String& s) { fPending = s; Synchronize(doStatus); }
    void __fastcall doStatus()   { fForm->Status(fPending); }
    void __fastcall doApply()    { fForm->applyFix(fFix); }
    void __fastcall doAddPosSat(){ fForm->addPosSat(fProgRow); }
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
    const std::size_t need = (std::size_t)(fTrackMs + 2) * n;   // samples one channel needs

    // Every channel tracks the SAME ~36 s window and differs only by its
    // sub-millisecond code-phase offset, so read the span ONCE into a shared,
    // read-only buffer (instead of re-reading ~1.3 GB per satellite) and let
    // each tracker index into it at its own offset.
    std::size_t maxOff = 0;
    for (std::size_t k = 0; k < fSats.size(); ++k)
        if ((std::size_t)fSats[k].codePhaseSamp > maxOff)
            maxOff = (std::size_t)fSats[k].codePhaseSamp;
    const std::size_t bufLen = need + maxOff;          // covers [off, off+need) for every PRN

    std::vector<std::int8_t> buf;
    try { buf.resize(bufLen); }
    catch (...) { fMsg = L"ERROR: out of memory for the sample buffer.";
                  Synchronize(doFail); Synchronize(reenable); return; }
    f.seekg(0, std::ios::beg);
    f.read(reinterpret_cast<char*>(buf.data()), (std::streamsize)bufLen);
    const std::size_t got = (std::size_t)f.gcount();

    // Track every satellite IN PARALLEL: tracking is pure CPU, independent per
    // PRN, and all channels share the one read-only buffer. A bounded pool
    // (<= hardware threads) pulls PRNs off an atomic work index; each result
    // lands in its own slot (no locking), and completion messages go through a
    // small queue that ONLY this (VCL) thread drains + posts via Synchronize.
    const int N = (int)fSats.size();
    std::vector<PerSat> out((std::size_t)N);
    std::atomic<int>    nextJob(0);
    std::atomic<int>    doneCount(0);
    std::mutex                   msgMu;
    std::vector<String>          msgQ;
    std::vector<PosSatProgress>  rowQ;
    auto pushSat = [&](const String& s, const PosSatProgress& p) {
        std::lock_guard<std::mutex> g(msgMu);
        msgQ.push_back(s);
        rowQ.push_back(p);
    };

    auto worker = [&]() {
        for (;;) {
            if (Terminated) break;
            const int k = nextJob.fetch_add(1);
            if (k >= N) break;
            const gps::AcqResult& a = fSats[(std::size_t)k];
            PosSatProgress prog; prog.prn = a.prn;

            const std::size_t off = (std::size_t)a.codePhaseSamp;
            if (off >= got) {
                prog.state = 0;
                pushSat(String(L"    PRN ") + IntToStr(a.prn) + L": no data", prog);
                doneCount.fetch_add(1); continue;
            }
            const std::int8_t* sp = buf.data() + off;
            const std::size_t  sl = got - off;

            const double fd = gps::refineDoppler(sp, sl, a.prn, a.doppler, rcfg);
            gps::TrackChannel ch(a.prn, fd, tcfg);
            std::vector<gps::TrackEpoch> ep = ch.run(sp, sl, fTrackMs);

            if (!ep.empty()) {              // mean Doppler over the settled tail
                std::size_t s0 = ep.size() * 9 / 10; double dsum = 0; int dc = 0;
                for (std::size_t i = s0; i < ep.size(); ++i) { dsum += ep[i].doppler; ++dc; }
                prog.doppler = dc ? dsum / dc : 0.0;
            }

            gps::BitSync bs = gps::findBitSync(ep);
            prog.cn0 = gps::estimateCN0(ep, bs.valid ? bs.offset : 0);
            if (!bs.valid) {
                prog.state = 1;
                pushSat(String(L"    PRN ") + IntToStr(a.prn) + L": no bit sync", prog);
                doneCount.fetch_add(1); continue;
            }
            std::vector<int> bits = gps::demodulateBits(ep, bs.offset);
            gps::NavDecode nd = gps::decodeNav(bits, a.prn);
            prog.state = nd.eph.valid ? 3 : 2;
            { char b[96]; std::snprintf(b, sizeof(b), "    PRN %d: %d subframes, ephemeris %s",
                  a.prn, (int)nd.subframes.size(), nd.eph.valid ? "OK" : "incomplete");
              pushSat(String(b), prog); }
            if (nd.eph.valid) {
                PvtChan c;
                c.prn = a.prn; c.codePhaseSamp = a.codePhaseSamp; c.bitOffset = bs.offset;
                c.eph = nd.eph; c.ep = std::move(ep); c.subs = nd.subframes;
                out[(std::size_t)k].chan = std::move(c);
                out[(std::size_t)k].valid = true;
            }
            doneCount.fetch_add(1);
        }
    };

    unsigned hw = std::thread::hardware_concurrency();
    int nWorkers = N;
    if (hw && nWorkers > (int)hw) nWorkers = (int)hw;
    { char b[128]; std::snprintf(b, sizeof(b),
          "  tracking %d satellites in parallel on %d threads (%d ms each)...",
          N, nWorkers, fTrackMs); status(String(b)); }

    std::vector<std::thread> pool;
    pool.reserve((std::size_t)nWorkers);
    for (int i = 0; i < nWorkers; ++i) pool.emplace_back(worker);

    // Drain status lines + per-PRN table rows while the pool runs. ONLY this
    // (VCL) thread touches the GUI, via Synchronize; the lock is never held
    // across a Synchronize call.
    auto drainOnce = [&]() {
        std::vector<String>         msgs;
        std::vector<PosSatProgress> rows;
        { std::lock_guard<std::mutex> g(msgMu); msgs.swap(msgQ); rows.swap(rowQ); }
        for (std::size_t i = 0; i < msgs.size(); ++i) status(msgs[i]);
        for (std::size_t i = 0; i < rows.size(); ++i) { fProgRow = rows[i]; Synchronize(doAddPosSat); }
    };
    while (doneCount.load() < N && !Terminated) { ::Sleep(150); drainOnce(); }
    for (std::size_t i = 0; i < pool.size(); ++i) pool[i].join();
    drainOnce();

    // Assemble surviving channels in acquisition order (deterministic).
    std::vector<PvtChan> chans;
    for (int k = 0; k < N; ++k)
        if (out[(std::size_t)k].valid) chans.push_back(std::move(out[(std::size_t)k].chan));

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

    // Look angles for the sky plot: rotate each SV's ECEF offset into the
    // receiver's local East/North/Up, then to azimuth (from N, CW) / elevation.
    {
        const double DEG  = 3.14159265358979323846 / 180.0;
        const double latR = sol.lat * DEG, lonR = sol.lon * DEG;
        const double sLat = std::sin(latR), cLat = std::cos(latR);
        const double sLon = std::sin(lonR), cLon = std::cos(lonR);
        for (std::size_t i = 0; i < fFix.rows.size() && i < sats.size(); ++i) {
            const double dx = sats[i].x - sol.x;
            const double dy = sats[i].y - sol.y;
            const double dz = sats[i].z - sol.z;
            const double e  = -sLon * dx + cLon * dy;
            const double nN = -sLat * cLon * dx - sLat * sLon * dy + cLat * dz;
            const double u  =  cLat * cLon * dx + cLat * sLon * dy + sLat * dz;
            double az = std::atan2(e, nN) / DEG; if (az < 0.0) az += 360.0;
            const double el = std::atan2(u, std::sqrt(e * e + nN * nN)) / DEG;
            fFix.rows[i].azDeg   = az;
            fFix.rows[i].elDeg   = el;
            fFix.rows[i].elValid = (el >= 0.0);
        }
    }

    Synchronize(doApply);
    status(L"Position fix complete.");
    Synchronize(reenable);
}

//---------------------------------------------------------------------------
void TMainForm::applyFix(const GuiFix& fix)
{
    // The PRN / C/N0 / Doppler / Eph rows were filled live during the run; now
    // fill the pseudorange column for the satellites that made it into the fix.
    char pb[40];
    for (std::size_t i = 0; i < fix.rows.size(); ++i) {
        const GuiSatRow& r = fix.rows[i];
        int row = (r.prn >= 1 && r.prn <= 32) ? FPosRow[r.prn] : 0;
        if (row == 0) {                       // safety: not seen live -> append
            row = ++FPosCount;
            if (sgSats->RowCount < row + 1) sgSats->RowCount = row + 1;
            FPosRow[r.prn] = row;
            sgSats->Cells[0][row] = IntToStr(r.prn);
        }
        std::snprintf(pb, sizeof(pb), "%.3f", r.prKm);
        sgSats->Cells[4][row] = String(pb);
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

    // Refresh the sky plot with the satellites' look angles.
    FSkyRows = fix.rows;
    if (FchSky) FchSky->Repaint();
}

//---------------------------------------------------------------------------
// Add or refresh one PRN's row in the Position table as a parallel worker
// finishes it (PRN / C/N0 / Doppler / Eph). The pseudorange column is filled in
// later by applyFix once the fix is solved.
void TMainForm::addPosSat(const PosSatProgress& p)
{
    if (p.prn < 1 || p.prn > 32) return;

    int row = FPosRow[p.prn];
    if (row == 0) {
        row = ++FPosCount;                          // data rows are 1..FPosCount
        if (sgSats->RowCount < row + 1) sgSats->RowCount = row + 1;
        FPosRow[p.prn] = row;
    }

    char b[32];
    sgSats->Cells[0][row] = IntToStr(p.prn);
    if (p.state >= 1) {
        std::snprintf(b, sizeof(b), "%.1f",  p.cn0);     sgSats->Cells[1][row] = String(b);
        std::snprintf(b, sizeof(b), "%+.0f", p.doppler); sgSats->Cells[2][row] = String(b);
    } else {
        sgSats->Cells[1][row] = L"--";
        sgSats->Cells[2][row] = L"--";
    }
    sgSats->Cells[3][row] = (p.state == 3) ? L"OK"
                          : (p.state == 2) ? L"partial"
                          : (p.state == 1) ? L"no sync" : L"no data";
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

    buildPositionSky();                 // create the Position layout + sky plot (once)

    // Reset the satellite table for live, per-PRN updates during the parallel run.
    for (int i = 0; i <= 32; ++i) FPosRow[i] = 0;
    FPosCount = 0;
    sgSats->RowCount = 2;
    for (int r = 1; r < sgSats->RowCount; ++r)
        for (int c = 0; c < sgSats->ColCount; ++c) sgSats->Cells[c][r] = L"";
    sgSats->Cells[0][0] = L"PRN";
    sgSats->Cells[1][0] = L"C/N0";
    sgSats->Cells[2][0] = L"Doppler";
    sgSats->Cells[3][0] = L"Eph";
    sgSats->Cells[4][0] = L"Pseudorange km";
    memoFix->Clear();
    FSkyRows.clear();                   // drop the previous fix's sky dots
    if (FchSky) FchSky->Repaint();

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
