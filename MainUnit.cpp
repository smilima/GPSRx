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
#include <memory>
#include <System.IniFiles.hpp>
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
// DPI scaling for runtime-created controls.
//
// The app is PerMonitorV2 DPI-aware and the form is Scaled, so VCL scales the
// .dfm-streamed controls (and fonts) from the 96-DPI design to the monitor DPI.
// But controls we create in code and place with literal SetBounds() pixels are
// NOT auto-scaled - they keep their 96-DPI coordinates while inheriting the
// already-scaled font, so on a 1.5x display their boxes are too small for the
// text. dpx() scales a design pixel to the current screen DPI; runtime popup
// forms (born at 96 DPI) are instead scaled wholesale via TForm::ScaleForPPI in
// dpiShow(), which is exactly what VCL does for designed forms.
//---------------------------------------------------------------------------
static int dpx(int v)
{
    return (int)((long long)v * Vcl::Forms::Screen->PixelsPerInch / 96);
}

//---------------------------------------------------------------------------
// Recursively scale a runtime popup's child controls from 96-DPI design
// coordinates to the current screen DPI. We scale ONLY geometry (bounds, and
// grid column/row sizes) - NOT fonts: a runtime form's fonts already render at
// the device DPI (points are physical), so scaling them too would double up.
// Anchors are pinned to top-left during each move so SetBounds is absolute and
// akRight/akBottom children land correctly, then restored for live resizing.
// The form itself is pre-sized with dpx() in makeInspector.
static void scaleChildrenForDpi(Vcl::Controls::TWinControl* parent)
{
    const int num = Vcl::Forms::Screen->PixelsPerInch, den = 96;
    if (num == den) return;
    for (int i = 0; i < parent->ControlCount; ++i) {
        Vcl::Controls::TControl* c = parent->Controls[i];
        TAnchors keep = c->Anchors;
        c->Anchors = TAnchors() << akLeft << akTop;
        c->SetBounds(MulDiv(c->Left, num, den), MulDiv(c->Top, num, den),
                     MulDiv(c->Width, num, den), MulDiv(c->Height, num, den));
        c->Anchors = keep;
        if (c->InheritsFrom(__classid(TStringGrid))) {
            TStringGrid* g = static_cast<TStringGrid*>(c);
            g->DefaultRowHeight = MulDiv(g->DefaultRowHeight, num, den);
            // Scale each column's effective width. NB: do NOT set DefaultColWidth
            // here - that resets explicit per-column widths to a uniform value.
            for (int col = 0; col < g->ColCount; ++col)
                g->ColWidths[col] = MulDiv(g->ColWidths[col], num, den);
        }
        if (c->InheritsFrom(__classid(Vcl::Controls::TWinControl)))
            scaleChildrenForDpi(static_cast<Vcl::Controls::TWinControl*>(c));
    }
}

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

    // --- DAC streaming + settings ---
    FStream = NULL;
    FDeadStream = NULL;
    loadSettings();                                  // fills FDacSampleRate / FIfHz from GPSRx.ini

    // View -> Settings menu (built at runtime, so no .dfm edit is needed).
    TMenuItem* miView = new TMenuItem(MainMenu1);
    miView->Caption = L"View";
    MainMenu1->Items->Add(miView);
    TMenuItem* miSettings = new TMenuItem(miView);
    miSettings->Caption = L"Settings...";
    miSettings->OnClick = settingsClick;
    miView->Add(miSettings);

    // "Stream from DAC" button on the top panel, to the right of the file edit.
    FbtnStream = new TButton(this);
    FbtnStream->Parent  = Panel1;
    FbtnStream->Caption = L"Stream from DAC";
    FbtnStream->SetBounds(editFile->Left + editFile->Width + dpx(12), editFile->Top - dpx(2), dpx(130), dpx(27));
    FbtnStream->OnClick = btnStreamClick;

    // --- Advanced inspection: "Advanced" checkbox + a hidden tab of inspectors ---
    FAdvThread = NULL; FAdvDead = NULL; FAdvReady = false;
    FchkAdvanced = new TCheckBox(this);
    FchkAdvanced->Parent  = Panel1;
    FchkAdvanced->Caption = L"Advanced";
    FchkAdvanced->SetBounds(FbtnStream->Left + FbtnStream->Width + dpx(16), editFile->Top + dpx(2), dpx(90), dpx(21));
    FchkAdvanced->OnClick = advCheckClick;

    tsAdvanced = new TTabSheet(PageControl1);
    tsAdvanced->PageControl = PageControl1;
    tsAdvanced->Caption     = L"Advanced";
    tsAdvanced->TabVisible  = false;

    FlblAdv = new TLabel(this);
    FlblAdv->Parent = tsAdvanced;
    FlblAdv->SetBounds(dpx(10), dpx(8), dpx(940), dpx(44));
    FlblAdv->AutoSize = false; FlblAdv->WordWrap = true;
    FlblAdv->Caption =
        L"GPS signal inspection lab - teaching views of every receiver stage.  "
        L"Journey / RF / Acquisition / C-A Code work straight from the open capture; "
        L"the rest (Tracking, Nav, Ephemeris, Almanac, PVT) need \"Analyze\", which tracks + "
        L"decodes every acquired satellite for ~30 s and then enables them.";

    FbtnAnalyze = new TButton(this);
    FbtnAnalyze->Parent = tsAdvanced;
    FbtnAnalyze->SetBounds(dpx(10), dpx(56), dpx(230), dpx(30));
    FbtnAnalyze->Caption = L"Analyze (track + decode all)";
    FbtnAnalyze->OnClick = advAnalyzeClick;

    {
        int idx = 0;
        // (caption, handler, needsAnalysis)
        struct Item { const wchar_t* cap; TNotifyEvent ev; bool needData; };
        auto add = [&](const String& cap, TNotifyEvent ev, bool needData) {
            TButton* b = new TButton(this);
            b->Parent = tsAdvanced;
            b->SetBounds(dpx(10), dpx(100 + idx * 34), dpx(230), dpx(28));
            b->Caption = cap;
            b->OnClick = ev;
            if (needData) { b->Enabled = false; FAdvButtons.push_back(b); }
            ++idx;
        };
        add(L"Signal Journey Overview", advJourneyClick, false);
        add(L"RF && Spectrum",          advRfClick,      false);
        add(L"Acquisition Search Surface", advAcqClick,  false);
        add(L"C/A Code && Correlation", advCodeClick,  false);
        add(L"Tracking Loops Lab",      advTrackClick, true);
        add(L"Nav Frame && Bits",       advNavClick,   true);
        add(L"Ephemeris Decoder",       advEphClick,   true);
        add(L"Almanac && SF4/5 Pages",  advAlmClick,   true);
        add(L"PVT Solver Lab",          advPvtClick,   true);
    }

    Status(L"Ready. Open a .bin or .sim capture from File > Open, then click Acquire.");
}
//---------------------------------------------------------------------------
// Settings persistence (GPSRx.ini next to the executable). The DAC sample rate
// maps to the receiver sample rate fs; the IF maps to ifFreq. Defaults match the
// bundled capture so the simulated stream decodes out of the box.
void TMainForm::loadSettings()
{
    const String ini = ChangeFileExt(Application->ExeName, L".ini");
    std::unique_ptr<TIniFile> f(new TIniFile(ini));
    FDacSampleRate = f->ReadFloat(L"Stream", L"DacSampleRateHz", 38192000.0);
    FIfHz          = f->ReadFloat(L"Stream", L"IfHz",             9550000.0);
}
//---------------------------------------------------------------------------
void TMainForm::saveSettings()
{
    const String ini = ChangeFileExt(Application->ExeName, L".ini");
    std::unique_ptr<TIniFile> f(new TIniFile(ini));
    f->WriteFloat(L"Stream", L"DacSampleRateHz", FDacSampleRate);
    f->WriteFloat(L"Stream", L"IfHz",            FIfHz);
    f->UpdateFile();
}
//---------------------------------------------------------------------------
// View -> Settings: a small modal dialog (built in code) for the two rates.
void __fastcall TMainForm::settingsClick(TObject* Sender)
{
    std::unique_ptr<TForm> dlg(new TForm((TComponent*)NULL));
    dlg->Caption      = L"Settings";
    dlg->BorderStyle  = bsDialog;
    dlg->Position     = poMainFormCenter;
    dlg->ClientWidth  = dpx(340);
    dlg->ClientHeight = dpx(150);

    TLabel* l1 = new TLabel(dlg.get());
    l1->Parent = dlg.get(); l1->SetBounds(16, 20, 170, 20); l1->Caption = L"DAC sample rate (Hz)";
    TEdit* e1 = new TEdit(dlg.get());
    e1->Parent = dlg.get(); e1->SetBounds(190, 16, 134, 24); e1->Text = FloatToStr(FDacSampleRate);

    TLabel* l2 = new TLabel(dlg.get());
    l2->Parent = dlg.get(); l2->SetBounds(16, 58, 170, 20); l2->Caption = L"IF (Hz)";
    TEdit* e2 = new TEdit(dlg.get());
    e2->Parent = dlg.get(); e2->SetBounds(190, 54, 134, 24); e2->Text = FloatToStr(FIfHz);

    TButton* ok = new TButton(dlg.get());
    ok->Parent = dlg.get(); ok->Caption = L"OK"; ok->Default = true;
    ok->ModalResult = mrOk; ok->SetBounds(150, 106, 80, 28);
    TButton* cancel = new TButton(dlg.get());
    cancel->Parent = dlg.get(); cancel->Caption = L"Cancel"; cancel->Cancel = true;
    cancel->ModalResult = mrCancel; cancel->SetBounds(240, 106, 80, 28);

    scaleChildrenForDpi(dlg.get());            // scale the runtime dialog geometry
    dlg->ActiveControl = e1;
    if (dlg->ShowModal() == mrOk) {
        double dr = FDacSampleRate, iff = FIfHz;
        try { dr  = StrToFloat(e1->Text); } catch (...) {}
        try { iff = StrToFloat(e2->Text); } catch (...) {}
        if (dr  > 0) FDacSampleRate = dr;
        if (iff > 0) FIfHz          = iff;
        saveSettings();
        char b[120];
        std::snprintf(b, sizeof(b), "Settings saved: DAC %.3f MHz, IF %.3f MHz.",
                      FDacSampleRate / 1e6, FIfHz / 1e6);
        Status(String(b));
    }
}
//---------------------------------------------------------------------------
void TMainForm::Status(const String& s)
{
    // Cap the log so continuous streaming cannot grow the memo unbounded.
    if (memoResults->Lines->Count > 2000) memoResults->Lines->Delete(0);
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
// Fill the acquisition bar chart from a complete result set (used while streaming).
void TMainForm::showAcqResults(const std::vector<gps::AcqResult>& results)
{
    beginAcquisition();
    for (std::size_t i = 0; i < results.size(); ++i) addAcqResult(results[i]);
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
    const int gx = sgChannels->Left + sgChannels->Width + dpx(8);
    pnl->SetBounds(gx, sgChannels->Top,
                   tsTracking->ClientWidth  - gx - dpx(6),
                   tsTracking->ClientHeight - sgChannels->Top - dpx(6));
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
    spIQ->Width   = dpx(6);
    spIQ->Align   = alLeft;
    spIQ->MinSize = dpx(80);

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
    spRow->Height  = dpx(6);
    spRow->Align   = alTop;
    spRow->MinSize = dpx(80);

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
    pnlPos->SetBounds(dpx(8), sgSats->Top, tsPosition->ClientWidth - dpx(16),
                      tsPosition->ClientHeight - sgSats->Top - dpx(8));
    pnlPos->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;

    // Satellite table (left). Re-parented into the container and given the
    // live-update column widths.
    sgSats->Parent = pnlPos;
    sgSats->Top = 0; sgSats->Left = 0;
    sgSats->Align = alLeft;
    sgSats->DefaultColWidth = dpx(80);
    sgSats->ColWidths[0] = dpx(40);
    sgSats->ColWidths[1] = dpx(64);
    sgSats->ColWidths[2] = dpx(76);
    sgSats->ColWidths[3] = dpx(60);
    sgSats->ColWidths[4] = dpx(130);

    // Divider between the table and the summary memo (Left before Align).
    TSplitter* spP1 = new TSplitter(this);
    spP1->Parent  = pnlPos;
    spP1->Left    = sgSats->Width + 1;
    spP1->Width   = dpx(6);
    spP1->Align   = alLeft;
    spP1->MinSize = dpx(120);

    // Fix-summary memo (middle).
    memoFix->Parent = pnlPos;
    memoFix->Top    = 0;
    memoFix->Left   = spP1->Left + spP1->Width + 1;
    memoFix->Width  = dpx(256);
    memoFix->Align  = alLeft;

    // Divider between the memo and the sky plot.
    TSplitter* spP2 = new TSplitter(this);
    spP2->Parent  = pnlPos;
    spP2->Left    = memoFix->Left + memoFix->Width + 1;
    spP2->Width   = dpx(6);
    spP2->Align   = alLeft;
    spP2->MinSize = dpx(120);

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
// Clear the Tracking grid + cached channels (shared by Track Acquired + streaming).
void TMainForm::resetTracking()
{
    buildTrackingCharts();
    FTracked.clear();
    sgChannels->RowCount = 2;
    for (int r = 1; r < sgChannels->RowCount; ++r)
        for (int c = 0; c < sgChannels->ColCount; ++c) sgChannels->Cells[c][r] = L"";
    sgChannels->Cells[0][0] = L"PRN";
    sgChannels->Cells[1][0] = L"Doppler";
    sgChannels->Cells[2][0] = L"C/N0";
    sgChannels->Cells[3][0] = L"Status";
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

    resetTracking();

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

// Form pseudoranges at the subframe boundary common to the most channels and
// solve least-squares PVT -> lat/lon/alt + per-satellite rows (incl. az/el for
// the sky plot). GUI-free and deterministic; shared by the single-fix worker
// (TPvtThread) and the continuous streaming worker (TStreamThread). Returns a
// GuiFix with ok=false if fewer than 4 channels align to a common subframe.
static GuiFix solveFixFromChannels(const std::vector<PvtChan>& chans, double fs)
{
    GuiFix fix;
    const double C = 299792458.0;

    std::map<int, std::vector<std::pair<int,int> > > byTow;   // tow -> [(chIdx, bitIndex)]
    for (int ci = 0; ci < (int)chans.size(); ++ci)
        for (std::size_t j = 0; j < chans[ci].subs.size(); ++j)
            byTow[chans[ci].subs[j].towCount].push_back(
                std::make_pair(ci, chans[ci].subs[j].bitIndex));

    int bestTow = -1; std::size_t bestCnt = 0;
    for (std::map<int, std::vector<std::pair<int,int> > >::iterator it = byTow.begin();
         it != byTow.end(); ++it)
        if (it->second.size() > bestCnt) { bestCnt = it->second.size(); bestTow = it->first; }
    if (bestTow < 0) return fix;                                // no parity-passing subframe

    const double samplesPerCode = std::round(fs / 1000.0);
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
    if (ttms.size() < 4) return fix;                           // ok stays false

    const double mn = std::floor(*std::min_element(ttms.begin(), ttms.end()));
    std::vector<double> pr(ttms.size());
    fix.bestTow = bestTow;
    for (std::size_t i = 0; i < ttms.size(); ++i) {
        ttms[i] = ttms[i] - mn + START_OFFSET;                 // ms
        pr[i]   = ttms[i] * (C / 1000.0);                      // meters
        GuiSatRow row;
        row.prn  = usedPrn[i];
        row.prKm = pr[i] / 1000.0;
        row.x = sats[i].x / 1e3; row.y = sats[i].y / 1e3; row.z = sats[i].z / 1e3;
        row.svClkUs = sats[i].clockBias * 1e6;
        fix.rows.push_back(row);
    }

    gps::PvtSolution sol = gps::solvePvt(pr, sats);
    fix.ok         = sol.ok;
    fix.nSats      = (int)pr.size();
    fix.x = sol.x; fix.y = sol.y; fix.z = sol.z;
    fix.lat = sol.lat; fix.lon = sol.lon; fix.alt = sol.alt;
    fix.clockBiasM = sol.clockBias;
    fix.gdop       = sol.gdop;
    fix.iterations = sol.iterations;
    fix.residRms   = sol.residRms;
    fix.radiusKm   = std::sqrt(sol.x*sol.x + sol.y*sol.y + sol.z*sol.z) / 1e3;

    // Look angles for the sky plot: ECEF offset -> local East/North/Up -> az/el.
    const double DEG  = 3.14159265358979323846 / 180.0;
    const double latR = sol.lat * DEG, lonR = sol.lon * DEG;
    const double sLat = std::sin(latR), cLat = std::cos(latR);
    const double sLon = std::sin(lonR), cLon = std::cos(lonR);
    for (std::size_t i = 0; i < fix.rows.size() && i < sats.size(); ++i) {
        const double dx = sats[i].x - sol.x, dy = sats[i].y - sol.y, dz = sats[i].z - sol.z;
        const double e  = -sLon * dx + cLon * dy;
        const double nN = -sLat * cLon * dx - sLat * sLon * dy + cLat * dz;
        const double u  =  cLat * cLon * dx + cLat * sLon * dy + sLat * dz;
        double az = std::atan2(e, nN) / DEG; if (az < 0.0) az += 360.0;
        const double el = std::atan2(u, std::sqrt(e * e + nN * nN)) / DEG;
        fix.rows[i].azDeg   = az;
        fix.rows[i].elDeg   = el;
        fix.rows[i].elValid = (el >= 0.0);
    }
    return fix;
}

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

    fFix = solveFixFromChannels(chans, tcfg.fs);
    if (!fFix.ok) {
        fMsg = L"Position: could not align >= 4 channels to a common subframe.";
        Synchronize(doFail); Synchronize(reenable); return;
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
// Clear + relabel the Position table (shared by Compute Fix and streaming).
void TMainForm::resetPositionTable()
{
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

    resetPositionTable();
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
// Streaming "DAC" receiver
//---------------------------------------------------------------------------
// TStreamThread - a continuous receiver. It stands in for a live DAC IQ stream
// by looping over the loaded capture: each pass acquires at the current read
// offset, tracks every visible satellite in parallel, decodes, solves PVT, and
// updates the Position tab live; then it advances the offset (wrapping at EOF)
// and repeats until stopped. Re-acquiring each pass lets the satellite set
// follow what is "in view". The file read is the only DAC-specific seam: swap it
// for a real device buffer and the rest is unchanged.
//---------------------------------------------------------------------------
class TStreamThread : public TThread
{
public:
    __fastcall TStreamThread(TMainForm* AForm, const String& AFile,
                             const gps::AcqConfig& ACfg, int ATrackMs)
        : TThread(true), fForm(AForm), fFile(AFile), fCfg(ACfg),
          fTrackMs(ATrackMs), fStop(false), fIter(0)
    {
        FreeOnTerminate = false;   // owner stops + WaitFor + deletes (see streamDone / ~TMainForm)
    }
    void requestStop() { fStop = true; }
    const std::atomic<bool>& stopFlag() const { return fStop; }

protected:
    void __fastcall Execute();

private:
    TMainForm*        fForm;
    String            fFile;
    gps::AcqConfig    fCfg;
    int               fTrackMs;
    std::atomic<bool> fStop;
    int               fIter;
    String            fPending;
    GuiFix            fFix;
    PosSatProgress    fProgRow;
    std::vector<gps::AcqResult> fAcqResults;
    GuiTrackedChannel fTrackedCh;

    void status(const String& s) { fPending = s; Synchronize(doStatus); }
    void __fastcall doStatus()        { fForm->Status(fPending); }
    void __fastcall doReset()         { fForm->resetPositionTable(); }
    void __fastcall doAddSat()        { fForm->addPosSat(fProgRow); }
    void __fastcall doApply()         { fForm->applyFix(fFix); }
    void __fastcall doShowAcq()       { fForm->showAcqResults(fAcqResults); }
    void __fastcall doResetTracking() { fForm->resetTracking(); }
    void __fastcall doAddTracked()    { fForm->addTrackedChannel(fTrackedCh); }

    void runPass(std::ifstream& f, long long offset, std::vector<std::int8_t>& buf);
};

void __fastcall TStreamThread::Execute()
{
    // Execute just returns when finished/aborted; the OnTerminate handler
    // (TMainForm::streamDone) resets the GUI and manages this thread's lifetime.
    std::ifstream f(AnsiString(fFile).c_str(), std::ios::binary);
    if (!f) { status(L"ERROR: cannot open the stream source file."); return; }
    f.seekg(0, std::ios::end);
    const long long fileLen = (long long)f.tellg();

    const int n = (int)std::lround(fCfg.fs * 1.0e-3);
    const long long need = (long long)(fTrackMs + 2) * n;
    if (need >= fileLen) {
        status(L"ERROR: stream source too short for the integration window.");
        return;
    }

    // One big sample buffer, reused across passes (instead of reallocating ~1.3
    // GB every pass).
    std::vector<std::int8_t> buf;
    try { buf.resize((std::size_t)need); }
    catch (...) { status(L"ERROR: out of memory for the stream buffer."); return; }

    status(L"Streaming from DAC (simulated): continuous acquire / track / fix. Click Stop to end.");

    const long long step = (long long)(fTrackMs / 4) * n;   // advance ~1/4 window per pass
    long long offset = 0;
    while (!fStop && !Terminated) {
        ++fIter;
        runPass(f, offset, buf);
        offset += step;
        if (offset + need > fileLen) offset = 0;            // wrap -> loop the file
        for (int s = 0; s < 8 && !fStop && !Terminated; ++s) ::Sleep(100);   // interruptible pause
    }
    status(L"Streaming stopped.");
}

void TStreamThread::runPass(std::ifstream& f, long long offset, std::vector<std::int8_t>& buf)
{
    gps::TrackConfig tcfg; tcfg.fs = fCfg.fs; tcfg.ifFreq = fCfg.ifFreq;
    gps::AcqConfig   rcfg; rcfg.fs = tcfg.fs; rcfg.ifFreq = tcfg.ifFreq;
    const int n = (int)std::lround(tcfg.fs * 1.0e-3);

    // --- acquire at this offset (re-acquire every pass) ---
    std::vector<std::int8_t> acq((std::size_t)n * fCfg.numMs);
    f.clear(); f.seekg((std::streamoff)offset, std::ios::beg);
    f.read(reinterpret_cast<char*>(acq.data()), (std::streamsize)acq.size());
    if ((std::size_t)f.gcount() < acq.size()) return;
    std::vector<gps::AcqResult> results = gps::acquireAll(acq.data(), acq.size(), fCfg);
    std::vector<gps::AcqResult> found;
    for (std::size_t i = 0; i < results.size(); ++i)
        if (results[i].found) found.push_back(results[i]);

    { char b[100]; std::snprintf(b, sizeof(b),
          "[stream #%d] %d satellites in view; tracking...", fIter, (int)found.size());
      status(String(b)); }
    fAcqResults = results;
    Synchronize(doShowAcq);          // fill the Acquisition-tab bar chart
    Synchronize(doResetTracking);    // clear the Tracking tab for this pass
    Synchronize(doReset);            // clear the Position table for this pass
    if (found.size() < 4) { status(L"  fewer than 4 in view this pass - waiting."); return; }

    // --- read the integration window into the shared buffer at this offset ---
    const std::size_t need = (std::size_t)(fTrackMs + 2) * n;
    if (buf.size() < need) return;                           // safety (buffer sized in Execute)
    f.clear(); f.seekg((std::streamoff)offset, std::ios::beg);
    f.read(reinterpret_cast<char*>(buf.data()), (std::streamsize)need);
    const std::size_t got = (std::size_t)f.gcount();

    // --- parallel track + decode (same machinery as the single Compute Fix) ---
    const int N = (int)found.size();
    std::vector<PerSat>         out((std::size_t)N);
    std::atomic<int>            nextJob(0), doneCount(0);
    std::mutex                  mu;
    std::vector<String>            mq;
    std::vector<PosSatProgress>    rq;
    std::vector<GuiTrackedChannel> tq;
    auto pushSat = [&](const String& s, const PosSatProgress& p) {
        std::lock_guard<std::mutex> g(mu); mq.push_back(s); rq.push_back(p);
    };
    auto pushSatTracked = [&](const String& s, const PosSatProgress& p, GuiTrackedChannel& gc) {
        std::lock_guard<std::mutex> g(mu); mq.push_back(s); rq.push_back(p); tq.push_back(std::move(gc));
    };
    auto worker = [&]() {
        for (;;) {
            if (fStop || Terminated) break;
            const int k = nextJob.fetch_add(1);
            if (k >= N) break;
            const gps::AcqResult& a = found[(std::size_t)k];
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
            std::vector<gps::TrackEpoch> ep = ch.run(sp, sl, fTrackMs, &fStop);
            if (fStop || Terminated) { doneCount.fetch_add(1); continue; }  // aborted -> skip

            // Channel stats for the Tracking-tab grid + lock indicator (mirrors the
            // single-track worker): mean Doppler, I/Q power ratio, Doppler span.
            const int Nn = (int)ep.size();
            const int s0 = (Nn > 100) ? 100 : 0;
            double sIP = 0, sQP = 0, dMin = 1e9, dMax = -1e9, dSum = 0; int m = 0;
            for (int i = s0; i < Nn; ++i) {
                sIP += std::fabs(ep[i].iP); sQP += std::fabs(ep[i].qP);
                const double d = ep[i].doppler; dSum += d;
                if (d < dMin) dMin = d; if (d > dMax) dMax = d; ++m;
            }
            const double finalDop = m ? dSum / m : 0.0;
            prog.doppler = finalDop;
            const double ratio = sIP / (sQP > 0 ? sQP : 1);

            gps::BitSync bs = gps::findBitSync(ep);
            prog.cn0 = gps::estimateCN0(ep, bs.valid ? bs.offset : 0);
            const bool locked = (m > 0) && (ratio > 3.0) && ((dMax - dMin) < 200.0) && bs.valid;

            GuiTrackedChannel gc;
            gc.prn = a.prn; gc.finalDoppler = finalDop; gc.avgCn0 = prog.cn0; gc.locked = locked;
            gc.epochs = ep;     // copy for the Tracking-tab plots (ep is moved into the fix below)

            if (!bs.valid) {
                prog.state = 1;
                pushSatTracked(String(L"    PRN ") + IntToStr(a.prn) + L": no bit sync", prog, gc);
                doneCount.fetch_add(1); continue;
            }
            std::vector<int> bits = gps::demodulateBits(ep, bs.offset);
            gps::NavDecode nd = gps::decodeNav(bits, a.prn);
            prog.state = nd.eph.valid ? 3 : 2;
            pushSatTracked(String(L"    PRN ") + IntToStr(a.prn) +
                    (nd.eph.valid ? L": ephemeris OK" : L": ephemeris incomplete"), prog, gc);
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
    int nW = N; if (hw && nW > (int)hw) nW = (int)hw;
    std::vector<std::thread> pool; pool.reserve((std::size_t)nW);
    for (int i = 0; i < nW; ++i) pool.emplace_back(worker);

    auto drain = [&]() {
        std::vector<String> ms; std::vector<PosSatProgress> rs; std::vector<GuiTrackedChannel> ts;
        { std::lock_guard<std::mutex> g(mu); ms.swap(mq); rs.swap(rq); ts.swap(tq); }
        for (std::size_t i = 0; i < ms.size(); ++i) status(ms[i]);
        for (std::size_t i = 0; i < rs.size(); ++i) { fProgRow = rs[i]; Synchronize(doAddSat); }
        for (std::size_t i = 0; i < ts.size(); ++i) { fTrackedCh = std::move(ts[i]); Synchronize(doAddTracked); }
    };
    while (doneCount.load() < N && !fStop && !Terminated) { ::Sleep(120); drain(); }
    for (std::size_t i = 0; i < pool.size(); ++i) pool[i].join();
    drain();

    if (fStop || Terminated) return;

    std::vector<PvtChan> chans;
    for (int k = 0; k < N; ++k)
        if (out[(std::size_t)k].valid) chans.push_back(std::move(out[(std::size_t)k].chan));
    if (chans.size() < 4) { status(L"  fewer than 4 full ephemerides this pass."); return; }

    fFix = solveFixFromChannels(chans, tcfg.fs);
    if (!fFix.ok) { status(L"  could not align >= 4 channels this pass."); return; }
    Synchronize(doApply);
    { char b[120]; std::snprintf(b, sizeof(b),
          "[stream #%d] FIX: Lat %.6f, Lon %.6f, Alt %.0f m  (%d sats, GDOP %.2f)",
          fIter, fFix.lat, fFix.lon, fFix.alt, fFix.nSats, fFix.gdop); status(String(b)); }
}

//---------------------------------------------------------------------------
void __fastcall TMainForm::btnStreamClick(TObject* Sender)
{
    if (FStream) {                                  // already streaming -> stop
        static_cast<TStreamThread*>(FStream)->requestStop();
        FbtnStream->Enabled = false;
        FbtnStream->Caption = L"Stopping...";
        return;
    }
    if (FFilePath.IsEmpty()) {
        Status(L"Open a capture first (it stands in for the live DAC stream).");
        return;
    }

    PageControl1->ActivePage = tsPosition;          // jump to the Position tab
    buildPositionSky();
    resetPositionTable();
    memoFix->Clear();
    FSkyRows.clear(); if (FchSky) FchSky->Repaint();

    btnAcquire->Enabled = false;                    // no manual ops while streaming
    btnTrack->Enabled   = false;
    btnFix->Enabled     = false;

    gps::AcqConfig cfg;
    cfg.fs = FDacSampleRate; cfg.ifFreq = FIfHz;
    cfg.numMs = 2; cfg.dopplerStep = 500.0; cfg.threshold = 2.5;

    Status(L"Stream from DAC starting (simulated from " + ExtractFileName(FFilePath) + L")...");
    { char b[90]; std::snprintf(b, sizeof(b), "  DAC %.3f MHz, IF %.3f MHz, 36 s integration per pass.",
          FDacSampleRate / 1e6, FIfHz / 1e6); Status(String(b)); }

    if (FDeadStream) { delete FDeadStream; FDeadStream = NULL; }   // reap the previous worker

    FbtnStream->Caption = L"Stop Streaming";
    TStreamThread* w = new TStreamThread(this, FFilePath, cfg, 36000);
    w->OnTerminate = streamDone;
    FStream = w;
    w->Start();
}

//---------------------------------------------------------------------------
void TMainForm::streamStopped()
{
    FbtnStream->Caption = L"Stream from DAC";
    FbtnStream->Enabled = true;
    btnAcquire->Caption = L"Acquire";        btnAcquire->Enabled = true;
    btnTrack->Caption   = L"Track Acquired"; btnTrack->Enabled   = true;
    btnFix->Caption     = L"Compute Fix";    btnFix->Enabled     = true;
}

//---------------------------------------------------------------------------
// OnTerminate for the stream worker (fires on the main thread after Execute
// returns - for Stop, an error, or shutdown). Resets the GUI and parks the
// (non-FreeOnTerminate) thread object in FDeadStream for deferred deletion: a
// thread cannot delete itself from within its own OnTerminate.
void __fastcall TMainForm::streamDone(TObject* Sender)
{
    if (FStream == Sender) FStream = NULL;
    if (FDeadStream && FDeadStream != Sender) delete FDeadStream;  // reap the prior finished worker
    FDeadStream = static_cast<TThread*>(Sender);
    streamStopped();
}

//---------------------------------------------------------------------------
__fastcall TMainForm::~TMainForm()
{
    // Stop + wait for the stream worker before the form (and its GUI) is torn
    // down, so it can never Synchronize to a destroyed form. The abort flag makes
    // the in-flight track bail within a ms; WaitFor() on the main thread keeps
    // pumping Synchronize so there is no deadlock. OnTerminate is cleared first so
    // streamDone does not run mid-teardown.
    if (FStream) {
        TStreamThread* s = static_cast<TStreamThread*>(FStream);
        s->OnTerminate = NULL;     // streamDone must not run during teardown
        s->requestStop();
        s->WaitFor();              // fast (abort flag); pumps Synchronize -> no deadlock
        delete s;
        FStream = NULL;            // clear only after the worker is fully gone
    }
    if (FDeadStream) { delete FDeadStream; FDeadStream = NULL; }
    if (FAdvThread) {
        TThread* a = FAdvThread; FAdvThread = NULL;
        a->OnTerminate = NULL;
        // TAdvThread has requestStop(); cast is safe (defined below in this unit)
        // but to avoid an incomplete-type forward ref here we just terminate+wait.
        a->Terminate();
        a->WaitFor();
        delete a;
    }
    if (FAdvDead) { delete FAdvDead; FAdvDead = NULL; }
}

//===========================================================================
// Advanced inspection suite
//===========================================================================
static int comboPrn(TComboBox* cb)
{
    if (!cb || cb->ItemIndex < 0) return 0;
    return (int)(NativeInt)cb->Items->Objects[cb->ItemIndex];
}

// Worker that tracks + decodes every acquired satellite and caches the full
// per-PRN telemetry/nav detail + almanac + fix for the inspector popups. Mirrors
// the Compute-Fix worker but RETAINS everything. FreeOnTerminate=false; results
// are pulled by TMainForm::advDone (its OnTerminate) on the main thread.
class TAdvThread : public TThread
{
public:
    __fastcall TAdvThread(TMainForm* AForm, const String& AFile,
                          const std::vector<gps::AcqResult>& ASats,
                          const gps::AcqConfig& ACfg, int ATrackMs)
        : TThread(true), fForm(AForm), fFile(AFile), fSats(ASats),
          fCfg(ACfg), fTrackMs(ATrackMs)
    {
        FreeOnTerminate = false;
    }
    std::vector<AdvPrn> result;     // pulled by advDone on the main thread
    gps::AlmanacSet     almanac;
    GuiFix              fix;
    bool                ok = false;

protected:
    void __fastcall Execute();

private:
    TMainForm*                  fForm;
    String                      fFile;
    std::vector<gps::AcqResult> fSats;
    gps::AcqConfig              fCfg;
    int                         fTrackMs;
    String                      fPending;
    void status(const String& s) { fPending = s; Synchronize(doStatus); }
    void __fastcall doStatus()   { fForm->Status(fPending); }
};

void __fastcall TAdvThread::Execute()
{
    std::ifstream f(AnsiString(fFile).c_str(), std::ios::binary);
    if (!f) { status(L"Advanced: cannot open file."); return; }

    gps::TrackConfig tcfg; tcfg.fs = fCfg.fs; tcfg.ifFreq = fCfg.ifFreq;
    gps::AcqConfig   rcfg; rcfg.fs = tcfg.fs; rcfg.ifFreq = tcfg.ifFreq;
    const int n = (int)std::lround(tcfg.fs * 1.0e-3);

    std::size_t maxOff = 0;
    for (std::size_t k = 0; k < fSats.size(); ++k)
        if ((std::size_t)fSats[k].codePhaseSamp > maxOff) maxOff = (std::size_t)fSats[k].codePhaseSamp;
    const std::size_t need = (std::size_t)(fTrackMs + 2) * n;
    const std::size_t bufLen = need + maxOff;
    std::vector<std::int8_t> buf;
    try { buf.resize(bufLen); } catch (...) { status(L"Advanced: out of memory."); return; }
    f.seekg(0, std::ios::beg);
    f.read(reinterpret_cast<char*>(buf.data()), (std::streamsize)bufLen);
    const std::size_t got = (std::size_t)f.gcount();

    const int N = (int)fSats.size();
    std::vector<AdvPrn>  out((std::size_t)N);
    std::atomic<int>     nextJob(0), doneCount(0);
    std::mutex           mu;
    std::vector<String>  mq;
    auto worker = [&]() {
        for (;;) {
            if (Terminated) break;
            const int k = nextJob.fetch_add(1);
            if (k >= N) break;
            const gps::AcqResult& a = fSats[(std::size_t)k];
            const std::size_t off = (std::size_t)a.codePhaseSamp;
            if (off >= got) { doneCount.fetch_add(1); continue; }
            const std::int8_t* sp = buf.data() + off;
            const std::size_t  sl = got - off;
            const double fd = gps::refineDoppler(sp, sl, a.prn, a.doppler, rcfg);
            gps::TrackChannel ch(a.prn, fd, tcfg);
            std::vector<gps::TrackEpoch> ep = ch.run(sp, sl, fTrackMs, NULL);
            if (Terminated) { doneCount.fetch_add(1); continue; }

            AdvPrn ap;
            ap.prn = a.prn; ap.codePhaseSamp = a.codePhaseSamp;
            if (!ep.empty()) {
                std::size_t s0 = ep.size() * 9 / 10; double ds = 0; int dc = 0;
                for (std::size_t i = s0; i < ep.size(); ++i) { ds += ep[i].doppler; ++dc; }
                ap.doppler = dc ? ds / dc : 0.0;
            }
            gps::BitSync bs = gps::findBitSync(ep);
            ap.cn0 = gps::estimateCN0(ep, bs.valid ? bs.offset : 0);
            ap.bitOffset = bs.valid ? bs.offset : 0;
            ap.epochs = std::move(ep);
            if (bs.valid) {
                ap.bits   = gps::demodulateBits(ap.epochs, bs.offset);
                ap.nav    = gps::decodeNav(ap.bits, a.prn);
                ap.detail = gps::inspectNav(ap.bits, a.prn);
                ap.valid  = true;
            }
            out[(std::size_t)k] = std::move(ap);
            { char b[64]; std::snprintf(b, sizeof(b), "  analyzed PRN %d", a.prn);
              std::lock_guard<std::mutex> g(mu); mq.push_back(String(b)); }
            doneCount.fetch_add(1);
        }
    };
    unsigned hw = std::thread::hardware_concurrency();
    int nW = N; if (hw && nW > (int)hw) nW = (int)hw;
    std::vector<std::thread> pool; pool.reserve((std::size_t)nW);
    for (int i = 0; i < nW; ++i) pool.emplace_back(worker);
    while (doneCount.load() < N && !Terminated) {
        ::Sleep(150);
        std::vector<String> batch;
        { std::lock_guard<std::mutex> g(mu); batch.swap(mq); }
        for (std::size_t i = 0; i < batch.size(); ++i) status(batch[i]);
    }
    for (std::size_t i = 0; i < pool.size(); ++i) pool[i].join();
    if (Terminated) return;

    std::vector<int> allBits;
    std::vector<PvtChan> chans;
    for (int k = 0; k < N; ++k) {
        if (out[(std::size_t)k].prn == 0) continue;
        if (out[(std::size_t)k].valid) {
            for (int b : out[(std::size_t)k].bits) allBits.push_back(b);
            if (out[(std::size_t)k].nav.eph.valid) {
                PvtChan c;
                c.prn = out[(std::size_t)k].prn;
                c.codePhaseSamp = out[(std::size_t)k].codePhaseSamp;
                c.bitOffset = out[(std::size_t)k].bitOffset;
                c.eph  = out[(std::size_t)k].nav.eph;
                c.ep   = out[(std::size_t)k].epochs;           // copy (solve needs sampleIndex)
                c.subs = out[(std::size_t)k].nav.subframes;
                chans.push_back(std::move(c));
            }
        }
        result.push_back(std::move(out[(std::size_t)k]));
    }
    almanac = gps::decodeAlmanac(allBits);
    if (chans.size() >= 4) fix = solveFixFromChannels(chans, tcfg.fs);
    ok = !result.empty();
}

//---------------------------------------------------------------------------
void __fastcall TMainForm::advCheckClick(TObject* Sender)
{
    tsAdvanced->TabVisible = FchkAdvanced->Checked;
    if (FchkAdvanced->Checked) PageControl1->ActivePage = tsAdvanced;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::inspectorClose(TObject* Sender, TCloseAction& Action)
{
    Action = caFree;
}
//---------------------------------------------------------------------------
TForm* TMainForm::makeInspector(const String& title, int w, int h)
{
    TForm* f = new TForm(Application);
    f->Caption  = title;
    f->Width = dpx(w); f->Height = dpx(h);   // pre-size the form in scaled pixels
    f->Position = poMainFormCenter;
    f->OnClose  = inspectorClose;
    return f;
}
//---------------------------------------------------------------------------
// Show a runtime-built popup after DPI-scaling its whole control subtree from
// the 96-DPI design coordinates to the monitor DPI (the popup is born at 96).
// This is the same scaling VCL applies to .dfm forms, so charts, grids, images
// and nested panels all scale correctly in one call.
void TMainForm::dpiShow(TForm* f)
{
    scaleChildrenForDpi(f);   // scale child geometry 96 -> screen DPI (fonts already correct)
    f->Show();
}
//---------------------------------------------------------------------------
const AdvPrn* TMainForm::advFind(int prn) const
{
    for (std::size_t i = 0; i < FAdv.size(); ++i)
        if (FAdv[i].prn == prn) return &FAdv[i];
    return NULL;
}
//---------------------------------------------------------------------------
void TMainForm::advFillPrnCombo(TComboBox* cb, bool ephemerisOnly)
{
    cb->Items->Clear();
    for (std::size_t i = 0; i < FAdv.size(); ++i) {
        if (ephemerisOnly && !FAdv[i].nav.eph.valid) continue;
        cb->Items->AddObject(L"PRN " + IntToStr(FAdv[i].prn), (TObject*)(NativeInt)FAdv[i].prn);
    }
    if (cb->Items->Count > 0) cb->ItemIndex = 0;
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::advAnalyzeClick(TObject* Sender)
{
    if (FAdvThread) { Status(L"Advanced: analysis already running."); return; }
    if (!FHaveResults)       { Status(L"Acquire satellites first (Acquisition tab)."); return; }
    if (FFilePath.IsEmpty()) { Status(L"No file loaded."); return; }

    std::vector<gps::AcqResult> found;
    for (int prn = 1; prn <= 32; ++prn)
        if (FResults[prn].found) found.push_back(FResults[prn]);
    if (found.empty()) { Status(L"No acquired satellites to analyze."); return; }

    if (FAdvDead) { delete FAdvDead; FAdvDead = NULL; }

    gps::AcqConfig cfg = configForFile(FFilePath);
    FbtnAnalyze->Enabled = false;
    FbtnAnalyze->Caption = L"Analyzing...";
    char b[100];
    std::snprintf(b, sizeof(b), "Advanced: tracking + decoding %d satellites for inspection (~30 s)...",
                  (int)found.size());
    Status(String(b));

    TAdvThread* w = new TAdvThread(this, FFilePath, found, cfg, 36000);
    w->OnTerminate = advDone;
    FAdvThread = w;
    w->Start();
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::advDone(TObject* Sender)
{
    TAdvThread* t = static_cast<TAdvThread*>(Sender);
    if (FAdvThread == Sender) FAdvThread = NULL;
    if (t->ok) {
        FAdv        = std::move(t->result);
        FAdvAlmanac = t->almanac;
        FAdvFix     = t->fix;
        FAdvReady   = true;
        for (std::size_t i = 0; i < FAdvButtons.size(); ++i) FAdvButtons[i]->Enabled = true;
        char b[100];
        std::snprintf(b, sizeof(b), "Advanced: analysis complete - %d satellites cached, inspectors enabled.",
                      (int)FAdv.size());
        Status(String(b));
    } else {
        Status(L"Advanced: analysis did not complete.");
    }
    FbtnAnalyze->Enabled = true;
    FbtnAnalyze->Caption = L"Analyze (track + decode all)";
    if (FAdvDead && FAdvDead != Sender) delete FAdvDead;
    FAdvDead = static_cast<TThread*>(Sender);
}

//===========================================================================
// Inspector: C/A Code & Correlation  (standalone - needs no analysis)
//===========================================================================
void __fastcall TMainForm::advCodeClick(TObject* Sender)
{
    TForm* f = makeInspector(L"C/A Code & Correlation", 920, 660);

    TLabel* la = new TLabel(f); la->Parent = f; la->SetBounds(12, 12, 48, 20); la->Caption = L"PRN A:";
    FcodePrnA = new TComboBox(f); FcodePrnA->Parent = f; FcodePrnA->SetBounds(60, 8, 70, 24);
    FcodePrnA->Style = csDropDownList;
    TLabel* lb = new TLabel(f); lb->Parent = f; lb->SetBounds(150, 12, 70, 20); lb->Caption = L"vs PRN B:";
    FcodePrnB = new TComboBox(f); FcodePrnB->Parent = f; FcodePrnB->SetBounds(220, 8, 70, 24);
    FcodePrnB->Style = csDropDownList;
    for (int p = 1; p <= 32; ++p) {
        FcodePrnA->Items->AddObject(IntToStr(p), (TObject*)(NativeInt)p);
        FcodePrnB->Items->AddObject(IntToStr(p), (TObject*)(NativeInt)p);
    }
    FcodePrnA->ItemIndex = 0; FcodePrnB->ItemIndex = 1;
    FcodePrnA->OnChange = advCodeChange; FcodePrnB->OnChange = advCodeChange;
    FcodeLbl = new TLabel(f); FcodeLbl->Parent = f; FcodeLbl->SetBounds(310, 12, 590, 20); FcodeLbl->AutoSize = false;

    FcodeChart = new TChart(f); FcodeChart->Parent = f; FcodeChart->SetBounds(12, 44, 894, 230);
    FcodeChart->Anchors = TAnchors() << akLeft << akTop << akRight;
    FcodeChart->View3D = false; FcodeChart->Legend->Visible = false;
    FcodeChart->BottomAxis->Title->Caption = L"chip index"; FcodeChart->LeftAxis->Title->Caption = L"chip value";
    TFastLineSeries* cs = new TFastLineSeries(FcodeChart); FcodeChart->AddSeries(cs);

    FcorrChart = new TChart(f); FcorrChart->Parent = f; FcorrChart->SetBounds(12, 284, 894, 290);
    FcorrChart->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;
    FcorrChart->View3D = false; FcorrChart->Legend->Visible = true; FcorrChart->Legend->Alignment = laBottom;
    FcorrChart->Title->Text->Text = L"Circular correlation over all 1023 code-phase lags";
    FcorrChart->BottomAxis->Title->Caption = L"code-phase lag (chips)";
    FcorrChart->LeftAxis->Title->Caption = L"correlation";
    TFastLineSeries* au = new TFastLineSeries(FcorrChart); FcorrChart->AddSeries(au); au->Title = L"auto-corr(A)";
    TFastLineSeries* xc = new TFastLineSeries(FcorrChart); FcorrChart->AddSeries(xc); xc->Title = L"cross-corr(A,B)";

    TMemo* m = new TMemo(f); m->Parent = f; m->SetBounds(12, 580, 894, 44);
    m->Anchors = TAnchors() << akLeft << akRight << akBottom; m->ReadOnly = true;
    m->Lines->Add(L"A 1023-chip Gold code repeats every 1 ms at 1.023 Mcps (~37.33 samples/chip at 38.192 MHz). "
                  L"Auto-correlation is a 1023-high, 1-chip-wide spike = the ~30 dB processing gain that lifts the "
                  L"satellite out of the noise; cross-correlation between PRNs stays low and bounded (CDMA).");
    advCodeChange(NULL);
    dpiShow(f);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::advCodeChange(TObject* Sender)
{
    if (!FcodeChart || !FcorrChart) return;
    int pa = comboPrn(FcodePrnA), pb = comboPrn(FcodePrnB);
    if (pa < 1) pa = 1; if (pb < 1) pb = 2;
    std::vector<std::int8_t> a = gps::generateCACode(pa);
    std::vector<std::int8_t> b = gps::generateCACode(pb);
    const int L = gps::CA_CODE_LENGTH;

    TChartSeries* cs = FcodeChart->Series[0]; cs->Clear();
    for (int i = 0; i < 64 && i < (int)a.size(); ++i) cs->AddXY((double)i, (double)a[i], L"", clTeeColor);
    FcodeChart->Title->Text->Text = String(L"C/A code chips (first 64 of 1023) - PRN ") + IntToStr(pa);

    TChartSeries* au = FcorrChart->Series[0]; au->Clear();
    TChartSeries* xc = FcorrChart->Series[1]; xc->Clear();
    int autoMax = 0, crossMax = 0;
    for (int lag = 0; lag < L; ++lag) {
        int sa = 0, sx = 0;
        for (int i = 0; i < L; ++i) { int j = i + lag; if (j >= L) j -= L; sa += a[i] * a[j]; sx += a[i] * b[j]; }
        au->AddXY((double)lag, (double)sa, L"", clTeeColor);
        xc->AddXY((double)lag, (double)sx, L"", clTeeColor);
        if (lag != 0 && std::abs(sa) > autoMax) autoMax = std::abs(sa);
        if (std::abs(sx) > crossMax) crossMax = std::abs(sx);
    }
    char buf[220];
    std::snprintf(buf, sizeof(buf),
        "PRN %d: auto peak = %d (lag 0), max sidelobe = %d;  max |cross| vs PRN %d = %d  ->  ~%.0f dB separation",
        pa, L, autoMax, pb, crossMax, 20.0 * std::log10((double)L / (crossMax > 0 ? crossMax : 1)));
    FcodeLbl->Caption = String(buf);
}

//===========================================================================
// Inspector: Tracking Loops Lab  (uses cached epochs)
//===========================================================================
void __fastcall TMainForm::advTrackClick(TObject* Sender)
{
    if (!FAdvReady) { Status(L"Click Analyze first."); return; }
    TForm* f = makeInspector(L"Tracking Loops Lab", 960, 700);

    TLabel* l = new TLabel(f); l->Parent = f; l->SetBounds(12, 12, 36, 20); l->Caption = L"PRN:";
    FtrkPrn = new TComboBox(f); FtrkPrn->Parent = f; FtrkPrn->SetBounds(50, 8, 80, 24); FtrkPrn->Style = csDropDownList;
    advFillPrnCombo(FtrkPrn, false);
    FtrkPrn->OnChange = advTrackChange;
    FtrkLbl = new TLabel(f); FtrkLbl->Parent = f; FtrkLbl->SetBounds(150, 12, 790, 20); FtrkLbl->AutoSize = false;

    FtrkIQ = new TChart(f); FtrkIQ->Parent = f; FtrkIQ->SetBounds(12, 44, 360, 320);
    FtrkIQ->Anchors = TAnchors() << akLeft << akTop; FtrkIQ->View3D = false; FtrkIQ->Legend->Visible = false;
    FtrkIQ->Title->Text->Text = L"Prompt I/Q constellation";
    FtrkIQ->BottomAxis->Title->Caption = L"I (prompt)"; FtrkIQ->LeftAxis->Title->Caption = L"Q (prompt)";
    TPointSeries* iq = new TPointSeries(FtrkIQ); FtrkIQ->AddSeries(iq);
    iq->Pointer->Size = 1; iq->Pointer->Style = psCircle;

    FtrkDisc = new TChart(f); FtrkDisc->Parent = f; FtrkDisc->SetBounds(380, 44, 566, 320);
    FtrkDisc->Anchors = TAnchors() << akLeft << akTop << akRight; FtrkDisc->View3D = false;
    FtrkDisc->Legend->Visible = true; FtrkDisc->Legend->Alignment = laBottom;
    FtrkDisc->Title->Text->Text = L"Loop discriminators vs time";
    FtrkDisc->BottomAxis->Title->Caption = L"epoch (ms)";
    TFastLineSeries* pll = new TFastLineSeries(FtrkDisc); FtrkDisc->AddSeries(pll); pll->Title = L"PLL (cycles)";
    TFastLineSeries* dll = new TFastLineSeries(FtrkDisc); FtrkDisc->AddSeries(dll); dll->Title = L"DLL (chips)";

    FtrkObs = new TChart(f); FtrkObs->Parent = f; FtrkObs->SetBounds(12, 374, 934, 296);
    FtrkObs->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom; FtrkObs->View3D = false;
    FtrkObs->Legend->Visible = true; FtrkObs->Legend->Alignment = laBottom;
    FtrkObs->Title->Text->Text = L"Observables: Doppler (Hz, left)  /  C/N0 (dB-Hz, right)";
    FtrkObs->BottomAxis->Title->Caption = L"epoch (ms)";
    FtrkObs->LeftAxis->Title->Caption = L"Doppler (Hz)"; FtrkObs->RightAxis->Title->Caption = L"C/N0 (dB-Hz)";
    TFastLineSeries* dop = new TFastLineSeries(FtrkObs); FtrkObs->AddSeries(dop); dop->Title = L"Doppler";
    TFastLineSeries* cn0 = new TFastLineSeries(FtrkObs); FtrkObs->AddSeries(cn0); cn0->Title = L"C/N0"; cn0->VertAxis = aRightAxis;

    advTrackChange(NULL);
    dpiShow(f);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::advTrackChange(TObject* Sender)
{
    if (!FtrkIQ || !FtrkDisc || !FtrkObs) return;
    const AdvPrn* a = advFind(comboPrn(FtrkPrn));
    if (!a) return;
    const std::vector<gps::TrackEpoch>& ep = a->epochs;
    const int N = (int)ep.size();

    TChartSeries* iq = FtrkIQ->Series[0]; iq->Clear();
    const int s = (N > 50) ? 50 : 0;
    const int stepIQ = std::max(1, (N - s) / 1500);
    for (int i = s; i < N; i += stepIQ) iq->AddXY(ep[i].iP, ep[i].qP, L"", clNavy);

    TChartSeries* pll = FtrkDisc->Series[0]; pll->Clear();
    TChartSeries* dll = FtrkDisc->Series[1]; dll->Clear();
    TChartSeries* dop = FtrkObs->Series[0]; dop->Clear();
    TChartSeries* cn0 = FtrkObs->Series[1]; cn0->Clear();
    const int stepT = std::max(1, N / 800);
    for (int i = 0; i < N; i += stepT) {
        pll->AddXY((double)i, ep[i].pllDisc, L"", clTeeColor);
        dll->AddXY((double)i, ep[i].dllDisc, L"", clTeeColor);
        dop->AddXY((double)i, ep[i].doppler, L"", clTeeColor);
        cn0->AddXY((double)i, ep[i].cn0,     L"", clTeeColor);
    }
    char b[240];
    std::snprintf(b, sizeof(b),
        "PRN %d: %d ms tracked, mean Doppler %+.0f Hz, C/N0 ~%.1f dB-Hz.  Two I/Q clusters = carrier-locked BPSK "
        "(prompt-I sign = the 50 bps nav bits); the discriminators settle to noise around zero.",
        a->prn, N, a->doppler, a->cn0);
    FtrkLbl->Caption = String(b);
}

//===========================================================================
// Inspector: Nav Frame & Bits  (uses cached NavDetail)
//===========================================================================
void __fastcall TMainForm::advNavClick(TObject* Sender)
{
    if (!FAdvReady) { Status(L"Click Analyze first."); return; }
    TForm* f = makeInspector(L"Nav Frame & Bits", 960, 700);

    TLabel* l = new TLabel(f); l->Parent = f; l->SetBounds(12, 12, 36, 20); l->Caption = L"PRN:";
    FnavPrn = new TComboBox(f); FnavPrn->Parent = f; FnavPrn->SetBounds(50, 8, 80, 24); FnavPrn->Style = csDropDownList;
    advFillPrnCombo(FnavPrn, false);
    FnavPrn->OnChange = advNavChange;

    TLabel* l2 = new TLabel(f); l2->Parent = f; l2->SetBounds(12, 40, 460, 18);
    l2->Caption = L"Parity-passing subframes:";
    FnavSubs = new TStringGrid(f); FnavSubs->Parent = f; FnavSubs->SetBounds(12, 60, 470, 250);
    FnavSubs->Anchors = TAnchors() << akLeft << akTop << akBottom;
    FnavSubs->ColCount = 6; FnavSubs->FixedCols = 0; FnavSubs->RowCount = 2;
    FnavSubs->Options = FnavSubs->Options << goRowSelect;

    FnavWords = new TStringGrid(f); FnavWords->Parent = f; FnavWords->SetBounds(496, 60, 450, 250);
    FnavWords->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;
    FnavWords->ColCount = 4; FnavWords->FixedCols = 0; FnavWords->RowCount = 11;

    TLabel* l3 = new TLabel(f); l3->Parent = f; l3->SetBounds(12, 320, 460, 18);
    l3->Caption = L"Recovered bit stream (subframe boundaries marked):";
    FnavDump = new TMemo(f); FnavDump->Parent = f; FnavDump->SetBounds(12, 340, 934, 330);
    FnavDump->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;
    FnavDump->ReadOnly = true; FnavDump->ScrollBars = ssBoth;
    FnavDump->Font->Name = L"Consolas"; FnavDump->Font->Size = 9;

    advNavChange(NULL);
    dpiShow(f);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::advNavChange(TObject* Sender)
{
    if (!FnavSubs || !FnavWords || !FnavDump) return;
    const AdvPrn* a = advFind(comboPrn(FnavPrn));
    if (!a) return;
    const gps::NavDetail& d = a->detail;

    FnavSubs->RowCount = (int)d.subframes.size() + 1;
    const wchar_t* hdr[6] = { L"SF", L"TOW", L"page", L"alert/AS", L"words OK", L"bit idx" };
    for (int c = 0; c < 6; ++c) FnavSubs->Cells[c][0] = hdr[c];
    for (std::size_t i = 0; i < d.subframes.size(); ++i) {
        const gps::NavSubframe& s = d.subframes[i];
        int okw = 0; for (int w = 0; w < 10; ++w) if (s.words[w].parityOk) ++okw;
        char b[24];
        FnavSubs->Cells[0][i + 1] = IntToStr(s.id);
        FnavSubs->Cells[1][i + 1] = IntToStr(s.towCount);
        FnavSubs->Cells[2][i + 1] = (s.id == 4 || s.id == 5) ? IntToStr(s.page) : String(L"-");
        FnavSubs->Cells[3][i + 1] = String(s.alert ? L"A" : L"-") + (s.antiSpoof ? L"/AS" : L"/-");
        std::snprintf(b, sizeof(b), "%d/10", okw); FnavSubs->Cells[4][i + 1] = String(b);
        FnavSubs->Cells[5][i + 1] = IntToStr(s.bitIndex);
    }

    const wchar_t* wh[4] = { L"word", L"raw (hex)", L"data (hex)", L"parity" };
    for (int c = 0; c < 4; ++c) FnavWords->Cells[c][0] = wh[c];
    if (!d.subframes.empty()) {
        const gps::NavSubframe& s = d.subframes[0];
        for (int w = 0; w < 10; ++w) {
            char b[16];
            FnavWords->Cells[0][w + 1] = IntToStr(w + 1);
            std::snprintf(b, sizeof(b), "%08X", s.words[w].raw);  FnavWords->Cells[1][w + 1] = String(b);
            std::snprintf(b, sizeof(b), "%06X", s.words[w].data); FnavWords->Cells[2][w + 1] = String(b);
            FnavWords->Cells[3][w + 1] = s.words[w].parityOk ? L"OK" : L"FAIL";
        }
    }

    FnavDump->Lines->BeginUpdate();
    FnavDump->Clear();
    char hl[140];
    std::snprintf(hl, sizeof(hl), "PRN %d  polarity=%s  parityFails=%d  subframes=%d  (preamble 0x8B; 30 bits/word, 10 words = 300-bit/6 s subframe)",
                  a->prn, d.polarity ? "inverted" : "upright", d.totalParityFails, (int)d.subframes.size());
    FnavDump->Lines->Add(String(hl));
    for (std::size_t i = 0; i < d.subframes.size(); ++i) {
        const gps::NavSubframe& s = d.subframes[i];
        char sh[80];
        std::snprintf(sh, sizeof(sh), "-- Subframe %d  TOW=%d  page=%d --", s.id, s.towCount, s.page);
        FnavDump->Lines->Add(String(sh));
        for (int w = 0; w < 10; ++w) {
            String line = L"  w";
            if (w + 1 < 10) line += L" ";
            line += IntToStr(w + 1) + L" ";
            std::uint32_t r = s.words[w].raw;
            for (int bk = 29; bk >= 0; --bk) { line += ((r >> bk) & 1) ? L"1" : L"0"; if (bk % 6 == 0) line += L" "; }
            line += s.words[w].parityOk ? L"OK" : L"FAIL";
            FnavDump->Lines->Add(line);
        }
    }
    FnavDump->Lines->EndUpdate();
}

//===========================================================================
// Inspector: Ephemeris Decoder & SV Clock  (uses cached Ephemeris)
//===========================================================================
void __fastcall TMainForm::advEphClick(TObject* Sender)
{
    if (!FAdvReady) { Status(L"Click Analyze first."); return; }
    TForm* f = makeInspector(L"Ephemeris Decoder & SV Clock", 960, 700);

    TLabel* l = new TLabel(f); l->Parent = f; l->SetBounds(12, 12, 36, 20); l->Caption = L"PRN:";
    FephPrn = new TComboBox(f); FephPrn->Parent = f; FephPrn->SetBounds(50, 8, 80, 24); FephPrn->Style = csDropDownList;
    advFillPrnCombo(FephPrn, true);
    FephPrn->OnChange = advEphChange;

    FephGrid = new TStringGrid(f); FephGrid->Parent = f; FephGrid->SetBounds(12, 44, 934, 500);
    FephGrid->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;
    FephGrid->ColCount = 4; FephGrid->FixedCols = 0; FephGrid->RowCount = 2;
    FephGrid->ColWidths[0] = 150; FephGrid->ColWidths[1] = 220; FephGrid->ColWidths[2] = 130; FephGrid->ColWidths[3] = 420;

    FephMemo = new TMemo(f); FephMemo->Parent = f; FephMemo->SetBounds(12, 552, 934, 118);
    FephMemo->Anchors = TAnchors() << akLeft << akRight << akBottom;
    FephMemo->ReadOnly = true; FephMemo->Font->Name = L"Consolas"; FephMemo->Font->Size = 9;

    advEphChange(NULL);
    dpiShow(f);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::advEphChange(TObject* Sender)
{
    if (!FephGrid || !FephMemo) return;
    const AdvPrn* a = advFind(comboPrn(FephPrn));
    if (!a || !a->nav.eph.valid) return;
    const gps::Ephemeris& e = a->nav.eph;

    struct Row { const wchar_t* field; double val; const wchar_t* units; const wchar_t* meaning; };
    Row rows[] = {
        { L"WN",        (double)e.weekNumber, L"week",   L"GPS week number (mod 1024)" },
        { L"IODC",      (double)e.iodc,       L"-",      L"Issue Of Data, Clock" },
        { L"IODE",      (double)e.iode,       L"-",      L"Issue Of Data, Ephemeris (== IODC LSBs)" },
        { L"toc",       e.toc,                L"s",      L"clock reference time of week" },
        { L"af0",       e.af0,                L"s",      L"SV clock bias" },
        { L"af1",       e.af1,                L"s/s",    L"SV clock drift" },
        { L"af2",       e.af2,                L"s/s^2",  L"SV clock drift rate" },
        { L"TGD",       e.tgd,                L"s",      L"group delay differential" },
        { L"toe",       e.toe,                L"s",      L"ephemeris reference time of week" },
        { L"sqrtA",     e.sqrtA,              L"sqrt(m)",L"square root of semi-major axis" },
        { L"e",         e.ecc,                L"-",      L"orbit eccentricity" },
        { L"M0",        e.m0,                 L"rad",    L"mean anomaly at toe" },
        { L"deltaN",    e.deltaN,             L"rad/s",  L"mean motion correction" },
        { L"Omega0",    e.omega0,             L"rad",    L"longitude of ascending node @ week start" },
        { L"OmegaDot",  e.omegaDot,           L"rad/s",  L"rate of right ascension" },
        { L"i0",        e.i0,                 L"rad",    L"inclination at toe" },
        { L"IDOT",      e.idot,               L"rad/s",  L"rate of inclination" },
        { L"omega",     e.omega,              L"rad",    L"argument of perigee" },
        { L"Cuc",       e.cuc,                L"rad",    L"cos harmonic, arg of latitude" },
        { L"Cus",       e.cus,                L"rad",    L"sin harmonic, arg of latitude" },
        { L"Crc",       e.crc,                L"m",      L"cos harmonic, orbit radius" },
        { L"Crs",       e.crs,                L"m",      L"sin harmonic, orbit radius" },
        { L"Cic",       e.cic,                L"rad",    L"cos harmonic, inclination" },
        { L"Cis",       e.cis,                L"rad",    L"sin harmonic, inclination" },
    };
    const int nr = (int)(sizeof(rows) / sizeof(rows[0]));
    FephGrid->RowCount = nr + 1;
    FephGrid->Cells[0][0] = L"Field"; FephGrid->Cells[1][0] = L"Value";
    FephGrid->Cells[2][0] = L"Units"; FephGrid->Cells[3][0] = L"Meaning";
    for (int i = 0; i < nr; ++i) {
        char v[40];
        std::snprintf(v, sizeof(v), "%.10g", rows[i].val);
        FephGrid->Cells[0][i + 1] = rows[i].field;
        FephGrid->Cells[1][i + 1] = String(v);
        FephGrid->Cells[2][i + 1] = rows[i].units;
        FephGrid->Cells[3][i + 1] = rows[i].meaning;
    }

    const double a_m = e.sqrtA * e.sqrtA;
    const double PI = 3.14159265358979;
    FephMemo->Lines->BeginUpdate(); FephMemo->Clear();
    char b[180];
    std::snprintf(b, sizeof(b), "Derived:  a = sqrtA^2 = %.1f m (%.1f km),  i0 = %.2f deg,  Omega0 = %.2f deg,  period ~ %.0f min",
                  a_m, a_m / 1e3, e.i0 * 180.0 / PI, e.omega0 * 180.0 / PI,
                  2.0 * PI * std::sqrt(a_m * a_m * a_m / 3.986005e14) / 60.0);
    FephMemo->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "Sanity: e<0.03? %s   sqrtA~5153? %s   i0~0.97rad? %s   IODE==IODC&0xFF? %s",
                  (e.ecc >= 0 && e.ecc < 0.03) ? "yes" : "NO",
                  (e.sqrtA > 5000 && e.sqrtA < 5300) ? "yes" : "NO",
                  (e.i0 > 0.8 && e.i0 < 1.2) ? "yes" : "NO",
                  (e.iode == (e.iodc & 0xFF)) ? "yes" : "NO");
    FephMemo->Lines->Add(String(b));
    FephMemo->Lines->Add(L"SV clock: dt_sv(t) = af0 + af1*(t-toc) + af2*(t-toc)^2 + relativistic - TGD.  "
                         L"Each field is a scaled integer (2^-n) broadcast in subframes 1/2/3.");
    FephMemo->Lines->EndUpdate();
}

//===========================================================================
// Inspector: Almanac & SF4/5 Pages  (uses cached AlmanacSet)
//===========================================================================
void __fastcall TMainForm::advAlmClick(TObject* Sender)
{
    if (!FAdvReady) { Status(L"Click Analyze first."); return; }
    TForm* f = makeInspector(L"Almanac & SF4/5 Pages", 940, 660);

    TStringGrid* g = new TStringGrid(f); g->Parent = f; g->SetBounds(12, 12, 916, 420);
    g->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;
    g->ColCount = 8; g->FixedCols = 0; g->RowCount = 2;
    const wchar_t* hdr[8] = { L"PRN", L"sqrtA", L"a (km)", L"e", L"i0 (deg)", L"OmegaDot", L"af0 (s)", L"health" };
    int colw[8] = { 50, 90, 90, 100, 80, 110, 110, 60 };
    for (int c = 0; c < 8; ++c) { g->Cells[c][0] = hdr[c]; g->ColWidths[c] = colw[c]; }
    int row = 0;
    for (int prn = 1; prn <= 32; ++prn) {
        const gps::Almanac& a = FAdvAlmanac.alm[prn];
        if (!a.valid) continue;
        ++row; if (g->RowCount < row + 1) g->RowCount = row + 1;
        char b[40];
        g->Cells[0][row] = IntToStr(prn);
        std::snprintf(b, sizeof(b), "%.3f", a.sqrtA);                 g->Cells[1][row] = String(b);
        std::snprintf(b, sizeof(b), "%.1f", a.sqrtA * a.sqrtA / 1e3); g->Cells[2][row] = String(b);
        std::snprintf(b, sizeof(b), "%.6f", a.ecc);                  g->Cells[3][row] = String(b);
        std::snprintf(b, sizeof(b), "%.2f", a.i0 * 180.0 / 3.14159265); g->Cells[4][row] = String(b);
        std::snprintf(b, sizeof(b), "%.3e", a.omegaDot);             g->Cells[5][row] = String(b);
        std::snprintf(b, sizeof(b), "%.3e", a.af0);                  g->Cells[6][row] = String(b);
        g->Cells[7][row] = IntToStr(a.health);
    }

    TMemo* m = new TMemo(f); m->Parent = f; m->SetBounds(12, 440, 916, 178);
    m->Anchors = TAnchors() << akLeft << akRight << akBottom; m->ReadOnly = true;
    m->Font->Name = L"Consolas"; m->Font->Size = 9;
    m->Lines->BeginUpdate();
    m->Lines->Add(L"Almanac = coarse, long-validity reduced ephemeris for the WHOLE constellation, carried across "
                  L"subframe 4/5 pages (vs precise ~2 hr ephemeris for one SV). Used for fast cold-start aiding.");
    String ps = L"Pages captured (SV-ID/page): ";
    for (std::size_t i = 0; i < FAdvAlmanac.pagesSeen.size() && i < 40; ++i) ps += IntToStr(FAdvAlmanac.pagesSeen[i]) + L" ";
    m->Lines->Add(ps);
    String wnLine = L"Almanac ref week (WNa) = ";
    wnLine += (FAdvAlmanac.refWeek >= 0) ? IntToStr(FAdvAlmanac.refWeek) : String(L"(not in this capture)");
    wnLine += L".  Iono page: "; wnLine += FAdvAlmanac.haveIono ? L"yes" : L"no";
    wnLine += L".  UTC page: ";   wnLine += FAdvAlmanac.haveUtc  ? L"yes" : L"no";
    m->Lines->Add(wnLine);
    if (FAdvAlmanac.haveIono) {
        char b[160];
        std::snprintf(b, sizeof(b), "Klobuchar alpha: %.3e %.3e %.3e %.3e   beta: %.0f %.0f %.0f %.0f",
                      FAdvAlmanac.alpha[0], FAdvAlmanac.alpha[1], FAdvAlmanac.alpha[2], FAdvAlmanac.alpha[3],
                      FAdvAlmanac.beta[0], FAdvAlmanac.beta[1], FAdvAlmanac.beta[2], FAdvAlmanac.beta[3]);
        m->Lines->Add(String(b));
    }
    m->Lines->Add(L"NOTE: the full almanac (all 32 PRNs + iono/UTC) spans the 25-page cycle = 12.5 minutes, so a "
                  L"short capture only shows the few pages it contains - which is why almanac download is slow.");
    m->Lines->EndUpdate();
    dpiShow(f);
}

//===========================================================================
// Inspector: PVT Solver Lab  (uses cached fix)
//===========================================================================
void __fastcall TMainForm::advPvtClick(TObject* Sender)
{
    if (!FAdvReady)  { Status(L"Click Analyze first."); return; }
    if (!FAdvFix.ok) { Status(L"No position fix in the cached analysis (need >= 4 satellites)."); return; }
    TForm* f = makeInspector(L"PVT Solver Lab", 940, 640);

    TStringGrid* g = new TStringGrid(f); g->Parent = f; g->SetBounds(12, 12, 916, 360);
    g->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;
    g->ColCount = 7; g->FixedCols = 0; g->RowCount = (int)FAdvFix.rows.size() + 1;
    const wchar_t* hdr[7] = { L"PRN", L"pseudorange (km)", L"SV X (km)", L"SV Y (km)", L"SV Z (km)", L"SV clk (us)", L"elev (deg)" };
    int colw[7] = { 50, 150, 130, 130, 130, 110, 90 };
    for (int c = 0; c < 7; ++c) { g->Cells[c][0] = hdr[c]; g->ColWidths[c] = colw[c]; }
    for (std::size_t i = 0; i < FAdvFix.rows.size(); ++i) {
        const GuiSatRow& r = FAdvFix.rows[i];
        char b[40]; const int row = (int)i + 1;
        g->Cells[0][row] = IntToStr(r.prn);
        std::snprintf(b, sizeof(b), "%.3f", r.prKm);     g->Cells[1][row] = String(b);
        std::snprintf(b, sizeof(b), "%.0f", r.x);        g->Cells[2][row] = String(b);
        std::snprintf(b, sizeof(b), "%.0f", r.y);        g->Cells[3][row] = String(b);
        std::snprintf(b, sizeof(b), "%.0f", r.z);        g->Cells[4][row] = String(b);
        std::snprintf(b, sizeof(b), "%+.3f", r.svClkUs); g->Cells[5][row] = String(b);
        std::snprintf(b, sizeof(b), "%.1f", r.elDeg);    g->Cells[6][row] = String(b);
    }

    TMemo* m = new TMemo(f); m->Parent = f; m->SetBounds(12, 380, 916, 218);
    m->Anchors = TAnchors() << akLeft << akRight << akBottom; m->ReadOnly = true;
    m->Font->Name = L"Consolas"; m->Font->Size = 9;
    m->Lines->BeginUpdate();
    char b[180];
    std::snprintf(b, sizeof(b), "FIX: Lat %.6f deg, Lon %.6f deg, Alt %.1f m   (ECEF %.1f, %.1f, %.1f m)",
                  FAdvFix.lat, FAdvFix.lon, FAdvFix.alt, FAdvFix.x, FAdvFix.y, FAdvFix.z);
    m->Lines->Add(String(b));
    std::snprintf(b, sizeof(b), "GDOP %.2f   iterations %d   residual RMS %.1f m   Rx clock bias %.1f m (%.3f us)",
                  FAdvFix.gdop, FAdvFix.iterations, FAdvFix.residRms, FAdvFix.clockBiasM, FAdvFix.clockBiasM / 299792458.0 * 1e6);
    m->Lines->Add(String(b));
    m->Lines->Add(L"");
    m->Lines->Add(L"Pseudorange = (code phase + bit/subframe TOW) -> common-TOW alignment + 68.802 ms nominal travel,");
    m->Lines->Add(L"x c. It is 'pseudo' because every range shares the receiver clock bias = the 4th unknown, so >= 4");
    m->Lines->Add(L"satellites are needed. Gauss-Newton least squares solves [x,y,z,clk] from Earth-centre in ~5-6 iters,");
    m->Lines->Add(L"with each SV's clock (af0/1/2 + relativistic - TGD) and the Sagnac earth-rotation correction applied.");
    m->Lines->EndUpdate();
    dpiShow(f);
}

//===========================================================================
// Shared helper: read the first 'maxSamples' int8 IF samples of the open
// capture (used by the signal-level inspectors that work without "Analyze").
//===========================================================================
bool TMainForm::advReadIF(std::vector<std::int8_t>& out, int maxSamples)
{
    out.clear();
    if (FFilePath.IsEmpty() || maxSamples <= 0) return false;
    std::ifstream f(AnsiString(FFilePath).c_str(), std::ios::binary);
    if (!f) return false;
    out.resize((std::size_t)maxSamples);
    f.read(reinterpret_cast<char*>(out.data()), (std::streamsize)maxSamples);
    out.resize((std::size_t)f.gcount());
    return !out.empty();
}

// Blue->cyan->green->yellow->red heat ramp for the correlation surface.
static void heatColor(float v, unsigned char& r, unsigned char& g, unsigned char& b)
{
    if (v < 0) v = 0; if (v > 1) v = 1;
    float x = v * 4.0f;
    if (x < 1.0f)      { r = 0;                       g = 0;                       b = (unsigned char)(128 + 127 * x); }
    else if (x < 2.0f) { r = 0;                       g = (unsigned char)(255 * (x - 1)); b = 255; }
    else if (x < 3.0f) { r = (unsigned char)(255 * (x - 2)); g = 255;             b = (unsigned char)(255 * (3 - x)); }
    else               { r = 255;                     g = (unsigned char)(255 * (4 - x)); b = 0; }
}

//===========================================================================
// Inspector: Acquisition Search Surface  (one PRN, straight from the capture)
//===========================================================================
void __fastcall TMainForm::advAcqClick(TObject* Sender)
{
    if (FFilePath.IsEmpty()) { Status(L"Open a capture first (File > Open)."); return; }
    TForm* f = makeInspector(L"Acquisition Search Surface", 984, 720);

    TLabel* l = new TLabel(f); l->Parent = f; l->SetBounds(12, 12, 36, 20); l->Caption = L"PRN:";
    FacqPrn = new TComboBox(f); FacqPrn->Parent = f; FacqPrn->SetBounds(50, 8, 90, 24); FacqPrn->Style = csDropDownList;
    int defIdx = -1, cnt = 0;
    for (int p = 1; p <= 32; ++p) {
        FacqPrn->Items->AddObject(L"PRN " + IntToStr(p), (TObject*)(NativeInt)p);
        if (defIdx < 0 && p <= 32 && FResults[p].found) defIdx = cnt;   // first acquired PRN
        ++cnt;
    }
    FacqPrn->ItemIndex = (defIdx >= 0) ? defIdx : 0;
    FacqPrn->OnChange = advAcqChange;

    FacqLbl = new TLabel(f); FacqLbl->Parent = f; FacqLbl->SetBounds(150, 12, 820, 20); FacqLbl->AutoSize = false;

    TLabel* hl = new TLabel(f); hl->Parent = f; hl->SetBounds(12, 38, 600, 16);
    hl->Caption = L"Correlation surface  (x = code phase 0..1023 chips,  y = Doppler high->low top->bottom,  bright = strong)";
    FacqImg = new TImage(f); FacqImg->Parent = f; FacqImg->SetBounds(12, 56, 600, 362);
    FacqImg->Stretch = true; FacqImg->Proportional = false;

    FacqDop = new TChart(f); FacqDop->Parent = f; FacqDop->SetBounds(624, 56, 348, 362);
    FacqDop->Anchors = TAnchors() << akLeft << akTop << akRight;
    FacqDop->View3D = false; FacqDop->Legend->Visible = false;
    FacqDop->Title->Text->Text = L"Doppler cut @ peak code phase";
    FacqDop->BottomAxis->Title->Caption = L"Doppler (Hz)";
    FacqDop->LeftAxis->Title->Caption = L"correlation (norm)";
    TFastLineSeries* ds = new TFastLineSeries(FacqDop); FacqDop->AddSeries(ds);

    FacqCode = new TChart(f); FacqCode->Parent = f; FacqCode->SetBounds(12, 430, 960, 172);
    FacqCode->Anchors = TAnchors() << akLeft << akTop << akRight;
    FacqCode->View3D = false; FacqCode->Legend->Visible = false;
    FacqCode->Title->Text->Text = L"Code-phase cut @ peak Doppler  (the thumbtack)";
    FacqCode->BottomAxis->Title->Caption = L"code phase (chips)";
    FacqCode->LeftAxis->Title->Caption = L"correlation (norm)";
    TFastLineSeries* csl = new TFastLineSeries(FacqCode); FacqCode->AddSeries(csl);

    TMemo* m = new TMemo(f); m->Parent = f; m->SetBounds(12, 610, 960, 72);
    m->Anchors = TAnchors() << akLeft << akRight << akBottom; m->ReadOnly = true;
    m->Lines->Add(L"Parallel-code-phase search: for every Doppler trial the IF carrier is wiped off and ONE FFT circularly "
                  L"correlates against the local C/A replica - testing all ~38192 code phases at once, swept over Doppler. "
                  L"A satellite in view shows a sharp 2-D peak (its Doppler + code phase); an absent PRN shows only a flat "
                  L"noise floor. Pick an acquired PRN, then an absent one, to see the difference.");
    advAcqChange(NULL);
    dpiShow(f);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::advAcqChange(TObject* Sender)
{
    if (!FacqImg || !FacqCode || !FacqDop) return;
    int prn = comboPrn(FacqPrn); if (prn < 1) prn = 1;

    gps::AcqConfig cfg = configForFile(FFilePath);
    const int n = (int)std::lround(cfg.fs * 1.0e-3);
    std::vector<std::int8_t> raw;
    if (!advReadIF(raw, n * cfg.numMs + 64) || (int)raw.size() < n * cfg.numMs) {
        FacqLbl->Caption = L"Could not read enough samples from the capture."; return;
    }

    Screen->Cursor = crHourGlass;
    gps::AcqSurface s;
    bool okSurf = true;
    try { s = gps::acquireSurface(prn, raw.data(), raw.size(), cfg, 512); }
    catch (...) { okSurf = false; }
    Screen->Cursor = crDefault;
    if (!okSurf || s.nBins <= 0 || s.nCols <= 0) { FacqLbl->Caption = L"Surface computation failed."; return; }

    // --- heatmap: rows = Doppler (high at top), cols = code phase ---
    std::unique_ptr<Graphics::TBitmap> bmp(new Graphics::TBitmap());
    bmp->PixelFormat = pf32bit;
    bmp->Width  = s.nCols;
    bmp->Height = s.nBins;
    for (int y = 0; y < s.nBins; ++y) {
        int srcBin = s.nBins - 1 - y;                 // flip so +Doppler is at top
        unsigned char* line = (unsigned char*)bmp->ScanLine[y];
        const float* row = &s.mag[(std::size_t)srcBin * s.nCols];
        for (int x = 0; x < s.nCols; ++x) {
            unsigned char r, g, b; heatColor(row[x], r, g, b);
            line[x * 4 + 0] = b; line[x * 4 + 1] = g; line[x * 4 + 2] = r; line[x * 4 + 3] = 255;
        }
    }
    FacqImg->Picture->Bitmap->Assign(bmp.get());

    TChartSeries* csl = FacqCode->Series[0]; csl->Clear();
    for (int c = 0; c < s.nCols; ++c) csl->AddXY(s.codeChips[c], s.codeSlice[c], L"", clNavy);
    TChartSeries* ds = FacqDop->Series[0]; ds->Clear();
    for (int b = 0; b < s.nBins; ++b) ds->AddXY(s.dopplerHz[b], s.dopplerSlice[b], L"", clNavy);

    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "PRN %d: peak Doppler %+.0f Hz, code phase %.1f chips, peak/2nd-peak ratio %.1f  ->  %s",
        prn, s.peakDopplerHz, s.peakCodeChip, s.peakRatio,
        (s.peakRatio >= cfg.threshold ? "ACQUIRED (clear thumbtack peak)"
                                      : "no clear peak (PRN likely not in view)"));
    FacqLbl->Caption = String(buf);
}

//===========================================================================
// Inspector: RF & Spectrum  (raw IF time series + Welch PSD, from the capture)
//===========================================================================
void __fastcall TMainForm::advRfClick(TObject* Sender)
{
    if (FFilePath.IsEmpty()) { Status(L"Open a capture first (File > Open)."); return; }
    gps::AcqConfig cfg = configForFile(FFilePath);

    const int L   = 4096;                 // Welch segment length (power of two)
    const int K   = 16;                   // segments
    const int hop = L / 2;                // 50% overlap
    const int need = (K - 1) * hop + L;   // samples required
    std::vector<std::int8_t> raw;
    if (!advReadIF(raw, need + 64) || (int)raw.size() < need) {
        Status(L"Capture too short for a spectrum."); return;
    }

    TForm* f = makeInspector(L"RF & Spectrum", 968, 688);

    TChart* ct = new TChart(f); ct->Parent = f; ct->SetBounds(12, 12, 944, 244);
    ct->Anchors = TAnchors() << akLeft << akTop << akRight;
    ct->View3D = false; ct->Legend->Visible = false;
    ct->Title->Text->Text = L"Raw IF samples (first 600) - looks like noise";
    ct->BottomAxis->Title->Caption = L"sample index";
    ct->LeftAxis->Title->Caption = L"int8 amplitude";
    TFastLineSeries* ts = new TFastLineSeries(ct); ct->AddSeries(ts);
    for (int i = 0; i < 600 && i < (int)raw.size(); ++i) ts->AddXY((double)i, (double)raw[i], L"", clNavy);
    FrfTime = ct;

    TChart* cp = new TChart(f); cp->Parent = f; cp->SetBounds(12, 264, 944, 318);
    cp->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;
    cp->View3D = false; cp->Legend->Visible = true; cp->Legend->Alignment = laBottom;
    cp->Title->Text->Text = L"Power spectral density (Welch, Hann window, 16 averages)";
    cp->BottomAxis->Title->Caption = L"frequency (MHz)";
    cp->LeftAxis->Title->Caption = L"power (dB)";
    TFastLineSeries* ps = new TFastLineSeries(cp); cp->AddSeries(ps); ps->Title = L"PSD";
    TLineSeries* ifm = new TLineSeries(cp); cp->AddSeries(ifm); ifm->Title = L"IF"; ifm->Color = clRed;
    FrfPsd = cp;

    // --- Welch PSD ---
    const double TWO_PI = 6.283185307179586;
    std::vector<double> win(L); double winPow = 0.0;
    for (int i = 0; i < L; ++i) { win[i] = 0.5 - 0.5 * std::cos(TWO_PI * i / (L - 1)); winPow += win[i] * win[i]; }
    double mean = 0.0; for (int i = 0; i < need; ++i) mean += raw[i]; mean /= need;

    std::vector<double> psd((std::size_t)(L / 2), 0.0);
    std::vector<dsp::cd> seg((std::size_t)L);
    for (int sgi = 0; sgi < K; ++sgi) {
        const int base = sgi * hop;
        for (int i = 0; i < L; ++i) seg[i] = dsp::cd(((double)raw[base + i] - mean) * win[i], 0.0);
        dsp::Fft::transform(seg, false);
        for (int k = 0; k < L / 2; ++k) psd[k] += std::norm(seg[k]);
    }
    double pmax = -1e300, pmin = 1e300;
    const double scale = 1.0 / ((double)K * (double)L * winPow + 1e-12);
    for (int k = 1; k < L / 2; ++k) {
        double db = 10.0 * std::log10(psd[k] * scale + 1e-12);
        double fMHz = (double)k * cfg.fs / L / 1.0e6;
        ps->AddXY(fMHz, db, L"", clNavy);
        if (db > pmax) pmax = db; if (db < pmin) pmin = db;
    }
    // vertical IF marker
    const double ifMHz = cfg.ifFreq / 1.0e6;
    ifm->AddXY(ifMHz, pmin, L"", clRed);
    ifm->AddXY(ifMHz, pmax, L"", clRed);

    TMemo* m = new TMemo(f); m->Parent = f; m->SetBounds(12, 590, 944, 90);
    m->Anchors = TAnchors() << akLeft << akRight << akBottom; m->ReadOnly = true;
    char b[200];
    std::snprintf(b, sizeof(b),
        "Sampled at fs = %.3f MHz; the C/A signal sits at IF = %.3f MHz (red line) - but ~16 dB BELOW the noise floor,",
        cfg.fs / 1.0e6, cfg.ifFreq / 1.0e6);
    m->Lines->Add(String(b));
    m->Lines->Add(L"so the spectrum is just band-limited noise with NO visible carrier. That is normal for GPS: the signal");
    m->Lines->Add(L"arrives weaker than the thermal noise. Only the ~30 dB processing gain from despreading (correlating");
    m->Lines->Add(L"against the 1.023 Mcps PRN code) lifts it out - exactly what the Acquisition Surface & C/A Code labs show.");
    dpiShow(f);
}

//===========================================================================
// Inspector: Signal Journey Overview  (teaching flow + glossary, no data)
//===========================================================================
void __fastcall TMainForm::advJourneyClick(TObject* Sender)
{
    TForm* f = makeInspector(L"Signal Journey - from sky to fix", 1000, 720);

    TLabel* hdr = new TLabel(f); hdr->Parent = f; hdr->SetBounds(12, 8, 970, 22);
    hdr->Font->Style = TFontStyles() << fsBold; hdr->Font->Size = 11;
    hdr->Caption = L"How a GPS fix is made - the journey of one satellite's signal (follow 1 -> 8)";

    auto box = [&](int x, int y, const String& title, const String& sub) {
        TPanel* p = new TPanel(f); p->Parent = f; p->SetBounds(x, y, 220, 74);
        p->BevelOuter = bvRaised; p->ParentBackground = false; p->Color = (TColor)0x00EAF2F2;
        TLabel* t = new TLabel(p); t->Parent = p; t->SetBounds(8, 6, 204, 16);
        t->Caption = title; t->Font->Style = TFontStyles() << fsBold; t->Transparent = true;
        t->ShowAccelChar = false;
        TLabel* su = new TLabel(p); su->Parent = p; su->SetBounds(8, 26, 204, 44);
        su->Caption = sub; su->WordWrap = true; su->AutoSize = false; su->Font->Size = 8; su->Transparent = true;
    };
    auto arrow = [&](int x, int y, const String& a) {
        TLabel* l = new TLabel(f); l->Parent = f; l->SetBounds(x, y, 26, 26);
        l->Caption = a; l->Font->Size = 16; l->Font->Style = TFontStyles() << fsBold; l->Font->Color = clNavy;
    };

    const int xs[4] = { 12, 252, 492, 732 };
    const int y1 = 40, y2 = 156;
    // Row 1 (left -> right): stages 1..4
    box(xs[0], y1, L"1. Antenna / RF front-end", L"L1 1575.42 MHz down-converted to IF 9.55 MHz, sampled 38.192 Msps as int8.");
    box(xs[1], y1, L"2. Acquisition",            L"FFT parallel-code-phase search -> which PRNs, coarse Doppler & code phase.");
    box(xs[2], y1, L"3. Tracking",               L"Costas PLL (carrier) + DLL (code) lock and follow each satellite over time.");
    box(xs[3], y1, L"4. Bit & Frame Sync",       L"20 ms -> 1 nav bit (50 bps); find the 0x8B preamble = subframe start.");
    arrow(xs[0] + 222, y1 + 24, L"→");
    arrow(xs[1] + 222, y1 + 24, L"→");
    arrow(xs[2] + 222, y1 + 24, L"→");
    arrow(xs[3] + 96,  y1 + 78, L"↓");      // down from stage 4 to stage 5
    // Row 2 (right -> left): stages 5..8 (snake)
    box(xs[3], y2, L"5. Nav Decode (LNAV)",      L"Parity-check words; read TLM/HOW, ephemeris (SF1-3) and almanac (SF4-5).");
    box(xs[2], y2, L"6. SV Position & Clock",    L"Kepler propagate ephemeris -> each satellite's ECEF position and clock.");
    box(xs[1], y2, L"7. Pseudoranges",           L"Code phase + decoded TOW -> range to each SV on a common receiver clock.");
    box(xs[0], y2, L"8. PVT Solve",              L"Least squares on >=4 ranges -> receiver X/Y/Z/clock -> Lat / Lon / Alt.");
    arrow(xs[2] + 222, y2 + 24, L"←");
    arrow(xs[1] + 222, y2 + 24, L"←");
    arrow(xs[0] + 222, y2 + 24, L"←");

    TMemo* g = new TMemo(f); g->Parent = f; g->SetBounds(12, 250, 970, 430);
    g->Anchors = TAnchors() << akLeft << akTop << akRight << akBottom;
    g->ReadOnly = true; g->ScrollBars = ssVertical; g->Font->Name = L"Segoe UI"; g->Font->Size = 9;
    g->Lines->BeginUpdate();
    g->Lines->Add(L"GLOSSARY - key ideas behind each stage (open the matching inspector to see it on the real capture):");
    g->Lines->Add(L"");
    g->Lines->Add(L"C/A code (Coarse/Acquisition): each satellite has a unique 1023-chip Gold code at 1.023 Mcps that");
    g->Lines->Add(L"   repeats every 1 ms. Its sharp autocorrelation is the key to CDMA and to ~30 dB of processing gain.");
    g->Lines->Add(L"Spreading / despreading: the 50 bps data is multiplied by the fast code (spread over ~2 MHz), burying it");
    g->Lines->Add(L"   under the noise. Correlating against a local replica (despreading) collapses it back and lifts it out.");
    g->Lines->Add(L"Doppler: satellite motion shifts the carrier by +/- ~5 kHz; acquisition searches Doppler x code phase.");
    g->Lines->Add(L"Acquisition surface: a 2-D correlation map; a satellite in view is a single sharp peak (the thumbtack).");
    g->Lines->Add(L"Tracking loops: the Costas PLL keeps the carrier wiped (insensitive to the 180-deg data flips); the DLL");
    g->Lines->Add(L"   keeps the local code aligned to within a fraction of a chip. Locked I/Q forms two BPSK clusters.");
    g->Lines->Add(L"C/N0: carrier-to-noise-density (dB-Hz); ~37-45 dB-Hz for a healthy GPS signal after despreading.");
    g->Lines->Add(L"Nav message (LNAV): 50 bps, 30-bit words, 10 words/subframe (6 s), 5 subframes/frame (30 s),");
    g->Lines->Add(L"   25 pages (12.5 min). Each word ends in 6 parity bits (IS-GPS-200 Hamming/XOR).");
    g->Lines->Add(L"TLM / HOW: word 1 carries the 0x8B preamble (Telemetry); word 2 (Hand-Over Word) carries the TOW count.");
    g->Lines->Add(L"Ephemeris (SF 1-3): precise Keplerian orbit + clock for THIS satellite, valid ~2 hours (IODE/IODC tag it).");
    g->Lines->Add(L"Almanac (SF 4-5): coarse, long-life orbit for the WHOLE constellation - used for fast cold-start aiding.");
    g->Lines->Add(L"Pseudorange: speed-of-light x signal travel time. 'Pseudo' because every range shares one unknown -");
    g->Lines->Add(L"   the receiver clock bias - so a 4th satellite is needed to solve it alongside X, Y, Z.");
    g->Lines->Add(L"PVT: Position-Velocity-Time. Gauss-Newton least squares on the pseudoranges, with SV clock and the");
    g->Lines->Add(L"   Sagnac (earth-rotation) correction applied, converges in ~5-6 iterations to an ECEF position -> WGS-84.");
    g->Lines->Add(L"GDOP: geometry dilution of precision - how satellite geometry amplifies range error into position error.");
    g->Lines->EndUpdate();
    dpiShow(f);
}
//---------------------------------------------------------------------------
void __fastcall TMainForm::Exit2Click(TObject *Sender)
{
	Close();
}
//---------------------------------------------------------------------------

