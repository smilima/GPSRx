//---------------------------------------------------------------------------

#ifndef MainUnitH
#define MainUnitH
//---------------------------------------------------------------------------
#include <System.Classes.hpp>
#include <Vcl.Controls.hpp>
#include <Vcl.StdCtrls.hpp>
#include <Vcl.Forms.hpp>
#include <Vcl.ExtCtrls.hpp>
#include <Vcl.ComCtrls.hpp>
#include <Vcl.Grids.hpp>
#include <Vcl.Dialogs.hpp>
#include <Vcl.Menus.hpp>
#include <VCLTee.Chart.hpp>
#include <VCLTee.Series.hpp>
#include <VCLTee.TeEngine.hpp>
#include <VCLTee.TeeProcs.hpp>
#include <vector>
#include "Acquisition.h"
#include "Tracking.h"
#include "NavMessage.h"
//---------------------------------------------------------------------------
// One tracked satellite's results for the Tracking-tab display.
struct GuiTrackedChannel {
	int    prn = 0;
	double finalDoppler = 0;
	double avgCn0 = 0;
	bool   locked = false;
	std::vector<gps::TrackEpoch> epochs;
};
//---------------------------------------------------------------------------
// One satellite's contribution to the position fix (for the sat table).
struct GuiSatRow {
	int    prn = 0;
	double prKm = 0;            // pseudorange (km)
	double x = 0, y = 0, z = 0; // SV ECEF position (km)
	double svClkUs = 0;         // SV clock correction (us)
	double azDeg = 0, elDeg = 0;// look angle from the receiver (deg), for the sky plot
	bool   elValid = false;     // above the horizon -> plotted
};
// The least-squares position fix, carried from the worker to the UI.
struct GuiFix {
	bool   ok = false;
	int    nSats = 0;
	int    bestTow = 0;
	double x = 0, y = 0, z = 0;      // receiver ECEF (m)
	double lat = 0, lon = 0, alt = 0;// WGS-84 (deg, deg, m)
	double clockBiasM = 0;           // receiver clock bias (m)
	double gdop = 0;
	int    iterations = 0;
	double residRms = 0;             // post-fit residual RMS (m)
	double radiusKm = 0;             // ECEF radius (km)
	std::vector<GuiSatRow> rows;
};
//---------------------------------------------------------------------------
// Per-PRN progress pushed from a parallel PVT worker as each satellite finishes
// (so the Position table can fill in live, before the fix is solved).
struct PosSatProgress {
	int    prn = 0;
	double cn0 = 0;       // dB-Hz
	double doppler = 0;   // Hz
	int    state = 0;     // 0 no-data, 1 no-sync, 2 eph-incomplete, 3 eph-OK
};
//---------------------------------------------------------------------------
// Per-PRN data captured by the Advanced "Analyze" pass, for the inspector popups.
struct AdvPrn {
	int       prn = 0;
	bool      valid = false;
	long long codePhaseSamp = 0;
	int       bitOffset = 0;
	double    doppler = 0;
	double    cn0 = 0;
	std::vector<gps::TrackEpoch> epochs;   // full 36 s tracking telemetry
	std::vector<int>             bits;      // demodulated 50 bps stream
	gps::NavDecode               nav;       // ephemeris + subframe refs
	gps::NavDetail               detail;    // per-word/parity breakdown
};
//---------------------------------------------------------------------------
class TMainForm : public TForm
{
__published:	// IDE-managed Components
	TEdit *editFile;
	TMemo *memoResults;
	TPanel *Panel1;
	TMainMenu *MainMenu1;
	TMenuItem *File1;
	TMenuItem *File2;
	TMenuItem *Close1;
	TMenuItem *Close2;
	TMenuItem *Exit1;
	TMenuItem *Exit2;
	TOpenDialog *OpenDialog1;
	TSplitter *Splitter1;
	TLabel *lblFile;
	TPageControl *PageControl1;
	TTabSheet *tsAcquisition;
	TTabSheet *tsTracking;
	TTabSheet *tsPosition;
	TButton *btnAcquire;
	TChart *Chart1;
	TBarSeries *Series1;
	TButton *btnTrack;
	TStringGrid *sgChannels;
	TButton *btnFix;
	TStringGrid *sgSats;
	TMemo *memoFix;
	void __fastcall btnAcquireClick(TObject *Sender);
	void __fastcall File2Click(TObject *Sender);
	void __fastcall btnTrackClick(TObject *Sender);
	void __fastcall sgChannelsSelectCell(TObject *Sender, System::LongInt ACol,
	                                     System::LongInt ARow, bool &CanSelect);
	void __fastcall btnFixClick(TObject *Sender);
private:	// User declarations
	String         FFilePath;        // full path of the loaded capture (name-only shown in editFile)
	bool           FHaveResults;     // true once a sky search has populated the chart
	gps::AcqResult FResults[33];     // per-PRN acquisition results, index = PRN (1..32), for hover lookup
	void __fastcall Chart1MouseMove(TObject *Sender, TShiftState Shift, int X, int Y);

	// --- Tracking tab ---
	std::vector<GuiTrackedChannel> FTracked;
	bool             FTrackChartsBuilt;
	TChart          *FchIQ;          // I/Q constellation
	TChart          *FchPromptI;     // prompt-I (nav bits) over time
	TChart          *FchTrend;       // Doppler + C/N0 trends
	TChart          *FchSky;         // sky plot (az/el), custom-drawn in FchSkyAfterDraw
	TPointSeries    *FiqSeries;
	TFastLineSeries *FpromptSeries;
	TFastLineSeries *FdopSeries;
	TFastLineSeries *Fcn0Series;
	std::vector<GuiSatRow> FSkyRows; // satellites + look angles from the latest fix
	int              FPosRow[33];    // Position-table row for each PRN (0 = not shown yet)
	int              FPosCount;      // data rows currently in the Position table
	void buildTrackingCharts();
	void buildPositionSky();         // create the Position-tab layout + sky plot (once)
	void plotChannel(int idx);
	void __fastcall FchSkyAfterDraw(TObject *Sender);

	// --- DAC streaming + settings ---
	double           FDacSampleRate; // Hz  (DAC / stream sample rate -> fs)
	double           FIfHz;          // Hz  (intermediate frequency -> ifFreq)
	TButton*         FbtnStream;     // runtime "Stream from DAC" button on Panel1
	TThread*         FStream;        // running continuous-receiver worker (NULL when idle)
	TThread*         FDeadStream;    // finished stream worker awaiting deletion (not FreeOnTerminate)
	void loadSettings();
	void saveSettings();
	void __fastcall settingsClick(TObject* Sender);
	void __fastcall btnStreamClick(TObject* Sender);
	void __fastcall streamDone(TObject* Sender);   // stream worker OnTerminate handler

	// --- Advanced inspection suite ---
	TCheckBox*          FchkAdvanced;
	TTabSheet*          tsAdvanced;
	TButton*            FbtnAnalyze;
	TLabel*             FlblAdv;
	TThread*            FAdvThread;     // running Analyze worker (NULL when idle)
	TThread*            FAdvDead;       // finished Analyze worker awaiting deletion
	std::vector<AdvPrn> FAdv;           // per analyzed PRN
	gps::AlmanacSet     FAdvAlmanac;
	GuiFix              FAdvFix;
	bool                FAdvReady;
	std::vector<TButton*> FAdvButtons;  // inspector launch buttons (enabled after Analyze)
	// interactive inspector controls (one popup of each kind at a time)
	TComboBox*  FcodePrnA;  TComboBox* FcodePrnB;  TChart* FcodeChart;  TChart* FcorrChart;  TLabel* FcodeLbl;
	TComboBox*  FnavPrn;     TStringGrid* FnavSubs; TStringGrid* FnavWords; TMemo* FnavDump;
	TComboBox*  FephPrn;     TStringGrid* FephGrid; TMemo* FephMemo;
	TComboBox*  FtrkPrn;     TChart* FtrkIQ; TChart* FtrkDisc; TChart* FtrkObs; TLabel* FtrkLbl;
	TComboBox*  FacqPrn;     TImage* FacqImg; TChart* FacqCode; TChart* FacqDop; TLabel* FacqLbl;
	TChart*     FrfTime;     TChart* FrfPsd;  TLabel* FrfLbl;

	void __fastcall advCheckClick(TObject* Sender);
	void __fastcall advAnalyzeClick(TObject* Sender);
	void __fastcall advDone(TObject* Sender);          // Analyze worker OnTerminate
	void __fastcall inspectorClose(TObject* Sender, TCloseAction& Action);
	TForm* makeInspector(const String& title, int w, int h);
	void   dpiShow(TForm* f);   // DPI-scale a runtime popup's subtree, then Show()
	const AdvPrn* advFind(int prn) const;
	void advFillPrnCombo(TComboBox* cb, bool ephemerisOnly);
	void __fastcall advCodeClick(TObject* Sender);     // C/A code & correlation
	void __fastcall advNavClick(TObject* Sender);      // nav frame & bits
	void __fastcall advEphClick(TObject* Sender);      // ephemeris decoder
	void __fastcall advAlmClick(TObject* Sender);      // almanac & SF4/5
	void __fastcall advTrackClick(TObject* Sender);    // tracking loops lab
	void __fastcall advPvtClick(TObject* Sender);      // PVT solver lab
	void __fastcall advAcqClick(TObject* Sender);      // acquisition search surface
	void __fastcall advRfClick(TObject* Sender);       // RF & spectrum
	void __fastcall advJourneyClick(TObject* Sender);  // signal journey overview
	void __fastcall advCodeChange(TObject* Sender);
	void __fastcall advNavChange(TObject* Sender);
	void __fastcall advEphChange(TObject* Sender);
	void __fastcall advTrackChange(TObject* Sender);
	void __fastcall advAcqChange(TObject* Sender);
	bool advReadIF(std::vector<std::int8_t>& out, int maxSamples); // first samples of the capture
public:		// User declarations
	__fastcall TMainForm(TComponent* Owner);
	__fastcall ~TMainForm();
	// Called from worker threads via Synchronize (main thread):
	void Status(const String& s);                                   // append one status line to the memo
	void beginAcquisition();                                        // clear the chart for a new sky search
	void addAcqResult(const gps::AcqResult& r);                     // add one PRN's bar as it is acquired (real time)
	void addTrackedChannel(const GuiTrackedChannel& gc);            // add a tracked sat row + plot if first
	void applyFix(const GuiFix& fix);                               // fill the sat table + fix summary
	void addPosSat(const PosSatProgress& p);                        // add/update one PRN row live during the fix
	void resetPositionTable();                                      // clear + relabel the Position table
	void streamStopped();                                           // re-enable the UI when streaming ends
	void showAcqResults(const std::vector<gps::AcqResult>& results);// fill the acq bar chart from a result set
	void resetTracking();                                           // clear the Tracking grid + cached channels
};
//---------------------------------------------------------------------------
extern PACKAGE TMainForm *MainForm;
//---------------------------------------------------------------------------
#endif
