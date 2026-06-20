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
	TPointSeries    *FiqSeries;
	TFastLineSeries *FpromptSeries;
	TFastLineSeries *FdopSeries;
	TFastLineSeries *Fcn0Series;
	void buildTrackingCharts();
	void plotChannel(int idx);
public:		// User declarations
	__fastcall TMainForm(TComponent* Owner);
	// Called from worker threads via Synchronize (main thread):
	void Status(const String& s);                                   // append one status line to the memo
	void ApplyResults(const std::vector<gps::AcqResult>& results);  // fill the bar chart + cache results
	void addTrackedChannel(const GuiTrackedChannel& gc);            // add a tracked sat row + plot if first
	void applyFix(const GuiFix& fix);                               // fill the sat table + fix summary
};
//---------------------------------------------------------------------------
extern PACKAGE TMainForm *MainForm;
//---------------------------------------------------------------------------
#endif
