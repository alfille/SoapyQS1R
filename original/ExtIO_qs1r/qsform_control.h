//---------------------------------------------------------------------------

#ifndef qsform_controlH
#define qsform_controlH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <ExtCtrls.hpp>
#include "qs_io_libusb.h"
#include <ComCtrls.hpp>
#include <inifiles.hpp>
//---------------------------------------------------------------------------

class Tqsform : public TForm
{
__published:	// IDE-managed Components
	TRadioGroup *rgSampleRate;
	TGroupBox *GroupBox1;
	TCheckBox *cbPGA;
	TCheckBox *cbRAND;
	TCheckBox *cbDITH;
	TGroupBox *GroupBox2;
	TEdit *editClockFreq;
	TUpDown *udLevelCal;
	TUpDown *udClockCorrection;
	TEdit *editLevelCal;
	TLabel *labelLevelCal;
	TLabel *Hz;
	TLabel *Label1;
	void __fastcall cbPGAClick(TObject *Sender);
	void __fastcall cbRANDClick(TObject *Sender);
	void __fastcall cbDITHClick(TObject *Sender);
	void __fastcall rgSampleRateClick(TObject *Sender);
	void __fastcall udClockCorrectionClick(TObject *Sender, TUDBtnType Button);
	void __fastcall udLevelCalClick(TObject *Sender, TUDBtnType Button);
	void __fastcall FormDestroy(TObject *Sender);
	void __fastcall FormCreate(TObject *Sender);
private:	// User declarations
	TIniFile *Settings;
public:		// User declarations
	__fastcall Tqsform(TComponent* Owner);
	int cic1_deci;
	int cic2_deci;
	int sample_rate;
	int old_rate;
	long clockcorrection;
	long levelcorrection;
	bool need_freq_update;
	bool allow_hide;
};
//---------------------------------------------------------------------------
extern PACKAGE Tqsform *qsform;
//---------------------------------------------------------------------------
#endif
