//---------------------------------------------------------------------------

#ifndef qsformH
#define qsformH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <inifiles.hpp>

//---------------------------------------------------------------------------

extern int MB_CONTROL1_BR;

class Tqsformmain : public TForm
{
__published:	// IDE-managed Components
	TComboBox *cbSampleRate;
	TLabel *Label1;
	TCheckBox *chkPGA;
	TCheckBox *chkRAND;
	TCheckBox *chkDITH;
	TLabel *Label2;
	TEdit *editClkCorrection;
	void __fastcall cbSampleRateChange(TObject *Sender);
	void __fastcall chkPGAClick(TObject *Sender);
	void __fastcall chkRANDClick(TObject *Sender);
	void __fastcall chkDITHClick(TObject *Sender);
	void __fastcall editClkCorrectionChange(TObject *Sender);
	void __fastcall FormCreate(TObject *Sender);
	void __fastcall FormDestroy(TObject *Sender);
private:	// User declarations
	TIniFile * settings;
public:		// User declarations
	__fastcall Tqsformmain(TComponent* Owner);
	long clockcorrection;
	long samplerate;
	long old_rate;
	long levelcorrection;
	bool need_freq_update;
	bool allow_hide;
};
//---------------------------------------------------------------------------
extern PACKAGE Tqsformmain *qsformmain;
//---------------------------------------------------------------------------
#endif
