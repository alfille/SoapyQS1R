//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "qsform_control.h"

//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
Tqsform *qsform;
//---------------------------------------------------------------------------
__fastcall Tqsform::Tqsform(TComponent* Owner)
	: TForm(Owner)
{
	Settings = new TIniFile(ExtractFilePath(Application->ExeName) + "wr_qs1r.ini");

	allow_hide = Settings->ReadBool("Form", "AllowHide", 0);

	sample_rate = Settings->ReadInteger("DDC", "SampleRate", 1250000);
	old_rate = -1;
	clockcorrection = Settings->ReadInteger("Calibration", "ClockCorrection", 0);
	levelcorrection = Settings->ReadInteger("Calibration", "LevelCorrection", 9);
	need_freq_update = true;

	cbPGA->Checked = Settings->ReadBool("ADC", "PGA", true);
	cbRAND->Checked = Settings->ReadBool("ADC", "RAND", true);
	cbDITH->Checked = Settings->ReadBool("ADC", "DITH", false);

	if (cbPGA->Checked) {
		writeMultibusInt(MB_PGA_REG,0x1);
	} else
	{
		writeMultibusInt(MB_PGA_REG,0x0);
	}

	if (cbRAND->Checked) {
		writeMultibusInt(MB_RAND_REG,0x1);
	}  else
	{
		writeMultibusInt(MB_RAND_REG,0x0);
	}

	if (cbDITH->Checked) {
		writeMultibusInt(MB_DITH_REG,0x1);
	} else
	{
		writeMultibusInt(MB_DITH_REG,0x0);
	}

	switch (sample_rate) {
		case 2500000:  // BW:2000000
			cic1_deci = 5;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 0;
			break;
		case 1953125:  // BW: 1562500
			cic1_deci = 16;
			cic2_deci = 2;
			rgSampleRate->ItemIndex = 1;
			break;
		case 1562500: // BW: 1250000
			cic1_deci = 10;
			cic2_deci = 4;
			rgSampleRate->ItemIndex = 2;
			break;
		case 1250000:  // BW: 1000000
			cic1_deci = 10;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 3;
			break;
		case 625000:  // BW: 500000
			cic1_deci = 20;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 4;
			break;
		case 312500: // BW: 250000
			cic1_deci = 40;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 5;
			break;
		case 250000: // BW: 200000
			cic1_deci = 50;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 6;
			break;
		case 156250:  // BW: 125000
			cic1_deci = 80;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 7;
			break;
		case 125000: // BW 100000
			cic1_deci = 100;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 8;
			break;
		case 62500:  // BW: 50000
			cic1_deci = 200;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 9;
			break;
		case 50000:  // BW: 40000
			cic1_deci = 250;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 10;
			break;
		case 25000:  // BW: 20000
			cic1_deci = 500;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 11;
			break;
		case 12500:  // BW: 10000
			cic1_deci = 1000;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 12;
			break;
	default:
        	cic1_deci = 10;
			cic2_deci = 5;
			rgSampleRate->ItemIndex = 0;
			break;
		;
	}

	writeMultibusInt(MB_CIC1_DEC, cic1_deci);
	writeMultibusInt(MB_CIC2_DEC, cic2_deci);

	udClockCorrection->Position = clockcorrection;
	udLevelCal->Position = levelcorrection;
}
//---------------------------------------------------------------------------
void __fastcall Tqsform::cbPGAClick(TObject *Sender)
{
	if (cbPGA->Checked) {
		writeMultibusInt(MB_PGA_REG,0x1);
	}
	else
	{
		writeMultibusInt(MB_PGA_REG,0x0);
	}
	Settings->WriteBool("ADC", "PGA", cbPGA->Checked);
}
//---------------------------------------------------------------------------
void __fastcall Tqsform::cbRANDClick(TObject *Sender)
{
	if (cbRAND->Checked) {
		writeMultibusInt(MB_RAND_REG,0x1);
	}
	else
	{
		writeMultibusInt(MB_RAND_REG,0x0);
	}
	Settings->WriteBool("ADC", "RAND", cbRAND->Checked);
}
//---------------------------------------------------------------------------
void __fastcall Tqsform::cbDITHClick(TObject *Sender)
{
	if (cbDITH->Checked) {
		writeMultibusInt(MB_DITH_REG,0x1);
	}
	else
	{
        writeMultibusInt(MB_DITH_REG,0x0);
	}
	Settings->WriteBool("ADC", "DITH", cbDITH->Checked);
}
//---------------------------------------------------------------------------
void __fastcall Tqsform::rgSampleRateClick(TObject *Sender)
{
	switch (rgSampleRate->ItemIndex) {
		case 0:
			cic1_deci = 5;
			cic2_deci = 5;
			sample_rate = 2500000;
			break;
		case 1:
			cic1_deci = 16;
			cic2_deci = 2;
			sample_rate = 1953125;
			break;
		case 2:
			cic1_deci = 10;
			cic2_deci = 4;
			sample_rate = 1562500;
			break;
		case 3:
			cic1_deci = 10;
			cic2_deci = 5;
			sample_rate = 1250000;
			break;
		case 4:
			cic1_deci = 20;
			cic2_deci = 5;
			sample_rate = 625000;
			break;
		case 5:
			cic1_deci = 40;
			cic2_deci = 5;
			sample_rate = 312500;
			break;
		case 6:
			cic1_deci = 50;
			cic2_deci = 5;
			sample_rate = 250000;
			break;
		case 7:
			cic1_deci = 80;
			cic2_deci = 5;
			sample_rate = 156250;
			break;
		case 8:
			cic1_deci = 100;
			cic2_deci = 5;
			sample_rate = 125000;
			break;
		case 9:
			cic1_deci = 200;
			cic2_deci = 5;
			sample_rate = 62500;
			break;
		case 10:
			cic1_deci = 250;
			cic2_deci = 5;
			sample_rate = 50000;
			break;
		case 11:
			cic1_deci = 500;
			cic2_deci = 5;
			sample_rate = 25000;
			break;
		case 12:
			cic1_deci = 1000;
			cic2_deci = 5;
			sample_rate = 12500;
			break;
	default:
        	cic1_deci = 10;
			cic2_deci = 5;
			sample_rate = 1250000;
			rgSampleRate->ItemIndex = 0;
			break;
        ;
	}
	Settings->WriteInteger("DDC", "SampleRate", sample_rate);
}
//---------------------------------------------------------------------------

void __fastcall Tqsform::udClockCorrectionClick(TObject *Sender, TUDBtnType Button)
{
	clockcorrection = (long)udClockCorrection->Position;
	Settings->WriteInteger("Calibration", "ClockCorrection", clockcorrection);
	need_freq_update = true;
}
//---------------------------------------------------------------------------

void __fastcall Tqsform::udLevelCalClick(TObject *Sender, TUDBtnType Button)
{
	levelcorrection = (long)udLevelCal->Position;
	Settings->WriteInteger("Calibration", "LevelCorrection", levelcorrection);
}
//---------------------------------------------------------------------------

void __fastcall Tqsform::FormDestroy(TObject *Sender)
{
	Settings->WriteInteger("Form", "PosTop", this->Top);
	Settings->WriteInteger("Form", "PosLeft", this->Left);
	Settings->UpdateFile();
	delete Settings;
}
//---------------------------------------------------------------------------

void __fastcall Tqsform::FormCreate(TObject *Sender)
{
	this->Top = Settings->ReadInteger("Form", "PosTop", 540);
	this->Left = Settings->ReadInteger("Form", "PosLeft", 750);
	this->Update();
}
//---------------------------------------------------------------------------


