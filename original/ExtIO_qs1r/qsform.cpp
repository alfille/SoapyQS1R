//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "qsform.h"
#include "qs_io_libusb.h"

//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
Tqsformmain *qsformmain;
//---------------------------------------------------------------------------
__fastcall Tqsformmain::Tqsformmain(TComponent* Owner)
	: TForm(Owner)
{   
	settings = new TIniFile( ExtractFilePath(Application->ExeName) + "wr_qs1r.ini" );

	clockcorrection = settings->ReadFloat( "Calibration", "ClkCorrection", -1410.0 );
	samplerate = settings->ReadInteger( "DDC", "SampleRate", 125000 );
	allow_hide = settings->ReadBool( "Form", "AllowHide", true );
	old_rate = 0;
	need_freq_update = true;

	cbSampleRate->Items->Add("25000");
	cbSampleRate->Items->Add("50000");
	cbSampleRate->Items->Add("125000");
	cbSampleRate->Items->Add("250000");
	cbSampleRate->Items->Add("500000");
	cbSampleRate->Items->Add("625000");
	cbSampleRate->Items->Add("1250000");
	cbSampleRate->Items->Add("1562500");
	cbSampleRate->Items->Add("2500000");

	chkPGA->Checked = settings->ReadBool( "DDC", "PGA", true );;
	chkRAND->Checked = settings->ReadBool( "DDC", "RAND", true );;
	chkDITH->Checked = settings->ReadBool( "DDC", "DITH", false );;

	int index = 0;

	switch ( samplerate ) {
	case 25000:
		index = 0;
		break;
	case 50000:
		index = 1;
		break;
	case 125000:
		index = 2;
		break;
	case 250000:
		index = 3;
		break;
	case 500000:
		index = 4;
		break;
	case 625000:
		index = 5;
		break;
	case 1250000:
		index = 6;
		break;
	case 1562500:
		index = 7;
		break;
	case 2500000:
		index = 8;
		break;
	default:
		;
	}

	cbSampleRate->ItemIndex = index;
	
	editClkCorrection->Text = clockcorrection;

	writeMultibusInt( MB_CONTRL1, MB_CONTROL1_BR );
}
//---------------------------------------------------------------------------
void __fastcall Tqsformmain::cbSampleRateChange(TObject *Sender)
{
	if ( cbSampleRate->ItemIndex == 0 ) {
        
	}
	switch ( cbSampleRate->ItemIndex) {
	case 0:
		samplerate = 25000;
		break;
	case 1:
		samplerate = 50000;
		break;
	case 2:
		samplerate = 125000;
		break;
	case 3:
		samplerate = 250000;
		break;
	case 4:
		samplerate = 500000;
		break;
	case 5:
		samplerate = 625000;
		break;
	case 6:
		samplerate = 1250000;
		break;
	case 7:
		samplerate = 1562500;
		break;
	case 8:
		samplerate = 2500000;
		break;
	default:
    	samplerate = 50000;
		;
	}
	settings->WriteInteger( "DDC", "SampleRate", samplerate ); 
}
//---------------------------------------------------------------------------

void __fastcall Tqsformmain::chkPGAClick(TObject *Sender)
{
	if ( chkPGA->Checked ) {
		MB_CONTROL1_BR |= PGA;
	}
	else
	{
		MB_CONTROL1_BR &= ~PGA;
	}
	writeMultibusInt( MB_CONTRL1, MB_CONTROL1_BR );
	settings->WriteBool( "DDC", "PGA", chkPGA->Checked ) ;
}
//---------------------------------------------------------------------------


void __fastcall Tqsformmain::chkRANDClick(TObject *Sender)
{
	if ( chkRAND->Checked ) {
		MB_CONTROL1_BR |= RANDOM;
	}
	else
	{
		MB_CONTROL1_BR &= ~RANDOM;
	}
	writeMultibusInt( MB_CONTRL1, MB_CONTROL1_BR );
	settings->WriteBool( "DDC", "RAND", chkRAND->Checked ) ;
}
//---------------------------------------------------------------------------

void __fastcall Tqsformmain::chkDITHClick(TObject *Sender)
{
	if ( chkDITH->Checked ) {
		MB_CONTROL1_BR |= DITHER;
	}
	else
	{
		MB_CONTROL1_BR &= ~DITHER;
	}
	writeMultibusInt( MB_CONTRL1, MB_CONTROL1_BR );
	settings->WriteBool( "DDC", "DITH", chkDITH->Checked ) ;
}
//---------------------------------------------------------------------------

void __fastcall Tqsformmain::editClkCorrectionChange(TObject *Sender)
{
	try
	{
		clockcorrection = editClkCorrection->Text.ToDouble();
		need_freq_update = true;
		settings->WriteFloat( "Calibration", "ClkCorrection", clockcorrection );
	}
	catch (...) { }
}
//---------------------------------------------------------------------------

void __fastcall Tqsformmain::FormCreate(TObject *Sender)
{
	this->Top = settings->ReadInteger( "Form", "PosTop", 0 );
	this->Left = settings->ReadInteger( "Form", "PosLeft", 0 );
	this->Update();
}
//---------------------------------------------------------------------------


void __fastcall Tqsformmain::FormDestroy(TObject *Sender)
{
	settings->WriteInteger( "Form", "PosTop", this->Top );
	settings->WriteInteger( "Form", "PosLeft", this->Left );
	settings->UpdateFile();	
}
//---------------------------------------------------------------------------

