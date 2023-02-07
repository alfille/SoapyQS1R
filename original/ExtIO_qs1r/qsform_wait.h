//---------------------------------------------------------------------------

#ifndef qsform_waitH
#define qsform_waitH
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <ExtCtrls.hpp>
//---------------------------------------------------------------------------
class TQSWaitForm : public TForm
{
__published:	// IDE-managed Components
	TPanel *pnlStatus;
	TLabel *lblWait;
private:	// User declarations
public:		// User declarations
	__fastcall TQSWaitForm(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TQSWaitForm *QSWaitForm;
//---------------------------------------------------------------------------
#endif
