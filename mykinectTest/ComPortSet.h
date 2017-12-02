#pragma once
#include "afxwin.h"


// CComPortSet dialog

class CComPortSet : public CDialogEx
{
	DECLARE_DYNAMIC(CComPortSet)

public:
	CComPortSet(CWnd* pParent = NULL);   // standard constructor
	virtual ~CComPortSet();
	
	int selectComPortID;
// Dialog Data
	enum { IDD = IDD_DIALOG_COMSET };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonOk();
	CComboBox m_comselect;
	CComboBox m_combrad;
	afx_msg void OnBnClickedButtonCancel();
	virtual BOOL OnInitDialog();
};
