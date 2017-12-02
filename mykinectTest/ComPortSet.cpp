// ComPortSet.cpp : implementation file
//

#include "stdafx.h"
#include "mykinectTest.h"
#include "ComPortSet.h"
#include "afxdialogex.h"


// CComPortSet dialog

IMPLEMENT_DYNAMIC(CComPortSet, CDialogEx)

CComPortSet::CComPortSet(CWnd* pParent /*=NULL*/)
	: CDialogEx(CComPortSet::IDD, pParent)
{
	  selectComPortID=4;
}

CComPortSet::~CComPortSet()
{
}

void CComPortSet::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO1, m_comselect);
	DDX_Control(pDX, IDC_COMBO2, m_combrad);
}


BEGIN_MESSAGE_MAP(CComPortSet, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON_OK, &CComPortSet::OnBnClickedButtonOk)
	ON_BN_CLICKED(IDC_BUTTON_CANCEL, &CComPortSet::OnBnClickedButtonCancel)
END_MESSAGE_MAP()


// CComPortSet message handlers


void CComPortSet::OnBnClickedButtonOk()
{
	// TODO: Add your control notification handler code here
	UpdateData();
	selectComPortID=m_comselect.GetCurSel()+1;
	return CDialogEx::OnOK();
}


void CComPortSet::OnBnClickedButtonCancel()
{
	// TODO: Add your control notification handler code here
	return CDialogEx::OnCancel();
}


BOOL CComPortSet::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// TODO:  Add extra initialization here
	m_comselect.SetCurSel(selectComPortID-1);
	m_combrad.SetCurSel(0);
	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}
