// UserLocationDlg.cpp : ���� �����Դϴ�.
//

#include "stdafx.h"
#include "Celluon_demo_mfc.h"
#include "UserLocationDlg.h"
#include "afxdialogex.h"


// UserLocationDlg ��ȭ �����Դϴ�.

IMPLEMENT_DYNAMIC(UserLocationDlg, CDialogEx)

UserLocationDlg::UserLocationDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(UserLocationDlg::IDD, pParent)
	, m_userX(0)
	, m_userY(0)
	, m_userZ(0)
{

}

UserLocationDlg::~UserLocationDlg()
{
}

void UserLocationDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT1, m_userX);
	DDX_Text(pDX, IDC_EDIT2, m_userY);
	DDX_Text(pDX, IDC_EDIT3, m_userZ);
}


BEGIN_MESSAGE_MAP(UserLocationDlg, CDialogEx)
END_MESSAGE_MAP()


// UserLocationDlg �޽��� ó�����Դϴ�.
