#pragma once


// UserLocationDlg ��ȭ �����Դϴ�.

class UserLocationDlg : public CDialogEx
{
	DECLARE_DYNAMIC(UserLocationDlg)

public:
	UserLocationDlg(CWnd* pParent = NULL);   // ǥ�� �������Դϴ�.
	virtual ~UserLocationDlg();

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_DIALOG_USER_LOCATION };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

	DECLARE_MESSAGE_MAP()
public:
	int m_userX;
	int m_userY;
	int m_userZ;
};
