#pragma once


// UserLocationDlg 대화 상자입니다.

class UserLocationDlg : public CDialogEx
{
	DECLARE_DYNAMIC(UserLocationDlg)

public:
	UserLocationDlg(CWnd* pParent = NULL);   // 표준 생성자입니다.
	virtual ~UserLocationDlg();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_DIALOG_USER_LOCATION };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

	DECLARE_MESSAGE_MAP()
public:
	int m_userX;
	int m_userY;
	int m_userZ;
};
