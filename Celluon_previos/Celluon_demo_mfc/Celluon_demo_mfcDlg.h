
// Celluon_demo_mfcDlg.h : header file
//

#pragma once

#include "Application.h"

// CCelluon_demo_mfcDlg dialog
class CCelluon_demo_mfcDlg : public CDialogEx
{
// Construction
public:
	CCelluon_demo_mfcDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_CELLUON_DEMO_MFC_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support

// 추가 함수들
public:
	void Init();
// 추가 변수들
protected:
	cv::VideoCapture m_videoCapture;
	MyWarping m_warping;
	CString m_strFilePath;
	KinectController m_kinect;
	Point2f m_BoundaryMathedPoint[5];
	Point3d m_userLocation;

// Implementation
protected:
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnLoad();
	afx_msg void OnBnClickedBtnPlay();
	afx_msg void OnBnClickedBtnDepth();
	afx_msg void OnBnClickedBtnRun();
	afx_msg void OnBnClickedBtnUserlocation();
};
