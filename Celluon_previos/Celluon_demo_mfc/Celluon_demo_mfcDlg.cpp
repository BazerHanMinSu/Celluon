
// Celluon_demo_mfcDlg.cpp : implementation file
//

#include "stdafx.h"
#include "Celluon_demo_mfc.h"
#include "Celluon_demo_mfcDlg.h"
#include "afxdialogex.h"
#include "UserLocationDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define KB_ENTER 13
#define KB_ESC 27


// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CCelluon_demo_mfcDlg dialog



CCelluon_demo_mfcDlg::CCelluon_demo_mfcDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CCelluon_demo_mfcDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CCelluon_demo_mfcDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CCelluon_demo_mfcDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_LOAD, &CCelluon_demo_mfcDlg::OnBnClickedBtnLoad)
	ON_BN_CLICKED(IDC_BTN_PLAY, &CCelluon_demo_mfcDlg::OnBnClickedBtnPlay)
	ON_BN_CLICKED(IDC_BTN_DEPTH, &CCelluon_demo_mfcDlg::OnBnClickedBtnDepth)
	ON_BN_CLICKED(IDC_BTN_RUN, &CCelluon_demo_mfcDlg::OnBnClickedBtnRun)
	ON_BN_CLICKED(IDC_BTN_USERLOCATION, &CCelluon_demo_mfcDlg::OnBnClickedBtnUserlocation)
END_MESSAGE_MAP()


// CCelluon_demo_mfcDlg message handlers

BOOL CCelluon_demo_mfcDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	Init();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CCelluon_demo_mfcDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CCelluon_demo_mfcDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CCelluon_demo_mfcDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CCelluon_demo_mfcDlg::Init()
{
	m_videoCapture;
	m_strFilePath = "";
	m_userLocation.x = USER_X;
	m_userLocation.y = USER_Y;
	m_userLocation.z = USER_Z;
}

void CCelluon_demo_mfcDlg::OnBnClickedBtnLoad()
{
	// TODO: Add your control notification handler code here

	if (m_videoCapture.isOpened())
	{
		m_videoCapture.release();
		//AfxMessageBox(L"비디오 정보가 이미 있습니다.");
	}

	CFileDialog dlg(true, NULL, NULL, NULL, NULL, NULL);

	if (dlg.DoModal() == IDOK)
	{
		m_strFilePath = dlg.GetPathName(); 
		// CString to string
		CT2CA pszConvertedAnsiString(m_strFilePath);
		// Construct a std::string using the LPCSTR input
		std::string s(pszConvertedAnsiString);

		m_videoCapture.open(s);
	}

	if (!m_videoCapture.isOpened())
	{
		AfxMessageBox(L"비디오 정보를 불러올 수 없습니다.");
		return;
	}
}


void CCelluon_demo_mfcDlg::OnBnClickedBtnPlay()
{
	// TODO: Add your control notification handler code here

	if (!m_videoCapture.isOpened())
	{
		AfxMessageBox(L"비디오를 재생할 수 없습니다.");
		return;
	}

	bool isFullScreen = false;

	cv::namedWindow("Window Mode: ENTER     EXIT: ESC", CV_WINDOW_NORMAL);
	cv::moveWindow("Window Mode: ENTER     EXIT: ESC", 0, 0);
	cv::setWindowProperty("Window Mode: ENTER     EXIT: ESC", CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);

	cv::Mat frame;
	cv::Mat dst;
	m_videoCapture >> frame;

	Point TopLeft = Point(m_BoundaryMathedPoint[TOPLEFT].x, m_BoundaryMathedPoint[TOPLEFT].y);
	Point TopRight = Point(m_BoundaryMathedPoint[TOPRIGHT].x, m_BoundaryMathedPoint[TOPRIGHT].y);
	Point BottomRight = Point(m_BoundaryMathedPoint[BOTTOMRIGHT].x, m_BoundaryMathedPoint[BOTTOMRIGHT].y);
	Point BottomLeft = Point(m_BoundaryMathedPoint[BOTTOMLEFT].x, m_BoundaryMathedPoint[BOTTOMLEFT].y);

	//Point TopLeft = Point(0, 0);
	//Point TopRight = Point(1280, 0);
	//Point BottomRight = Point(1280, 720);
	//Point BottomLeft = Point(0, 720);
	resize(frame, frame, Size(PIXEL_OF_SCREEN_X - 1, PIXEL_OF_SCREEN_Y - 1));
	m_warping.CalcTransformMatrix(frame, TopLeft, TopRight, BottomLeft, BottomRight);
	int val;
	while (1)
	{
		m_videoCapture >> frame;
		if (frame.empty())
		{
			break;
		}
		resize(frame, frame, Size(PIXEL_OF_SCREEN_X-1, PIXEL_OF_SCREEN_Y-1));
		m_warping.CalcWarping(frame, dst);
		cv::imshow("Window Mode: ENTER     EXIT: ESC", dst);
		//cv::imwrite("asdf.jpg", dst);
		//return;
		val = cv::waitKey(33);

		if (val == KB_ENTER)
		{
			if (isFullScreen)
				cv::setWindowProperty("Window Mode: ENTER     EXIT: ESC", CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);
			else
				cv::setWindowProperty("Window Mode: ENTER     EXIT: ESC", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			isFullScreen = !isFullScreen;
		}
		else if (val == KB_ESC)
			break;
	}
	cv::destroyWindow("Window Mode: ENTER     EXIT: ESC");
	m_videoCapture.release();
}


void CCelluon_demo_mfcDlg::OnBnClickedBtnDepth()
{
	// TODO: Add your control notification handler code here

	
	cv::Mat depth_img;
	int val;

	cv::namedWindow("Depth Image");

	while (1)
	{
		m_kinect.UpdateKinectDepthFrame();
		depth_img = m_kinect.GetKinectDepthFrame();

		cv::imshow("Depth Image", depth_img);
		val = waitKey(1);
		if (val == KB_ESC)
			break;
	}
	cv::destroyWindow("Depth Image");
	
}


void CCelluon_demo_mfcDlg::OnBnClickedBtnRun()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	Application app;
	app.SetUserLocation(m_userLocation.x, m_userLocation.y, m_userLocation.z);
	app.Run();
	for (int i = 0; i < 5; i++)
		m_BoundaryMathedPoint[i] = app.m_BoundaryMathedPoint[i];
	AfxMessageBox(L"Get m_BoundaryMathedPoint");
}


void CCelluon_demo_mfcDlg::OnBnClickedBtnUserlocation()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	UserLocationDlg userDlg;

	if (userDlg.DoModal() == IDOK)
	{
		CString x;
		x.Format(L"(%d, %d, %d)", userDlg.m_userX, userDlg.m_userY, userDlg.m_userZ);
		AfxMessageBox(x);
		m_userLocation.x = userDlg.m_userX;
		m_userLocation.y = userDlg.m_userY;
		m_userLocation.z = userDlg.m_userZ;
	}
	else
	{
		CString x;
		x.Format(L"(%d, %d, %d)", 0, 0, 0);
		AfxMessageBox(x);
		m_userLocation.x = 0;
		m_userLocation.y = 0;
		m_userLocation.z = 0;
	}
}
