
// Celluon_demo_mfc.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CCelluon_demo_mfcApp:
// See Celluon_demo_mfc.cpp for the implementation of this class
//

class CCelluon_demo_mfcApp : public CWinApp
{
public:
	CCelluon_demo_mfcApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CCelluon_demo_mfcApp theApp;