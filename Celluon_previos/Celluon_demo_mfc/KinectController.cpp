#include "stdafx.h"
#include "KinectController.h"

BOOL bThreadLoop = TRUE;
BOOL bNormalize = FALSE;

template<class T>
void SafeKinectInterfaceRelease(T*& pInterface)
{
	if (pInterface != nullptr)
	{
		pInterface->Release();
		pInterface = nullptr;
	}
}

template<class T1, class T2>
void AcquireKinectFrame(T1*& pReader, T2*& pFrame)
{
	HRESULT hr = E_PENDING;
	T2* pTempFrame = nullptr;

	while (bThreadLoop && hr == E_PENDING)
	{
		hr = pReader->AcquireLatestFrame(&pTempFrame);
	}

	if (SUCCEEDED(hr))
	{
		pFrame = pTempFrame;
	}
	else
	{
		pFrame = nullptr;
	}
}

KinectController::KinectController()
{
	m_isInitSucessful = Init();
	if (FAILED(m_isInitSucessful)) { printf("%ld\n", m_isInitSucessful);	return; }
	else printf("success : Init\n");
}

KinectController::~KinectController()
{
	Release();
}

HRESULT KinectController::Init()
{
	HRESULT hr;
	m_pKinectSensor = nullptr;
	m_pColorFrameReader = nullptr;
	m_pDepthFrameReader = nullptr;
	m_pColorFrameSource = nullptr;
	m_pDepthFrameSource = nullptr;
	m_pCoordinateMapper = nullptr;

	// Sensor init
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr)) { printf("%ld\n", hr);	return 1; }
	else printf("success : GetDefaultKinectSensor\n");

	hr = m_pKinectSensor->Open();
	if (FAILED(hr)) { printf("%ld\n", hr);	return 1; }
	else printf("success : m_pKinectSensor->Open\n");

	// Color and depth frame init
	hr = m_pKinectSensor->get_ColorFrameSource(&m_pColorFrameSource);
	if (FAILED(hr)) { printf("%ld\n", hr);	return 1; }
	else printf("success : m_pKinectSensor->get_ColorFrameSource\n");

	hr = m_pKinectSensor->get_DepthFrameSource(&m_pDepthFrameSource);
	if (FAILED(hr)) { printf("%ld\n", hr);	return 1; }
	else printf("success : m_pKinectSensor->get_DepthFrameSource\n");

	if (m_pColorFrameSource != nullptr)
	{
		hr = m_pColorFrameSource->OpenReader(&m_pColorFrameReader);
		if (FAILED(hr)) { printf("%ld\n", hr);	return 1; }
		else printf("success : m_pColorFrameSource->OpenReader\n");

		SafeKinectInterfaceRelease(m_pColorFrameSource);
	}

	if (m_pDepthFrameSource != nullptr)
	{
		hr = m_pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		if (FAILED(hr)) { printf("%ld\n", hr);	return 1; }
		else printf("success : m_pDepthFrameSource->OpenReader\n");

		SafeKinectInterfaceRelease(m_pDepthFrameSource);
	}

	hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
	if (FAILED(hr)) { printf("%ld\n", hr);	return 1; }
	else printf("success : m_pKinectSensor->get_CoordinateMapper\n");

	m_matKinectColor = new Mat;
	m_matKinectDepth = new Mat;
	m_matKinectColorDepth = new Mat;

	return hr;
}

void KinectController::Release()
{
	SafeKinectInterfaceRelease(m_pColorFrameReader);
	SafeKinectInterfaceRelease(m_pDepthFrameReader);
	SafeKinectInterfaceRelease(m_pKinectSensor);

	if (m_matKinectColor != nullptr)
	{
		delete m_matKinectColor;
		m_matKinectColor = nullptr;
	}

	if (m_matKinectDepth != nullptr)
	{
		delete m_matKinectDepth;
		m_matKinectDepth = nullptr;
	}

	delete m_matKinectColor;
	delete m_matKinectDepth;
	delete m_matKinectColorDepth;
}

void KinectController::UpdateKinectColorFrame()
{
	HRESULT hr;
	IColorFrame* pColorFrame = nullptr;
	IFrameDescription *pFrameDescription = nullptr;
	INT height;
	INT width;
	ColorImageFormat imageFormat = ColorImageFormat_None;
	UINT sizeBuffer = 0;
	BYTE* buffer = nullptr;


	// Init frame
	AcquireKinectFrame(m_pColorFrameReader, pColorFrame);

	if (pColorFrame != nullptr)
	{
		pColorFrame->get_FrameDescription(&pFrameDescription);
		pColorFrame->get_RawColorImageFormat(&imageFormat);

		pFrameDescription->get_Height(&height);
		pFrameDescription->get_Width(&width);

		*m_matKinectColor = Mat::zeros(height, width, CV_8UC4);

		if (pColorFrame != nullptr)
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				pColorFrame->AccessRawUnderlyingBuffer(&sizeBuffer, &buffer);
				memcpy(m_matKinectColor->data, buffer,
					m_matKinectColor->dataend - m_matKinectColor->datastart);
			}
			else
				pColorFrame->CopyConvertedFrameDataToArray(
				m_matKinectColor->dataend - m_matKinectColor->datastart,
				m_matKinectColor->data, ColorImageFormat_Bgra);
		}

	}

	SafeKinectInterfaceRelease(pFrameDescription);
	SafeKinectInterfaceRelease(pColorFrame);
}

void KinectController::UpdateKinectDepthFrame()
{
	HRESULT hr;
	IDepthFrame* pDepthFrame = nullptr;
	IFrameDescription *pFrameDescription = nullptr;
	INT height;
	INT width;
	USHORT distanceDepthMin;
	USHORT distanceDepthMax;
	UINT sizeBuffer = 0;
	UINT16* buffer = nullptr;


	// Init frame
	AcquireKinectFrame(m_pDepthFrameReader, pDepthFrame);

	if (pDepthFrame != nullptr)
	{
		pDepthFrame->get_FrameDescription(&pFrameDescription);
		//pDepthFrame->get_DepthMinReliableDistance(&distanceDepthMin);
		pDepthFrame->get_DepthMaxReliableDistance(&distanceDepthMax);

		pFrameDescription->get_Height(&height);
		pFrameDescription->get_Width(&width);

		*m_matKinectDepth = Mat::zeros(height, width, CV_16UC1);


		if (pDepthFrame != nullptr)
		{
			pDepthFrame->AccessUnderlyingBuffer(&sizeBuffer, &buffer);
			memcpy(m_matKinectDepth->data, buffer, m_matKinectDepth->dataend - m_matKinectDepth->datastart);

			//ushort dMax, dMin;
			//pDepthFrame->get_DepthMaxReliableDistance(&dMax); // 4500
			//pDepthFrame->get_DepthMinReliableDistance(&dMin); // 500
			//double scale = 65535. / (dMax - dMin);

			//m_matKinectDepth->convertTo(*m_matKinectDepth, CV_16UC1, scale);
		}
	}

	SafeKinectInterfaceRelease(pFrameDescription);
	SafeKinectInterfaceRelease(pDepthFrame);
}

Mat KinectController::GetKinectColorFrame()
{
	return m_matKinectColor->clone();
}
Mat KinectController::GetKinectDepthFrame()
{
	//ushort dMax, dMin;
	//pDepthFrame->get_DepthMaxReliableDistance(&dMax); // 4500
	//pDepthFrame->get_DepthMinReliableDistance(&dMin); // 500

	Mat normalizeImage;
	double scale = 65535. / (4500 - 500);

	m_matKinectDepth->convertTo(normalizeImage, CV_16UC1, scale);

	return normalizeImage;
}

Mat KinectController::GetKinectOriginalDepthFrame()
{
	return m_matKinectDepth->clone();
}

Mat KinectController::GetKinectColorDepthFrame()
{
	HRESULT hr;

	IColorFrame* pColorFrame = nullptr;
	IFrameDescription *pFrameDescription = nullptr;


	IDepthFrame* pDepthFrame = nullptr;
	INT color_height;
	INT color_width;
	INT depth_height;
	INT depth_width;

	// Init frame
	AcquireKinectFrame(m_pColorFrameReader, pColorFrame);
	pColorFrame->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Height(&color_height);
	pFrameDescription->get_Width(&color_width);

	SafeKinectInterfaceRelease(pFrameDescription);

	// Init frame
	AcquireKinectFrame(m_pDepthFrameReader, pDepthFrame);
	pDepthFrame->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Height(&depth_height);
	pFrameDescription->get_Width(&depth_width);

	DepthSpacePoint* pDepthCoordinate = new DepthSpacePoint[color_width*color_height];

	hr = m_pCoordinateMapper->MapColorFrameToDepthSpace(depth_width*depth_height,
		(UINT16*)m_matKinectDepth->data,
		color_width*color_height, pDepthCoordinate);

	if (SUCCEEDED(hr))
	{
		int max = 0;
		m_matKinectColor->copyTo(*m_matKinectColorDepth);
		for (int i = 0; i < color_width*color_height; i++)
		{
			DepthSpacePoint dsp = pDepthCoordinate[i];
			int depthX = (int)floor(dsp.X + 0.5);
			int depthY = (int)floor(dsp.Y + 0.5);
			//printf("%d , %d\n", depthX, depthY);

			if (depthX >= 0 && depthX < depth_width&&depthY >= 0 && depthY < depth_height)
			{
				ushort* depthSrc = (ushort*)m_matKinectDepth->data;
				ushort val = depthSrc[depthY*depth_width + depthX];
				if (val < 1500)
				{
				}
				else
				{
					m_matKinectColorDepth->data[i * 4 + 0] = 0;
					m_matKinectColorDepth->data[i * 4 + 1] = 0;
					m_matKinectColorDepth->data[i * 4 + 2] = 0;
					m_matKinectColorDepth->data[i * 4 + 3] = 0;
				}
			}
			else
			{
				m_matKinectColorDepth->data[i * 4 + 0] = 0;
				m_matKinectColorDepth->data[i * 4 + 1] = 0;
				m_matKinectColorDepth->data[i * 4 + 2] = 0;
				m_matKinectColorDepth->data[i * 4 + 3] = 0;
			}
		}
		printf("max : %d\n", max);
		//	waitKey();
	}

	SafeKinectInterfaceRelease(pColorFrame);
	SafeKinectInterfaceRelease(pDepthFrame);
	SafeKinectInterfaceRelease(pFrameDescription);
	delete[] pDepthCoordinate;

	imshow("m_matKinectColorDepth", *m_matKinectColorDepth);
	waitKey(1);

	return m_matKinectColorDepth->clone();
}