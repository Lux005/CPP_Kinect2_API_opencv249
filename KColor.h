#pragma once
#include "Common.h"

class kColor
{
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;

public:
	kColor();
	~kColor();
	// Current Kinect
	IKinectSensor*          m_pKinectSensor;

	// Color reader 
	IColorFrameReader*      m_pColorFrameReader;
	RGBQUAD*				m_pColorRGBX;

	HRESULT                 Update(cv::Mat &cvframe);

	HRESULT                 InitializeDefaultSensor();

	//	void					ProcessColor(INT64 nTime, cv::Mat* pBuffer, int nWidth, int nHeight);
};
