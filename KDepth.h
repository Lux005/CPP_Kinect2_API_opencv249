#pragma once
#include "Common.h"

class kDepth
{
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

public:
	/// <summary>
	/// Constructor
	/// </summary>
	kDepth();

	~kDepth();
	// Current Kinect
	IKinectSensor*          m_pKinectSensor;

	// Depth reader
	IDepthFrameReader*      m_pDepthFrameReader;

	RGBQUAD*                m_pDepthRGBX;

	void                    Update(cv::Mat &cvframe);

	HRESULT                 InitializeDefaultSensor();

};

