
// mykinectTestDlg.h : 头文件
//

#pragma once

#include <opencv2/opencv.hpp>
#include "NuiApi.h"
#include <FaceTrackLib.h>
#include "cmscomm.h"
#include "RobotChControl.h"
#include <fstream> 
using namespace std;
using namespace cv;
// CmykinectTestDlg 对话框
class CmykinectTestDlg : public CDialogEx
{
// 构造
public:
	CRobotChControl m_RobotChControl;
	CmykinectTestDlg(CWnd* pParent = NULL);	// 标准构造函数
	void MatToCImage(Mat& mat, CImage& cImage);
	void DisplayImage(CWnd* m_pMyWnd,const CImage& image);
	bool Is_open;
	int m_Port;

	bool m_bRecord;
	ofstream m_os;
	
	int m_recordFrameNum;

	bool m_bSelectedSkeleton;//是否跟踪到骨架
	

	//显示人脸表情
	 IFTFaceTracker* pFT;
	 IFTResult* pFTResult;
	 IFTImage* pColorFrame; 
	 IFTImage* pDepthImage;
	 bool m_KinectSensorPresent;

	 float m_scale;
	 float m_rotationXYZ[3];
	 float m_translationXYZ[3];
	

	 FT_SENSOR_DATA sensorData;

	 FT_VECTOR3D m_NeckPoint[NUI_SKELETON_COUNT];
     FT_VECTOR3D m_HeadPoint[NUI_SKELETON_COUNT];
     bool        m_SkeletonTracked[NUI_SKELETON_COUNT];
     FLOAT       m_ZoomFactor;   // video frame zoom factor (it is 1.0f if there is no zoom)
     POINT       m_ViewOffset;   // Offset of the view from the top left corner.

    HRESULT     GetVideoConfiguration(FT_CAMERA_CONFIG* videoConfig);
    HRESULT     GetDepthConfiguration(FT_CAMERA_CONFIG* depthConfig);
	void SetCenterOfImage(IFTResult* pResult);
	float                       m_XCenterFace;
    float                       m_YCenterFace;
	   FT_VECTOR3D                 m_hint3D[2];
	 
	   void GetHeadData();
	bool m_LastTrackSucceeded;
	HRESULT GetClosestHint(FT_VECTOR3D* pHint3D);
	HRESULT GetClosestSkeleton(FT_VECTOR3D* pHint3D,int &selectedSkeleton);
	BOOL ShowImage(IFTImage* colorImage,CWnd* pMyWnd);
	CvFont font;
   
	 
    
	//保存每帧图像
    Mat colorImage;
	Mat depthImage;
	Mat skeletonImage;
	Mat mask;

	//定义三个事件句柄
	HANDLE colorEvent;
	HANDLE depthEvent;
	HANDLE skeletonEvent;
	
	//定义三个线程句柄
	HANDLE colorStreamHandle;
	HANDLE depthStreamHandle;
	
	  
   void getColorImage(HANDLE &colorEvent,HANDLE &colorStreamHandle, Mat &colorImage);
   void getDepthImage(HANDLE &depthEvent, HANDLE &depthStreamHandle, Mat &depthImage);
   void GetSkeletonData(HANDLE &skeletonEvent,Mat &skeletonImage, Mat &colorImage, Mat &depthImage);
   void drawSkeleton(Mat &image,CvPoint pointSet[],int witchone);



	HRESULT hr;
	//HRESULT 是一种简单的数据类型，通常被属性和 ATL 用作返回值。
	bool m_bStart;
	 HANDLE        m_hThNuiProcess;
	static DWORD WINAPI     Nui_ProcessThread( LPVOID pParam);
    DWORD WINAPI            Nui_ProcessThread( );
// 对话框数据
	enum { IDD = IDD_MYKINECTTEST_DIALOG };
	
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButOpenkinect();
	
	afx_msg void OnBnClickedButStopkinect();
	virtual BOOL DestroyWindow();

	CMSComm m_com;
	afx_msg void OnBnClickedOpenport();
	afx_msg void OnBnClickedCloseport();
	afx_msg void OnBnClickedButtonOnline();
	afx_msg void OnBnClickedButtonOffline();
	afx_msg void OnBnClickedButtonReset();
	DECLARE_EVENTSINK_MAP()
	void OnCommMscomm1();
	CString IDC_EDIT_RXDATA;
	CString m_controlAngle;
	//是否镜像
	bool m_bMirror;
	afx_msg void OnBnClickedMirro1();
	afx_msg void OnBnClickedBtnStartRecord();
	afx_msg void OnBnClickedBtnEndRecord();
	afx_msg void OnEnChangeEditControlangle();
	afx_msg void OnEnChangeEdit1();
};
