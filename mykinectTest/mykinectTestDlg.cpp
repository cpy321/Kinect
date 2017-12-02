
// mykinectTestDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "mykinectTest.h"
#include "mykinectTestDlg.h"
#include "afxdialogex.h"
#include "ComPortSet.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#define PI 3.1415
using namespace cv;

bool tracked[NUI_SKELETON_COUNT]={FALSE};   
CvPoint skeletonPoint[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT]={cvPoint(0,0)};
//OpenCV的基本数据类型之一，表示一个坐标为整数的二维点，是一个包含integer类型成员x和y的简单结构体
//SkeletonPoint类型表示骨骼点的位置信息。SkeletonPoint是一个结构体，包含X、Y、Z三个数据成员，用以存储骨骼点的三维坐标
CvPoint colorPoint[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT]={cvPoint(0,0)};  


  HRESULT VisualizeFacetracker(IFTImage* pColorImg, IFTResult* pAAMRlt, UINT32 color)
{
    if (!pColorImg->GetBuffer() || !pAAMRlt)
    {
        return E_POINTER;
    }

    // Insufficient data points to render face data
    FT_VECTOR2D* pPts2D;
    UINT pts2DCount;
    HRESULT hr = pAAMRlt->Get2DShapePoints(&pPts2D, &pts2DCount);
    if (FAILED(hr))
    {
        return hr;
    }

    if (pts2DCount < 86)
    {
        return E_INVALIDARG;
    }


    POINT* pFaceModel2DPoint = reinterpret_cast<POINT*>(_malloca(sizeof(POINT) * pts2DCount));
    if (!pFaceModel2DPoint)
    {
        return E_OUTOFMEMORY;
    }


    for (UINT ipt = 0; ipt < pts2DCount; ++ipt)
    {
        pFaceModel2DPoint[ipt].x = LONG(pPts2D[ipt].x + 0.5f);
        pFaceModel2DPoint[ipt].y = LONG(pPts2D[ipt].y + 0.5f);
    }

    for (UINT ipt = 0; ipt < 8; ++ipt)
    {
        POINT ptStart = pFaceModel2DPoint[ipt];
        POINT ptEnd = pFaceModel2DPoint[(ipt+1)%8];
        pColorImg->DrawLine(ptStart, ptEnd, color, 1);
    }

    for (UINT ipt = 8; ipt < 16; ++ipt)
    {
        POINT ptStart = pFaceModel2DPoint[ipt];
        POINT ptEnd = pFaceModel2DPoint[(ipt-8+1)%8+8];
        pColorImg->DrawLine(ptStart, ptEnd, color, 1);
    }

    for (UINT ipt = 16; ipt < 26; ++ipt)
    {
        POINT ptStart = pFaceModel2DPoint[ipt];
        POINT ptEnd = pFaceModel2DPoint[(ipt-16+1)%10+16];
        pColorImg->DrawLine(ptStart, ptEnd, color, 1);
    }

    for (UINT ipt = 26; ipt < 36; ++ipt)
    {
        POINT ptStart = pFaceModel2DPoint[ipt];
        POINT ptEnd = pFaceModel2DPoint[(ipt-26+1)%10+26];
        pColorImg->DrawLine(ptStart, ptEnd, color, 1);
    }

    for (UINT ipt = 36; ipt < 47; ++ipt)
    {
        POINT ptStart = pFaceModel2DPoint[ipt];
        POINT ptEnd = pFaceModel2DPoint[ipt+1];
        pColorImg->DrawLine(ptStart, ptEnd, color, 1);
    }

    for (UINT ipt = 48; ipt < 60; ++ipt)
    {
        POINT ptStart = pFaceModel2DPoint[ipt];
        POINT ptEnd = pFaceModel2DPoint[(ipt-48+1)%12+48];
        pColorImg->DrawLine(ptStart, ptEnd, color, 1);
    }

    for (UINT ipt = 60; ipt < 68; ++ipt)
    {
        POINT ptStart = pFaceModel2DPoint[ipt];
        POINT ptEnd = pFaceModel2DPoint[(ipt-60+1)%8+60];
        pColorImg->DrawLine(ptStart, ptEnd, color, 1);
    }

    for (UINT ipt = 68; ipt < 86; ++ipt)
    {
        POINT ptStart = pFaceModel2DPoint[ipt];
        POINT ptEnd = pFaceModel2DPoint[ipt+1];
        pColorImg->DrawLine(ptStart, ptEnd, color, 1);
    }
    _freea(pFaceModel2DPoint);

    return hr;
}

   HRESULT VisualizeFaceModel(IFTImage* pColorImg, IFTModel* pModel, FT_CAMERA_CONFIG const* pCameraConfig, FLOAT const* pSUCoef, 
	   FLOAT zoomFactor, POINT viewOffset, IFTResult* pAAMRlt, UINT32 color)
   {
	   if (!pColorImg || !pModel || !pCameraConfig || !pSUCoef || !pAAMRlt)
	   {
		   return E_POINTER;
	   }

	   HRESULT hr = S_OK;
	   UINT vertexCount = pModel->GetVertexCount();
	   FT_VECTOR2D* pPts2D = reinterpret_cast<FT_VECTOR2D*>(_malloca(sizeof(FT_VECTOR2D) * vertexCount));
	   if (pPts2D)
	   {
		   FLOAT *pAUs;
		   UINT auCount;
		   hr = pAAMRlt->GetAUCoefficients(&pAUs, &auCount);
		   if (SUCCEEDED(hr))
		   {
			   FLOAT scale, rotationXYZ[3], translationXYZ[3];
			   hr = pAAMRlt->Get3DPose(&scale, rotationXYZ, translationXYZ);
			   if (SUCCEEDED(hr))
			   {
				   hr = pModel->GetProjectedShape(pCameraConfig, zoomFactor, viewOffset, pSUCoef, pModel->GetSUCount(), pAUs, auCount, 
					   scale, rotationXYZ, translationXYZ, pPts2D, vertexCount);
				   if (SUCCEEDED(hr))
				   {
					   POINT* p3DMdl   = reinterpret_cast<POINT*>(_malloca(sizeof(POINT) * vertexCount));
					   if (p3DMdl)
					   {
						   for (UINT i = 0; i < vertexCount; ++i)
						   {
							   p3DMdl[i].x = LONG(pPts2D[i].x + 0.5f);
							   p3DMdl[i].y = LONG(pPts2D[i].y + 0.5f);
						   }

						   FT_TRIANGLE* pTriangles;
						   UINT triangleCount;
						   hr = pModel->GetTriangles(&pTriangles, &triangleCount);
						   if (SUCCEEDED(hr))
						   {
							   struct EdgeHashTable
							   {
								   UINT32* pEdges;
								   UINT edgesAlloc;

								   void Insert(int a, int b) 
								   {
									   UINT32 v = (min(a, b) << 16) | max(a, b);
									   UINT32 index = (v + (v << 8)) * 49157, i;
									   for (i = 0; i < edgesAlloc - 1 && pEdges[(index + i) & (edgesAlloc - 1)] && v != pEdges[(index + i) & (edgesAlloc - 1)]; ++i)
									   {
									   }
									   pEdges[(index + i) & (edgesAlloc - 1)] = v;
								   }
							   } eht;

							   eht.edgesAlloc = 1 << UINT(log(2.f * (1 + vertexCount + triangleCount)) / log(2.f));
							   eht.pEdges = reinterpret_cast<UINT32*>(_malloca(sizeof(UINT32) * eht.edgesAlloc));
							   if (eht.pEdges)
							   {
								   ZeroMemory(eht.pEdges, sizeof(UINT32) * eht.edgesAlloc);
								   for (UINT i = 0; i < triangleCount; ++i)
								   { 
									   eht.Insert(pTriangles[i].i, pTriangles[i].j);
									   eht.Insert(pTriangles[i].j, pTriangles[i].k);
									   eht.Insert(pTriangles[i].k, pTriangles[i].i);
								   }
								   for (UINT i = 0; i < eht.edgesAlloc; ++i)
								   {
									   if(eht.pEdges[i] != 0)
									   {
										   pColorImg->DrawLine(p3DMdl[eht.pEdges[i] >> 16], p3DMdl[eht.pEdges[i] & 0xFFFF], color, 1);
									   }
								   }
								   _freea(eht.pEdges);
							   }

							   // Render the face rect in magenta
							   RECT rectFace;
							   hr = pAAMRlt->GetFaceRect(&rectFace);
							   if (SUCCEEDED(hr))
							   {
								   POINT leftTop = {rectFace.left, rectFace.top};
								   POINT rightTop = {rectFace.right - 1, rectFace.top};
								   POINT leftBottom = {rectFace.left, rectFace.bottom - 1};
								   POINT rightBottom = {rectFace.right - 1, rectFace.bottom - 1};
								   UINT32 nColor = 0xff00ff;
								   SUCCEEDED(hr = pColorImg->DrawLine(leftTop, rightTop, nColor, 1)) &&
									   SUCCEEDED(hr = pColorImg->DrawLine(rightTop, rightBottom, nColor, 1)) &&
									   SUCCEEDED(hr = pColorImg->DrawLine(rightBottom, leftBottom, nColor, 1)) &&
									   SUCCEEDED(hr = pColorImg->DrawLine(leftBottom, leftTop, nColor, 1));
							   }
						   }

						   _freea(p3DMdl); 
					   }
					   else
					   {
						   hr = E_OUTOFMEMORY;
					   }
				   }
			   }
		   }
		   _freea(pPts2D);
	   }
	   else
	   {
		   hr = E_OUTOFMEMORY;
	   }
	   return hr;
   }


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

string CString2StdString(const CString& cstr)  
{     
    CT2A str(cstr);  
    return string(str.m_psz);  
}  
class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
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


// CmykinectTestDlg 对话框

CmykinectTestDlg::CmykinectTestDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CmykinectTestDlg::IDD, pParent)
	, IDC_EDIT_RXDATA(_T(""))
	, m_bMirror(true)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_bStart=false;
	m_hThNuiProcess=NULL;


	pFT = 0;
	m_hWnd = NULL;
	pFTResult = NULL;
	pColorFrame = NULL;
	pDepthImage = NULL;


	m_ZoomFactor = 1.0f;
	m_ViewOffset.x = 0;
	m_ViewOffset.y = 0;
	m_LastTrackSucceeded=false;

	Is_open=false;
	m_Port=4;
	m_KinectSensorPresent=false;
	m_scale=0;
	m_translationXYZ[0]=m_translationXYZ[1]=m_translationXYZ[2]=0;
	m_rotationXYZ[0]=m_rotationXYZ[1]=m_rotationXYZ[2]=0;
	m_controlAngle = _T("");
	//是否镜像
    m_bSelectedSkeleton=false;
	m_bRecord=false;
}

void CmykinectTestDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_MSCOMM1, m_com);
	DDX_Text(pDX, IDC_EDIT1, IDC_EDIT_RXDATA);
	DDX_Text(pDX, IDC_EDIT_CONTROLANGLE, m_controlAngle);
}

BEGIN_MESSAGE_MAP(CmykinectTestDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUT_OPENKINECT, &CmykinectTestDlg::OnBnClickedButOpenkinect)

	ON_BN_CLICKED(IDC_BUT_STOPKINECT, &CmykinectTestDlg::OnBnClickedButStopkinect)
	
	ON_BN_CLICKED(IDC_OPENPORT, &CmykinectTestDlg::OnBnClickedOpenport)
	ON_BN_CLICKED(IDC_CLOSEPORT, &CmykinectTestDlg::OnBnClickedCloseport)
	ON_BN_CLICKED(IDC_BUTTON_ONLINE, &CmykinectTestDlg::OnBnClickedButtonOnline)
	ON_BN_CLICKED(IDC_BUTTON_OFFLINE, &CmykinectTestDlg::OnBnClickedButtonOffline)
	ON_BN_CLICKED(IDC_BUTTON_RESET, &CmykinectTestDlg::OnBnClickedButtonReset)
	ON_BN_CLICKED(IDC_MIRRO1, &CmykinectTestDlg::OnBnClickedMirro1)
	ON_BN_CLICKED(IDC_BTN_START_RECORD, &CmykinectTestDlg::OnBnClickedBtnStartRecord)
	ON_BN_CLICKED(IDC_BTN_END_RECORD, &CmykinectTestDlg::OnBnClickedBtnEndRecord)
	ON_EN_CHANGE(IDC_EDIT_CONTROLANGLE, &CmykinectTestDlg::OnEnChangeEditControlangle)
	ON_EN_CHANGE(IDC_EDIT1, &CmykinectTestDlg::OnEnChangeEdit1)
END_MESSAGE_MAP()


// CmykinectTestDlg 消息处理程序

BOOL CmykinectTestDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
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

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	ShowWindow(SW_MINIMIZE);
	//m_bMirror=false;

	// TODO: 在此添加额外的初始化代码
	//机器人位置初始化
	m_RobotChControl.ReadNormalPosition("normalaction.TXT");
	
    double hscale = 1.0;
    double vscale = 1.0;
    int linewidth = 2;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,hscale,vscale,0,linewidth);


 
	colorImage.create(480,640,CV_8UC3);
	depthImage.create(240,320,CV_8UC3);
	skeletonImage.create(240,320,CV_8UC3);
	mask.create(240,320,CV_8UC3);

	//1.定义事件句柄
	 colorEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
	 depthEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
	 skeletonEvent=CreateEvent(NULL,TRUE,FALSE,NULL);

	 colorStreamHandle=NULL;
	 depthStreamHandle=NULL;


	//2.初始化
	hr=NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR|NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX|NUI_INITIALIZE_FLAG_USES_SKELETON);

	if(FAILED(hr)){
         MessageBox( _T("摄像头初始化失败"),_T("错误"));
	   return hr;
	}
	//Return Value
    //Type: HRESULT
    //Returns S_OK if successful; otherwise, returns a failure code.

   //3、打开KINECT设备
   //3.1 打开彩色图信息通道，并用colorStreamHandle保存该流的句柄，以便于以后读取 
   hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, NULL, 4, colorEvent, &colorStreamHandle);   
   if( hr != S_OK )     
    {     
         MessageBox(_T("打开彩色摄像头失败"),_T("错误"));   
         NuiShutdown();          
		return hr;     
    }  
   //3.2 打开深度图信息通道，并用depthStreamHandle保存该流的句柄，以便于以后读取 
    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, NULL, 2, depthEvent, &depthStreamHandle);   
    if( hr != S_OK )     
    {     
         MessageBox(_T("打开深度摄像头失败"),_T("错误"));   
         NuiShutdown();          
		 return hr;     
    }  
	//3.3 设置骨骼跟踪，并用colorStreamHandle保存该流的句柄，以便于以后读取 
	 hr = NuiSkeletonTrackingEnable( skeletonEvent, 0 );//打开骨骼跟踪事件     
	 //Returns S_OK if successful; otherwise, returns one of the following failure codes.
     //
     if( hr != S_OK )     
    {     
        MessageBox(_T("设置骨骼跟踪识别"),_T("错误")); 
        NuiShutdown();   
       return hr;     
    }   

	 ///添加的人脸代码
	  pFT = FTCreateFaceTracker();
	  if(!pFT)
	  {
		// Handle errors
	  }
	   
	  // FT_CAMERA_CONFIG myColorCameraConfig = {640, 480, 1.0}; // width, height, focal length
       //FT_CAMERA_CONFIG myDepthCameraConfig = {640, 480}; // width, height

	  FT_CAMERA_CONFIG videoConfig = {640, 480, NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS}; // width, height, focal length
      FT_CAMERA_CONFIG depthConfig; // width, height
      depthConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;  
	  depthConfig.Width = 320;  
	  depthConfig.Height = 240;//貌似这里一定要填，而且要填对才行！！  
		
    HRESULT hr = pFT->Initialize(&videoConfig, &depthConfig, NULL, NULL);
		  if( FAILED(hr) )
		  {
			// Handle errors
		  }

	  // Create IFTResult to hold a face tracking result
	  pFTResult = NULL;
	  hr = pFT->CreateFTResult(&pFTResult);
	  if(FAILED(hr))
	  {
		// Handle errors
	  }
	  // prepare Image and SensorData for 640x480 RGB images
	  pColorFrame = FTCreateImage();
	  if(!pColorFrame)
	  {
		return E_OUTOFMEMORY;
	  }
		DWORD width = 0;
		DWORD height = 0;

		NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, width, height);

		hr = pColorFrame->Allocate(videoConfig.Width, videoConfig.Height, FTIMAGEFORMAT_UINT8_B8G8R8X8);
		if (FAILED(hr))
		{
			return hr;
		}
		pDepthImage = FTCreateImage();
		if (!pDepthImage)
		{
			return E_OUTOFMEMORY;
		}

		NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_320x240, width, height);

		hr = pDepthImage->Allocate(depthConfig.Width, depthConfig.Height, FTIMAGEFORMAT_UINT16_D13P3);
		if (FAILED(hr))
		{
			return hr;
		}

		  for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
		{
			m_HeadPoint[i] = m_NeckPoint[i] = FT_VECTOR3D(0, 0, 0);
			m_SkeletonTracked[i] = false;
		}
		 
		m_hint3D[0] = m_hint3D[1] = FT_VECTOR3D(0, 0, 0);
		SetCenterOfImage(NULL);
		((CButton *)GetDlgItem(IDC_MIRRO1))->SetCheck(1);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CmykinectTestDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CmykinectTestDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CmykinectTestDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CmykinectTestDlg::OnBnClickedButOpenkinect()
{
	// TODO: 在此添加控件通知处理程序代码 
	if(m_bStart )
		return;
	if ( NULL != m_hThNuiProcess )
    {
        WaitForSingleObject( m_hThNuiProcess, INFINITE );
        CloseHandle( m_hThNuiProcess );
    } 
	m_bStart=TRUE;
	m_KinectSensorPresent=TRUE;
	m_hThNuiProcess = CreateThread( NULL, 0, Nui_ProcessThread, this, 0, NULL );
    return ;  
}
// 实现cv::Mat 结构到 CImage结构的转化  
void CmykinectTestDlg::MatToCImage(Mat& mat, CImage& cImage)  
{  
    int width = mat.cols;  
    int height = mat.rows;  
    int channels = mat.channels();  
  
    cImage.Destroy();//这一步是防止重复利用造成内存问题  
    cImage.Create(width,height,8*channels);  
  
    uchar* ps;  
    uchar* pimg = (uchar*)cImage.GetBits(); //获取CImage的像素存贮区的指针  
    int step = cImage.GetPitch();//每行的字节数,注意这个返回值有正有负  
      
    // 如果是1个通道的图像(灰度图像) DIB格式才需要对调色板设置    
    // CImage中内置了调色板，我们要对他进行赋值：  
    if (1 == channels)  
    {  
        RGBQUAD* ColorTable;    
        int MaxColors=256;    
        //这里可以通过CI.GetMaxColorTableEntries()得到大小(如果你是CI.Load读入图像的话)    
        ColorTable = new RGBQUAD[MaxColors];    
        cImage.GetColorTable(0,MaxColors,ColorTable);//这里是取得指针    
        for (int i=0; i<MaxColors; i++)    
        {    
            ColorTable[i].rgbBlue = (BYTE)i;    
            //BYTE和uchar一回事，但MFC中都用它    
            ColorTable[i].rgbGreen = (BYTE)i;    
            ColorTable[i].rgbRed = (BYTE)i;    
        }    
        cImage.SetColorTable(0,MaxColors,ColorTable);    
        delete []ColorTable;    
    }  
      
  
    for (int i = 0; i < height; i++)  
    {  
        ps = mat.ptr<uchar>(i);  
        for (int j = 0; j < width; j++)  
        {  
            if (1 == channels)  
            {  
                *(pimg + i*step + j) = ps[j];  
                //*(pimg + i*step + j) = 105;  
            }  
            else if (3 == channels)  
            {  
                *(pimg + i*step + j*3) = ps[j*3];  
                *(pimg + i*step + j*3 + 1) = ps[j*3 + 1];  
                *(pimg + i*step + j*3 + 2) = ps[j*3 + 2];  
            } 
			else if (4 == channels)  
            {  
                *(pimg + i*step + j*4) = ps[j*4];  
                *(pimg + i*step + j*4 + 1) = ps[j*4 + 1];  
                *(pimg + i*step + j*4 + 2) = ps[j*4 + 2];  
				*(pimg + i*step + j*4 + 3) = ps[j*4 + 2]; 
            }  
        }  
    }  
    
}  

// 显示图像到指定窗口  
void CmykinectTestDlg::DisplayImage(CWnd* m_pMyWnd,const CImage& image)  
{  
      
    CDC *m_pDC = m_pMyWnd->GetDC();//获取窗口所拥有的设备上下文，用于显示图像  
    m_pMyWnd->UpdateWindow();  
  
    CRect rc;  
    m_pMyWnd->GetWindowRect(&rc);  
  
    /*InvalidateRect(m_pMyWnd->m_hWnd,&rc,true);*/  
    int nwidth = rc.Width();  
    int nheight = rc.Height();  
  
    int fixed_width = min(image.GetWidth(),nwidth);  
    int fixed_height = min(image.GetHeight(),nheight);  
  
    double ratio_w = fixed_width / (double)image.GetWidth();  
    double ratio_h = fixed_height / (double)image.GetHeight();  
    double ratio = min(ratio_w,ratio_h);  
  
    int show_width = (int)(image.GetWidth() * ratio);  
    int show_height = (int)(image.GetHeight() * ratio);  
  
    int offsetx = (nwidth - show_width) / 2;  
    int offsety = (nheight - show_height) / 2;  
  
    ::SetStretchBltMode(m_pDC->GetSafeHdc(),COLORONCOLOR);//设置位图的伸缩模式  
    image.StretchBlt(m_pDC->GetSafeHdc(),offsetx,offsety,show_width,show_height,  
        0,0,image.GetWidth(),image.GetHeight(),SRCCOPY);  
}  




void CmykinectTestDlg::OnBnClickedButStopkinect()
{
	// TODO: 在此添加控件通知处理程序代码
	if(m_bStart)
	{
		 m_bStart=false;
	     /*NuiShutdown();*/
		 if ( NULL != m_hThNuiProcess )
		{
			WaitForSingleObject( m_hThNuiProcess, INFINITE );
			CloseHandle( m_hThNuiProcess );
			m_hThNuiProcess=NULL;
		}
	}
}
DWORD WINAPI CmykinectTestDlg::Nui_ProcessThread( LPVOID pParam )
{
    CmykinectTestDlg *pthis = (CmykinectTestDlg *)pParam;
    return pthis->Nui_ProcessThread( );
}
DWORD WINAPI CmykinectTestDlg::Nui_ProcessThread( )
{  	  
	//4、开始读取彩色图数据 
	 while(m_bStart)   
    {   
        
		if(WaitForSingleObject(colorEvent, 0)==0)   
             getColorImage(colorEvent, colorStreamHandle, colorImage);
		       // GotVideoAlert();
        if(WaitForSingleObject(depthEvent, 0)==0)   
            getDepthImage(depthEvent, depthStreamHandle, depthImage);   
		   //这里使用INFINITE是为了避免没有激活skeletonEvent而跳过此代码出现colorimage频闪的现象   
        if(WaitForSingleObject(skeletonEvent, INFINITE)==0)  
          {
			  GetSkeletonData(skeletonEvent, skeletonImage, colorImage, depthImage);  
			  if(m_bSelectedSkeleton) //跟踪到骨架
			  {   GetHeadData();
				  m_RobotChControl.SetHeadPose(m_scale,m_rotationXYZ,m_translationXYZ);
			      if(m_bSelectedSkeleton)
					{ 
						m_RobotChControl.MapActionToRobot(m_controlAngle,m_bMirror);
						m_bSelectedSkeleton=false;

						m_RobotChControl.SendControlValueToComm(&this->m_com,m_RobotChControl.m_oldChValue,48);
						if(m_bRecord){
						    CString str;
							
							str.Format("00:00:00:%d, ",m_recordFrameNum++);
							m_os<<endl<<str;
							for(int k=0;k<48;k++)
							{
								 m_os<<m_RobotChControl.m_oldChValue[k]<<", ";
							     
							}

						}
					}
			  }
		    /*if(m_Port)
			{ 
				float a7=m_rotationXYZ[1],b7=m_rotationXYZ[0],c7=m_rotationXYZ[2];
				if(a7>45)
					a7=45;
				if(a7<-45)
					a7=-45;
				oldChValue[9]=(a7-45)/-90.0*(255);		

				if(b7>5)
					b7=5;
				if(b7<-40)
					b7=-40;
				if(b7<-15)
				   oldChValue[10]=(b7+40)/-25.0*(127)+255;			   
				else
				   oldChValue[10]=(b7-5)/-20.0*(128);			   

				//if(c7>35)
				//	c7=35;
				//if(c7<-35)
				//	c7=-35;
				//if(c7<0)
				//{ 
				//	if(c7<-10)
				//		oldChValue[7]=(c7+40)/30.0*(-127)+255;
				//	else
				//		oldChValue[7]=(c7+10)/10.0*(-128)+128;				
				//	oldChValue[8]=128;
				//
				//}			   
				//else
				//{
				//}
				 SendControlValueToComm(oldChValue,48);
				 
			}*/
			  ShowImage(pColorFrame,GetDlgItem(IDC_SHOW_COLOR));
		}
		//str.Format(_T("t:%.1lf,%.1lf,%.1lf"),m_translationXYZ[0],m_translationXYZ[1],m_translationXYZ[2]);
		//textPos.y=90;
		//cvPutText(&img,(LPSTR)(LPCTSTR)str, textPos, &font,textColor);


	   /* 
		Mat tempMat(pColorFrame->GetHeight(),pColorFrame->GetWidth(),CV_8UC4,pColorFrame->GetBuffer());
		//imshow("faceTracking",tempMat);;
		CImage cImage;
		MatToCImage(tempMat,  cImage);
		DisplayImage(GetDlgItem(IDC_SHOW_COLOR),cImage);
		//cImage.Save(_T("C:\\sample1024.bmp"));  
	    cImage.Destroy();

        //imshow("faceTracking",tempMat);  

		CImage cImage1;
	    MatToCImage(depthImage,  cImage1);
		DisplayImage(GetDlgItem(IDC_SHOW_IMAGE_DEPTH),cImage1);
	    cImage1.Destroy();

		CImage cImage2;
	    MatToCImage(skeletonImage,  cImage2);
		DisplayImage(GetDlgItem(IDC_SHOW_IMAGE_SKELETON),cImage2);
	    cImage2.Destroy();//这一步是防止重复利用造成内存问题
		*/
		
		/*Mat tempMat(pDepthImage->GetHeight(),pDepthImage->GetWidth(),CV_8UC4,pDepthImage->GetBuffer());  
		CImage cImage1;
	    MatToCImage(depthImage,  cImage1);
		DisplayImage(GetDlgItem(IDC_SHOW_IMAGE_DEPTH),cImage1);
	    cImage1.Destroy();
	  */

		/*	CImage cImage;
		MatToCImage(colorImage,  cImage);
		DisplayImage(GetDlgItem(IDC_SHOW_COLOR),cImage);
		cImage.Destroy();
		*/
		//CImage cImage1;
	 //   MatToCImage(depthImage,  cImage1);
		//DisplayImage(GetDlgItem(IDC_SHOW_IMAGE_DEPTH),cImage1);
	 //   cImage1.Destroy();//
		//
		//CImage cImage2;
	 //   MatToCImage(skeletonImage,  cImage2);
		//DisplayImage(GetDlgItem(IDC_SHOW_IMAGE_SKELETON),cImage2);
	 //   cImage2.Destroy();//这一步是防止重复利用造成内存问题
			
		/*CImage cImage3;
	    MatToCImage(mask,  cImage3);
		DisplayImage(GetDlgItem(IDC_SHOW_IMAGE_MARK),cImage3);
	    cImage3.Destroy();//这一步是防止重复利用造成内存问题
		*/
    }   
    
	return 1;
}

BOOL CmykinectTestDlg::DestroyWindow()
{
	// TODO: 在此添加专用代码和/或调用基类
	if(pFT)
	   pFT->Release();
    pFT = NULL;

    if(pColorFrame)
    {
        pColorFrame->Release();
        pColorFrame = NULL;
    }

    if(pDepthImage) 
    {
        pDepthImage->Release();
        pDepthImage = NULL;
    }

    if(pFTResult)
    {
        pFTResult->Release();
        pFTResult = NULL;
    }

	

	  //7、关闭NUI链接   
    if(m_bStart)
	{ 
		NuiShutdown();
	}

	//销毁三个事件句柄
	if(colorEvent && (colorEvent!= INVALID_HANDLE_VALUE))
	  {
		  CloseHandle(colorEvent );
          colorEvent = NULL;
	}
	if( depthEvent && (depthEvent!= INVALID_HANDLE_VALUE))
	  {
		  CloseHandle(depthEvent );
          depthEvent = NULL;
	  }
	if(skeletonEvent && (skeletonEvent!= INVALID_HANDLE_VALUE))
	  {
		  CloseHandle(skeletonEvent );
          skeletonEvent = NULL;
      }

	return CDialogEx::DestroyWindow();
}
void CmykinectTestDlg::getColorImage(HANDLE &colorEvent, HANDLE &colorStreamHandle, Mat &colorImage)   
{   
    const NUI_IMAGE_FRAME *colorFrame = NULL;    
    NuiImageStreamGetNextFrame(colorStreamHandle, 0, &colorFrame);   
    INuiFrameTexture *pTexture = colorFrame->pFrameTexture;        
    NUI_LOCKED_RECT LockedRect;   
    pTexture->LockRect(0, &LockedRect, NULL, 0);
    if( LockedRect.Pitch != 0 )   
	{   
		memcpy(pColorFrame->GetBuffer(), PBYTE(LockedRect.pBits), min(pColorFrame->GetBufferSize(), UINT(pTexture->BufferLen())));
       
		for (int i=0; i<colorImage.rows; i++)   
        {  
            uchar *ptr = colorImage.ptr<uchar>(i);  //第i行的指针                    
            //每个字节代表一个颜色信息，直接使用uchar  
            uchar *pBuffer = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;  
            for (int j=0; j<colorImage.cols; j++)   
            {   
                ptr[3*j] = pBuffer[4*j];  //内部数据是4个字节，0-1-2是BGR，第4个现在未使用   
                ptr[3*j+1] = pBuffer[4*j+1];   
                ptr[3*j+2] = pBuffer[4*j+2];   
            }   
        }   
    }   
    else   
    {   
        //MessageBox(_T("捕捉色彩图像出现错误"));   
    }  
    pTexture->UnlockRect(0);   
    NuiImageStreamReleaseFrame(colorStreamHandle, colorFrame );  
}   
 void CmykinectTestDlg::getDepthImage(HANDLE &depthEvent, HANDLE &depthStreamHandle, Mat &depthImage)   
{   
    const NUI_IMAGE_FRAME *depthFrame = NULL;   
    NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame);   
    INuiFrameTexture *pTexture = depthFrame->pFrameTexture;     
   
    NUI_LOCKED_RECT LockedRect;   
    pTexture->LockRect(0, &LockedRect, NULL, 0);     
	if (LockedRect.Pitch)
	{   // Copy depth frame to face tracking
		memcpy(pDepthImage->GetBuffer(), PBYTE(LockedRect.pBits), min(pDepthImage->GetBufferSize(), UINT(pTexture->BufferLen())));
	}
	else
	{
		OutputDebugString( _T("Buffer length of received depth texture is bogus\r\n") );
	}
	
    RGBQUAD q;   
  
    if( LockedRect.Pitch != 0 )   
    {   
        for (int i=0; i<depthImage.rows; i++)   
        {   
            uchar *ptr = depthImage.ptr<uchar>(i);   
            uchar *pBuffer = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;  
            USHORT *pBufferRun = (USHORT*) pBuffer;   
              
            for (int j=0; j<depthImage.cols; j++)   
            {   
                int player = pBufferRun[j]&7;   
                int data = (pBufferRun[j]&0xfff8) >> 3;   
                   
               uchar imageData = 255-(uchar)(256*data/0x0fff);   
               q.rgbBlue = q.rgbGreen = q.rgbRed = 0;   
   
                switch(player)   
                {   
                    case 0:     
                        q.rgbRed = imageData / 2;     
                        q.rgbBlue = imageData / 2;     
                        q.rgbGreen = imageData / 2;     
                       break;     
                    case 1:      
                        q.rgbRed = imageData;     
                        break;     
                    case 2:     
                        q.rgbGreen = imageData;     
                        break;     
                    case 3:     
                        q.rgbRed = imageData / 4;     
                        q.rgbGreen = q.rgbRed*4;  //这里利用乘的方法，而不用原来的方法可以避免不整除的情况   
                        q.rgbBlue = q.rgbRed*4;  //可以在后面的getTheContour()中配合使用，避免遗漏一些情况   
                        break;     
                   case 4:     
                        q.rgbBlue = imageData / 4;    
                        q.rgbRed = q.rgbBlue*4;     
                        q.rgbGreen = q.rgbBlue*4;     
                       break;     
                    case 5:     
                       q.rgbGreen = imageData / 4;    
                        q.rgbRed = q.rgbGreen*4;     
                        q.rgbBlue = q.rgbGreen*4;     
                        break;     
                    case 6:     
                        q.rgbRed = imageData / 2;     
                        q.rgbGreen = imageData / 2;      
                        q.rgbBlue = q.rgbGreen*2;     
                        break;     
                    case 7:     
                        q.rgbRed = 255 - ( imageData / 2 );     
                        q.rgbGreen = 255 - ( imageData / 2 );     
                        q.rgbBlue = 255 - ( imageData / 2 );   
               }      
               ptr[3*j] = q.rgbBlue;   
               ptr[3*j+1] = q.rgbGreen;   
               ptr[3*j+2] = q.rgbRed;   
            }   
        }   
    }   
    else   
    {   
		//MessageBox(_T("捕捉深度图像出现错误"));   
    }   
      
    pTexture->UnlockRect(0);  
    NuiImageStreamReleaseFrame(depthStreamHandle, depthFrame);    
}   

void CmykinectTestDlg::GetSkeletonData(HANDLE &skeletonEvent, Mat &skeletonImage, Mat &colorImage, Mat &depthImage)   
{   
    NUI_SKELETON_FRAME skeletonFrame = {0};  
    bool bFoundSkeleton = false;    
   
   if(NuiSkeletonGetNextFrame( 0, &skeletonFrame ) == S_OK )     
   {     
       for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )     
        {     
            if( skeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )   
            {     
               bFoundSkeleton = true;     

			   m_SkeletonTracked[i] = true;

			   m_HeadPoint[i].x = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].x;
			   m_HeadPoint[i].y = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].y;
			   m_HeadPoint[i].z = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].z;

			   m_NeckPoint[i].x = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].x;
			   m_NeckPoint[i].y = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].y;
			   m_NeckPoint[i].z = skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].z;

              // break;   
            }
			else{
				m_HeadPoint[i] = m_NeckPoint[i] = FT_VECTOR3D(0, 0, 0);
				m_SkeletonTracked[i] = false;
			}
        }     
    }   
    else   
    {    //MessageBox(_T("没有找到合适的骨骼帧"));  
         return;    
    }   
   
    if( !bFoundSkeleton )     
    {     
        return;    
    }     
   
    NuiTransformSmooth(&skeletonFrame, NULL);//平滑骨骼帧,消除抖动     
   
	skeletonImage.setTo(0);     

	int selectedSkeletonId=0;
	GetClosestSkeleton(m_hint3D,selectedSkeletonId);
	if( selectedSkeletonId !=-1 
		&& skeletonFrame.SkeletonData[selectedSkeletonId].eTrackingState == NUI_SKELETON_TRACKED 
		&& skeletonFrame.SkeletonData[selectedSkeletonId].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_SHOULDER_CENTER] != NUI_SKELETON_POSITION_NOT_TRACKED)  //没有发现最近的骨骼
     { 
		 m_RobotChControl.SetSkeletonData(skeletonFrame.SkeletonData[selectedSkeletonId]);
		 m_bSelectedSkeleton=true;//已经跟踪到骨架
	
	}
	/*for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )     
    {     
        if( skeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED &&     
            skeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[NUI_SKELETON_POSITION_SHOULDER_CENTER] != NUI_SKELETON_POSITION_NOT_TRACKED)     
        {   
			m_RobotChControl.SetSkeletonData(skeletonFrame.SkeletonData[i]);

             float fx, fy;     
   
            for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)//所有的坐标转化为深度图的坐标     
            {     
                NuiTransformSkeletonToDepthImage(skeletonFrame.SkeletonData[i].SkeletonPositions[j], &fx, &fy );     
                skeletonPoint[i][j].x = (int)fx;     
                skeletonPoint[i][j].y = (int)fy;     
            }     
   
            for (int j=0; j<NUI_SKELETON_POSITION_COUNT ; j++)     
            {     
                if (skeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[j] != NUI_SKELETON_POSITION_NOT_TRACKED)//跟踪点一用有三种状态：1没有被跟踪到，2跟踪到，3根据跟踪到的估计到     
                {     
                    LONG colorx, colory;   
                    NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480, 0,    
                        skeletonPoint[i][j].x, skeletonPoint[i][j].y, 0,&colorx, &colory);   
                    colorPoint[i][j].x = int(colorx);  
                    colorPoint[i][j].y = int(colory); //存储坐标点   
                    circle(colorImage, colorPoint[i][j], 4, cvScalar(0, 255, 255), 1, 8, 0);   
                    circle(skeletonImage, skeletonPoint[i][j], 3, cvScalar(0, 255, 255), 1, 8, 0);   
   
                    tracked[i] = TRUE;   
                }   
            }   
			
				//add by hz
			double a=0,b=0,c=0,a1=0,b1=0,c1=0,a2=0,b2=0,c2=0,d2=0,a3=0;
			double a4=0,b4=0;
			CalculateLeftArm(skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT],
				             skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_LEFT],
				             skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_WRIST_LEFT],
			                  a,b,c);			
			CalculateLeftArm(skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
				            skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT],
				            skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT],
			                a1,b1,c1);			
			CalculateRightLeg(skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HIP_LEFT],
							skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_KNEE_LEFT],
							skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_LEFT],
							skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT],
							a2,b2,c2,d2);
			CalculateBow(skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER],
							skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SPINE],
			                a3);
			CalculateUpbody(skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT],
				           skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
						   a4,b4);
			double a5=0,b5=0;
			CalculateWaist(skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER],
								skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SPINE],
								skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
								skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HIP_LEFT],
								a5,b5);
            double a6=0,b6=0,c6=0,d6=0;   
			CalculateShoulder(skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER],
				skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT],
				skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
				a6,
				b6,
				c6,
				d6);
           
			double a8=0;  
			CacluateHead(skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER],
				         skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD],a8);
			
			IplImage img;
			img=skeletonImage;

		   if(m_Port)		   
		   {   
			   ////z
			   if(a1<10)
				   a1=10;
			   if(a1>160)
				   a1=160;
			   oldChValue[13]=(a1-10)/150.0*255;

			   if(a1<20)
				   a1=20;
			   if(a1>90)
				   a1=90;
			   if(a1>85)
				    oldChValue[13]=10;
			   else if(a1>75)
				    oldChValue[13]=(85-a1)/10.0*10+10;
			   else 
				   oldChValue[13]=(75-a1)/55.0*235+20;


		       if(a<10)
				   a=10;
			   if(a>160)
				   a=160;
			   oldChValue[24]=(a-10)/150.0*255;

			   if(a<20)
				   a=20;
			   if(a>90)
				   a=90;
			   if(a>85)
				   oldChValue[24]=10;
			   else if(a>75)
				   oldChValue[24]=(85-a)/10.0*10+10;
			   else 
				   oldChValue[24]=(75-a)/55.0*235+20;

			   	   //x
			    b1=180-b1;
			   if(b1>=85)
				   oldChValue[14]=130;
			   if(b1>=75)
				 oldChValue[14]=(85-b1)/10.0*40+130;
			  else
			     oldChValue[14]=(75-b1)/60.0*90+160;
			   if( oldChValue[14]>255)
				   oldChValue[14]=255;

			   if(b<10)
				   b=10;
			   if(b>90)
				   b=90;
			   if(b>=85)
				   oldChValue[25]=130;
			   if(b>=75)
				   oldChValue[25]=(85-b)/10.0*40+130;
			   else
				   oldChValue[25]=(75-b)/60.0*90+160;
			   if( oldChValue[25]>255)
				    oldChValue[25]=255;



			   if(c<10)
				   c=10;
			   if(c>160)
				   c=160;
			   oldChValue[27]=(c-10)/150.0*255;

			   if(c1<10)
				   c1=10;
			   if(c1>160)
				   c1=160;
				   oldChValue[16]=(c1-10)/150.0*255;

//腰
			   if(a3<0)
				   a3=0;
			   if(a3>35)
				   a3=35;
			   oldChValue[33]=a3/35.0*255.0;

     //       //腿部动作
				 //z方向
				   if(a2>90)
					   a2=90;
				   if(a2<40)
					   a2=40;
				   oldChValue[40]=(90-a2)/50.0*200+55;//(a2-10)/150.0*255;

				   //x方向
				   if(b2<10)
					   b2=10;
				   if(b2>20)
					   b2=20;
				   oldChValue[41]=(b2-10)/10*180+70;

				   //////弯曲
				   if(c2<5)
					   c2=5;
				   if(c2>35)
					   c2=35;
				   oldChValue[43]=(c2-5)/30.0*255;

				   //////弯曲
				   if(d2<10)
					   d2=10;
				   if(d2>130)
					   d2=130;
				   if(d2<70)
            		    oldChValue[42]=(d2-10)/60.0*70+10;
				   else
				     oldChValue[42]=(d2-70)/60.0*180+70;

	//上半身    //////弯曲
				   if(a4<70)
					   a4=70;
				   if(a4>110)
					   a4=110;
				   oldChValue[34]=(a4-70)/40.0*255;

				   if(b4<40)
					   b4=40;
				   if(b4>140)
					   b4=140;
				   oldChValue[35]=(b4-40)/100.0*255;
	//腰动作: 
				 if(a5<0)
					   a5=0;
				   if(a5>110)
					   a5=110;
				   if(a5<=90)
				   {
				
				       oldChValue[37]=(a5-75)/15.0*255;
					   oldChValue[36]=250;
				   }
				   else {
					   oldChValue[36]=(a5-110)/-20.0*255;
					   oldChValue[37]=250;
				   }
				    
				   if(b5>65)
					   b5=65;
				   if(b5<30)
					   b5=30;
				   if(b5>60)
				   {
					   oldChValue[39]=(b5-65)/5.0*(-80);
				   }else
					   oldChValue[39]=-(b5-60)/30.0*175+80;
	//肩动作: 
				   if(a6>125)
					   a6=125;
				   if(a6<115)
					   a6=115;
				    oldChValue[23]=(a6-115)/-10.0*(255)+255;
				   

					if(b6>180)
						b6=180;
					if(b6<160)
						b6=110;
					oldChValue[22]=(b6-160)/-20.0*(255)+255;

					if(c6>125)
						c6=125;
					if(c6<115)
						c6=115;
					oldChValue[12]=(c6-115)/-10.0*(255)+255;


					if(d6>180)
						d6=180;
					if(d6<160)
						d6=110;
					oldChValue[11]=(d6-160)/-20.0*(255)+255;		


					if(a8>110)
					  a8=110;
					if(a8<70)
					  a8=70;
					if(a8>90)
					{ 
						
						oldChValue[7]=(a8-110)/-20.0*(128);
									
					    oldChValue[8]=128;

					}			   
					else
					{

						oldChValue[8]=(a8-70)/20.0*(128);

						oldChValue[7]=128;
					}

			   //SendControlValueToComm(oldChValue,48);
		   }
			CString str;
		    str.Format(_T("左手:（上下=%.0lf,前后=%.0lf,肘关节=%.0lf）"),a,b,c);
			m_controlAngle=str;	   
	     
			str.Format(_T("\r\n右手（上下=%.0lf,前后=%.0lf,肘关节=%.0lf）"),a1,b1,c1);
			m_controlAngle+=str;
					 
			
			str.Format(_T("\r\n腿部（向前=%.0lf,支开=%.0lf,膝关节=%.0lf）"),a2,b2,c2,d2);
			m_controlAngle+=str;

			
			str.Format(_T("\r\n腰(鞠躬):%.0lf"),a3);
			m_controlAngle+=str;
		
			str.Format(_T("\r\n上半身:倾斜=%.0lf,旋转=%.0lf"),a4,b4);
			m_controlAngle+=str;

			str.Format(_T("\r\n腰部:前倾后仰=%.0lf,倾斜=%.0lf"),a5,b5);
			m_controlAngle+=str;

			//沿着左 肩部前倾和耸肩
			str.Format(_T("\r\n肩部:左肩耸=%.0lf,左肩向前=%.0lf,右肩耸=%.0lf,右肩向前=%.0lf"),a6,b6,c6,d6);
			m_controlAngle+=str;
		
			str.Format(_T("\r\n头部:左右点=%.0lf"),a8);
			m_controlAngle+=str;

			//drawSkeleton(colorImage, colorPoint[i], i);    
			
			drawSkeleton(skeletonImage, skeletonPoint[i], i); 
			
			
		}  
    }*/     
}   
void CmykinectTestDlg::drawSkeleton(Mat &image, CvPoint pointSet[], int whichone)   
{   
    CvScalar color;   
    switch(whichone) //跟踪不同的人显示不同的颜色   
    {   
    case 0:   
        color = cvScalar(255);   
        break;   
    case 1:   
        color = cvScalar(0,255);   
        break;   
    case 2:   
        color = cvScalar(0, 0, 255);   
        break;   
    case 3:   
        color = cvScalar(255, 255, 0);   
        break;   
    case 4:   
        color = cvScalar(255, 0, 255);   
        break;   
    case 5:   
        color = cvScalar(0, 255, 255);   
        break;   
    }   
   
    if((pointSet[NUI_SKELETON_POSITION_HEAD].x!=0 || pointSet[NUI_SKELETON_POSITION_HEAD].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HEAD], pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_SPINE].x!=0 || pointSet[NUI_SKELETON_POSITION_SPINE].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SPINE], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_SPINE].x!=0 || pointSet[NUI_SKELETON_POSITION_SPINE].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HIP_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_CENTER].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SPINE], pointSet[NUI_SKELETON_POSITION_HIP_CENTER], color, 2);   
   
    //左上肢   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_LEFT], pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_ELBOW_LEFT], pointSet[NUI_SKELETON_POSITION_WRIST_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_WRIST_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HAND_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_HAND_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_WRIST_LEFT], pointSet[NUI_SKELETON_POSITION_HAND_LEFT], color, 2);   
   
    //右上肢   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_CENTER], pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_SHOULDER_RIGHT], pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_ELBOW_RIGHT], pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HAND_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_HAND_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_WRIST_RIGHT], pointSet[NUI_SKELETON_POSITION_HAND_RIGHT], color, 2);   
   
    //左下肢   
    if((pointSet[NUI_SKELETON_POSITION_HIP_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HIP_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HIP_CENTER], pointSet[NUI_SKELETON_POSITION_HIP_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_HIP_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HIP_LEFT], pointSet[NUI_SKELETON_POSITION_KNEE_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_KNEE_LEFT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_KNEE_LEFT], pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT].y!=0) &&    
        (pointSet[NUI_SKELETON_POSITION_FOOT_LEFT].x!=0 || pointSet[NUI_SKELETON_POSITION_FOOT_LEFT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_ANKLE_LEFT], pointSet[NUI_SKELETON_POSITION_FOOT_LEFT], color, 2);   
   
    //右下肢   
    if((pointSet[NUI_SKELETON_POSITION_HIP_CENTER].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_CENTER].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HIP_CENTER], pointSet[NUI_SKELETON_POSITION_HIP_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_HIP_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_HIP_RIGHT], pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT],color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_KNEE_RIGHT], pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT], color, 2);   
    if((pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT].y!=0) &&   
        (pointSet[NUI_SKELETON_POSITION_FOOT_RIGHT].x!=0 || pointSet[NUI_SKELETON_POSITION_FOOT_RIGHT].y!=0))   
        line(image, pointSet[NUI_SKELETON_POSITION_ANKLE_RIGHT], pointSet[NUI_SKELETON_POSITION_FOOT_RIGHT], color, 2);   
}   


HRESULT CmykinectTestDlg::GetVideoConfiguration(FT_CAMERA_CONFIG* videoConfig)
{
    if (!videoConfig)
    {
        return E_POINTER;
    }

    UINT width = pColorFrame ? pColorFrame->GetWidth() : 0;
    UINT height =  pColorFrame ? pColorFrame->GetHeight() : 0;
    FLOAT focalLength = 0.f;

    if(width == 640 && height == 480)
    {
        focalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
    }
    else if(width == 1280 && height == 960)
    {
        focalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.f;
    }

    if(focalLength == 0.f)
    {
        return E_UNEXPECTED;
    }


    videoConfig->FocalLength = focalLength;
    videoConfig->Width = width;
    videoConfig->Height = height;
    return(S_OK);
}

HRESULT CmykinectTestDlg::GetDepthConfiguration(FT_CAMERA_CONFIG* depthConfig)
{
    if (!depthConfig)
    {
        return E_POINTER;
    }

    UINT width = pDepthImage ? pDepthImage->GetWidth() : 0;
    UINT height =  pDepthImage ? pDepthImage->GetHeight() : 0;
    FLOAT focalLength = 0.f;

    if(width == 80 && height == 60)
    {
        focalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS / 4.f;
    }
    else if(width == 320 && height == 240)
    {
        focalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS;
    }
    else if(width == 640 && height == 480)
    {
        focalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * 2.f;
    }

    if(focalLength == 0.f)
    {
        return E_UNEXPECTED;
    }

    depthConfig->FocalLength = focalLength;
    depthConfig->Width = width;
    depthConfig->Height = height;

    return S_OK;
}
// We compute here the nominal "center of attention" that is used when zooming the presented image.
void CmykinectTestDlg::SetCenterOfImage(IFTResult* pResult)
{
    float centerX = ((float)pColorFrame->GetWidth())/2.0f;
    float centerY = ((float)pColorFrame->GetHeight())/2.0f;
    if (pResult)
    {
        if (SUCCEEDED(pResult->GetStatus()))
        {
            RECT faceRect;
            pResult->GetFaceRect(&faceRect);
            centerX = (faceRect.left+faceRect.right)/2.0f;
            centerY = (faceRect.top+faceRect.bottom)/2.0f;
        }
        m_XCenterFace += 0.02f*(centerX-m_XCenterFace);
        m_YCenterFace += 0.02f*(centerY-m_YCenterFace);
    }
    else
    {
        m_XCenterFace = centerX;
        m_YCenterFace = centerY;
    }
}

// Get a video image and process it.
void CmykinectTestDlg::GetHeadData()
{
    HRESULT hrFT = E_FAIL;
	// Do face tracking
    //if (SUCCEEDED(hrCopy))
    {
        FT_SENSOR_DATA sensorData( pColorFrame, pDepthImage,m_ZoomFactor,&m_ViewOffset);

         FT_VECTOR3D* hint = NULL;
        if (SUCCEEDED(GetClosestHint(m_hint3D)))
        {
            hint = m_hint3D;
        }
        if (m_LastTrackSucceeded)
        {
            hrFT = pFT->ContinueTracking(&sensorData, hint, pFTResult);
        }
        else
        {
            hrFT = pFT->StartTracking(&sensorData, NULL, hint, pFTResult);
        }
    }
    
    m_LastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(pFTResult->GetStatus());
    if (m_LastTrackSucceeded)
    {
        FLOAT* pSU = NULL;
        UINT numSU;
        BOOL suConverged;
        pFT->GetShapeUnits(NULL, &pSU, &numSU, &suConverged);
        POINT viewOffset = {0, 0};
        FT_CAMERA_CONFIG cameraConfig;
        if (m_KinectSensorPresent)
        {
            GetVideoConfiguration(&cameraConfig);
        }
        else
        {
            cameraConfig.Width = 640;
            cameraConfig.Height = 480;
            cameraConfig.FocalLength = 500.0f;
        }			
		pFTResult->Get3DPose(&m_scale,m_rotationXYZ,m_translationXYZ);

		/*IFTModel* ftModel;
		HRESULT hr = pFT->GetFaceModel(&ftModel);
		if (SUCCEEDED(hr))
		{
		hr = VisualizeFaceModel(pColorFrame, ftModel, &cameraConfig, pSU, 1.0, viewOffset, pFTResult, 0x00FFFF00);
		ftModel->Release();
		}*/
	   //HRESULT hr = VisualizeFacetracker(pColorFrame, pFTResult,0x00FFFF00);
    }
    else
    {
        pFTResult->Reset();
	}
    SetCenterOfImage(pFTResult);
}
HRESULT CmykinectTestDlg::GetClosestHint(FT_VECTOR3D* pHint3D)
{
    int selectedSkeleton = -1;
    float smallestDistance = 0;

    if (!pHint3D)
    {
        return(E_POINTER);
    }

    if (pHint3D[1].x == 0 && pHint3D[1].y == 0 && pHint3D[1].z == 0)
    {
        // Get the skeleton closest to the camera
        for (int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
        {
            if (m_SkeletonTracked[i] && (smallestDistance == 0 || m_HeadPoint[i].z < smallestDistance))
            {
                smallestDistance = m_HeadPoint[i].z;
                selectedSkeleton = i;
            }
        }
    }
    else
    {   // Get the skeleton closest to the previous position
        for (int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
        {
            if (m_SkeletonTracked[i])
            {
                float d = abs(m_HeadPoint[i].x - pHint3D[1].x) +
                    abs(m_HeadPoint[i].y - pHint3D[1].y) +
                    abs(m_HeadPoint[i].z - pHint3D[1].z);
                if (smallestDistance == 0 || d < smallestDistance)
                {
                    smallestDistance = d;
                    selectedSkeleton = i;
                }
            }
        }
    }
	
	if (selectedSkeleton == -1)
    {
        return E_FAIL;
    }

    pHint3D[0] = m_NeckPoint[selectedSkeleton];
    pHint3D[1] = m_HeadPoint[selectedSkeleton];

    return S_OK;
}
HRESULT CmykinectTestDlg::GetClosestSkeleton(FT_VECTOR3D* pHint3D,int &selectedSkeleton)
{
   
	float smallestDistance = 0;
	 selectedSkeleton = -1;
	if (!pHint3D)
	{
		return(E_POINTER);
	}

	if (pHint3D[1].x == 0 && pHint3D[1].y == 0 && pHint3D[1].z == 0)
	{
		// Get the skeleton closest to the camera
		for (int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
		{
			if (m_SkeletonTracked[i] && (smallestDistance == 0 || m_HeadPoint[i].z < smallestDistance))
			{
				smallestDistance = m_HeadPoint[i].z;
				selectedSkeleton = i;
			}
		}
	}
	else
	{   // Get the skeleton closest to the previous position
		for (int i = 0 ; i < NUI_SKELETON_COUNT ; i++ )
		{
			if (m_SkeletonTracked[i])
			{
				float d = abs(m_HeadPoint[i].x - pHint3D[1].x) +
					abs(m_HeadPoint[i].y - pHint3D[1].y) +
					abs(m_HeadPoint[i].z - pHint3D[1].z);
				if (smallestDistance == 0 || d < smallestDistance)
				{
					smallestDistance = d;
					selectedSkeleton = i;
				}
			}
		}
	}

	if (selectedSkeleton == -1)
	{
		return E_FAIL;
	}

	return S_OK;
}
BOOL CmykinectTestDlg::ShowImage(IFTImage* colorImage,CWnd* pMyWnd)
{
    BOOL ret = TRUE;
	if(pMyWnd==NULL)
		return false;
    CDC *m_pDC = pMyWnd->GetDC();//获取窗口所拥有的设备上下文，用于显示图像  
   
	CRect rc;  
    pMyWnd->GetWindowRect(&rc);  
	
	IFTImage*   pVideoBuffer;
	pVideoBuffer = FTCreateImage();
    int width = rc.Width();  
    int height = rc.Height();  
	 int originX=0;
	 int originY=0;
    if (colorImage)
    {
        int iWidth = colorImage->GetWidth();
        int iHeight = colorImage->GetHeight();
        if (iWidth > 0 && iHeight > 0)
        {
            int iTop = 0;
            int iBottom = iHeight;
            int iLeft = 0;
            int iRight = iWidth;

            // Keep a separate buffer.
            if (pVideoBuffer && SUCCEEDED(pVideoBuffer->Allocate(iWidth, iHeight, FTIMAGEFORMAT_UINT8_B8G8R8A8)))
            {
                // Copy do the video buffer while converting bytes
                colorImage->CopyTo(pVideoBuffer, NULL, 0, 0);

                // Compute the best approximate copy ratio.
                float w1 = (float)iHeight * (float)width;
                float w2 = (float)iWidth * (float)height;
                if (w2 > w1 && height > 0)
                {
                    // video image too wide
                    float wx = w1/height;
                    iLeft = (int)max(0, m_XCenterFace - wx / 2);
                    iRight = iLeft + (int)wx;
                    if (iRight > iWidth)
                    {
                        iRight = iWidth;
                        iLeft = iRight - (int)wx;
                    }
                }
                else if (w1 > w2 && width > 0)
                {
                    // video image too narrow
                    float hy = w2/width;
                    iTop = (int)max(0, m_YCenterFace - hy / 2);
                    iBottom = iTop + (int)hy;
                    if (iBottom > iHeight)
                    {
                        iBottom = iHeight;
                        iTop = iBottom - (int)hy;
                    }
                }

                int const bmpPixSize = pVideoBuffer->GetBytesPerPixel();
				SetStretchBltMode(m_pDC->GetSafeHdc() , HALFTONE);
                BITMAPINFO bmi = {sizeof(BITMAPINFO), iWidth, iHeight, 1, static_cast<WORD>(bmpPixSize * CHAR_BIT), BI_RGB, pVideoBuffer->GetStride() * iHeight, 5000, 5000, 0, 0};
                if (0 == StretchDIBits(m_pDC->GetSafeHdc(), originX, originY, width, height,
                    iLeft, iBottom, iRight-iLeft, iTop-iBottom, pVideoBuffer->GetBuffer(), &bmi, DIB_RGB_COLORS, SRCCOPY))
                {
                    ret = FALSE;
                }
            }
        }
    }
	if (pVideoBuffer)
    {
        pVideoBuffer->Release();
        pVideoBuffer = NULL;
    }
    return ret;
}

void CmykinectTestDlg::OnBnClickedOpenport()
{
	// TODO: Add your control notification handler code here
	CComPortSet dlg;
	if(IDOK==dlg.DoModal()){
	   m_Port=dlg.selectComPortID;
	}

	m_com.put_CommPort(m_Port);//设置串口
	m_com.put_InputMode(1);//设置数据读取格式为二进制
	m_com.put_Settings(_T("115200,n,8,1"));//传输参数
	m_com.put_RThreshold(1);//缓冲区内有一个字符就可以接收
	m_com.put_InBufferSize(4086);//接收缓冲区的大小2字节
	m_com.put_OutBufferSize(4086);//发送缓冲区大小2字节
	m_com.put_InBufferCount(0);//清空接收缓冲区
	if(!m_com.get_PortOpen())
	{
		m_com.put_PortOpen(true);//打开串口
		GetDlgItem(IDC_OPENPORT)->EnableWindow(FALSE);
		GetDlgItem(IDC_CLOSEPORT)->EnableWindow(TRUE);
		Is_open=true;


	}	
	else
		MessageBox(_T("Serial port already open!"),_T("Prompt"));

}


void CmykinectTestDlg::OnBnClickedCloseport()
{
	// TODO: Add your control notification handler code here

	if(m_com.get_PortOpen())
	{ 
	

		m_com.put_PortOpen(false);//关闭串口
	
		Is_open=false;
	

	}	

}


void CmykinectTestDlg::OnBnClickedButtonOnline()
{
	// TODO: Add your control notification handler code here
	if(Is_open==false)
	{
		MessageBox(_T("The serial port is not open！"),_T("Prompt"));
		return; 
	}
	m_RobotChControl.SetRobotOnline(&m_com);

}


void CmykinectTestDlg::OnBnClickedButtonOffline()
{
	// TODO: Add your control notification handler code here
	// TODO: Add your control notification handler code here
	if(Is_open==false)
	{
		MessageBox(_T("The serial port is not open！"),_T("Prompt"));
		return; 
	}
	m_RobotChControl.SetRobotOffline(&m_com);

}


void CmykinectTestDlg::OnBnClickedButtonReset()
{
	// TODO: Add your control notification handler code here
	if(Is_open==false)
	{
		MessageBox(_T("The serial port is not open！"),_T("Prompt"));
		return; 
	}
    m_RobotChControl.SetRobotReset(&m_com);
	
}

BEGIN_EVENTSINK_MAP(CmykinectTestDlg, CDialogEx)
	ON_EVENT(CmykinectTestDlg, IDC_MSCOMM1, 1, CmykinectTestDlg::OnCommMscomm1, VTS_NONE)
	END_EVENTSINK_MAP()


void CmykinectTestDlg::OnCommMscomm1()
{
	// TODO: Add your message handler code here
	//if(m_)
	VARIANT variant_inp;
	COleSafeArray safearray_inp;
	long len,k;
	byte rxdata[512];
	CString strtemp;
	if(m_com.get_CommEvent()==2){
		variant_inp=m_com.get_Input();
		safearray_inp=variant_inp;
		len=safearray_inp.GetOneDimSize();
		for(k=0;k<len;k++)
			safearray_inp.GetElement(&k,rxdata+k);
		for(k=0;k<len;k++)
		{
			char bt=*(char*)(rxdata+k);
			bt=bt+48;
			strtemp.Format(TEXT("%c"),bt);
			//strtemp.Format(_T("%c"),bt);
			IDC_EDIT_RXDATA+=strtemp;

		}


	}
	CString temp=(TEXT("\r\n"));
	IDC_EDIT_RXDATA+=temp;
	UpdateData(FALSE);
}




void CmykinectTestDlg::OnBnClickedMirro1()
{
	// TODO: Add your control notification handler code here
	UpdateData();
    m_bMirror=!m_bMirror;
	UpdateData(FALSE);
}


void CmykinectTestDlg::OnBnClickedBtnStartRecord()
{
	// TODO: Add your control notification handler code here
	//CFileDialog dlg(false,"TXT");
	CFileDialog dlg(FALSE, _T("TXT"), _T("hand"),OFN_OVERWRITEPROMPT|OFN_HIDEREADONLY,_T("*.txt||"), this);
	///dlg.m_ofn.lpstrTitle = _T("保存日记文件");
	if(IDOK==dlg.DoModal())
	{ 
       CString strName,strPath;
	   strPath=dlg.GetPathName();
	   m_os.open(strPath);
	   m_recordFrameNum=0;
	   m_bRecord=true;

	}
	
}


void CmykinectTestDlg::OnBnClickedBtnEndRecord()
{
	// TODO: Add your control notification handler code here
	if(m_bRecord)
	{
	   m_bRecord=false;
	   m_recordFrameNum=0;
	   m_os.close();
	}
}


void CmykinectTestDlg::OnEnChangeEditControlangle()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}


void CmykinectTestDlg::OnEnChangeEdit1()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialogEx::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}
