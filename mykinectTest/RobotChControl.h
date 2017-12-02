#pragma once
#include "NuiApi.h"
#include "cmscomm.h"

//机器人通道控制常量
#define  CH1_Eyebrows 0  //128,128,0,255
#define  CH2_Cheek    1  //64,64,0,255
#define	 CH3_Eyelid   2  //64,64,0,255
#define  CH4_EyeLR    3  //128,128,0,255
#define	 CH5_EyeUD    4  //128,128,0,255
#define	 CH6_Mouth    5  //64,64,0,255
#define	 CH7_MothShut 6  //32,32,0,255
#define	 CH8_NckTwist1 7  //128,128,0,255
#define	 CH9_NckTwist2 8  //128,128,0,255
#define	 CH10_NckLR    9  //128,128,0,255
#define	 CH11_NckUD    10 //128,128,0,255
#define	 CH12_LShldrFB 11 //0,0,0,255
#define	 CH13_LShldrUD 12 //0,0,0,255
#define	 CH14_LArmUD   13 //35,35,0,255
#define	 CH15_LShldrOC 14 //198,198,0,255
#define	 CH16_LUarm    15 //119,119,0,255
#define	 CH17_LElbw    16 //0,126,0,255
#define	 CH18_LFarm    17 //128,128,0,255
#define	 CH19_LWrist1  18 // 125,125,0,255
#define	 CH20_LWrist2  19 //125,125,0,255
#define	 CH21_LFinger1 20 //128,128,0,255
#define	 CH22_LFinger2 21 //128,128,0,255
#define	 CH23_RShldrFB 22 //0,0,0,255
#define	 CH24_RShldrUD 23 //0,0,0,255
#define	 CH25_RArmUD   24 //12,12,0,255
#define	 CH26_RShldrOC 25 //198,198,0,255
#define	 CH27_RUarm    26 //119,119,0,255
#define	 CH28_RElbw    27 //0,126,0,255
#define	 CH29_RFarm    28 //128,128,0,255
#define  CH30_RWrist1  29 //128,128,0,255
#define  CH31_RWrist2  30 //125,125,0,255
#define	 CH32_RFinger1 31 //128,128,0,255
#define	 CH33_RFinger2 32 //128,128,0,255
#define	 CH34_ChestFB  33   //0,0,0,255
#define	 CH35_BodyTilts 34  //125,125,0,255
#define	 CH36_BodyLR    35  //117,117,0,255
#define	 CH37_BodyFB    36  //155,155,0,255
#define	 CH38_WaistFB   37  //249,249,0,255
#define	 CH39_WaistLR   38  //0,0,0,255
#define	 CH40_WaistTilts 39 //79,79,0,255
#define	 CH41_RHipFB     40 //58,58,0,255
#define	 CH42_RHipOC     41 //81,81,0,255
#define	 CH43_RHipTwist  42 //76,76,0,255
#define	 CH44_RKnee      43 //0,0,0,255
#define	 CH45_WholeLR    44 //104,104,0,255
#define	 CH46_WholeFB    45 //86,86,0,255
#define	 CH47_WholeTilts 46 //0,0,0,255
#define	 CH48_NULL       47 //128,128,0,255

//机器人通信控制常量
#define  SM        0xFE   //在命令的最初始位置--起始符 
#define  EM        0x01   //在命令的最终端位置--终止符
#define	 ACK       0x06   //正确应答
#define  NAK       0x15   //非应答
#define	 ONLIN     0x55   //联机
#define	 OFLIN     0xDF   //下线
#define	 EMG0N     0x61   //紧急设置
#define	 EMG0F     0x62   //紧急解除
#define	 REFTX     0x74   //送通通道值 （发送）
#define	 INRQ      0x77   //读取通道值 （接收）
  
class CRobotChControl
{
public:
	bool m_bMirror;
	CRobotChControl(void);
	~CRobotChControl(void);

public:	
	//保留48个通道的值
	int m_oldChValue[48];
	//正常情况下动作值
	int m_normalAction[48];
	
	
	//人脸头部跟踪的几何特征，缩放比例、旋转、平移
	float m_scale;
	float m_rotationXYZ[3];
	float m_translationXYZ[3];
		////////////////////////////////////////////////

	//身体骨骼跟踪特征点坐标
	NUI_SKELETON_DATA m_skeletonData;
	

private:
	double LinearFunction(double x, double startx, double endx, double starty,double endy);
	void   SplitString(CString str, CString split,CStringArray& strGet);
/////////////////////////////////////////////////////////////////////////////////////////////////// 
//                                   表演者关节角度计算                                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
	//计算左右点头角度
	void CacluateHead(Vector4  &shoudlerC,Vector4  &head,double &headUpdown,double &headLeftRight,double &headTilt);

	//上半身动作,倾斜和旋转
	void CalculateUpbody(                      
		Vector4  &shoulderLeft,
		Vector4  &shoulderRight,
		double &upbodyTiltAngle,
		double &upbodyRotateAngle
		);
	//计算肩部动作（向前向上）
	void CalculateShoulder(Vector4  &shoudlerC,
		Vector4  &leftShoudler,
		Vector4  &rightShoudler,
		double &leftUpAngle,
		double &leftFrontAngle,
		double &rightUpAngle,
		double &rightFrontAngle
		);

	//计算左胳膊的角度
	void CalculateLeftArm( Vector4  &shoulder,
		                   Vector4  &elbow,
		                   Vector4  &hand,
						   double &front_Angle,
						   double &left_Angle,
						   double &elbow_Angle
		                 );
	//计算右胳膊的角度
	void CalculateRightArm( Vector4  &shoulder,
							Vector4  &elbow,
							Vector4  &hand,
							double &front_Angle,
							double &left_Angle,
							double &elbow_Angle
		);

	//计算腰的角度（前后、倾斜）
	void CalculateWaist(Vector4  &shoudlerC,
		Vector4  &spine,
		Vector4  &rightShoudler,
		Vector4  &leftHip,
		double &front_backAngle,
		double &tiltAngle
		);
	//计算腰的角度,控制不同通道
	void CalculateBow( Vector4  &shoudlerC,
		 Vector4  &spine,
		double &angle
		);
    //计算左腿的角度（前-踢腿、张开、角的旋转）
   void CalculateLeftLeg( Vector4  &hip,
						 Vector4  &knee,
						 Vector4  &ankle,
						 Vector4 &foot,
						 double &frontAngle,
						 double &leftRightAngle,
						 double &kneeAngle,
						 double &footAngle
					);
   //计算右腿的角度（前-踢腿、张开、角的旋转）
   void CalculateRightLeg( Vector4  &hip,
						   Vector4  &knee,
						   Vector4  &ankle,
						   Vector4 &foot,
						   double &frontAngle,
						   double &leftRightAngle,
						   double &kneeAngle,
						   double &footAngle
						   );


/////////////////////////////////////////////////////////////////////////////////////////////////// 
//                                     表演者向机器人动作映射                                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
   //头部通道控制，主要包括,[CH8]NckTwist1 [CH9]NckTwist2, [CH10]NckL/R,[CH11]NckU/D
   void MapHeadChanel(double updown,double leftright,double twist);
  
   //肩部通道控制，主要包括, CH12_LShldrFB  CH13_LShldrUD  CH23_RShldrFB CH24_RShldrUD   
   void MapShoulderChanel(double lShldrFB,double lShldrUD,double rShldrFB,double rShldrUD);
  
   //手臂通道控制，主要包括,  CH14_LArmUD ,CH15_LShldrOC ,CH17_LElbw ,CH25_RArmUD  CH26_RShldrOC CH28_RElbw  
   void MapArmChanel(double lArmUD,double lShldrOC,double lElbw,double rArmUD,double rShldrOC,double rElbw);
  
   //上半身通道控制，上半身倾斜和旋转。主要包括,  CH35_BodyTilts ,CH36_BodyLR  
   void MapUpbodyChanel(double upbodyTilt,double upbodyRotate);

   //腰部通道控制，主要包括,  	 CH37_BodyFB  CH38_WaistFB   CH40_WaistTilts 
   void MapWasitChanel(double waistFrontback,double waistRotate);

   //胸部通道控制，相当于鞠躬动作，这个动作跟CH37 CH38具有相似效果 主要包括,H34_ChestFB
   void MapBowChanel(double bowAngle);
 
   //右腿胸部通道控制，包括向前踢腿、向左张开、屈膝、脚摆动， 主要包括,CH41_RHipFB,CH42_RHipOC,CH44_RKnee,CH43_RHipTwist
   void MapRightLegChanel(double rHipFB,double rHipOC,double rRKnee,double rHipTwist);

public:  
///////////////////////////////////////////机器人控制///////////////////////////////////////////
	//从配置文件中读取机器人关节初始位置
   void ReadNormalPosition(CString filename);
   //机器人联机
   void SetRobotOnline(CMSComm *pMscomm);
   //机器人脱机
   void SetRobotOffline(CMSComm *pMscomm);
   //机器人复位
   void SetRobotReset(CMSComm *pMscomm);
   //向串口发送一帧数据（48个值，已经相关校验数据）
   int SendControlValueToComm(CMSComm *pMscomm,int sendData[],int chNum=48);
 //////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
   //接收头部跟踪的姿态坐标
   void SetHeadPose(float scale, float rotationXYZ[],float translationXYZ[]);
   //接收kinect发送的骨骼跟踪数据
   void SetSkeletonData(NUI_SKELETON_DATA skeletonData);
   //将表演者的动作映射到机器人
   void MapActionToRobot(CString &controlInfo,bool bMirror);
//////////////////////////////////////////////////////////////////////////////////////////////////
  
};

