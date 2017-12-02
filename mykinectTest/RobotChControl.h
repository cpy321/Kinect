#pragma once
#include "NuiApi.h"
#include "cmscomm.h"

//������ͨ�����Ƴ���
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

//������ͨ�ſ��Ƴ���
#define  SM        0xFE   //����������ʼλ��--��ʼ�� 
#define  EM        0x01   //����������ն�λ��--��ֹ��
#define	 ACK       0x06   //��ȷӦ��
#define  NAK       0x15   //��Ӧ��
#define	 ONLIN     0x55   //����
#define	 OFLIN     0xDF   //����
#define	 EMG0N     0x61   //��������
#define	 EMG0F     0x62   //�������
#define	 REFTX     0x74   //��ͨͨ��ֵ �����ͣ�
#define	 INRQ      0x77   //��ȡͨ��ֵ �����գ�
  
class CRobotChControl
{
public:
	bool m_bMirror;
	CRobotChControl(void);
	~CRobotChControl(void);

public:	
	//����48��ͨ����ֵ
	int m_oldChValue[48];
	//��������¶���ֵ
	int m_normalAction[48];
	
	
	//����ͷ�����ٵļ������������ű�������ת��ƽ��
	float m_scale;
	float m_rotationXYZ[3];
	float m_translationXYZ[3];
		////////////////////////////////////////////////

	//���������������������
	NUI_SKELETON_DATA m_skeletonData;
	

private:
	double LinearFunction(double x, double startx, double endx, double starty,double endy);
	void   SplitString(CString str, CString split,CStringArray& strGet);
/////////////////////////////////////////////////////////////////////////////////////////////////// 
//                                   �����߹ؽڽǶȼ���                                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
	//�������ҵ�ͷ�Ƕ�
	void CacluateHead(Vector4  &shoudlerC,Vector4  &head,double &headUpdown,double &headLeftRight,double &headTilt);

	//�ϰ�����,��б����ת
	void CalculateUpbody(                      
		Vector4  &shoulderLeft,
		Vector4  &shoulderRight,
		double &upbodyTiltAngle,
		double &upbodyRotateAngle
		);
	//����粿��������ǰ���ϣ�
	void CalculateShoulder(Vector4  &shoudlerC,
		Vector4  &leftShoudler,
		Vector4  &rightShoudler,
		double &leftUpAngle,
		double &leftFrontAngle,
		double &rightUpAngle,
		double &rightFrontAngle
		);

	//������첲�ĽǶ�
	void CalculateLeftArm( Vector4  &shoulder,
		                   Vector4  &elbow,
		                   Vector4  &hand,
						   double &front_Angle,
						   double &left_Angle,
						   double &elbow_Angle
		                 );
	//�����Ҹ첲�ĽǶ�
	void CalculateRightArm( Vector4  &shoulder,
							Vector4  &elbow,
							Vector4  &hand,
							double &front_Angle,
							double &left_Angle,
							double &elbow_Angle
		);

	//�������ĽǶȣ�ǰ����б��
	void CalculateWaist(Vector4  &shoudlerC,
		Vector4  &spine,
		Vector4  &rightShoudler,
		Vector4  &leftHip,
		double &front_backAngle,
		double &tiltAngle
		);
	//�������ĽǶ�,���Ʋ�ͬͨ��
	void CalculateBow( Vector4  &shoudlerC,
		 Vector4  &spine,
		double &angle
		);
    //�������ȵĽǶȣ�ǰ-���ȡ��ſ����ǵ���ת��
   void CalculateLeftLeg( Vector4  &hip,
						 Vector4  &knee,
						 Vector4  &ankle,
						 Vector4 &foot,
						 double &frontAngle,
						 double &leftRightAngle,
						 double &kneeAngle,
						 double &footAngle
					);
   //�������ȵĽǶȣ�ǰ-���ȡ��ſ����ǵ���ת��
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
//                                     ������������˶���ӳ��                                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
   //ͷ��ͨ�����ƣ���Ҫ����,[CH8]NckTwist1 [CH9]NckTwist2, [CH10]NckL/R,[CH11]NckU/D
   void MapHeadChanel(double updown,double leftright,double twist);
  
   //�粿ͨ�����ƣ���Ҫ����, CH12_LShldrFB  CH13_LShldrUD  CH23_RShldrFB CH24_RShldrUD   
   void MapShoulderChanel(double lShldrFB,double lShldrUD,double rShldrFB,double rShldrUD);
  
   //�ֱ�ͨ�����ƣ���Ҫ����,  CH14_LArmUD ,CH15_LShldrOC ,CH17_LElbw ,CH25_RArmUD  CH26_RShldrOC CH28_RElbw  
   void MapArmChanel(double lArmUD,double lShldrOC,double lElbw,double rArmUD,double rShldrOC,double rElbw);
  
   //�ϰ���ͨ�����ƣ��ϰ�����б����ת����Ҫ����,  CH35_BodyTilts ,CH36_BodyLR  
   void MapUpbodyChanel(double upbodyTilt,double upbodyRotate);

   //����ͨ�����ƣ���Ҫ����,  	 CH37_BodyFB  CH38_WaistFB   CH40_WaistTilts 
   void MapWasitChanel(double waistFrontback,double waistRotate);

   //�ز�ͨ�����ƣ��൱�ھϹ����������������CH37 CH38��������Ч�� ��Ҫ����,H34_ChestFB
   void MapBowChanel(double bowAngle);
 
   //�����ز�ͨ�����ƣ�������ǰ���ȡ������ſ�����ϥ���Űڶ��� ��Ҫ����,CH41_RHipFB,CH42_RHipOC,CH44_RKnee,CH43_RHipTwist
   void MapRightLegChanel(double rHipFB,double rHipOC,double rRKnee,double rHipTwist);

public:  
///////////////////////////////////////////�����˿���///////////////////////////////////////////
	//�������ļ��ж�ȡ�����˹ؽڳ�ʼλ��
   void ReadNormalPosition(CString filename);
   //����������
   void SetRobotOnline(CMSComm *pMscomm);
   //�������ѻ�
   void SetRobotOffline(CMSComm *pMscomm);
   //�����˸�λ
   void SetRobotReset(CMSComm *pMscomm);
   //�򴮿ڷ���һ֡���ݣ�48��ֵ���Ѿ����У�����ݣ�
   int SendControlValueToComm(CMSComm *pMscomm,int sendData[],int chNum=48);
 //////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////
   //����ͷ�����ٵ���̬����
   void SetHeadPose(float scale, float rotationXYZ[],float translationXYZ[]);
   //����kinect���͵Ĺ�����������
   void SetSkeletonData(NUI_SKELETON_DATA skeletonData);
   //�������ߵĶ���ӳ�䵽������
   void MapActionToRobot(CString &controlInfo,bool bMirror);
//////////////////////////////////////////////////////////////////////////////////////////////////
  
};

