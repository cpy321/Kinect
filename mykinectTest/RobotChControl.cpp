#include "StdAfx.h"
#include "RobotChControl.h"
#include "math.h"
#ifndef max
#define max(a,b)        (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#define PI 3.1415

/// @brief   空间点的实现
struct MyVector3D
{
	double X;
	double Y;
	double Z;
};

CRobotChControl::CRobotChControl(void)
{
  m_bMirror=true;
}


CRobotChControl::~CRobotChControl(void)
{
}

void CRobotChControl::ReadNormalPosition(CString filename)
{
	//读取机器人正常情况下各通道缺省动作值
	CStdioFile file;

	file.Open(filename,CFile::modeRead);

	//逐行读取字符串
	CString strText = "";
	CString szLine = "";

	while( file.ReadString( szLine ) )		
	{
		CStringArray strArray;
		if(szLine.Find(",")<0)
			continue;
		SplitString(szLine,",",strArray);
		int count = strArray.GetSize();
		for(int i=0;i<count;i++)
		{
			m_normalAction[i]=atoi(strArray[i]);

		}
		break;
	}	
	file.Close();
	 
}
//字符串分隔
void CRobotChControl::SplitString(CString str, CString split,CStringArray& strGet) 
{ 
	int pos = -1;
	pos = str.Find(split);
	while(pos != -1)
	{
		CString strSun = "";
		strSun = str.Left(pos);
		if(strSun.Find(":")>=0)
			;
		else 
			strGet.Add(strSun);
		str.Delete(0,pos+1);
		pos = str.Find(split);
	}
	if(str != "")
	{//最后剩下的字符串可能没有split这个标示，看你需要怎么处理了
		//这里也把他加入到strGet中
		strGet.Add(str);
	}
}
//机器人联机
void CRobotChControl::SetRobotOnline(CMSComm *pMscomm)
{
	CByteArray byteArray;//二进制
	byteArray.RemoveAll();
	byteArray.SetSize(3);
	byteArray.SetAt(0,SM);
	byteArray.SetAt(1,ONLIN);
	byteArray.SetAt(2,EM);
	pMscomm->put_Output(COleVariant(byteArray));//发送累加后的发送数据的校验和
}
//机器人脱机
void CRobotChControl::SetRobotOffline(CMSComm *pMscomm)
{
	// TODO: Add your control notification handler code here
	CByteArray byteArray;//二进制数组
	byteArray.RemoveAll();
	byteArray.SetSize(3);
	byteArray.SetAt(0,SM);
	byteArray.SetAt(1,OFLIN);
	byteArray.SetAt(2,EM);
	pMscomm->put_Output(COleVariant(byteArray));//发送累加后的发送数据的校验和
}
//机器人复位
void CRobotChControl::SetRobotReset(CMSComm *pMscomm)
{
	SendControlValueToComm(pMscomm,m_normalAction,48);
}
//线性映射函数
double CRobotChControl::LinearFunction(double x, double startx, double endx, double starty,double endy)
{
	return (x-startx)/(endx-startx)*(endy-starty)+starty;
}

//通过串口发送数据，注意通信协议
int CRobotChControl::SendControlValueToComm(CMSComm *pMscomm, int sendData[],int chNum)
{
	int count;
	count=chNum+5;
	CByteArray byteArray;//二进制数组
	byteArray.SetSize(count);
	byteArray.SetAt(0,SM);//起始符
	byteArray.SetAt(1,REFTX);
	byteArray.SetAt(2,chNum);//通道总数目

	BYTE sum,k,tempData;
	sum=chNum;

	for(int j=0;j<chNum;j++){
		if(sendData[j]<0)
			sendData[j]=0;
		else if(sendData[j]>255)
			 sendData[j]=255;
		byteArray.SetAt(3+j,sendData[j]);
		k=sendData[j];////计算发送数据的校验和
		sum=sum+k;
	} //组合每个通道值数据

	sum^=0xFF;
	sum++; 
	byteArray.SetAt(count-2,sum);//累加后的发送数据的校验和
	byteArray.SetAt(count-1,EM);//结束符

	pMscomm->put_Output(COleVariant(byteArray));//发送结束符

	//保留正在发送的数据
	for(int j=0;j<chNum;j++)
		m_oldChValue[j]=sendData[j];
	return 1;

}
//将kinect 跟踪的骨骼数据发送给机器人控制模块	
void CRobotChControl::SetSkeletonData(NUI_SKELETON_DATA skeletonData)
{
	m_skeletonData.eTrackingState=skeletonData.eTrackingState;
	m_skeletonData.dwTrackingID=skeletonData.dwTrackingID;
	m_skeletonData.dwEnrollmentIndex=skeletonData.dwEnrollmentIndex;
	m_skeletonData.dwUserIndex=skeletonData.dwUserIndex;
	m_skeletonData.Position=skeletonData.Position;
	for(int i=0;i<20;i++)
	  m_skeletonData.SkeletonPositions[i]=skeletonData.SkeletonPositions[i];
	for(int i=0;i<20;i++)
		m_skeletonData.eSkeletonPositionTrackingState[i]=skeletonData.eSkeletonPositionTrackingState[i];
	m_skeletonData.dwQualityFlags=skeletonData.dwQualityFlags;
}
//将kinect跟踪的头部姿态数据发送给机器人控制模块
void CRobotChControl::SetHeadPose(float scale, float rotationXYZ[],float translationXYZ[])
{
   m_scale=scale;
   for(int i=0;i<3;i++)
	   m_rotationXYZ[i]=rotationXYZ[i];
   for(int i=0;i<3;i++)
	   translationXYZ[i]=translationXYZ[i];
}

////////////////////////////////////////////////////////////////////////////////
// 计算表演者关节角度
////////////////////////////////////////////////////////////////////////////////
/**
 * @brief   计算表演者的左臂上3个舵机的角度.
 * @ingroup SceneDrawer
 * 
 * @param   shoulder 左肩膀的坐标
 * @param   elbow 左肘关节的坐标
 * @param   hand 左手(腕)的坐标
 * 
 * @details 所有坐标均采用向量法进行计算.
 * @param   frong_Angle 手向前伸
 * @param   left_Angle  手左右张开
 * @param   elbow_Angle  肘关节角度
 */
 void CRobotChControl::CalculateLeftArm( Vector4  &shoulder,
						Vector4  &elbow,
						Vector4  &hand,
						double &front_Angle,
						double &left_Angle,
						double &elbow_Angle
					)
{
		MyVector3D vector1;
		MyVector3D vector2;
		
		MyVector3D normal1;
		MyVector3D normal2;
		
		double deltaNormal1;
		double deltaNormal2;
		double deltaVector1;
		double deltaVector2;
		double cosAngle;

		  //沿着Z轴运动
        vector1.X = 0;
        vector1.Y =elbow.y - shoulder.y;
        vector1.Z = elbow.z - shoulder.z;

        vector2.X = 0.0;
        vector2.Y =0;
        vector2.Z =  100.00;
        
        deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
        deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

        if (deltaVector1 * deltaVector2 > 0.0)
        {
            cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			front_Angle=180-acos(cosAngle)*180.0/PI;
			
        }else
			front_Angle=-1;
		
			// g_controlRobot.RobotJointAngle[ROBOT_LEFT_SHOULDER_VERTICAL] = INVALID_JOINT_VALUE;
        
		//沿着X轴运动
        vector1.X = elbow.x - shoulder.x;
        vector1.Y = elbow.y - shoulder.y;
        vector1.Z = 0.0;

     	vector2.X =100.0;
        vector2.Y = 0;
        vector2.Z = 0.0;

        
        deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
        deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

        if (deltaVector1 * deltaVector2 > 0.0)
        {
            cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			left_Angle=180-acos(cosAngle)*180.0/PI;			
        }
        else
             left_Angle=-1;
			//g_controlRobot.RobotJointAngle[ROBOT_LEFT_SHOULDER_HORIZEN] = INVALID_JOINT_VALUE;

		//计算肘关节
        vector1.X = shoulder.x - elbow.x;
        vector1.Y = shoulder.y - elbow.y;
        vector1.Z = elbow.z - shoulder.z;

        vector2.X = hand.x - elbow.x;
        vector2.Y = hand.y - elbow.y;
        vector2.Z = elbow.z - hand.z;   

        deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
        deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

        if (deltaVector1 * deltaVector2 > 0.0)
        {
            cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			elbow_Angle=180-acos(cosAngle)*180.0/PI;
         }
        else
			elbow_Angle=-1;
                 
}
/**
 * @brief   计算表演者的右臂上3个舵机的角度.
 * @ingroup SceneDrawer 
 * @param   shoulder 右肩膀的坐标
 * @param   elbow 右肘关节的坐标
 * @param   hand 右手(腕)的坐标
 * 
 * @details 所有坐标均采用向量法进行计算.
 * @ret   frong_Angle 手向前伸
 * @ret   left_Angle  手左右张开
 * @ret   elbow_Angle  肘关节角度
 */

 void CRobotChControl::CalculateRightArm( Vector4  &shoulder,
						Vector4  &elbow,
						Vector4  &hand,
						double &front_Angle,
						double &left_Angle,
						double &elbow_Angle
					)
{
		MyVector3D vector1;
		MyVector3D vector2;
		
		MyVector3D normal1;
		MyVector3D normal2;
		
		double deltaNormal1;
		double deltaNormal2;
		double deltaVector1;
		double deltaVector2;
		double cosAngle;

	
		  //沿着Z轴运动
        vector1.X = 0;
        vector1.Y =elbow.y - shoulder.y;
        vector1.Z = elbow.z - shoulder.z;

        vector2.X = 0.0;
        vector2.Y =0;
        vector2.Z =  100.00;
        
        deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
        deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

        if (deltaVector1 * deltaVector2 > 0.0)
        {
            cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			front_Angle=180-acos(cosAngle)*180.0/PI;
			
        }else
			front_Angle=-1;
		
			// g_controlRobot.RobotJointAngle[ROBOT_LEFT_SHOULDER_VERTICAL] = INVALID_JOINT_VALUE;
        
		//沿着X轴运动
        vector1.X = elbow.x - shoulder.x;
        vector1.Y = elbow.y - shoulder.y;
        vector1.Z = 0.0;

        /*vector2.X = 0.0;
        vector2.Y = 100;
        vector2.Z = 0.0;*/

		vector2.X =-100.0;
        vector2.Y = 0;
        vector2.Z = 0.0;

        
        deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
        deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

        if (deltaVector1 * deltaVector2 > 0.0)
        {
            cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			left_Angle=180-acos(cosAngle)*180.0/PI;			
        }
        else
             left_Angle=-1;
		
		//计算肘关节
        vector1.X = shoulder.x - elbow.x;
        vector1.Y = shoulder.y - elbow.y;
        vector1.Z = elbow.z - shoulder.z;

        vector2.X = hand.x - elbow.x;
        vector2.Y = hand.y - elbow.y;
        vector2.Z = elbow.z - hand.z;   

        deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
        deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

        if (deltaVector1 * deltaVector2 > 0.0)
        {
            cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			elbow_Angle=180-acos(cosAngle)*180.0/PI;
         }
        else
			elbow_Angle=-1;
                 
}
 /**
 * @brief   计算表演者的左腿角度.
 * @param  hip   左胯骨的坐标
 * @param  knee  左膝关节的坐标
 * @param  ankle 左踝关节的坐标
 * @param  foot  左脚关节的坐标

 * @ret    frontAngle 腿向前伸
 * @ret    leftRightAngle  腿左右张开
 * @ret    kneeAngle  膝关节角度
 * @ret    footAngle  踝关节角度
 */
  void CRobotChControl::CalculateLeftLeg( Vector4  &hip,
						 Vector4  &knee,
						 Vector4  &ankle,
						 Vector4 &foot,
						 double &frontAngle,
						 double &leftRightAngle,
						 double &kneeAngle,
						 double &footAngle
					)
  {
	  MyVector3D vector1;
	  MyVector3D vector2;

	  MyVector3D normal1;
	  MyVector3D normal2;

	  double deltaNormal1;
	  double deltaNormal2;
	  double deltaVector1;
	  double deltaVector2;
	  double cosAngle;

	 
	  //沿着Z轴运动 相当于踢腿动作,垂直站立为90度
	  vector1.X = 0;
	  vector1.Y =knee.y - hip.y;
	  vector1.Z = knee.z - hip.z;

	  vector2.X = 0.0;
	  vector2.Y = 0;
	  vector2.Z = -100;
	  deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	  deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	  if (deltaVector1 * deltaVector2 > 0.0)
	  {
		  cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
		  frontAngle=acos(cosAngle)*180.0/PI;

	  }else
		  frontAngle=-1;

	 //沿着X轴运动
	  vector1.X = knee.x - hip.x;
	  vector1.Y = knee.y - hip.y;
	  vector1.Z = 0.0;

	  vector2.X = 0.0;
	  vector2.Y = -100;
	  vector2.Z = 0.0;

	  deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	  deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	  if (deltaVector1 * deltaVector2 > 0.0)
	  {
		  cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
		  leftRightAngle=acos(cosAngle)*180.0/PI;			
	  }
	  else
		  leftRightAngle=-1;
	 

	  //计算knee关节
	  vector1.X = hip.x - knee.x;
	  vector1.Y = hip.y - knee.y;
	  vector1.Z = hip.z-knee.z;

	  vector2.X = ankle.x - knee.x;
	  vector2.Y = ankle.y - knee.y;
	  vector2.Z = ankle.z-knee.z ;   

	  deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	  deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	  if (deltaVector1 * deltaVector2 > 0.0)
	  {
		  cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
		  kneeAngle=180-acos(cosAngle)*180.0/PI;
	  }
	  else
		  kneeAngle=-1;

	  //沿着foot的运动 相当于脚的旋转动作（与z轴关系）
	  vector1.X = foot.x-ankle.x;
	  vector1.Y =0;//foot.y - ankle.y;
	  vector1.Z = foot.z - ankle.z;

	  vector2.X = -100.0;
	  vector2.Y = 0;
	  vector2.Z = 0;
	  deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	  deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	  if (deltaVector1 * deltaVector2 > 0.0)
	  {
		  cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
		  footAngle=acos(cosAngle)*180.0/PI;

	  }else
		  footAngle=-1;
   
  
  }

						  /**
 * @brief   计算表演者的右腿角度.
 * @param  hip   右胯骨的坐标
 * @param  knee  右膝关节的坐标
 * @param  ankle 右踝关节的坐标
 * @param  foot  右脚关节的坐标

 * @ret    frontAngle 腿向前伸
 * @ret    leftRightAngle  腿左右张开
 * @ret    kneeAngle  膝关节角度
 * @ret    footAngle  踝关节角度
 */

  void CRobotChControl::CalculateRightLeg( Vector4  &hip,
						 Vector4  &knee,
						 Vector4  &ankle,
						 Vector4 &foot,
						 double &frontAngle,
						 double &leftRightAngle,
						 double &kneeAngle,
						 double &footAngle
					)
  {
	  MyVector3D vector1;
	  MyVector3D vector2;

	  MyVector3D normal1;
	  MyVector3D normal2;

	  double deltaNormal1;
	  double deltaNormal2;
	  double deltaVector1;
	  double deltaVector2;
	  double cosAngle;

	 
	  //沿着Z轴运动 相当于踢腿动作,垂直站立为90度
	  vector1.X = 0;
	  vector1.Y =knee.y - hip.y;
	  vector1.Z = knee.z - hip.z;

	  vector2.X = 0.0;
	  vector2.Y = 0;
	  vector2.Z = -100;
	  deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	  deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	  if (deltaVector1 * deltaVector2 > 0.0)
	  {
		  cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
		  frontAngle=acos(cosAngle)*180.0/PI;

	  }else
		  frontAngle=-1;

	 //沿着X轴运动
	  vector1.X = knee.x - hip.x;
	  vector1.Y = knee.y - hip.y;
	  vector1.Z = 0.0;

	  vector2.X = 0.0;
	  vector2.Y = -100;
	  vector2.Z = 0.0;

	  deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	  deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	  if (deltaVector1 * deltaVector2 > 0.0)
	  {
		  cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
		  leftRightAngle=acos(cosAngle)*180.0/PI;			
	  }
	  else
		  leftRightAngle=-1;
	 

	  //计算knee关节
	  vector1.X = hip.x - knee.x;
	  vector1.Y = hip.y - knee.y;
	  vector1.Z = hip.z-knee.z;

	  vector2.X = ankle.x - knee.x;
	  vector2.Y = ankle.y - knee.y;
	  vector2.Z = ankle.z-knee.z ;   

	  deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	  deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	  if (deltaVector1 * deltaVector2 > 0.0)
	  {
		  cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
		  kneeAngle=180-acos(cosAngle)*180.0/PI;
	  }
	  else
		  kneeAngle=-1;

	  //沿着foot的运动 相当于脚的旋转动作（与z轴关系）
	  vector1.X = foot.x-ankle.x;
	  vector1.Y =0;//foot.y - ankle.y;
	  vector1.Z = foot.z - ankle.z;

	  vector2.X = 100.0;
	  vector2.Y = 0;
	  vector2.Z = 0;
	  deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	  deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	  if (deltaVector1 * deltaVector2 > 0.0)
	  {
		  cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
		  footAngle=acos(cosAngle)*180.0/PI;

	  }else
		  footAngle=-1;
   
  
  }


/**
 * @brief   计算表演者鞠躬的角度.
 * @param   shoudler center 肩中心坐标
 * @param   spine 脊梁的坐标

 * @ret    angle 鞠躬的角度
 */
   void CRobotChControl::CalculateBow( Vector4  &shoudlerC,
				      Vector4  &spine,
				      double &angle
					  )
   {

		MyVector3D vector1;
		MyVector3D vector2;

		double deltaVector1;
		double deltaVector2;
		double cosAngle;


		//沿着Z轴运动 相当于鞠躬动作,正常为0度
		vector1.X = shoudlerC.x-spine.x;
		vector1.Y =shoudlerC.y-spine.y ;
		vector1.Z = shoudlerC.z - spine.z;

		vector2.X = 0.0;
		vector2.Y = 100;
		vector2.Z = 0;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			angle=acos(cosAngle)*180.0/PI;

		}else
			angle=-1;

   }
  
   /**
 * @brief   计算表演者上半身动作
 * @param   leftShoudler 左肩中坐标
 * @param   rightShoudler 右肩中坐标
 * @ret   upbodyTiltAngle  上半身倾斜
 * @ret   waistRotateAngle 上半身旋转

 */
  void CRobotChControl::CalculateUpbody(                      
	   Vector4  &shoulderLeft,
	   Vector4  &shoulderRight,
	   double &upbodyTiltAngle,
	   double &upbodyRotateAngle
	   )
   {
	   	MyVector3D vector1;
		MyVector3D vector2;
		
		MyVector3D normal1;
		MyVector3D normal2;
		
		double deltaNormal1;
		double deltaNormal2;
		double deltaVector1;
		double deltaVector2;
		double cosAngle;

	
       //沿着y轴运动  上半身倾斜
        vector1.X =shoulderLeft.x-shoulderRight.x;
        vector1.Y =shoulderLeft.y-shoulderRight.y;
        vector1.Z = 0.0;

        vector2.X = 0.0;
        vector2.Y =100;
        vector2.Z =  0.00;
        
        deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
        deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

        if (deltaVector1 * deltaVector2 > 0.0)
        {
            cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			upbodyTiltAngle=acos(cosAngle)*180.0/PI;
			
        }else
			upbodyTiltAngle=-1;
		
			// g_controlRobot.RobotJointAngle[ROBOT_LEFT_SHOULDER_VERTICAL] = INVALID_JOINT_VALUE;
        
		//沿着z轴运动
		vector1.X =shoulderLeft.x-shoulderRight.x;
		vector1.Y =0;
		vector1.Z =shoulderLeft.z-shoulderRight.z;

        /*vector2.X = 0.0;
        vector2.Y = 100;
        vector2.Z = 0.0;*/

		vector2.X =0;
        vector2.Y = 0;
        vector2.Z = 100.0;

        
        deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
        deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

        if (deltaVector1 * deltaVector2 > 0.0)
        {
            cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			upbodyRotateAngle=acos(cosAngle)*180.0/PI;			
        }
        else
             upbodyRotateAngle=-1;		

   }

   /**
 * @brief   计算表演者计算肩向前向上角度
 * @param   shoudlerC 肩中心坐标
 * @param   leftShoudler 左肩中坐标
 * @param   rightShoudler 右肩中坐标
 * @ret   leftUpAngle      肩向上
 * @ret   leftFrontAngle   肩向前
 * @ret   rightUpAngle     肩向上
 * @ret   rightFrontAngle  肩向前
 */
   void CRobotChControl::CalculateShoulder(Vector4  &shoudlerC,
	   Vector4  &leftShoudler,
	   Vector4  &rightShoudler,
	   double &leftUpAngle,
	   double &leftFrontAngle,
	   double &rightUpAngle,
	   double &rightFrontAngle
	   )
   {
	   MyVector3D vector1;
	   MyVector3D vector2;

	   double deltaVector1;
	   double deltaVector2;
	   double cosAngle;


	   //沿着左 肩部前倾和耸肩
	   vector1.X = leftShoudler.x-shoudlerC.x;
	   vector1.Y =leftShoudler.y-shoudlerC.y ;
	   vector1.Z = 0;

	   vector2.X = 0;
	   vector2.Y = 100.0;
	   vector2.Z = 0;

	   deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	   deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	   if (deltaVector1 * deltaVector2 > 0.0)
	   {
		   cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);

		   leftUpAngle=acos(cosAngle)*180.0/PI;

	   }else
		   leftUpAngle=-1;

	   vector1.X = leftShoudler.x-shoudlerC.x ;
	   vector1.Y =0 ;
	   vector1.Z = leftShoudler.z-shoudlerC.z;

	   vector2.X = 100;
	   vector2.Y = 0.0;
	   vector2.Z = 0;

	   deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	   deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	   if (deltaVector1 * deltaVector2 > 0.0)
	   {
		   cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);

		   leftFrontAngle=acos(cosAngle)*180.0/PI;

	   }else
		   leftFrontAngle=-1;

	   //沿着右 肩部前倾和耸肩
	   vector1.X = rightShoudler.x-shoudlerC.x;
	   vector1.Y =rightShoudler.y-shoudlerC.y ;
	   vector1.Z = 0;

	   vector2.X = 0;
	   vector2.Y = 100.0;
	   vector2.Z = 0;

	   deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	   deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	   if (deltaVector1 * deltaVector2 > 0.0)
	   {
		   cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);

		   rightUpAngle=acos(cosAngle)*180.0/PI;

	   }else
		   rightUpAngle=-1;

	   vector1.X = rightShoudler.x-shoudlerC.x ;
	   vector1.Y =0 ;
	   vector1.Z = rightShoudler.z-shoudlerC.z;

	   vector2.X = -100;
	   vector2.Y = 0.0;
	   vector2.Z = 0;

	   deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	   deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	   if (deltaVector1 * deltaVector2 > 0.0)
	   {
		   cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);

		   rightFrontAngle=acos(cosAngle)*180.0/PI;

	   }else
		   rightFrontAngle=-1;
   }
/**
  //计算表演者头部左右偏的角度
 * @brief   计算表演者头的角度.

 * @param   shoudler center 肩中心坐标
 * @param   head 头部坐标
   @ret    headUpdown 头部上下的角度
   @ret    headLeftRight 头部左右的角度
   @ret    headTilt 头部左右偏的角度
 */
   void CRobotChControl::CacluateHead(Vector4  &shoudlerC,
	               Vector4  &head,double &headUpdown,double &headLeftRight,double &headTilt)
   {
	   headUpdown=m_rotationXYZ[0];
	   headLeftRight=m_rotationXYZ[1];

	   MyVector3D vector1;
	   MyVector3D vector2;

	   double deltaVector1;
	   double deltaVector2;
	   double cosAngle;


	   //头部左右偏的角度
	   vector1.X = head.x-shoudlerC.x;
	   vector1.Y =head.y-shoudlerC.y ;
	   vector1.Z = 0;

	   vector2.X = 100.0;
	   vector2.Y = 0;
	   vector2.Z = 0;

	   deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
	   deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

	   if (deltaVector1 * deltaVector2 > 0.0)
	   {
		   cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);

		   headTilt=acos(cosAngle)*180.0/PI;

	   }else
		   headTilt=-1;

   }
   ////////////////////////////////////////////////////////////////////////////////
/**
 * @brief   计算表演者腰部角度.
 * @param   shoudler center 肩中心坐标
 * @param   spine 脊梁的坐标
 * @param    rightShoudler 右肩坐标
 * @param   leftHip      左胯骨坐标
 * @ret      front_backAngle 前后角度
 * @ret      tiltAngle       倾斜角度（通过右肩和左胯骨计算）
 */
 void CRobotChControl::CalculateWaist( Vector4  &shoudlerC,
				    Vector4  &spine,
					Vector4  &rightShoudler,
					Vector4  &leftHip,
				    double &front_backAngle,
					double &tiltAngle
					  )
{
		MyVector3D vector1;
		MyVector3D vector2;
		
		double deltaVector1;
		double deltaVector2;
		double cosAngle;

		       
		//沿着腰部前倾和后仰,正常为255度，注意对应的是关节37,38
		vector1.X = shoudlerC.x-spine.x;
        vector1.Y =shoudlerC.y-spine.y ;
        vector1.Z = shoudlerC.z - spine.z;

        vector2.X = 0.0;
        vector2.Y = 0;
        vector2.Z = 100;

        deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
        deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

        if (deltaVector1 * deltaVector2 > 0.0)
        {
            cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);
			
			front_backAngle=acos(cosAngle)*180.0/PI;
			
        }else
			front_backAngle=-1;

		//沿着腰部倾斜,正常为40度，注意对应的是关节40
		vector1.X = rightShoudler.x-leftHip.x;
		vector1.Y =rightShoudler.y-leftHip.y ;
		vector1.Z = rightShoudler.z - leftHip.z;

		vector2.X = 100.0;
		vector2.Y = 0;
		vector2.Z = 0;

		deltaVector1 = sqrt(vector1.X * vector1.X + vector1.Y * vector1.Y + vector1.Z * vector1.Z);
		deltaVector2 = sqrt(vector2.X * vector2.X + vector2.Y * vector2.Y + vector2.Z * vector2.Z);

		if (deltaVector1 * deltaVector2 > 0.0)
		{
			cosAngle = (vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z) / (deltaVector1 * deltaVector2);

			tiltAngle=acos(cosAngle)*180.0/PI;

		}else
			tiltAngle=-1;

}
  ////////////////////////////////////////////////////////////////////////////////
/**
 * @brief   机器人头部通道控制，主要包括,[CH8]NckTwist1 [CH9]NckTwist2, [CH10]NckL/R,[CH11]NckU/D
 * @param  updown    头上下角度
 * @param  leftright 头左右角度
 * @param    twist   头左右点头
 */
 
 void CRobotChControl::MapHeadChanel(double updown,double leftright,double twist)
 {
	 
	 //左右点头
	 if(twist>110)
		 twist=110;
	 if(twist<70)
		 twist=70;
	 if(twist>90)
	 { 
		if(m_bMirror) 
			 m_oldChValue[CH8_NckTwist1]=LinearFunction(twist,110,90,0,128);
		else
			m_oldChValue[CH8_NckTwist1]=LinearFunction(twist,110,90,255,128);

		m_oldChValue[CH9_NckTwist2]=128;
	 }			   
	 else
	 {
		 if(m_bMirror)
			 m_oldChValue[CH9_NckTwist2]=LinearFunction(twist,90,70,128,0);
		 else
			  m_oldChValue[CH9_NckTwist2]=LinearFunction(twist,90,70,128,255);
		 m_oldChValue[CH8_NckTwist1]=128;
	 }

	 //头左右
	 if(leftright>45)
		 leftright=45;
	 if(leftright<-45)
		 leftright=-45;
	 if(m_bMirror) 
           m_oldChValue[CH10_NckLR]=LinearFunction(leftright,45,-45,255,0);
	 else
		m_oldChValue[CH10_NckLR]=LinearFunction(leftright,-45,45,255,0);		
	
	 //头上下
	 if(updown>5)
		 updown=5;
	 if(updown<-40)
		 updown=-40;
	 if(updown<-15)
		 m_oldChValue[CH11_NckUD]=LinearFunction(updown,-40,-15,255,128);			   
	 else
		 m_oldChValue[CH11_NckUD]=LinearFunction(updown,-15,5,128,0);				   

	 //if(c7>35)
	 //	c7=35;
	 //if(c7<-35)
	 //	c7=-35;
	 //if(c7<0)
	 //{ 
	 //	if(c7<-10)
	 //		m_oldChValue[7]=(c7+40)/30.0*(-127)+255;
	 //	else
	 //		m_oldChValue[7]=(c7+10)/10.0*(-128)+128;				
	 //	m_oldChValue[8]=128;
	 //
	 //}			   
	 //else
	 //{
	 //}

 }

 //肩部通道控制，主要包括, CH12_LShldrFB  CH13_LShldrUD  CH23_RShldrFB CH24_RShldrUD   
 void CRobotChControl::MapShoulderChanel(double lShldrFB,double lShldrUD,double rShldrFB,double rShldrUD)
 {
	 //肩动作:
	 if(lShldrUD>125)
		 lShldrUD=125;
	 if(lShldrUD<115)
		 lShldrUD=115;
	 m_oldChValue[CH13_LShldrUD]=LinearFunction(lShldrUD,115,125,255,0);


	 if(lShldrFB>180)
		 lShldrFB=180;
	 if(lShldrFB<160)
		 lShldrFB=160;
	 m_oldChValue[CH12_LShldrFB]=LinearFunction(lShldrFB,160,180,255,0);		


	 if(rShldrUD>125)
		 rShldrUD=125;
	 if(rShldrUD<115)
		 rShldrUD=115;
	 m_oldChValue[CH24_RShldrUD]=LinearFunction(rShldrUD,115,125,255,0);


	 if(rShldrFB>180)
		 rShldrFB=180;
	 if(rShldrFB<160)
		 rShldrFB=160;
	 m_oldChValue[CH23_RShldrFB]=LinearFunction(rShldrFB,160,180,255,0);

 }
 void CRobotChControl::MapArmChanel(double lArmUD,double lShldrOC,double lElbw,double rArmUD,double rShldrOC,double rElbw)
 {
	 //手臂上下
	 if(lArmUD<20)
		 lArmUD=20;
	 if(lArmUD>90)
		 lArmUD=90;
	 if(lArmUD>85)
		 m_oldChValue[CH14_LArmUD]=10;
	 else if(lArmUD>75)
		 m_oldChValue[CH14_LArmUD]=LinearFunction(lArmUD,85,75,10,20);
	 else 
		 m_oldChValue[CH14_LArmUD]=LinearFunction(lArmUD,75,20,20,255);


	 if(rArmUD<20)
		 rArmUD=20;
	 if(rArmUD>90)
		 rArmUD=90;
	 if(rArmUD>85)
		 m_oldChValue[CH25_RArmUD]=10;
	 else if(rArmUD>75)
		 m_oldChValue[CH25_RArmUD]=LinearFunction(rArmUD,85,75,10,20);
	 else 
		 m_oldChValue[CH25_RArmUD]=LinearFunction(rArmUD,75,20,20,255);

	 //手臂前伸
	 //lShldrOC=180-lShldrOC;
	 //if(lShldrOC>=85)
		// m_oldChValue[CH15_LShldrOC]=130;
	 //if(lShldrOC>=75)
		// m_oldChValue[CH15_LShldrOC]=LinearFunction(lShldrOC,75,85,170,130);//LinearFunction(85-lShldrOC)/10.0*40+130;
	 //else
		// m_oldChValue[CH15_LShldrOC]=LinearFunction(lShldrOC,10,75,255,170);//=(75-lShldrOC)/60.0*90+160;
	 if(lShldrOC<10)
		 lShldrOC=10;
	 if(lShldrOC>90)
		 lShldrOC=90;
	 if(lShldrOC>=85)
		 m_oldChValue[CH15_LShldrOC]=130;
	 if(rShldrOC>=75)
		 m_oldChValue[CH15_LShldrOC]=LinearFunction(lShldrOC,75,85,170,130);//(85-)/10.0*40+130;
	 else
		 m_oldChValue[CH15_LShldrOC]=LinearFunction(lShldrOC,10,75,255,170);

	 if(rShldrOC<10)
		 rShldrOC=10;
	 if(rShldrOC>90)
		 rShldrOC=90;
	 if(rShldrOC>=85)
		 m_oldChValue[CH26_RShldrOC]=130;
	 if(rShldrOC>=75)
		 m_oldChValue[CH26_RShldrOC]=LinearFunction(rShldrOC,75,85,170,130);//(85-)/10.0*40+130;
	 else
		 m_oldChValue[CH26_RShldrOC]=LinearFunction(rShldrOC,10,75,255,170);

	 //手肘弯曲
	 if(rElbw<10)
		 rElbw=10;
	 if(rElbw>160)
		 rElbw=160;
	 m_oldChValue[CH28_RElbw]=LinearFunction(rElbw,10,160,0,255);

	 if(lElbw<10)
		 lElbw=10;
	 if(lElbw>160)
		 lElbw=160;
	 m_oldChValue[CH17_LElbw]=LinearFunction(lElbw,10,160,0,255);
 }
 //上半身通道控制，上半身倾斜和旋转。主要包括,  CH35_BodyTilts ,CH36_BodyLR  
 void CRobotChControl::MapUpbodyChanel(double upbodyTilt,double upbodyRotate)
 { //上半身通道控制，    //////弯曲
	 if(upbodyTilt<70)
		 upbodyTilt=70;
	 if(upbodyTilt>110)
		 upbodyTilt=110;
	 if(m_bMirror)
		 m_oldChValue[CH35_BodyTilts]=LinearFunction(upbodyTilt,70,110,255,0);

	 else
		 m_oldChValue[CH35_BodyTilts]=LinearFunction(upbodyTilt,70,110,0,255);

	 if(upbodyRotate<40)
		 upbodyRotate=40;
	 if(upbodyRotate>140)
		 upbodyRotate=140;
	 if(m_bMirror)
		 m_oldChValue[CH36_BodyLR]=LinearFunction(upbodyRotate,40,140,255,0);
	 else
		m_oldChValue[CH36_BodyLR]=LinearFunction(upbodyRotate,40,140,0,255);
 }


 //腰部通道控制，主要包括, CH37_BodyFB  CH38_WaistFB   CH40_WaistTilts 
 //注意 CH37_BodyFB 和 CH38_WaistFB 控制效果相似 ，故只取了其中一个控制
 void CRobotChControl::MapWasitChanel(double waistFrontback,double waistRotate)
 {
	 //腰前后动作: 
	 if(waistFrontback<75)
		 waistFrontback=75;
	 if(waistFrontback>110)
		 waistFrontback=110;
	 if(waistFrontback<=90)
	 {

		 m_oldChValue[CH38_WaistFB]=LinearFunction(waistFrontback,75,90,0,255);
		 m_oldChValue[CH37_BodyFB]=250;
	 }
	 else {
		 m_oldChValue[CH37_BodyFB]=LinearFunction(waistFrontback,90,110,255,0);
		 m_oldChValue[CH38_WaistFB]=250;
	 }
	 //腰左右旋转动作
	
		 if(waistRotate>65)
			 waistRotate=65;
		 if(waistRotate<30)
			 waistRotate=30;
		 if(waistRotate>60)
		 {

			 m_oldChValue[CH40_WaistTilts]=LinearFunction(waistRotate,60,65,80,75);

		 }
		 else
		 { 

			 m_oldChValue[CH40_WaistTilts]=LinearFunction(waistRotate,30,60,255,80);

		 }
	 
 }
 //控制胸部通道控制，相当于鞠躬动作，这个动作跟CH37 CH38具有相似效果 主要包括,H34_ChestFB
 void CRobotChControl::MapBowChanel(double bowAngle)
 {
	 //鞠躬动作
	 if(bowAngle<0)
		 bowAngle=0;
	 if(bowAngle>35)
		 bowAngle=35;
	 m_oldChValue[CH34_ChestFB]=LinearFunction(bowAngle,0,35,0,255);
 }

 //右腿胸部通道控制，包括向前踢腿、向左张开、屈膝、脚摆动， 主要包括,CH41_RHipFB,CH42_RHipOC,CH44_RKnee,CH43_RHipTwist
 void CRobotChControl::MapRightLegChanel(double rHipFB,double rHipOC,double rRKnee,double rHipTwist)
 {
	 //腿部动作
	 //向前踢腿
	 if(rHipFB>90)
		 rHipFB=90;
	 if(rHipFB<40)
		 rHipFB=40;
	 m_oldChValue[CH41_RHipFB]=LinearFunction(rHipFB,90,40,55,255);

	 //向左张开
	 if(rHipOC<10)
		 rHipOC=10;
	 if(rHipOC>20)
		 rHipOC=20;
	 m_oldChValue[CH42_RHipOC]=LinearFunction(rHipOC,10,20,70,255);

	 //屈膝
	 if(rRKnee<5)
		 rRKnee=5;
	 if(rRKnee>35)
		 rRKnee=35;
	 m_oldChValue[CH44_RKnee]=LinearFunction(rRKnee,5,35,0,255);

	 //脚跟旋转
	 if(rHipTwist<10)
		 rHipTwist=10;
	 if(rHipTwist>130)
		 rHipTwist=130;
	 if(rHipTwist<70)
		 m_oldChValue[CH43_RHipTwist]=LinearFunction(rHipTwist,10,70,10,80);
	 else
		 m_oldChValue[CH43_RHipTwist]=LinearFunction(rHipTwist,70,130,80,255);
 }

 //将表演者的动作映射到机器人
 void CRobotChControl::MapActionToRobot(CString &controlInfo,bool bMirror /*是否镜像*/)
 {

	  CString str;
	  m_bMirror=bMirror;

	  //计算头部角度
	  double headUpdown=0.0,headLeftRight=0.0,headTilt=0.0;
	  CacluateHead(m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER],
		           m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HEAD],headUpdown,headLeftRight,headTilt);
	 //映射头部角度
	  MapHeadChanel(headUpdown,headLeftRight,headTilt);
	  str.Format(_T("头部:上下=%.0lf,左右=%.0lf,左右偏=%.0lf"),headUpdown,headLeftRight,headTilt);
	  controlInfo=str;

	 //计算肩部动作（向前向上）
	  double 	 leftUpAngle=0.0, leftFrontAngle=0.0,rightUpAngle=0.0,rightFrontAngle=0.0;	  
	
	  CalculateShoulder(m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER],
		  m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT],
		  m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
		  leftUpAngle,
		  leftFrontAngle,
		  rightUpAngle,
		  rightFrontAngle);
	  //肩部通道控制，主要包括, CH12_LShldrFB  CH13_LShldrUD  CH23_RShldrFB CH24_RShldrUD   
	
	  if(m_bMirror)
		   MapShoulderChanel(leftFrontAngle, leftUpAngle, rightFrontAngle, rightUpAngle);	 
	  else		 
	       MapShoulderChanel( rightFrontAngle, rightUpAngle,leftFrontAngle, leftUpAngle);
	  	  //沿着肩部前倾和耸肩
	  str.Format(_T("\r\n肩部:左肩耸=%.0lf,左肩向前=%.0lf,右肩耸=%.0lf,右肩向前=%.0lf"),leftUpAngle,leftFrontAngle,rightUpAngle,rightFrontAngle);
	  controlInfo+=str;
	 
	//计算左胳膊的角度
	  double   leftArmFront_Angle=0.0, leftArmLeft_Angle=0.0,leftArmElbow_Angle=0.0;
	  CalculateLeftArm( m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT],
		  m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_LEFT],
		  m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_LEFT],
		  leftArmFront_Angle,
		  leftArmLeft_Angle,
		  leftArmElbow_Angle);

	  //计算右胳膊的角度
	  double   rightArmFront_Angle=0.0, rightArmLeft_Angle=0.0,rightArmElbow_Angle=0.0;
	  CalculateRightArm(m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
		  m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT],
		  m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT],
		  rightArmFront_Angle,
		  rightArmLeft_Angle,
		  rightArmElbow_Angle);
	  //手臂通道控制，主要包括,  CH14_LArmUD ,CH15_LShldrOC ,CH17_LElbw ,CH25_RArmUD  CH26_RShldrOC CH28_RElbw  
	  if(m_bMirror)
	       MapArmChanel(leftArmFront_Angle, leftArmLeft_Angle,leftArmElbow_Angle, rightArmFront_Angle,rightArmLeft_Angle, rightArmElbow_Angle);
	  else
		   MapArmChanel(rightArmFront_Angle,rightArmLeft_Angle, rightArmElbow_Angle,leftArmFront_Angle, leftArmLeft_Angle,leftArmElbow_Angle);

	  str.Format(_T("\r\n左胳膊:（上下=%.0lf,前后=%.0lf,肘关节=%.0lf）"),leftArmLeft_Angle,leftArmFront_Angle,leftArmElbow_Angle);
	  controlInfo+=str;	   
	  str.Format(_T("\r\n右胳膊:（上下=%.0lf,前后=%.0lf,肘关节=%.0lf）"),rightArmLeft_Angle,rightArmFront_Angle,rightArmElbow_Angle);
	  controlInfo+=str;

	 
	   //上半身动作,倾斜和旋转
	  double upbodyTilt=0.0, upbodyRotate=0.0;
	  CalculateUpbody(m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT],
		              m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
					  upbodyTilt,
					  upbodyRotate);
	 //上半身通道控制，上半身倾斜和旋转。主要包括,  CH35_BodyTilts ,CH36_BodyLR  
	  MapUpbodyChanel(upbodyTilt, upbodyRotate);
	  str.Format(_T("\r\n上半身:倾斜=%.0lf,旋转=%.0lf"),upbodyTilt,upbodyRotate);
	  controlInfo+=str;

     	  //计算腰的角度（前后、倾斜）
	  double waistFrontback=0.0, waistRotate=0.0;
	  if(m_bMirror) 
		  CalculateWaist(m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER],
		                 m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SPINE],
						 m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT],
						 m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HIP_LEFT],
						 waistFrontback,
						 waistRotate );
	  else
		 { CalculateWaist(m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER],
		                 m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SPINE],
						 m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT],
						 m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HIP_RIGHT],
						 waistFrontback,
						 waistRotate );
	       waistRotate=180-waistRotate;
	   }
	  //腰部通道控制，主要包括,  	 CH37_BodyFB  CH38_WaistFB   CH40_WaistTilts 
	   MapWasitChanel( waistFrontback, waistRotate);
	   str.Format(_T("\r\n腰部:前倾后仰=%.0lf,倾斜=%.0lf"),waistFrontback,waistRotate);
	   controlInfo+=str;

   //计算腰的角度,(想当于鞠躬)
	  double bowAngle=0.0;
	  CalculateBow( m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER],
		            m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_SPINE],
		            bowAngle);
	 //胸部通道控制，相当于鞠躬动作，这个动作跟CH37 CH38具有相似效果 主要包括,H34_ChestFB
	  MapBowChanel(bowAngle);
	  str.Format(_T("\r\n腰(鞠躬):%.0lf"),bowAngle);
	  controlInfo+=str;

	//计算右腿的角度（前-踢腿、张开、角的旋转）
     double rHipFB=0.0, rHipOC=0.0, rRKnee=0.0, rHipTwist=0.0;
	if(m_bMirror)
		CalculateRightLeg( m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HIP_RIGHT],
		m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_RIGHT],
		m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_RIGHT],
		m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_RIGHT],
		rHipFB,
		rHipOC,
		rRKnee,
		rHipTwist);
	else		
	    CalculateLeftLeg(m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_HIP_LEFT],
						m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_KNEE_LEFT],
						m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_LEFT],
						m_skeletonData.SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT],
						rHipFB,
						rHipOC,
						rRKnee,
						rHipTwist);



	 //右腿胸部通道控制，包括向前踢腿、向左张开、屈膝、脚摆动，主要包括,CH41_RHipFB,CH42_RHipOC,CH44_RKnee,CH43_RHipTwist
	  MapRightLegChanel( rHipFB, rHipOC, rRKnee, rHipTwist);
	  str.Format(_T("\r\n腿部:（向前=%.0lf,张开=%.0lf,膝关节=%.0lf,踝关节=%0.lf）"),rHipFB, rHipOC, rRKnee, rHipTwist);
	  controlInfo+=str;
	  
 }
