#pragma once
#include <Kinect.h>  
#include<iostream>  
#include<time.h>  
#include<fstream>
#include"generateBody.h"
#include"KinectJointFilter.h"

#define Joint_count 24
#pragma comment(lib, "kinect20.lib") 
using namespace std;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
mat JointTransform(Joint *prejoint);

bool detectJoint(HRESULT hResult, IBodyFrameReader *pBodyReader,mat &trans_joint)
{
	IBodyFrame *pBodyFrame = nullptr;
	hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
	if (SUCCEEDED(hResult)) {
		IBody *pBody[BODY_COUNT] = { 0 };

		hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
		if (SUCCEEDED(hResult)) {
			for (int count = 0; count < BODY_COUNT; count++) {
				BOOLEAN bTracked = false;
				hResult = pBody[count]->get_IsTracked(&bTracked);
				if (SUCCEEDED(hResult) && bTracked) {
					Joint joint[JointType::JointType_Count];
					/////////////////////////////     
					hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);//joint  

					trans_joint = JointTransform(joint);

					cout <<  " OK" << endl;
					return TRUE;
				}
			}
		}
		for (int count = 0; count < BODY_COUNT; count++) {
			SafeRelease(pBody[count]);
		}
	}
	SafeRelease(pBodyFrame);
	return FALSE;
}


//Joint transform

mat JointTransform(Joint *prejoint)
{
	mat tmp = zeros(24,3);

	//get index0
	tmp(0, 0) = prejoint[0].Position.X;
	tmp(1, 0) = prejoint[12].Position.X;
	tmp(2, 0) = prejoint[16].Position.X;

	tmp(4, 0) = prejoint[13].Position.X;
	tmp(5, 0) = prejoint[17].Position.X;
	tmp(6, 0) = prejoint[1].Position.X;
	tmp(7, 0) = prejoint[14].Position.X;
	tmp(8, 0) = prejoint[18].Position.X;

	tmp(12, 0) = prejoint[2].Position.X;

	tmp(15, 0) = prejoint[3].Position.X;
	tmp(16, 0) = prejoint[4].Position.X;
	tmp(17, 0) = prejoint[8].Position.X;
	tmp(18, 0) = prejoint[5].Position.X;
	tmp(19, 0) = prejoint[9].Position.X;
	tmp(20, 0) = prejoint[6].Position.X;
	tmp(21, 0) = prejoint[10].Position.X;



	//get index1
	tmp(0, 1) = prejoint[0].Position.Y;
	tmp(1, 1) = prejoint[12].Position.Y;
	tmp(2, 1) = prejoint[16].Position.Y;

	tmp(4, 1) = prejoint[13].Position.Y;
	tmp(5, 1) = prejoint[17].Position.Y;
	tmp(6, 1) = prejoint[1].Position.Y;
	tmp(7, 1) = prejoint[14].Position.Y;
	tmp(8, 1) = prejoint[18].Position.Y;

	tmp(12, 1) = prejoint[2].Position.Y;

	tmp(15, 1) = prejoint[3].Position.Y;
	tmp(16, 1) = prejoint[4].Position.Y;
	tmp(17, 1) = prejoint[8].Position.Y;
	tmp(18, 1) = prejoint[5].Position.Y;
	tmp(19, 1) = prejoint[9].Position.Y;
	tmp(20, 1) = prejoint[6].Position.Y;
	tmp(21, 1) = prejoint[10].Position.Y;

	//get index2
	tmp(0, 2) = prejoint[0].Position.Z;
	tmp(1, 2) = prejoint[12].Position.Z;
	tmp(2, 2) = prejoint[16].Position.Z;

	tmp(4, 2) = prejoint[13].Position.Z;
	tmp(5, 2) = prejoint[17].Position.Z;
	tmp(6, 2) = prejoint[1].Position.Z;
	tmp(7, 2) = prejoint[14].Position.Z;
	tmp(8, 2) = prejoint[18].Position.Z;

	tmp(12, 2) = prejoint[2].Position.Z;

	tmp(15, 2) = prejoint[3].Position.Z;
	tmp(16, 2) = prejoint[4].Position.Z;
	tmp(17, 2) = prejoint[8].Position.Z;
	tmp(18, 2) = prejoint[5].Position.Z;
	tmp(19, 2) = prejoint[9].Position.Z;
	tmp(20, 2) = prejoint[6].Position.Z;
	tmp(21, 2) = prejoint[10].Position.Z;
	return tmp;
}