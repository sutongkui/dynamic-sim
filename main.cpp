#include <iostream>
#include <windows.h>
#include <queue>
#include <Kinect.h>
#include <list>

#include "scene.h"
#include "SpringsBuilder.h"
#include "Simulator.h"
#include "parameter.h"
#include "./bvh/BVHAccel.h"
#include "generateBody.h"
#include "detectBody.h"
#include "KinectJointFilter.h"
#include "Mesh.h"
#include "VAOMesh.h"
#include "Cloth.h"
#include "ObjLoader.h"

// #define VLD_FORCE_ENABLE
// #include<vld.h>

using namespace std;

extern int StartNum = 100;
extern int StopNum = 300;
int NameofBody = StartNum;

string clothfile = "../smooth/2/";

queue<Mesh> BodyQueue;
queue<Vec4s> VQueue;
queue<Vec3s> NQueue;

HANDLE hMutex = NULL;

list<mat> smoothList;
list<mat>::iterator iter;

DoudouHead_Merge DoudouHead_Solver;

//Thread1 for Getting Every Frames Body
DWORD WINAPI GetBodyData(LPVOID pParam)
{
	for (int i = StartNum + 1; i <= StopNum; i++)
	{
		string file_name;
		char num[4];
		_itoa_s(i, num, 10);
		string s = num;
		file_name = clothfile + s + ".obj";

		ObjLoader loader;

		Mesh new_body;
		loader.load(new_body, file_name);
		new_body.scale(0.30f);
		new_body.translate(0.0f, 1.0f, 0.0f);
		BodyQueue.push(new_body);
	}

	return 0;
}

//Thread1 for Getting Every Frames Body
DWORD WINAPI DtoG(LPVOID pParam)
{
	//Initiate Template
	SMPL bodyTemplate = SMPL(MALE);
	cout << "SMPL::initial finished!" << endl;

	//Initiate Sensor  
	IKinectSensor *pSensor;
	HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&pSensor);
	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return -1;
	}

	IBodyFrameSource *pBodySource;
	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		return -1;
	}

	IBodyFrameReader *pBodyReader;
	hResult = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		return -1;
	}

	//mat pp = zeros(24, 3);
	//mat result=bodyTemplate.gen_pose_model(pp, TRUE);
	//bodyTemplate.write_to_obj(result, "MALE.obj");

	// Holt Double Exponential Smoothing Filter
	Sample::FilterDoubleExponential filter[BODY_COUNT];

	// Option : Setting Smoothing Parameter
	for (int count = 0; count < BODY_COUNT; count++) {
		float smoothing = 0.5f;          // [0..1], lower values closer to raw data
		float correction = 0.5f;         // [0..1], lower values slower to correct towards the raw data
		float prediction = 0.5f;         // [0..n], the number of frames to predict into the future
		float jitterRadius = 0.05f;       // The radius in meters for jitter reduction
		float maxDeviationRadius = 0.04f; // The maximum radius in meters that filtered positions are allowed to deviate from raw data

		filter[count].Init(smoothing, correction, prediction, jitterRadius, maxDeviationRadius);
	}

	//The label number of the first body detected by Kinect
	int BODY_LABEL = -1;

	StopWatch time;
	time.start();
	int counter = 1;
	bool tag = TRUE;
	bool first = TRUE;
	while (counter)
	{
		Vec4s vertex;
		Vec3s normal;
		//Obj new_body = BodyQueue.front();
		mat trans_joint;

		//bool judge = detectJoint(hResult, pBodyReader, joint);

		IBodyFrame *pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hResult)) {
			IBody *pBody[BODY_COUNT] = { 0 };

			hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
			if (SUCCEEDED(hResult)) {
				for (int count = 0; count < BODY_COUNT; count++) {
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked(&bTracked);
					if (bTracked&&SUCCEEDED(hResult) && BODY_LABEL == -1)
						BODY_LABEL = count;
					if (SUCCEEDED(hResult) && bTracked && count == BODY_LABEL) {
						//counter--;
						Joint joint[JointType::JointType_Count];
						/////////////////////////////     
						hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);//joint 

						//////////////////////// Filtered Joint//////////////////////////////////
						filter[count].Update(joint);
						const DirectX::XMVECTOR *vec = filter[count].GetFilteredJoints();
						for (int type = 0; type < JointType::JointType_Count; type++) {
							if (joint[type].TrackingState != TrackingState::TrackingState_NotTracked) {
								float x = 0.0f, y = 0.0f, z = 0.0f;
								DirectX::XMVectorGetXPtr(&x, vec[type]);
								DirectX::XMVectorGetYPtr(&y, vec[type]);
								DirectX::XMVectorGetZPtr(&z, vec[type]);
							}
						}
						////////////////////////////////////////////////////////////////////////
						//Get joint for genBody from kinect joint 
						trans_joint = JointTransform(joint);
						////////////////Transition from T-pose to first frame///////////////////////////////////
						if (first == TRUE) {
							mat pose = bodyTemplate.J_to_pose(trans_joint);
							float coefficient = 0.04f / max(max(pose));
							cout << coefficient << endl;
							mat transition = zeros(24, 3);
							int num = 0;
							while (max(max(abs(transition))) < max(max(abs(pose))))
							{
								//transition.print("t:");
								genFirstBody(transition, vertex, normal, bodyTemplate);
								transition += pose*coefficient;
								VQueue.push(vertex);
								NQueue.push(normal);
								num++;
							}
							cout << num << endl;
							first = FALSE;
						}
						//////////////////////////////////////////////////////////////////////////////////
						/////////////////////////////Smooth by List////////////////////////////////////////
						mat sum = zeros(24, 3);
						if (smoothList.size() < 5)
							smoothList.push_back(trans_joint);
						else {

							for (iter = smoothList.begin(); iter != smoothList.end(); ++iter)
							{
								sum += (*iter);
							}
							sum = sum / 5;
							smoothList.pop_front();
							smoothList.push_back(trans_joint);

							///////////////////////////////////////////////////////////////////////////

							genBodyVector(sum, vertex, normal, bodyTemplate);

							cout << "A new pose has been detected!" << endl;

							if (tag == TRUE) {
								VQueue.push(vertex);
								NQueue.push(normal);
								tag = FALSE;
								cout << "num:" << VQueue.size() << endl;
							}
							else tag = TRUE;
							time.stop();
							cout << "cost:" << time.elapsed_ms() << endl;
							time.restart();
						}
						//return TRUE;
					}
				}
			}
			for (int count = 0; count < BODY_COUNT; count++) {
				SafeRelease(pBody[count]);
			}
		}
		SafeRelease(pBodyFrame);

		//if (judge)
		//{
		//	genBody(joint, new_body);
		//	cout << "A new pose has been detected!" << endl;
		//}
		//else continue;

		//new_body.scale_translate(0.30, 0, 1.0, 0);
		//new_body.unified();
		//BodyQueue.push(new_body);
	}

	SafeRelease(pBodySource);
	SafeRelease(pBodyReader);;
	if (pSensor) {
		pSensor->Close();
	}
	SafeRelease(pSensor);

	return 0;
}

int main(int argc, char* *argv)
{
	Scene::instance().initialize(argc, argv); //initialize opengl 

	ObjLoader loader;
	Cloth cloth(SINGLE_LAYER_NOB);

	////²âÊÔÒÂ·þ
	//Obj cloth("../cloth/cloth.obj");    //pose0
	//cloth.scale_translate(0.31, 0, 1.95, 0.02);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/dress2/dress2-iso.obj",SINGLE_LAYER_NOB);  
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.24, 0, 1.2, 0.02); 
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/dress3/dress3.obj",SINGLE_LAYER_NOB);  
	//cloth.rotation(90, X);   
	//cloth.scale_translate(0.24, 0, 0.45, 0.02); 
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/dress-asymmetric/dress-asymmetric.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.25, 0, 1.10, 0.02);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/dress-victor/dress-victor.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   
	//cloth.scale_translate(0.25, 0, 1.60, 0.02);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/robe/robe.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.3, 0, 0.5, 0.0);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/tshirt/tshirt.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   
	//cloth.rotation(-5, Z);
	//cloth.scale_translate(0.26, 0, 1.18, -0.1);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/shirt/shirt.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   
	//cloth.rotation(-4, Z);
	//cloth.scale_translate(0.27, 0, 2.1, 0.15);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/skirt/skirt.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   
	//cloth.scale_translate(0.29, 0, 0.5, 0);
	//cloth.unified();

	loader.load(cloth, "../cloth_no_boundary/tshirt2/tshirt2.obj");
	cloth.rotation(90, 0, 0);
	cloth.rotation(0, 0, -4);
	cloth.scale(0.28f);
	cloth.translate(0, 0.9f, -2.2f);


	//Obj cloth("../cloth_no_boundary/shorts/shorts.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   //
	//cloth.scale_translate(0.29, 0, 0.5, 0);
	//cloth.unified();

	//Obj cloth("../cloth_no_boundary/vest/vest.obj", SINGLE_LAYER_NOB);
	//cloth.rotation(90, X);   
	//cloth.scale_translate(0.28, 0, 1.4, 0.02); 
	//cloth.unified();


	//string file;
	//char num_s[4];
	//_itoa_s(StartNum, num_s, 10);
	//string ss = num_s;
	//file = clothfile + ss + ".obj";
	//Obj body(file);

	VAOMesh body, head;

	loader.load(body, "../Template/MALE.obj");
	body.scale(0.3f);
	body.translate(0.0f, 0.6f, 0.0f);

	loader.load(head, "../DoudouHead/HeadColored.obj");
	DoudouHead_Solver.Init(head);


	Scene::instance().add(cloth);
	Scene::instance().add(body);
	Scene::instance().add(head);

	Scene::instance().initiate_body_template(body);
	Scene::instance().update_simulating_cloth(cloth);
	Scene::instance().update_simulating_body(body);

	HANDLE hThread1 = CreateThread(NULL, 0, DtoG, NULL, 0, NULL);
	CloseHandle(hThread1);
	hMutex = CreateMutex(NULL, FALSE, NULL);

	Scene::instance().render();

	return 0;
}


