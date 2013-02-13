/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

bool useSapGpuBroadphase = true;
extern bool useConvexHeightfield;
#include "btPgsJacobiSolver.h"
#include "../../rendering/rendertest/OpenGLInclude.h"
#ifdef _WIN32
#include "windows.h"
#endif

#include "CLPhysicsDemo.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "../basic_initialize/btOpenCLUtils.h"
#include "../broadphase_benchmark/findPairsOpenCL.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btAabbUtil2.h"
#include "../opencl/gpu_rigidbody_pipeline/btGpuNarrowPhaseAndSolver.h"
#include "../opencl/gpu_rigidbody_pipeline/btConvexUtility.h"
#include "../../dynamics/basic_demo/ConvexHeightFieldShape.h"
#include "../../dynamics/basic_demo/Stubs/AdlContact4.h"

//#define USE_GRID_BROADPHASE
//#ifdef USE_GRID_BROADPHASE
#include "../broadphase_benchmark/btGridBroadphaseCl.h"
//#else
#include "btGpuSapBroadphase.h"

//#endif //USE_GRID_BROADPHASE




#include "../broadphase_benchmark/btAabbHost.h"
#include "LinearMath/btQuickprof.h"

#define MSTRINGIFY(A) #A
static char* interopKernelString = 
#include "../broadphase_benchmark/integrateKernel.cl"

#define INTEROPKERNEL_SRC_PATH "opencl/broadphase_benchmark/integrateKernel.cl"
	
cl_kernel g_integrateTransformsKernel;
cl_kernel g_integrateTransformsKernel2;

btOpenCLDeviceInfo info;
const char* g_deviceName = 0;


bool runOpenCLKernels = true;

btGpuNarrowphaseAndSolver* g_narrowphaseAndSolver = 0;
ConvexHeightField* s_convexHeightField = 0 ;





btFindPairsIO gFpIO;

cl_context			g_cxMainContext;
cl_command_queue	g_cqCommandQue;
cl_device_id		g_device;





struct InternalData
{
	btOpenCLArray<btVector3>* m_linVelBuf;
	btOpenCLArray<btVector3>* m_angVelBuf;
	btOpenCLArray<float>* m_bodyTimes;
	
	btGridBroadphaseCl* m_BroadphaseGrid;
	btGpuSapBroadphase* m_BroadphaseSap;

	btOpenCLArray<btAABBHost>* m_localShapeAABBGPU;
	btAlignedObjectArray<btAABBHost>* m_localShapeAABBCPU;

	btAlignedObjectArray<btVector3>	m_linVelHost;
	btAlignedObjectArray<btVector3>	m_angVelHost;
	btAlignedObjectArray<float> m_bodyTimesHost;

	btPgsJacobiSolver*	m_pgsSolver;

	InternalData():m_linVelBuf(0),m_angVelBuf(0),m_bodyTimes(0),m_BroadphaseSap(0),m_BroadphaseGrid(0)
	{
		
	}
	~InternalData()
	{

	}
};


void InitCL(int preferredDeviceIndex, int preferredPlatformIndex, bool useInterop)
{
	void* glCtx=0;
	void* glDC = 0;

#ifdef _WIN32
	glCtx = wglGetCurrentContext();
	glDC = wglGetCurrentDC();
#else //!_WIN32
#ifndef __APPLE__
    GLXContext glCtx = glXGetCurrentContext();
    glDC = wglGetCurrentDC();//??
#endif
#endif //!_WIN32

    
	int ciErrNum = 0;
//#ifdef CL_PLATFORM_INTEL
//	cl_device_type deviceType = CL_DEVICE_TYPE_ALL;
//#else
	cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
//#endif

	

	if (useInterop)
	{
		g_cxMainContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, glCtx, glDC);
	} else
	{
		g_cxMainContext = btOpenCLUtils::createContextFromType(deviceType, &ciErrNum, 0,0,preferredDeviceIndex, preferredPlatformIndex);
	}


	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	int numDev = btOpenCLUtils::getNumDevices(g_cxMainContext);

	if (numDev>0)
	{
		g_device= btOpenCLUtils::getDevice(g_cxMainContext,0);
		g_cqCommandQue = clCreateCommandQueue(g_cxMainContext, g_device, 0, &ciErrNum);
		oclCHECKERROR(ciErrNum, CL_SUCCESS);
        
        btOpenCLUtils::printDeviceInfo(g_device);
		
		btOpenCLUtils::getDeviceInfo(g_device,&info);
		g_deviceName = info.m_deviceName;

	}

}



CLPhysicsDemo::CLPhysicsDemo(int maxShapeBufferCapacity, int maxNumObjects)
{
	m_maxShapeBufferCapacityInBytes = maxShapeBufferCapacity;
	m_maxNumPhysicsInstances = maxNumObjects;
	m_numPhysicsInstances=0;
	m_numDynamicPhysicsInstances = 0;

	m_data = new InternalData;
}

CLPhysicsDemo::~CLPhysicsDemo()
{
	delete m_narrowphaseAndSolver;
	m_narrowphaseAndSolver = 0;
}

void	CLPhysicsDemo::readbackBodiesToCpu()
{
	if (m_narrowphaseAndSolver)
		m_narrowphaseAndSolver->readbackAllBodiesToCpu();

	if (m_data->m_BroadphaseSap)
	{
		m_data->m_BroadphaseSap->readAabbsBackToCpu();
	}
}
	
void	CLPhysicsDemo::getObjectTransformFromCpu(float* position, float* orientation, int objectIndex)
{
	if (m_narrowphaseAndSolver)
		m_narrowphaseAndSolver->getObjectTransformFromCpu(position,orientation,objectIndex);
}



void CLPhysicsDemo::writeBodiesToGpu()
{
	if (m_data->m_BroadphaseSap)
		m_data->m_BroadphaseSap->writeAabbsToGpu();

	writeVelocitiesToGpu();
	
	if (m_narrowphaseAndSolver)
		m_narrowphaseAndSolver->writeAllBodiesToGpu();

	
}

int		CLPhysicsDemo::registerCompoundShape(btAlignedObjectArray<btGpuChildShape>* childShapes)
{
	
	int collidableIndex = m_narrowphaseAndSolver->allocateCollidable();
	btCollidable& col = m_narrowphaseAndSolver->getCollidableCpu(collidableIndex);
	col.m_shapeType = CollisionShape::SHAPE_COMPOUND_OF_CONVEX_HULLS;
	
	col.m_shapeIndex = m_narrowphaseAndSolver->registerCompoundShape(childShapes);
	col.m_numChildShapes = childShapes->size();
	
	
	btAABBHost aabbMin, aabbMax;
	btVector3 myAabbMin(1e30f,1e30f,1e30f);
	btVector3 myAabbMax(-1e30f,-1e30f,-1e30f);
	
	//compute local AABB of the compound of all children
	for (int i=0;i<childShapes->size();i++)
	{
		int childColIndex = childShapes->at(i).m_shapeIndex;
		btCollidable& childCol = m_narrowphaseAndSolver->getCollidableCpu(childColIndex);
		btAABBHost aabbMinLoc =m_data->m_localShapeAABBCPU->at(childColIndex*2);
		btAABBHost aabbMaxLoc =m_data->m_localShapeAABBCPU->at(childColIndex*2+1);

		btVector3 childLocalAabbMin(aabbMinLoc.fx,aabbMinLoc.fy,aabbMinLoc.fz);
		btVector3 childLocalAabbMax(aabbMaxLoc.fx,aabbMaxLoc.fy,aabbMaxLoc.fz);
		btVector3 aMin,aMax;
		btScalar margin(0.f);
		btTransform childTr;
		childTr.setIdentity();

		childTr.setOrigin(btVector3(childShapes->at(i).m_childPosition[0],
									childShapes->at(i).m_childPosition[1],
									childShapes->at(i).m_childPosition[2]));
		childTr.setRotation(btQuaternion(childShapes->at(i).m_childOrientation[0],
										 childShapes->at(i).m_childOrientation[1],
										 childShapes->at(i).m_childOrientation[2],
										 childShapes->at(i).m_childOrientation[3]));
		btTransformAabb(childLocalAabbMin,childLocalAabbMax,margin,childTr,aMin,aMax);
		myAabbMin.setMin(aMin);
		myAabbMax.setMax(aMax);		
	}
	
	aabbMin.fx = myAabbMin[0];//s_convexHeightField->m_aabb.m_min.x;
	aabbMin.fy = myAabbMin[1];//s_convexHeightField->m_aabb.m_min.y;
	aabbMin.fz= myAabbMin[2];//s_convexHeightField->m_aabb.m_min.z;
	aabbMin.uw = 0;
	
	aabbMax.fx = myAabbMax[0];//s_convexHeightField->m_aabb.m_max.x;
	aabbMax.fy = myAabbMax[1];//s_convexHeightField->m_aabb.m_max.y;
	aabbMax.fz= myAabbMax[2];//s_convexHeightField->m_aabb.m_max.z;
	aabbMax.uw = 0;
	
	m_data->m_localShapeAABBCPU->push_back(aabbMin);
	m_data->m_localShapeAABBGPU->push_back(aabbMin);
	
	m_data->m_localShapeAABBCPU->push_back(aabbMax);
	m_data->m_localShapeAABBGPU->push_back(aabbMax);
	clFinish(g_cqCommandQue);
	
	
	
	
	return collidableIndex;
	
}


	

int		CLPhysicsDemo::registerConcaveMesh(btAlignedObjectArray<btVector3>* vertices, btAlignedObjectArray<int>* indices,const float* scaling1)
{
	btVector3 scaling(scaling1[0],scaling1[1],scaling1[2]);

	int collidableIndex = m_narrowphaseAndSolver->allocateCollidable();
	btCollidable& col = m_narrowphaseAndSolver->getCollidableCpu(collidableIndex);
	
	col.m_shapeType = CollisionShape::SHAPE_CONCAVE_TRIMESH;
	col.m_shapeIndex = m_narrowphaseAndSolver->registerConcaveMeshShape(vertices,indices,col,scaling);

	

	btAABBHost aabbMin, aabbMax;
	btVector3 myAabbMin(1e30f,1e30f,1e30f);
	btVector3 myAabbMax(-1e30f,-1e30f,-1e30f);

	for (int i=0;i<vertices->size();i++)
	{
		btVector3 vtx(vertices->at(i)*scaling);
		myAabbMin.setMin(vtx);
		myAabbMax.setMax(vtx);
	}
	aabbMin.fx = myAabbMin[0];//s_convexHeightField->m_aabb.m_min.x;
	aabbMin.fy = myAabbMin[1];//s_convexHeightField->m_aabb.m_min.y;
	aabbMin.fz= myAabbMin[2];//s_convexHeightField->m_aabb.m_min.z;
	aabbMin.uw = 0;

	aabbMax.fx = myAabbMax[0];//s_convexHeightField->m_aabb.m_max.x;
	aabbMax.fy = myAabbMax[1];//s_convexHeightField->m_aabb.m_max.y;
	aabbMax.fz= myAabbMax[2];//s_convexHeightField->m_aabb.m_max.z;
	aabbMax.uw = 0;

	m_data->m_localShapeAABBCPU->push_back(aabbMin);
	m_data->m_localShapeAABBGPU->push_back(aabbMin);

	m_data->m_localShapeAABBCPU->push_back(aabbMax);
	m_data->m_localShapeAABBGPU->push_back(aabbMax);
	clFinish(g_cqCommandQue);

	return collidableIndex;
}



int		CLPhysicsDemo::registerConvexPolyhedron(btConvexUtility* utilPtr , bool noHeightField)
{
	int collidableIndex = m_narrowphaseAndSolver->allocateCollidable();

	btCollidable& col = m_narrowphaseAndSolver->getCollidableCpu(collidableIndex);
	col.m_shapeType = CollisionShape::SHAPE_CONVEX_HULL;
	col.m_shapeIndex = -1;
	
	int numFaces= utilPtr->m_faces.size();
	float4* eqn = new float4[numFaces];
	for (int i=0;i<numFaces;i++)
	{
		eqn[i].x = utilPtr->m_faces[i].m_plane[0];
		eqn[i].y = utilPtr->m_faces[i].m_plane[1];
		eqn[i].z = utilPtr->m_faces[i].m_plane[2];
		eqn[i].w = utilPtr->m_faces[i].m_plane[3];
	}
	printf("numFaces = %d\n", numFaces);

	if (noHeightField)
	{
		s_convexHeightField = 0;
	} else
	{
		s_convexHeightField = new ConvexHeightField(eqn,numFaces);
	}



	if (m_narrowphaseAndSolver)
	{
		btVector3 localCenter(0,0,0);
		for (int i=0;i<utilPtr->m_vertices.size();i++)
			localCenter+=utilPtr->m_vertices[i];
		localCenter*= (1.f/utilPtr->m_vertices.size());
		utilPtr->m_localCenter = localCenter;

		if (useConvexHeightfield)
		col.m_shapeIndex = m_narrowphaseAndSolver->registerConvexHeightfield(s_convexHeightField,col);
		else
		{
			col.m_shapeIndex = m_narrowphaseAndSolver->registerConvexHullShape(utilPtr,col);
			//col.m_shapeIndex = m_narrowphaseAndSolver->registerConvexHullShape(utilPtr,col);

		}
	}

	if (col.m_shapeIndex>=0)
	{
		btAABBHost aabbMin, aabbMax;
		btVector3 myAabbMin(1e30f,1e30f,1e30f);
		btVector3 myAabbMax(-1e30f,-1e30f,-1e30f);

		for (int i=0;i<utilPtr->m_vertices.size();i++)
		{
			myAabbMin.setMin(utilPtr->m_vertices[i]);
			myAabbMax.setMax(utilPtr->m_vertices[i]);
		}
		aabbMin.fx = myAabbMin[0];//s_convexHeightField->m_aabb.m_min.x;
		aabbMin.fy = myAabbMin[1];//s_convexHeightField->m_aabb.m_min.y;
		aabbMin.fz= myAabbMin[2];//s_convexHeightField->m_aabb.m_min.z;
		aabbMin.uw = 0;

		aabbMax.fx = myAabbMax[0];//s_convexHeightField->m_aabb.m_max.x;
		aabbMax.fy = myAabbMax[1];//s_convexHeightField->m_aabb.m_max.y;
		aabbMax.fz= myAabbMax[2];//s_convexHeightField->m_aabb.m_max.z;
		aabbMax.uw = 0;

		m_data->m_localShapeAABBCPU->push_back(aabbMin);
		m_data->m_localShapeAABBGPU->push_back(aabbMin);

		m_data->m_localShapeAABBCPU->push_back(aabbMax);
		m_data->m_localShapeAABBGPU->push_back(aabbMax);

		//m_data->m_localShapeAABB->copyFromHostPointer(&aabbMin,1,shapeIndex*2);
		//m_data->m_localShapeAABB->copyFromHostPointer(&aabbMax,1,shapeIndex*2+1);
		clFinish(g_cqCommandQue);
	}

	delete[] eqn;
	
	
	
	return collidableIndex;

}



int		CLPhysicsDemo::registerSphereShape(float radius)
{
	int collidableIndex = m_narrowphaseAndSolver->allocateCollidable();

	btCollidable& col = m_narrowphaseAndSolver->getCollidableCpu(collidableIndex);
	col.m_shapeType = CollisionShape::SHAPE_SPHERE;
	col.m_shapeIndex = 0;
	col.m_radius = radius;
	
	if (col.m_shapeIndex>=0)
	{
		btAABBHost aabbMin, aabbMax;
		btVector3 myAabbMin(-radius,-radius,-radius);
		btVector3 myAabbMax(radius,radius,radius);

		aabbMin.fx = myAabbMin[0];//s_convexHeightField->m_aabb.m_min.x;
		aabbMin.fy = myAabbMin[1];//s_convexHeightField->m_aabb.m_min.y;
		aabbMin.fz= myAabbMin[2];//s_convexHeightField->m_aabb.m_min.z;
		aabbMin.uw = 0;

		aabbMax.fx = myAabbMax[0];//s_convexHeightField->m_aabb.m_max.x;
		aabbMax.fy = myAabbMax[1];//s_convexHeightField->m_aabb.m_max.y;
		aabbMax.fz= myAabbMax[2];//s_convexHeightField->m_aabb.m_max.z;
		aabbMax.uw = 0;

		m_data->m_localShapeAABBCPU->push_back(aabbMin);
		m_data->m_localShapeAABBGPU->push_back(aabbMin);

		m_data->m_localShapeAABBCPU->push_back(aabbMax);
		m_data->m_localShapeAABBGPU->push_back(aabbMax);
		//m_data->m_localShapeAABB->copyFromHostPointer(&aabbMin,1,shapeIndex*2);
		//m_data->m_localShapeAABB->copyFromHostPointer(&aabbMax,1,shapeIndex*2+1);
		clFinish(g_cqCommandQue);
	}
	
	return collidableIndex;
}


int		CLPhysicsDemo::registerPlaneShape(const btVector3& planeNormal, float planeConstant)
{
	int collidableIndex = m_narrowphaseAndSolver->allocateCollidable();

	btCollidable& col = m_narrowphaseAndSolver->getCollidableCpu(collidableIndex);
	col.m_shapeType = CollisionShape::SHAPE_PLANE;
	col.m_shapeIndex = m_narrowphaseAndSolver->registerFace(planeNormal,planeConstant);
	col.m_radius = planeConstant;
	
	if (col.m_shapeIndex>=0)
	{
		btAABBHost aabbMin;
		aabbMin.fx = -1e30f;
		aabbMin.fy = -1e30f;
		aabbMin.fz = -1e30f;
		aabbMin.uw = 0;
		btAABBHost aabbMax;
		aabbMax.fx = 1e30f;
		aabbMax.fy = 1e30f;
		aabbMax.fz = 1e30f;
		aabbMax.uw = 0;
		m_data->m_localShapeAABBCPU->push_back(aabbMin);
		m_data->m_localShapeAABBGPU->push_back(aabbMin);
		m_data->m_localShapeAABBCPU->push_back(aabbMax);
		m_data->m_localShapeAABBGPU->push_back(aabbMax);
		clFinish(g_cqCommandQue);
	}
	
	return collidableIndex;
}


int		CLPhysicsDemo::registerConvexPolyhedron(const float* vertices, int strideInBytes, int numVertices, const float* scaling, bool noHeightField)
{
	btAlignedObjectArray<btVector3> verts;
	
	unsigned char* vts = (unsigned char*) vertices;
	for (int i=0;i<numVertices;i++)
	{
		float* vertex = (float*) &vts[i*strideInBytes];
		verts.push_back(btVector3(vertex[0]*scaling[0],vertex[1]*scaling[1],vertex[2]*scaling[2]));
	}

	btConvexUtility* utilPtr = new btConvexUtility();
	bool merge = true;
	if (numVertices)
	{
		utilPtr->initializePolyhedralFeatures(&verts[0],verts.size(),merge);
	}

#if 1
	for (int i=0;i<utilPtr->m_faces.size();i++)
	{
		if (utilPtr->m_faces[i].m_indices.size()>3)
		{
			btVector3 v0 = utilPtr->m_vertices[utilPtr->m_faces[i].m_indices[0]];
			btVector3 v1 = utilPtr->m_vertices[utilPtr->m_faces[i].m_indices[1]];
			btVector3 v2 = utilPtr->m_vertices[utilPtr->m_faces[i].m_indices[2]];

			btVector3 faceNormal = ((v1-v0).cross(v2-v0)).normalize();
			btScalar c = -faceNormal.dot(v0);
			printf("normal = %f,%f,%f, planeConstant = %f\n",faceNormal.getX(),faceNormal.getY(),faceNormal.getZ(),c);
			
			//utilPtr->m_faces[i].m_plane
		}
	}
#endif

	
	int shapeIndex = registerConvexPolyhedron(utilPtr,noHeightField);
	
	return shapeIndex;
}

int		CLPhysicsDemo::registerPhysicsInstance(float mass, const float* position, const float* orientation, int collidableIndex, int userIndex)
{
	btVector3 aabbMin(0,0,0),aabbMax(0,0,0);
	if (collidableIndex>=0)
	{
		btAABBHost hostLocalAabbMin = m_data->m_localShapeAABBCPU->at(collidableIndex*2);
		btAABBHost hostLocalAabbMax = m_data->m_localShapeAABBCPU->at(collidableIndex*2+1);
		btVector3 localAabbMin(hostLocalAabbMin.fx,hostLocalAabbMin.fy,hostLocalAabbMin.fz);
		btVector3 localAabbMax(hostLocalAabbMax.fx,hostLocalAabbMax.fy,hostLocalAabbMax.fz);
		
		btScalar margin = 0.01f;
		btTransform t;
		t.setIdentity();
		t.setOrigin(btVector3(position[0],position[1],position[2]));
		t.setRotation(btQuaternion(orientation[0],orientation[1],orientation[2],orientation[3]));
		
		btTransformAabb(localAabbMin,localAabbMax, margin,t,aabbMin,aabbMax);

		//(position[0],position[0],position[0]);

		
		//aabbMin -= btVector3(400.f,410.f,400.f);
		//aabbMax += btVector3(400.f,410.f,400.f);

	
		//btBroadphaseProxy* proxy = m_data->m_Broadphase->createProxy(aabbMin,aabbMax,collisionShapeIndex,userPointer,1,1,0,0);//m_dispatcher);
	
		if (useSapGpuBroadphase)
		{
			if (mass)
			{
				m_data->m_BroadphaseSap->createProxy(aabbMin,aabbMax,userIndex,1,1);//m_dispatcher);
			} else
			{
				m_data->m_BroadphaseSap->createLargeProxy(aabbMin,aabbMax,userIndex,1,1);//m_dispatcher);	
			}
		}
		else
		{
			void* userPtr = (void*)userIndex;
			m_data->m_BroadphaseGrid->createProxy(aabbMin,aabbMax,collidableIndex,userPtr ,1,1);//m_dispatcher);
		}
	}
			
	bool writeToGpu = false;
	int bodyIndex = -1;

	m_data->m_linVelHost.push_back(btVector3(0,0,0));
	m_data->m_angVelHost.push_back(btVector3(0,0,0));
	m_data->m_bodyTimesHost.push_back(0.f);
	
	

	if (m_narrowphaseAndSolver)
	{
		//bodyIndex = m_narrowphaseAndSolver->registerRigidBody(collisionShapeIndex,CollisionShape::SHAPE_CONVEX_HEIGHT_FIELD,mass,position,orientation,&aabbMin.getX(),&aabbMax.getX(),writeToGpu);
		bodyIndex = m_narrowphaseAndSolver->registerRigidBody(collidableIndex,mass,position,orientation,&aabbMin.getX(),&aabbMax.getX(),writeToGpu);
		

	}

	if (mass>0.f)
		m_numDynamicPhysicsInstances++;

	m_numPhysicsInstances++;
	return bodyIndex;
}

int CLPhysicsDemo::registerSoftbodyInstance(const btVector3& aabbMin, const btVector3& aabbMax, int collisionObjectIndex)
{
	assert(useSapGpuBroadphase);

	if (useSapGpuBroadphase)
	{
		m_data->m_BroadphaseSap->createProxy(aabbMin,aabbMax,collisionObjectIndex,1,1);
		m_numSoftbodyInstances++;
	}	

	return collisionObjectIndex;
}

void	CLPhysicsDemo::init(int preferredDevice, int preferredPlatform, bool useInterop)
{
	
	InitCL(preferredDevice,preferredPlatform,useInterop);


	m_narrowphaseAndSolver = new btGpuNarrowphaseAndSolver(g_cxMainContext,g_device,g_cqCommandQue);
	g_narrowphaseAndSolver = m_narrowphaseAndSolver;

	
	//adl::Solver<adl::TYPE_CL>::allocate(g_deviceCL->allocate(
	m_data->m_linVelBuf = new btOpenCLArray<btVector3>(g_cxMainContext,g_cqCommandQue,MAX_CONVEX_BODIES_CL,false);
	m_data->m_angVelBuf = new btOpenCLArray<btVector3>(g_cxMainContext,g_cqCommandQue,MAX_CONVEX_BODIES_CL,false);
	m_data->m_bodyTimes = new btOpenCLArray<float>(g_cxMainContext,g_cqCommandQue,MAX_CONVEX_BODIES_CL,false);
	
	m_data->m_localShapeAABBGPU = new btOpenCLArray<btAABBHost>(g_cxMainContext,g_cqCommandQue,MAX_CONVEX_SHAPES_CL,false);
	m_data->m_localShapeAABBCPU = new btAlignedObjectArray<btAABBHost>;
	

	

	
	
	
	int maxObjects = btMax(256,MAX_CONVEX_BODIES_CL);
	int maxPairsSmallProxy = 32;
	

	if (useSapGpuBroadphase)
	{
			m_data->m_BroadphaseSap = new btGpuSapBroadphase(g_cxMainContext ,g_device,g_cqCommandQue);//overlappingPairCache,btVector3(4.f, 4.f, 4.f), 128, 128, 128,maxObjects, maxObjects, maxPairsSmallProxy, 100.f, 128,
	} else
	{
		btOverlappingPairCache* overlappingPairCache=0;
		m_data->m_BroadphaseGrid = new btGridBroadphaseCl(overlappingPairCache,btVector3(4.f, 4.f, 4.f), 128, 128, 128,maxObjects, maxObjects, maxPairsSmallProxy, 100.f, 128,
			g_cxMainContext ,g_device,g_cqCommandQue);
	}		//g_cxMainContext ,g_device,g_cqCommandQue);
	

	m_data->m_pgsSolver = new btPgsJacobiSolver();
	

	cl_program prog = btOpenCLUtils::compileCLProgramFromString(g_cxMainContext,g_device,interopKernelString,0,"",INTEROPKERNEL_SRC_PATH);
	g_integrateTransformsKernel = btOpenCLUtils::compileCLKernelFromString(g_cxMainContext, g_device,interopKernelString, "integrateTransformsKernel" ,0,prog);
	g_integrateTransformsKernel2 = btOpenCLUtils::compileCLKernelFromString(g_cxMainContext, g_device,interopKernelString, "integrateTransformsKernel2" ,0,prog);

	initFindPairs(gFpIO, g_cxMainContext, g_device, g_cqCommandQue, MAX_CONVEX_BODIES_CL);

	


}
	


void CLPhysicsDemo::writeVelocitiesToGpu()
{
	m_data->m_linVelBuf->copyFromHost(m_data->m_linVelHost);
	m_data->m_angVelBuf->copyFromHost(m_data->m_angVelHost);
	m_data->m_bodyTimes->copyFromHost(m_data->m_bodyTimesHost);
	clFinish(g_cqCommandQue);
}




void	CLPhysicsDemo::cleanup()
{
	delete m_narrowphaseAndSolver;
	m_narrowphaseAndSolver = 0;

	delete m_data->m_linVelBuf;
	delete m_data->m_angVelBuf;
	delete m_data->m_bodyTimes;
	delete m_data->m_localShapeAABBCPU;
	delete m_data->m_localShapeAABBGPU;

	delete m_data->m_BroadphaseSap;
	delete m_data->m_BroadphaseGrid;
	delete m_data->m_pgsSolver;

	m_data=0;

	delete s_convexHeightField;
}



void	CLPhysicsDemo::setObjectTransform(const float* position, const float* orientation, int objectIndex)
{
	m_narrowphaseAndSolver->setObjectTransform(position, orientation, objectIndex);
}

void	CLPhysicsDemo::setObjectLinearVelocity(const float* linVel, int objectIndex)
{
	m_data->m_linVelHost[objectIndex].setValue(linVel[0],linVel[1],linVel[2]);
	m_data->m_linVelBuf->copyFromHostPointer((const btVector3*)linVel,1,objectIndex,true);
}


struct ConvexPolyhedronCL2
{
	btVector3		m_localCenter;
	btVector3		m_extents;
	btVector3		mC;
	btVector3		mE;

	btScalar		m_radius;
	int	m_faceOffset;
	int m_numFaces;
	int	m_numVertices;

	int m_vertexOffset;
	int	m_uniqueEdgesOffset;
	int	m_numUniqueEdges;
	int m_unused;	
};

#include "../../dynamics/basic_demo/Stubs/AdlRigidBody.h"

struct Body2
{

	float4 m_pos;
	Quaternion m_quat;
	float4 m_linVel;
	float4 m_angVel;

	u32 m_collidableIdx;
	float m_invMass;
	float m_restituitionCoeff;
	float m_frictionCoeff;
			
};


void	CLPhysicsDemo::stepSimulation()
{
	int sz = sizeof(ConvexPolyhedronCL2);
	int sz1 = sizeof(ConvexPolyhedronCL);
	btAssert(sz==sz1);

	int b1 = sizeof(Body2);
	int b2 = sizeof(RigidBodyBase::Body);
	btAssert(b1==b2);

	int ciErrNum=CL_SUCCESS;


	
	if (1 && m_numPhysicsInstances)
	{

		gFpIO.m_numObjects = m_numPhysicsInstances;
		
		
		if (useSapGpuBroadphase)
		{
			gFpIO.m_dAABB = m_data->m_BroadphaseSap->getAabbBuffer();
		} else
		{
			gFpIO.m_dAABB = m_data->m_BroadphaseGrid->getAabbBuffer();
		}
		gFpIO.m_dlocalShapeAABB = (cl_mem)m_data->m_localShapeAABBGPU->getBufferCL();
		gFpIO.m_numOverlap = 0;


		{
			BT_PROFILE("setupGpuAabbs");
			setupGpuAabbsFull(gFpIO,m_narrowphaseAndSolver->getBodiesGpu(), m_narrowphaseAndSolver->getCollidablesGpu() );
        //    setupGpuAabbsSimple(gFpIO);
		}

		{

		}

		if (1)
		{
			BT_PROFILE("calculateOverlappingPairs");
			
			if (useSapGpuBroadphase)
			{
				m_data->m_BroadphaseSap->calculateOverlappingPairs();
				gFpIO.m_numOverlap = m_data->m_BroadphaseSap->getNumOverlap();
				gFpIO.m_dAllOverlappingPairs = m_data->m_BroadphaseSap->getOverlappingPairBuffer();
			}
			else
			{
				m_data->m_BroadphaseGrid->calculateOverlappingPairs();
				gFpIO.m_dAllOverlappingPairs = m_data->m_BroadphaseGrid->getOverlappingPairBuffer();
				gFpIO.m_numOverlap = m_data->m_BroadphaseGrid->getNumOverlap();
			}
		}
		
		//printf("gFpIO.m_numOverlap = %d\n",gFpIO.m_numOverlap );
		if (gFpIO.m_numOverlap>=0 && gFpIO.m_numOverlap<MAX_BROADPHASE_COLLISION_CL)
		{
			//colorPairsOpenCL(gFpIO);

			if (runOpenCLKernels)
			{
				{
					BT_PROFILE("setupBodies");
					if (m_narrowphaseAndSolver)
						setupBodies(gFpIO, m_data->m_linVelBuf->getBufferCL(), m_data->m_angVelBuf->getBufferCL(), m_narrowphaseAndSolver->getBodiesGpu(), m_narrowphaseAndSolver->getBodyInertiasGpu());
				}
				
				{
					BT_PROFILE("computeContacts");
					if (m_narrowphaseAndSolver)
						m_narrowphaseAndSolver->computeContacts(gFpIO.m_dAllOverlappingPairs,gFpIO.m_numOverlap, gFpIO.m_dAABB,gFpIO.m_numObjects);
				}

				bool useOpenCLSolver = true;
				if (useOpenCLSolver)
				{
					BT_PROFILE("solve Contact Constraints (OpenCL)");
					if (m_narrowphaseAndSolver)
						m_narrowphaseAndSolver->solveContacts();
				} else
				{
					BT_PROFILE("solve Contact Constraints CPU/serial");
					if (m_narrowphaseAndSolver && m_data->m_pgsSolver && m_narrowphaseAndSolver->getNumContactsGpu())
					{
						btGpuNarrowphaseAndSolver* np = m_narrowphaseAndSolver;
						
						btAlignedObjectArray<RigidBodyBase::Body> hostBodies;
						btOpenCLArray<RigidBodyBase::Body> gpuBodies(g_cxMainContext,g_cqCommandQue,0,true);
						gpuBodies.setFromOpenCLBuffer(np->getBodiesGpu(),np->getNumBodiesGpu());
						gpuBodies.copyToHost(hostBodies);

						btAlignedObjectArray<RigidBodyBase::Inertia> hostInertias;
						btOpenCLArray<RigidBodyBase::Inertia> gpuInertias(g_cxMainContext,g_cqCommandQue,0,true);
						gpuInertias.setFromOpenCLBuffer(np->getBodyInertiasGpu(),np->getNumBodiesGpu());
						gpuInertias.copyToHost(hostInertias);

						btAlignedObjectArray<Contact4> hostContacts;
						btOpenCLArray<Contact4> gpuContacts(g_cxMainContext,g_cqCommandQue,0,true);
						gpuContacts.setFromOpenCLBuffer(np->getContactsGpu(),np->getNumContactsGpu());
						gpuContacts.copyToHost(hostContacts);

						{
							BT_PROFILE("pgsSolver::solveContacts");
							m_data->m_pgsSolver->solveContacts(np->getNumBodiesGpu(),&hostBodies[0],&hostInertias[0],np->getNumContactsGpu(),&hostContacts[0]);
						}
						
						gpuBodies.copyFromHost(hostBodies);


					}
					
					
				}
				
				{
					BT_PROFILE("copyBodyVelocities");
					if (m_narrowphaseAndSolver)
						copyBodyVelocities(gFpIO, m_data->m_linVelBuf->getBufferCL(), m_data->m_angVelBuf->getBufferCL(), m_narrowphaseAndSolver->getBodiesGpu());

				}
			}

		} else
		{
			printf("error, gFpIO.m_numOverlap = %d\n",gFpIO.m_numOverlap);
			btAssert(0);
		}


		{
			BT_PROFILE("integrateTransforms");

			if (runOpenCLKernels)
			{
				bool integrateOnGpu = true;
				if (integrateOnGpu)
				{
					int numObjects = m_numPhysicsInstances;
					int offset = m_maxShapeBufferCapacityInBytes/4;

					ciErrNum = clSetKernelArg(g_integrateTransformsKernel2, 0, sizeof(int), &offset);
					ciErrNum = clSetKernelArg(g_integrateTransformsKernel2, 1, sizeof(int), &numObjects);
					cl_mem bodyGpuBuffer = m_narrowphaseAndSolver->getBodiesGpu();
					ciErrNum = clSetKernelArg(g_integrateTransformsKernel2, 2, sizeof(cl_mem), (void*)&bodyGpuBuffer );
	
					cl_mem lv = m_data->m_linVelBuf->getBufferCL();
					cl_mem av = m_data->m_angVelBuf->getBufferCL();
					cl_mem btimes = m_data->m_bodyTimes->getBufferCL();

					ciErrNum = clSetKernelArg(g_integrateTransformsKernel2, 3, sizeof(cl_mem), (void*)&lv);
					ciErrNum = clSetKernelArg(g_integrateTransformsKernel2, 4, sizeof(cl_mem), (void*)&av);
					ciErrNum = clSetKernelArg(g_integrateTransformsKernel2, 5, sizeof(cl_mem), (void*)&btimes);
					
					
					

					size_t workGroupSize = 64;
					size_t	numWorkItems = workGroupSize*((m_numPhysicsInstances + (workGroupSize)) / workGroupSize);
				
					if (workGroupSize>numWorkItems)
						workGroupSize=numWorkItems;

					ciErrNum = clEnqueueNDRangeKernel(g_cqCommandQue, g_integrateTransformsKernel2, 1, NULL, &numWorkItems, &workGroupSize,0 ,0 ,0);
					oclCHECKERROR(ciErrNum, CL_SUCCESS);
				} else
				{
#ifdef _WIN32
					//debug velocity
					btAlignedObjectArray<btVector3> linvel;
					m_data->m_linVelBuf->copyToHost(linvel);
					for (int i=0;i<linvel.size();i++)
					{
						btAssert(_finite(linvel[i].x()));
					}
#endif
                    btAssert(0);

				}
			} 
		}

		
			

	}

	oclCHECKERROR(ciErrNum, CL_SUCCESS);

	

	if (runOpenCLKernels)
	{
		BT_PROFILE("clFinish");
		clFinish(g_cqCommandQue);
	}

	
}

cl_mem	CLPhysicsDemo::getBodiesGpu()
{
	return m_narrowphaseAndSolver->getBodiesGpu();
}

btGpuSapBroadphase* CLPhysicsDemo::getGpuSapBroadphase() 
{ 
	return m_data->m_BroadphaseSap; 
}