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

#ifndef GPU_NARROWPHASE_SOLVER_H
#define GPU_NARROWPHASE_SOLVER_H
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"

#include "btCollidable.h"

//#define MAX_CONVEX_BODIES_CL 8*1024
#define MAX_CONVEX_BODIES_CL 128*1024
//#define MAX_CONVEX_BODIES_CL 256*1024
//#define MAX_CONVEX_BODIES_CL 32*1024
//#define MAX_CONVEX_BODIES_CL 16*1024
//#define MAX_PAIRS_PER_BODY_CL 64
#define MAX_PAIRS_PER_BODY_CL 16
#define MAX_CONVEX_SHAPES_CL 8192
#define MAX_COMPOUND_CHILD_SHAPES 8192

#define MAX_FACES_PER_SHAPE 64
#define MAX_VERTICES_PER_FACE 64//mainly use for contact generation

#define MAX_BROADPHASE_COLLISION_CL (MAX_CONVEX_BODIES_CL*MAX_PAIRS_PER_BODY_CL)

/*
#define MAX_CONVEX_BODIES_CL 1024
#define MAX_PAIRS_PER_BODY_CL 32
#define MAX_CONVEX_SHAPES_CL 8192
#define MAX_BROADPHASE_COLLISION_CL (MAX_CONVEX_BODIES_CL*MAX_PAIRS_PER_BODY_CL)
*/

namespace adl
{
	struct DeviceCL;
};


struct	CustomDispatchData;

#include "../basic_initialize/btOpenCLInclude.h"

#include "btConvexUtility.h"
#include "../gpu_rigidbody_pipeline2/ConvexHullContact.h"


//#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "../../dynamics/basic_demo/Stubs/AdlMath.h"
#include "../../dynamics/basic_demo/Stubs/AdlContact4.h"
#include "../../dynamics/basic_demo/Stubs/AdlQuaternion.h"
#include "../../dynamics/basic_demo/Stubs/AdlRigidBody.h"

#include "../../dynamics/basic_demo/Stubs/ChNarrowPhase.h"
#include "../../dynamics/basic_demo/Stubs/Solver.h"

enum
{
	BT_SOLVER_N_SPLIT = 16,
	BT_SOLVER_N_BATCHES = 4,
	BT_SOLVER_N_OBJ_PER_SPLIT = 10,
	BT_SOLVER_N_TASKS_PER_BATCH = BT_SOLVER_N_SPLIT*BT_SOLVER_N_SPLIT,
};

struct	CustomDispatchData
{
	btOpenCLArray<ChNarrowphase::ShapeData>* m_ShapeBuffer;
#ifndef DISABLE_CONVEX_HEIGHTFIELD
	btAlignedObjectArray<ConvexHeightField*>* m_shapePointers;
#endif //DISABLE_CONVEX_HEIGHTFIELD
    
	btAlignedObjectArray<btConvexUtility*>* m_convexData;
    
	btAlignedObjectArray<ConvexPolyhedronCL> m_convexPolyhedra;
	btAlignedObjectArray<btVector3> m_uniqueEdges;
	btAlignedObjectArray<btVector3> m_convexVertices;
	btAlignedObjectArray<int> m_convexIndices;
    
	btOpenCLArray<ConvexPolyhedronCL>* m_convexPolyhedraGPU;
	btOpenCLArray<btVector3>* m_uniqueEdgesGPU;
	btOpenCLArray<btVector3>* m_convexVerticesGPU;
	btOpenCLArray<int>* m_convexIndicesGPU;
    
    btOpenCLArray<float4>* m_worldVertsB1GPU;
    btOpenCLArray<int4>* m_clippingFacesOutGPU;
    btOpenCLArray<float4>* m_worldNormalsAGPU;
    btOpenCLArray<float4>* m_worldVertsA1GPU;
    btOpenCLArray<float4>* m_worldVertsB2GPU;
    
	btAlignedObjectArray<btGpuChildShape> m_cpuChildShapes;
	btOpenCLArray<btGpuChildShape>*	m_gpuChildShapes;
    
	btAlignedObjectArray<btGpuFace> m_convexFaces;
	btOpenCLArray<btGpuFace>* m_convexFacesGPU;
    
	GpuSatCollision*	m_gpuSatCollision;
	
    
    
    
    
	btAlignedObjectArray<int2>* m_pBufPairsCPU;
    
	btOpenCLArray<int2>* m_convexPairsOutGPU;
	btOpenCLArray<int2>* m_planePairs;
    
	btOpenCLArray<Contact4>* m_pBufContactOutGPU;
	btAlignedObjectArray<Contact4>* m_pBufContactOutCPU;
	//adl::ChNarrowphase<adl::TYPE_CL>::Data* m_Data;
	ChNarrowphase* m_narrowPhase;
    
    
	btAlignedObjectArray<RigidBodyBase::Body>* m_bodyBufferCPU;
	btOpenCLArray<RigidBodyBase::Body>* m_bodyBufferGPU;
    
	btAlignedObjectArray<RigidBodyBase::Inertia>*	m_inertiaBufferCPU;
	btOpenCLArray<RigidBodyBase::Inertia>*	m_inertiaBufferGPU;
    
	Solver* m_solverGPU;
    
	btOpenCLArray<Constraint4>*		m_contactCGPU;
	void*			m_frictionCGPU;
    
	int m_numAcceleratedShapes;
	int m_numAcceleratedRigidBodies;
    
	btAlignedObjectArray<btCollidable>	m_collidablesCPU;
	btOpenCLArray<btCollidable>*	m_collidablesGPU;
    
    
};

class btGpuNarrowphaseAndSolver
{
protected:

	CustomDispatchData*	m_internalData;
	int m_acceleratedCompanionShapeIndex;
	int m_planeBodyIndex;
	int	m_static0Index;

	cl_context m_context;
	cl_device_id m_device;
	cl_command_queue m_queue;


public:

	


	btGpuNarrowphaseAndSolver(cl_context vtx, cl_device_id dev, cl_command_queue q);

	virtual ~btGpuNarrowphaseAndSolver(void);

	
	int registerCompoundShape(btAlignedObjectArray<btGpuChildShape>* childShapes);
	int registerFace(const btVector3& faceNormal, float faceConstant);
	int registerConcaveMeshShape(btAlignedObjectArray<btVector3>* vertices, btAlignedObjectArray<int>* indices, btCollidable& col, const float* scaling);
	int registerConcaveMeshShape(class objLoader* obj, btCollidable& col, const float* scaling);
	int registerConvexHullShape(class btConvexUtility* convexPtr, btCollidable& col);
	int registerConvexHeightfield(class ConvexHeightField* convexShape,btCollidable& col);
	int registerRigidBody(int collidableIndex, float mass, const float* position, const float* orientation, const float* aabbMin, const float* aabbMax,bool writeToGpu);
	void setObjectTransform(const float* position, const float* orientation , int bodyIndex);

	void	writeAllBodiesToGpu();

	void	readbackAllBodiesToCpu();
	void	getObjectTransformFromCpu(float* position, float* orientation , int bodyIndex) const;

	virtual void computeContacts(cl_mem broadphasePairs, int numBroadphasePairs, cl_mem aabbs, int numObjects);
	virtual void solveContacts();

	cl_mem	getBodiesGpu();
	int	getNumBodiesGpu() const;
	cl_mem	getBodyInertiasGpu();
	int	getNumBodyInertiasGpu() const;
	cl_mem	getCollidablesGpu();
	int		getNumCollidablesGpu() const;
	cl_mem	getContactsGpu();
	int	getNumContactsGpu() const;





	int allocateCollidable();

	btCollidable& getCollidableCpu(int collidableIndex);
	const btCollidable& getCollidableCpu(int collidableIndex) const;
	
	const CustomDispatchData* getCustomDispatchData() const { return m_internalData; }
};

#endif //GPU_NARROWPHASE_SOLVER_H