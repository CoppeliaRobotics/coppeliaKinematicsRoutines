#pragma once

#include <vector>
#include "mathDefines.h"
#include "7Vector.h"
#ifdef SIM_MATH_DOUBLE
    typedef double simReal;
#else
    typedef float simReal;
#endif

const simReal IK_DIVISION_FACTOR=simReal(100.0);

#define SIM_IS_BIT_SET(var,bit) (((var) & (1<<(bit)))!=0)
#define ik_handleflag_tipframe 0x00400000

int _getLoadingMapping(const std::vector<int>* map,int oldVal);

std::string ikGetLastError();
bool ikCreateEnvironment(int* environmentHandle=nullptr,bool protectedEnvironment=false);
bool ikLoad(const unsigned char* data,size_t dataLength);
bool ikSwitchEnvironment(int handle,bool allowAlsoProtectedEnvironment=false);
bool ikEraseEnvironment(int* switchedEnvironmentHandle=nullptr);
void ikReleaseBuffer(void* buffer);

bool ikGetObjectHandle(const char* objectName,int* objectHandle);
bool ikDoesObjectExist(const char* objectName);
bool ikEraseObject(int objectHandle);
bool ikGetObjectParent(int objectHandle,int* parentObjectHandle);
bool ikSetObjectParent(int objectHandle,int parentObjectHandle,bool keepInPlace);

bool ikCreateFrame(const char* frameName/*=nullptr*/,int* frameHandle);
bool ikGetLinkedFrame(int frameHandle,int* linkedFrameHandle);
bool ikSetLinkedFrame(int frameHandle,int linkedFrameHandle);

bool ikCreateJoint(const char* jointName/*=nullptr*/,int jointType,int* jointHandle);
bool ikGetJointMode(int jointHandle,int* mode);
bool ikSetJointMode(int jointHandle,int jointMode);
bool ikGetJointInterval(int jointHandle,bool* cyclic,simReal* intervalMinAndRange);
bool ikSetJointInterval(int jointHandle,bool cyclic,const simReal* intervalMinAndRange=nullptr);
bool ikGetJointScrewPitch(int jointHandle,simReal* pitch);
bool ikSetJointScrewPitch(int jointHandle,simReal pitch);
bool ikGetJointIkWeight(int jointHandle,simReal* ikWeight);
bool ikSetJointIkWeight(int jointHandle,simReal ikWeight);
bool ikGetJointMaxStepSize(int jointHandle,simReal* maxStepSize);
bool ikSetJointMaxStepSize(int jointHandle,simReal maxStepSize);
bool ikGetJointDependency(int jointHandle,int* dependencyJointHandle,simReal* offset,simReal* mult);
bool ikSetJointDependency(int jointHandle,int dependencyJointHandle,simReal offset=0.0,simReal mult=1.0);
bool ikGetJointPosition(int jointHandle,simReal* position);
bool ikSetJointPosition(int jointHandle,simReal position);
bool ikGetJointMatrix(int jointHandle,C4X4Matrix* matrix);
bool ikSetSphericalJointMatrix(int jointHandle,const C3X3Matrix* rotMatrix);
bool ikGetJointTransformation(int jointHandle,C7Vector* transf);
bool ikSetSphericalJointQuaternion(int jointHandle,const C4Vector* quaternion);

bool ikGetIkGroupHandle(const char* ikGroupName,int* ikGroupHandle);
bool ikDoesIkGroupExist(const char* ikGroupName);
bool ikCreateIkGroup(const char* ikGroupName/*=nullptr*/,int* ikGroupHandle);

bool ikGetIkGroupFlags(int ikGroupHandle,int* flags);
bool ikSetIkGroupFlags(int ikGroupHandle,int flags);
bool ikGetIkGroupCalculation(int ikGroupHandle,int* method,simReal* damping,int* maxIterations);
bool ikSetIkGroupCalculation(int ikGroupHandle,int method,simReal damping,int maxIterations);
bool ikGetIkGroupLimitThresholds(int ikGroupHandle,simReal* linearAndAngularThresholds);
bool ikSetIkGroupLimitThresholds(int ikGroupHandle,const simReal* linearAndAngularThresholds);

bool ikAddIkElement(int ikGroupHandle,int tipHandle,int* ikElementIndex);
bool ikGetIkElementEnabled(int ikGroupHandle,int ikElementIndex,bool* enabled);
bool ikSetIkElementEnabled(int ikGroupHandle,int ikElementIndex,bool enabled);
bool ikGetIkElementBase(int ikGroupHandle,int ikElementIndex,int* baseHandle,int* constraintsBaseHandle);
bool ikSetIkElementBase(int ikGroupHandle,int ikElementIndex,int baseHandle,int constraintsBaseHandle=-1);
bool ikGetIkElementConstraints(int ikGroupHandle,int ikElementIndex,int* constraints);
bool ikSetIkElementConstraints(int ikGroupHandle,int ikElementIndex,int constraints);
bool ikGetIkElementPrecision(int ikGroupHandle,int ikElementIndex,simReal* linearPrecision,simReal* angularPrecision);
bool ikSetIkElementPrecision(int ikGroupHandle,int ikElementIndex,simReal linearPrecision,simReal angularPrecision);
bool ikGetIkElementWeights(int ikGroupHandle,int ikElementIndex,simReal* linearWeight,simReal* angularWeight);
bool ikSetIkElementWeights(int ikGroupHandle,int ikElementIndex,simReal linearWeight,simReal angularWeight);

bool ikHandleIkGroup(int ikGroupHandle,int* result=nullptr);
bool ikComputeJacobian(int ikGroupHandle,int options,bool* success=nullptr);
simReal* ikGetJacobian(int ikGroupHandle,size_t* matrixSize);
bool ikGetManipulability(int ikGroupHandle,simReal* manip);

int ikGetConfigForTipPose(int ikGroupHandle,size_t jointCnt,const int* jointHandles,simReal thresholdDist,int maxIterations,simReal* retConfig,const simReal* metric=nullptr,bool(*validationCallback)(simReal*)=nullptr,const int* jointOptions=nullptr,const simReal* lowLimits=nullptr,const simReal* ranges=nullptr);

bool ikGetObjectTransformation(int objectHandle,int relativeToObjectHandle,C7Vector* transf);
bool ikSetObjectTransformation(int objectHandle,int relativeToObjectHandle,const C7Vector* transf);
bool ikGetObjectMatrix(int objectHandle,int relativeToObjectHandle,C4X4Matrix* matrix);
bool ikSetObjectMatrix(int objectHandle,int relativeToObjectHandle,const C4X4Matrix* matrix);
