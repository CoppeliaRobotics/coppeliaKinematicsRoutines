#pragma once

#include <vector>
#include <simMath/mathDefines.h>
#include <simMath/7Vector.h>
#include <string>

#define SIM_IS_BIT_SET(var,bit) (((var) & (1<<(bit)))!=0)
#define SIM_SET_BIT(var,bit) ((var) |= (1<<(bit)))
#define SIM_CLEAR_BIT(var,bit) ((var) &= (~(1<<(bit))))
#define SIM_SET_CLEAR_BIT(var,bit,on) ((on) ? SIM_SET_BIT((var),(bit)) : SIM_CLEAR_BIT((var),(bit)) )

#define ik_handleflag_tipdummy 0x00400000

#define ik_objecttype_joint 1
#define ik_objecttype_dummy 4

#define ik_jointmode_passive 0
#define ik_jointmode_ik 2

#define ik_jointtype_revolute 10
#define ik_jointtype_prismatic 11
#define ik_jointtype_spherical 12

#define ik_handle_world -1
#define ik_handle_all -2
#define ik_handle_parent -11

#define ik_constraint_x 1
#define ik_constraint_y 2
#define ik_constraint_z 4
#define ik_constraint_alpha_beta 8
#define ik_constraint_gamma 16
#define ik_constraint_position (1+2+4)
#define ik_constraint_orientation (8+16)
#define ik_constraint_pose (1+2+4+8+16)

#define ik_method_pseudo_inverse 0 /* tiny little bit of hard-coded damping */
#define ik_method_damped_least_squares 1
#define ik_method_jacobian_transpose 2
#define ik_method_undamped_pseudo_inverse 3

/* following 3 deprecated */
#define ik_result_not_performed 0
#define ik_result_success 1
#define ik_result_fail 2

#define ik_calc_notperformed 1
#define ik_calc_cannotinvert 2
#define ik_calc_notwithintolerance 16
#define ik_calc_stepstoobig 32
#define ik_calc_limithit 64
#define ik_calc_invalidcallbackdata 128

#define ik_group_enabled 1
#define ik_group_ignoremaxsteps 2
#define ik_group_restoreonbadlintol 4
#define ik_group_restoreonbadangtol 8
#define ik_group_stoponlimithit 16
#define ik_group_avoidlimits 64

void _setLastError(const char* errStr,const char* substr1=nullptr,const char* substr2=nullptr);
void _setLastError(const char* errStr,int intVal1,int intVal2=-1);
int _getLoadingMapping(const std::vector<int>* map,int oldVal);

std::string ikGetLastError();
void ikSetLogCallback(bool(*logCallback)(int,const char*,const char*));
void ikSetVerbosity(int level);
bool ikCreateEnvironment(int* environmentHandle=nullptr,int flags=0);
bool ikDuplicateEnvironment(int* duplicateEnvironmentHandle);
unsigned char* ikSave(size_t* dataLength);
bool ikLoad(const unsigned char* data,size_t dataLength);
bool ikSwitchEnvironment(int handle,bool allowAlsoProtectedEnvironment=false);
bool ikEraseEnvironment(int* switchedEnvironmentHandle=nullptr);
void ikReleaseBuffer(void* buffer);

bool ikGetObjectHandle(const char* objectName,int* objectHandle);
bool ikGetObjects(size_t index,int* objectHandle=nullptr,std::string* objectName=nullptr,bool* isJoint=nullptr,int* jointType=nullptr);
bool ikDoesObjectExist(const char* objectName);
bool ikEraseObject(int objectHandle);
bool ikGetObjectType(int objectHandle,int* objectType);
bool ikGetObjectParent(int objectHandle,int* parentObjectHandle);
bool ikSetObjectParent(int objectHandle,int parentObjectHandle,bool keepInPlace);

bool ikCreateDummy(const char* dummyName/*=nullptr*/,int* dummyHandle);
bool ikGetTargetDummy(int dummyHandle,int* targetDummyHandle);
bool ikSetTargetDummy(int dummyHandle,int targetDummyHandle);

bool ikCreateJoint(const char* jointName/*=nullptr*/,int jointType,int* jointHandle);
bool ikGetJointType(int jointHandle,int* theType);
bool ikGetJointMode(int jointHandle,int* mode);
bool ikSetJointMode(int jointHandle,int jointMode);
bool ikGetJointInterval(int jointHandle,bool* cyclic,double* intervalMinAndRange);
bool ikSetJointInterval(int jointHandle,bool cyclic,const double* intervalMinAndRange=nullptr);
bool ikGetJointScrewLead(int jointHandle,double* lead);
bool ikSetJointScrewLead(int jointHandle,double lead);
bool ikGetJointWeight(int jointHandle,double* ikWeight);
bool ikSetJointWeight(int jointHandle,double ikWeight);
bool ikGetJointLimitMargin(int jointHandle,double* m);
bool ikSetJointLimitMargin(int jointHandle,double m);
bool ikGetJointMaxStepSize(int jointHandle,double* maxStepSize);
bool ikSetJointMaxStepSize(int jointHandle,double maxStepSize);
bool ikGetJointDependency(int jointHandle,int* dependencyJointHandle,double* offset,double* mult);
bool ikSetJointDependency(int jointHandle,int dependencyJointHandle,double offset=0.0,double mult=1.0,double(*cb)(int ikEnv,int slaveJoint,double masterPos)=nullptr);
bool ikGetJointPosition(int jointHandle,double* position);
bool ikSetJointPosition(int jointHandle,double position);
bool ikGetJointMatrix(int jointHandle,C4X4Matrix* matrix);
bool ikSetSphericalJointMatrix(int jointHandle,const C3X3Matrix* rotMatrix);
bool ikGetJointTransformation(int jointHandle,C7Vector* transf);
bool ikSetSphericalJointQuaternion(int jointHandle,const C4Vector* quaternion);

bool ikGetGroupHandle(const char* ikGroupName,int* ikGroupHandle);
bool ikDoesGroupExist(const char* ikGroupName);
bool ikCreateGroup(const char* ikGroupName/*=nullptr*/,int* ikGroupHandle);
bool ikEraseGroup(int ikGroupHandle);

bool ikGetGroupFlags(int ikGroupHandle,int* flags);
bool ikSetGroupFlags(int ikGroupHandle,int flags);
bool ikGetGroupCalculation(int ikGroupHandle,int* method,double* damping,int* maxIterations);
bool ikSetGroupCalculation(int ikGroupHandle,int method,double damping,int maxIterations);
bool ikGetGroupJointLimitHits(int ikGroupHandle,std::vector<int>* jointHandles,std::vector<double>* underOrOvershots);
bool ikGetGroupJoints(int ikGroupHandle,std::vector<int>* jointHandles);

bool ikAddElement(int ikGroupHandle,int tipHandle,int* ikElementHandle);
bool ikEraseElement(int ikGroupHandle,int ikElementHandle);
bool ikGetElementFlags(int ikGroupHandle,int ikElementHandle,int* flags);
bool ikSetElementFlags(int ikGroupHandle,int ikElementHandle,int flags);
bool ikGetElementBase(int ikGroupHandle,int ikElementHandle,int* baseHandle,int* constraintsBaseHandle);
bool ikSetElementBase(int ikGroupHandle,int ikElementHandle,int baseHandle,int constraintsBaseHandle=-1);
bool ikGetElementConstraints(int ikGroupHandle,int ikElementHandle,int* constraints);
bool ikSetElementConstraints(int ikGroupHandle,int ikElementHandle,int constraints);
bool ikGetElementPrecision(int ikGroupHandle,int ikElementHandle,double* linearPrecision,double* angularPrecision);
bool ikSetElementPrecision(int ikGroupHandle,int ikElementHandle,double linearPrecision,double angularPrecision);
bool ikGetElementWeights(int ikGroupHandle,int ikElementHandle,double* linearWeight,double* angularWeight,double* elementWeight=nullptr);
bool ikSetElementWeights(int ikGroupHandle,int ikElementHandle,double linearWeight,double angularWeight,double elementWeight);

bool ikHandleGroups(const std::vector<int>* ikGroupHandles,int* result=nullptr,double* precision=nullptr,int(*cb)(const int*,double*,const int*,const int*,const int*,const int*,double*,double*,double*,int,int)=nullptr);
bool ikComputeJacobian(int baseHandle,int jointHandle,int constraints,const C7Vector* tipPose,const C7Vector* targetPose,const C7Vector* taltBasePose,std::vector<double>* jacobian,std::vector<double>* errorVect);
bool ikComputeGroupJacobian(int ikGroupHandle,std::vector<double>* jacobian,std::vector<double>* errorVect);
int ikFindConfig(int ikGroupHandle,size_t jointCnt,const int* jointHandles,double thresholdDist,int maxTimeInMs,double* retConfig,const double* metric=nullptr,bool(*validationCallback)(double*)=nullptr);

bool ikGetObjectTransformation(int objectHandle,int relativeToObjectHandle,C7Vector* transf);
bool ikSetObjectTransformation(int objectHandle,int relativeToObjectHandle,const C7Vector* transf);
bool ikGetObjectMatrix(int objectHandle,int relativeToObjectHandle,C4X4Matrix* matrix);
bool ikSetObjectMatrix(int objectHandle,int relativeToObjectHandle,const C4X4Matrix* matrix);

/* backward compatibility */
bool ikComputeJacobian_old(int ikGroupHandle,int options,bool* success=nullptr);
double* ikGetJacobian_old(int ikGroupHandle,size_t* matrixSize);
bool ikGetManipulability_old(int ikGroupHandle,double* manip);
int ikGetConfigForTipPose(int ikGroupHandle,size_t jointCnt,const int* jointHandles,double thresholdDist,int maxIterations,double* retConfig,const double* metric=nullptr,bool(*validationCallback)(double*)=nullptr,const int* jointOptions=nullptr,const double* lowLimits=nullptr,const double* ranges=nullptr);
bool ikGetJointIkWeight(int jointHandle,double* ikWeight);
bool ikSetJointIkWeight(int jointHandle,double ikWeight);
bool ikGetIkGroupHandle(const char* ikGroupName,int* ikGroupHandle);
bool ikDoesIkGroupExist(const char* ikGroupName);
bool ikCreateIkGroup(const char* ikGroupName,int* ikGroupHandle);
bool ikEraseIkGroup(int ikGroupHandle);
bool ikGetIkGroupFlags(int ikGroupHandle,int* flags);
bool ikSetIkGroupFlags(int ikGroupHandle,int flags);
bool ikGetIkGroupCalculation(int ikGroupHandle,int* method,double* damping,int* maxIterations);
bool ikSetIkGroupCalculation(int ikGroupHandle,int method,double damping,int maxIterations);
bool ikGetIkGroupJointLimitHits(int ikGroupHandle,std::vector<int>* jointHandles,std::vector<double>* underOrOvershots);
bool ikAddIkElement(int ikGroupHandle,int tipHandle,int* ikElementHandle);
bool ikEraseIkElement(int ikGroupHandle,int ikElementHandle);
bool ikGetIkElementFlags(int ikGroupHandle,int ikElementHandle,int* flags);
bool ikSetIkElementFlags(int ikGroupHandle,int ikElementHandle,int flags);
bool ikGetIkElementBase(int ikGroupHandle,int ikElementHandle,int* baseHandle,int* constraintsBaseHandle);
bool ikSetIkElementBase(int ikGroupHandle,int ikElementHandle,int baseHandle,int constraintsBaseHandle=-1);
bool ikGetIkElementConstraints(int ikGroupHandle,int ikElementHandle,int* constraints);
bool ikSetIkElementConstraints(int ikGroupHandle,int ikElementHandle,int constraints);
bool ikGetIkElementPrecision(int ikGroupHandle,int ikElementHandle,double* linearPrecision,double* angularPrecision);
bool ikSetIkElementPrecision(int ikGroupHandle,int ikElementHandle,double linearPrecision,double angularPrecision);
bool ikGetIkElementWeights(int ikGroupHandle,int ikElementHandle,double* linearWeight,double* angularWeight);
bool ikSetIkElementWeights(int ikGroupHandle,int ikElementHandle,double linearWeight,double angularWeight);
bool ikHandleIkGroup(int ikGroupHandle=ik_handle_all,int* result=nullptr,double* precision=nullptr,int(*cb)(const int*,double*,const int*,const int*,const int*,const int*,double*,double*,double*,int,int)=nullptr);
bool ikGetLinkedDummy(int dummyHandle,int* linkedDummyHandle);
bool ikSetLinkedDummy(int dummyHandle,int linkedDummyHandle);
bool ikGetJointScrewPitch(int jointHandle,double* pitch);
bool ikSetJointScrewPitch(int jointHandle,double pitch);
