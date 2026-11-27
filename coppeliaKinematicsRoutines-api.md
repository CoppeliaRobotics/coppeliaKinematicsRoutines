# Coppelia Kinematics Routines, API reference

All units, unless otherwise indicated, are specified in meters and radians.

## Alphabetical list

- [ikAddElement](#ikAddElement)
- [ikComputeGroupJacobian](#ikComputeGroupJacobian)
- [ikComputeJacobian](#ikComputeJacobian)
- [ikCreateDummy](#ikCreateDummy)
- [ikCreateEnvironment](#ikCreateEnvironment)
- [ikCreateGroup](#ikCreateGroup)
- [ikCreateJoint](#ikCreateJoint)
- [ikDoesGroupExist](#ikDoesGroupExist)
- [ikDoesObjectExist](#ikDoesObjectExist)
- [ikDuplicateEnvironment](#ikDuplicateEnvironment)
- [ikEraseEnvironment](#ikEraseEnvironment)
- [ikEraseObject](#ikEraseObject)
- [ikFindConfig](#ikFindConfig)
- [ikGetConfigForTipPose](#ikGetConfigForTipPose)
- [ikGetElementBase](#ikGetElementBase)
- [ikGetElementConstraints](#ikGetElementConstraints)
- [ikGetElementFlags](#ikGetElementFlags)
- [ikGetElementPrecision](#ikGetElementPrecision)
- [ikGetElementWeights](#ikGetElementWeights)
- [ikGetGroupCalculation](#ikGetGroupCalculation)
- [ikGetGroupFlags](#ikGetGroupFlags)
- [ikGetGroupHandle](#ikGetGroupHandle)
- [ikGetGroupJointLimitHits](#ikGetGroupJointLimitHits)
- [ikGetGroupJoints](#ikGetGroupJoints)
- [ikGetJointDependency](#ikGetJointDependency)
- [ikGetJointInterval](#ikGetJointInterval)
- [ikGetJointLimitMargin](#ikGetJointLimitMargin)
- [ikGetJointMatrix](#ikGetJointMatrix)
- [ikGetJointMaxStepSize](#ikGetJointMaxStepSize)
- [ikGetJointMode](#ikGetJointMode)
- [ikGetJointPosition](#ikGetJointPosition)
- [ikGetJointScrewLead](#ikGetJointScrewLead)
- [ikGetJointTransformation](#ikGetJointTransformation)
- [ikGetJointType](#ikGetJointType)
- [ikGetJointWeight](#ikGetJointWeight)
- [ikGetLastError](#ikGetLastError)
- [ikGetLinkedDummy](#ikGetLinkedDummy)
- [ikGetObjectHandle](#ikGetObjectHandle)
- [ikGetObjectMatrix](#ikGetObjectMatrix)
- [ikGetObjectParent](#ikGetObjectParent)
- [ikGetObjects](#ikGetObjects)
- [ikGetObjectTransformation](#ikGetObjectTransformation)
- [ikGetObjectType](#ikGetObjectType)
- [ikGetTargetDummy](#ikGetTargetDummy)
- [ikHandleGroups](#ikHandleGroups)
- [ikLoad](#ikLoad)
- [ikReleaseBuffer](#ikReleaseBuffer)
- [ikSave](#ikSave)
- [ikSetElementBase](#ikSetElementBase)
- [ikSetElementConstraints](#ikSetElementConstraints)
- [ikSetElementFlags](#ikSetElementFlags)
- [ikSetElementPrecision](#ikSetElementPrecision)
- [ikSetElementWeights](#ikSetElementWeights)
- [ikSetGroupCalculation](#ikSetGroupCalculation)
- [ikSetGroupFlags](#ikSetGroupFlags)
- [ikSetJointDependency](#ikSetJointDependency)
- [ikSetJointInterval](#ikSetJointInterval)
- [ikSetJointLimitMargin](#ikSetJointLimitMargin)
- [ikSetJointMaxStepSize](#ikSetJointMaxStepSize)
- [ikSetJointMode](#ikSetJointMode)
- [ikSetJointPosition](#ikSetJointPosition)
- [ikSetJointScrewLead](#ikSetJointScrewLead)
- [ikSetJointWeight](#ikSetJointWeight)
- [ikSetLinkedDummy](#ikSetLinkedDummy)
- [ikSetObjectMatrix](#ikSetObjectMatrix)
- [ikSetObjectParent](#ikSetObjectParent)
- [ikSetObjectTransformation](#ikSetObjectTransformation)
- [ikSetSphericalJointMatrix](#ikSetSphericalJointMatrix)
- [ikSetSphericalJointQuaternion](#ikSetSphericalJointQuaternion)
- [ikSetTargetDummy](#ikSetTargetDummy)
- [ikSwitchEnvironment](#ikSwitchEnvironment)

## Environment functions and helpers

- [ikCreateEnvironment](#ikCreateEnvironment)
- [ikEraseEnvironment](#ikEraseEnvironment)
- [ikDuplicateEnvironment](#ikDuplicateEnvironment)
- [ikSave](#ikSave)
- [ikLoad](#ikLoad)
- [ikSwitchEnvironment](#ikSwitchEnvironment)
- [ikGetLastError](#ikGetLastError)
- [ikReleaseBuffer](#ikReleaseBuffer)

## Objects

- [ikGetObjects](#ikGetObjects)
- [ikGetObjectHandle](#ikGetObjectHandle)
- [ikDoesObjectExist](#ikDoesObjectExist)
- [ikEraseObject](#ikEraseObject)
- [ikGetObjectParent](#ikGetObjectParent)
- [ikSetObjectParent](#ikSetObjectParent)
- [ikGetObjectTransformation](#ikGetObjectTransformation)
- [ikSetObjectTransformation](#ikSetObjectTransformation)
- [ikGetObjectType](#ikGetObjectType)
- [ikGetObjectMatrix](#ikGetObjectMatrix)
- [ikSetObjectMatrix](#ikSetObjectMatrix)

## Dummies

- [ikCreateDummy](#ikCreateDummy)
- [ikEraseObject](#ikEraseObject)
- [ikGetTargetDummy](#ikGetTargetDummy)
- [ikSetTargetDummy](#ikSetTargetDummy)

## Joints

- [ikCreateJoint](#ikCreateJoint)
- [ikEraseObject](#ikEraseObject)
- [ikGetJointPosition](#ikGetJointPosition)
- [ikSetJointPosition](#ikSetJointPosition)
- [ikGetJointMode](#ikGetJointMode)
- [ikSetJointMode](#ikSetJointMode)
- [ikGetJointInterval](#ikGetJointInterval)
- [ikSetJointInterval](#ikSetJointInterval)
- [ikGetJointDependency](#ikGetJointDependency)
- [ikSetJointDependency](#ikSetJointDependency)
- [ikGetJointWeight](#ikGetJointWeight)
- [ikSetJointWeight](#ikSetJointWeight)
- [ikGetJointLimitMargin](#ikGetJointLimitMargin)
- [ikSetJointLimitMargin](#ikSetJointLimitMargin)
- [ikGetJointMaxStepSize](#ikGetJointMaxStepSize)
- [ikSetJointMaxStepSize](#ikSetJointMaxStepSize)
- [ikGetJointScrewLead](#ikGetJointScrewLead)
- [ikSetJointScrewLead](#ikSetJointScrewLead)
- [ikGetJointTransformation](#ikGetJointTransformation)
- [ikSetSphericalJointQuaternion](#ikSetSphericalJointQuaternion)
- [ikGetJointType](#ikGetJointType)
- [ikGetJointMatrix](#ikGetJointMatrix)
- [ikSetSphericalJointMatrix](#ikSetSphericalJointMatrix)
- [ikGetGroupJoints](#ikGetGroupJoints)

## IK groups

- [ikCreateGroup](#ikCreateGroup)
- [ikGetGroupHandle](#ikGetGroupHandle)
- [ikDoesGroupExist](#ikDoesGroupExist)
- [ikGetGroupFlags](#ikGetGroupFlags)
- [ikSetGroupFlags](#ikSetGroupFlags)
- [ikGetGroupCalculation](#ikGetGroupCalculation)
- [ikGetGroupJointLimitHits](#ikGetGroupJointLimitHits)
- [ikGetGroupJoints](#ikGetGroupJoints)
- [ikSetGroupCalculation](#ikSetGroupCalculation)

## IK elements

- [ikAddElement](#ikAddElement)
- [ikGetElementFlags](#ikGetElementFlags)
- [ikSetElementFlags](#ikSetElementFlags)
- [ikGetElementConstraints](#ikGetElementConstraints)
- [ikSetElementConstraints](#ikSetElementConstraints)
- [ikGetElementBase](#ikGetElementBase)
- [ikSetElementBase](#ikSetElementBase)
- [ikGetElementPrecision](#ikGetElementPrecision)
- [ikSetElementPrecision](#ikSetElementPrecision)
- [ikGetElementWeights](#ikGetElementWeights)
- [ikSetElementWeights](#ikSetElementWeights)

## IK calculation

- [ikHandleGroups](#ikHandleGroups)
- [ikFindConfig](#ikFindConfig)
- [ikComputeGroupJacobian](#ikComputeGroupJacobian)
- [ikComputeJacobian](#ikComputeJacobian)

## Commands

### <a name="ikAddElement" id="ikAddElement"></a> ikAddElement

Adds a new IK element to an IK group.

#### Synopsis:

```cpp
bool ikAddElement(int ikGroupHandle,int tipHandle,int* ikElementHandle)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **tipHandle** (int): the handle of the dummy object that should act as the tip in the IK element.
- **ikElementHandle** (int*): the IK element handle in the IK group, in return.

#### Return values:

true in case of success.

#### See also:

[ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikComputeGroupJacobian" id="ikComputeGroupJacobian"></a> ikComputeGroupJacobian

Computes the Jacobian and error vector for an IK group

#### Synopsis:

```cpp
bool ikComputeGroupJacobian(int ikGroupHandle,std::vector* jacobian,std::vector* errorVect)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group
- **jacobian** (std::vector*): the jacobian in return (row major order). Each row corresponds to one constraint in following order: x, y, z, alpha, beta, gamma. Each column corresponds to one joint, from base to tip. Can be nullptr
- **errorVect** (std::vector*): the error vector between the target and the tip. Can be nullptr

#### Return values:

true in case of success.

#### See also:

[ikComputeJacobian](#ikComputeJacobian), [ikHandleGroups](#ikHandleGroups)

### <a name="ikComputeJacobian" id="ikComputeJacobian"></a> ikComputeJacobian

Computes the Jacobian and error vector for a kinematic chain

#### Synopsis:

```cpp
bool ikComputeJacobian(int baseHandle,int jointHandle,int constraints,const CPose* tipPose,const CPose* targetPose,const CPose* constrBasePose,std::vector* jacobian,std::vector* errorVect)
```

#### Parameters:

- **baseHandle** (int): the handle of the base of the kinematic chain, or -1 for the world
- **jointHandle** (int): the handle of the last joint in the kinematic chain, when going from base to tip
- **constraints** (int): a combination of following is possible: ik_constraint_x, ik_constraint_y, ik_constraint_z, ik_constraint_alpha_beta, ik_constraint_gamma. For convenience we also have ik_constraint_position=ik_constraint_x|ik_constraint_y|ik_constraint_z, ik_constraint_orientation=ik_constraint_alpha_beta|ik_constraint_gamma, and ik_constraint_pose=ik_constraint_position|ik_constraint_orientation.
- **tipPose** (const CPose*): the pose of the tip object, relative to the world
- **targetPose** (const CPose*): the pose of the target object, relative to the world. Can be nullptr, in which case the targetPose is the same as the tipPose
- **constrBasePose** (const CPose*): the pose that should serve as the reference frame for positional constraints, or nullptr to use the base object instead
- **jacobian** (std::vector*): the jacobian in return (row major order). Each row corresponds to one constraint in following order: x, y, z, alpha, beta, gamma. Each column corresponds to one joint, from base to tip. Can be nullptr
- **errorVect** (std::vector*): the error vector between the target and the tip. Can be nullptr

#### Return values:

true in case of success.

#### See also:

[ikComputeGroupJacobian](#ikComputeGroupJacobian), [ikHandleGroups](#ikHandleGroups)

### <a name="ikCreateDummy" id="ikCreateDummy"></a> ikCreateDummy

Creates a dummy object.

#### Synopsis:

```cpp
bool ikCreateDummy(const char* dummyName/*=nullptr*/,int* dummyHandle)
```

#### Parameters:

- **dummyName** (const char*, default: nullptr*/): the name of the dummy.
- **dummyHandle** (int*): the handle of the dummy, in return.

#### Return values:

true in case of success.

#### See also:

[ikDoesObjectExist](#ikDoesObjectExist), [ikCreateJoint](#ikCreateJoint), [ikEraseObject](#ikEraseObject)

### <a name="ikCreateEnvironment" id="ikCreateEnvironment"></a> ikCreateEnvironment

Creates an new IK environment, and switches to it.

#### Synopsis:

```cpp
bool ikCreateEnvironment(int* environmentHandle=nullptr,int flags=0)
```

#### Parameters:

- **environmentHandle** (int*, default: nullptr): the handle of the newly created environment.
- **flags** (int, default: 0): keep at 0

#### Return values:

true in case of success.

#### See also:

[ikEraseEnvironment](#ikEraseEnvironment), [ikSwitchEnvironment](#ikSwitchEnvironment), [ikDuplicateEnvironment](#ikDuplicateEnvironment), [ikLoad](#ikLoad)

### <a name="ikCreateGroup" id="ikCreateGroup"></a> ikCreateGroup

Creates an IK group.

#### Synopsis:

```cpp
bool ikCreateGroup(const char* ikGroupName/*=nullptr*/,int* ikGroupHandle)
```

#### Parameters:

- **ikGroupName** (const char*, default: nullptr*/): the name of the IK group.
- **ikGroupHandle** (int*): the handle of the IK group, in return.

#### Return values:

true in case of success.

#### See also:

[ikDoesGroupExist](#ikDoesGroupExist)

### <a name="ikCreateJoint" id="ikCreateJoint"></a> ikCreateJoint

Creates a joint object.

#### Synopsis:

```cpp
bool ikCreateJoint(const char* jointName/*=nullptr*/,int jointType,int* jointHandle)
```

#### Parameters:

- **jointName** (const char*, default: nullptr*/): the name of the joint.
- **jointType** (int): the type of the joint. supported types are ik_jointtype_revolute, ik_jointtype_prismatic and ik_jointtype_spherical.
- **jointHandle** (int*): the handle of the joint, in return.

#### Return values:

true in case of success.

#### See also:

[ikDoesObjectExist](#ikDoesObjectExist), [ikCreateDummy](#ikCreateJoint), [ikEraseObject](#ikEraseObject)

### <a name="ikDoesGroupExist" id="ikDoesGroupExist"></a> ikDoesGroupExist

Checks whether an IK group exists, based on its name.

#### Synopsis:

```cpp
bool ikDoesGroupExist(const char* ikGroupName)
```

#### Parameters:

- **ikGroupName** (const char*): the name of the IK group.

#### Return values:

true if the IK group exists.

#### See also:

[ikGetGroupHandle](#ikGetGroupHandle)

### <a name="ikDoesObjectExist" id="ikDoesObjectExist"></a> ikDoesObjectExist

Checks whether an object exists, based on its name.

#### Synopsis:

```cpp
bool ikDoesObjectExist(const char* objectName)
```

#### Parameters:

- **objectName** (const char*): the name of the object.

#### Return values:

true if the object exists.

#### See also:

[ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikDuplicateEnvironment" id="ikDuplicateEnvironment"></a> ikDuplicateEnvironment

Duplicate current IK environment. Can be used to operate on an environment without modifying the original environment

#### Synopsis:

```cpp
bool ikDuplicateEnvironment(int* newEnvironmentHandle)
```

#### Parameters:

- **newEnvironmentHandle** (int*): the handle of the newly created environment.

#### Return values:

true in case of success.

#### See also:

[ikEraseEnvironment](#ikEraseEnvironment), [ikSwitchEnvironment](#ikSwitchEnvironment), [ikDuplicateEnvironment](#ikDuplicateEnvironment), [ikCreateEnvironment](#ikCreateEnvironment)

### <a name="ikEraseEnvironment" id="ikEraseEnvironment"></a> ikEraseEnvironment

Erases an IK environment, and switches to another environment, if available.

#### Synopsis:

```cpp
bool ikEraseEnvironment(int* switchedEnvironmentHandle=nullptr)
```

#### Parameters:

- **switchedEnvironmentHandle** (int*, default: nullptr): the handle of the environment that was switched to, or -1 if there is no environment left.

#### Return values:

true in case of success.

#### See also:

[ikCreateEnvironment](#ikCreateEnvironment), [ikSwitchEnvironment](#ikSwitchEnvironment)

### <a name="ikEraseObject" id="ikEraseObject"></a> ikEraseObject

Erases an object.

#### Synopsis:

```cpp
bool ikEraseObject(int objectHandle)
```

#### Parameters:

- **objectHandle** (int): handle of the object.

#### Return values:

true in case of success.

#### See also:

[ikGetObjectHandle](#ikGetObjectHandle), [ikCreateDummy](#ikCreateDummy), [ikCreateJoint](#ikCreateJoint)

### <a name="ikFindConfig" id="ikFindConfig"></a> ikFindConfig

Searches for a manipulator configuration that matches a given end-effector position/orientation in space. Search is randomized.

#### Synopsis:

```cpp
int ikFindConfig(int ikGroupHandle,size_t jointCnt,const int* jointHandles,double thresholdDist,int maxSearchTimeInMs,double* retConfig,const double* metric=nullptr,bool(*validationCallback)(double*)=nullptr)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **jointCnt** (size_t): the number of joint handles provided in the jointHandles array.
- **jointHandles** (const int*): an array with jointCnt entries, that specifies the joint handles for the joints we wish to retrieve the values calculated by the IK.
- **thresholdDist** (double): a distance indicating when IK should be computed in order to try to bring the tip onto the target: since the search algorithm proceeds by generating random configurations, many of them produce a tip pose that is too far from the target pose to run IK successfully. Choosing a large value will result in slow calculations, choosing a small value might produce a smaller subset of solutions. Distance between two poses is calculated using a metric (see metric argument below).
- **maxSearchTimeInMs** (int): the maximum time in milliseconds before this function returns.
- **retConfig** (double*): an array with jointCnt entries, that will receive the IK calculated joint values, as specified by the jointHandles array.
- **metric** (const double*, default: nullptr): an array to 4 values indicating a metric used to compute pose-pose distances: distance=sqrt((dx*metric[0])^2+(dy*metric[1])^2+(dz*metric[2])^2+(angle*metric[3])^2).
- **validationCallback** (default: nullptr): a callback function taking as input argument the proposed joint values (i.e. a configuration), and as return value whether the configuration is valid (e.g. is not colliding).

#### Return values:

-1 in case of an error, 0 if no valid configuration was found, 1 otherwise.

#### See also:

[ikComputeJacobian](#ikComputeJacobian), [ikDuplicateEnvironment](#ikDuplicateEnvironment)

### <a name="ikGetConfigForTipPose" id="ikGetConfigForTipPose"></a> ikGetConfigForTipPose

Deprecated. Use ikFindConfig instead.

#### Synopsis:

```cpp
int ikGetConfigForTipPose(int ikGroupHandle,size_t jointCnt,const int* jointHandles,double thresholdDist,int maxIterations,double* retConfig,const double* metric=nullptr,bool(*validationCallback)(double*)=nullptr,const int* jointOptions=nullptr,const double* lowLimits=nullptr,const double* ranges=nullptr)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **jointCnt** (size_t): the number of joint handles provided in the jointHandles array.
- **jointHandles** (const int*): an array with jointCnt entries, that specifies the joint handles for the joints we wish to retrieve the values calculated by the IK.
- **thresholdDist** (double): a distance indicating when IK should be computed in order to try to bring the tip onto the target: since the search algorithm proceeds by generating random configurations, many of them produce a tip pose that is too far from the target pose to run IK successfully. Choosing a large value will result in slow calculations, choosing a small value might produce a smaller subset of solutions. Distance between two poses is calculated using a metric (see metric argument below).
- **maxIterations** (int): the maximum number of iterations before this function returns. Alternatively, one can specify an upper time limit, in milliseconds, after which the function returns with maxIterations=-timeLimitInMs.
- **retConfig** (double*): an array with jointCnt entries, that will receive the IK calculated joint values, as specified by the jointHandles array.
- **metric** (const double*, default: nullptr): an array to 4 values indicating a metric used to compute pose-pose distances: distance=sqrt((dx*metric[0])^2+(dy*metric[1])^2+(dz*metric[2])^2+(angle*metric[3])^2).
- **validationCallback** (default: nullptr): a callback function taking as input argument the proposed joint values (i.e. a configuration), and as return value whether the configuration is valid (e.g. is not colliding).
- **jointOptions** (const int*, default: nullptr): a bit-coded value corresponding to each specified joint handle. Bit 0 (i.e. 1) indicates the corresponding joint is dependent of another joint.
- **lowLimits** (const double*, default: nullptr): an optional array pointing to different low limit values for each specified joint. This can be useful when you wish to explore a sub-set of the joint's intervals.
- **ranges** (const double*, default: nullptr): an optional array pointing to different range values for each specified joint. This can be useful when you wish to explore a sub-set of the joint's intervals. If the range value is 0, then the lowLimit and range values are taken from the joint's properties. If the range value is negative, then the search interval will be centered around the current joint linear/angular position, with an extent of (-range).

#### Return values:

-1 in case of an error, 0 if no valid configuration was found, 1 otherwise.

#### See also:

[ikComputeJacobian](#ikComputeJacobian), [ikDuplicateEnvironment](#ikDuplicateEnvironment)

### <a name="ikGetElementBase" id="ikGetElementBase"></a> ikGetElementBase

Retrieves the base object of an IK element.

#### Synopsis:

```cpp
bool ikGetElementBase(int ikGroupHandle,int ikElementHandle,int* baseHandle,int* constraintsBaseHandle)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **baseHandle** (int*): the handle of the base object, in return, or -1 if the world is the base.
- **constraintsBaseHandle** (int*): the handle of the constraints base object, in return, relative to which the constraints are specified. Returns -1 if the constraints are relative to the base object.

#### Return values:

true in case of success.

#### See also:

[ikSetElementBase](#ikSetElementBase), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetElementConstraints" id="ikGetElementConstraints"></a> ikGetElementConstraints

Retrieves the constraints of an IK element.

#### Synopsis:

```cpp
bool ikGetElementConstraints(int ikGroupHandle,int ikElementHandle,int* constraints)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **constraints** (int*): the constraints, in return. A combination of following is possible: ik_constraint_x, ik_constraint_y, ik_constraint_z, ik_constraint_alpha_beta, ik_constraint_gamma. For convenience we also have ik_constraint_position=ik_constraint_x|ik_constraint_y|ik_constraint_z, ik_constraint_orientation=ik_constraint_alpha_beta|ik_constraint_gamma, and ik_constraint_pose=ik_constraint_position|ik_constraint_orientation.

#### Return values:

true in case of success.

#### See also:

[ikSetElementConstraints](#ikSetElementConstraints), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetElementFlags" id="ikGetElementFlags"></a> ikGetElementFlags

Retrieves various flags of an IK element.

#### Synopsis:

```cpp
bool ikGetElementFlags(int ikGroupHandle,int ikElementHandle,int* flags)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **flags** (int*): bit-coded flags, in return: bit0 set (1): the enabled state of the ik element.

#### Return values:

true in case of success.

#### See also:

[ikSetElementFlags](#ikSetElementFlags), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetElementPrecision" id="ikGetElementPrecision"></a> ikGetElementPrecision

Retrieves the precision settings of an IK element.

#### Synopsis:

```cpp
bool ikGetElementPrecision(int ikGroupHandle,int ikElementHandle,double* linearPrecision,double* angularPrecision)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **linearPrecision** (double*): the required linear precision, in return.
- **angularPrecision** (double*): the required angular precision, in return.

#### Return values:

true in case of success.

#### See also:

[ikSetElementPrecision](#ikSetElementPrecision), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetElementWeights" id="ikGetElementWeights"></a> ikGetElementWeights

Retrieves the desired linear and angular resolution weights of an IK element.

#### Synopsis:

```cpp
bool ikGetElementWeights(int ikGroupHandle,int ikElementHandle,double* linearWeight,double* angularWeight,double* elementWeight=nullptr)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **linearWeight** (double*): the linear resolution weight, in return.
- **angularWeight** (double*): the angular resolution weight, in return.
- **elementWeight** (double*, default: nullptr): the overall weight for that IK element, in return.

#### Return values:

true in case of success.

#### See also:

[ikSetElementWeights](#ikSetElementWeights), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetGroupCalculation" id="ikGetGroupCalculation"></a> ikGetGroupCalculation

Retrieves calculation properties for an IK group.

#### Synopsis:

```cpp
bool ikGetGroupCalculation(int ikGroupHandle,int* method,double* damping,int* maxIterations)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **method** (int*): the resolution method, in return. Possible values are ik_method_pseudo_inverse (features a tiny bit of hard-coded damping), ik_method_undamped_pseudo_inverse, ik_method_damped_least_squares and ik_method_jacobian_transpose.
- **damping** (double*): the damping, in case the resolution method is ik_method_damped_least_squares, in return.
- **maxIterations** (int*): the maximum number of iterations, in return.

#### Return values:

true in case of success.

#### See also:

[ikSetGroupCalculation](#ikSetGroupCalculation), [ikGetGroupHandle](#ikGetGroupHandle)

### <a name="ikGetGroupFlags" id="ikGetGroupFlags"></a> ikGetGroupFlags

Retrieves flags of an IK group.

#### Synopsis:

```cpp
bool ikGetGroupFlags(int ikGroupHandle,int* flags)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **flags** (int*): the flags of the IK group, in return, bit-coded:
  - ik_group_enabled: the group's enabled state
  - ik_group_ignoremaxsteps: the joints' max step sizes are ignored. Otherwise, when detected, an interpolation factor (originally at 1.0) is successively divided by 2.0 until the max. step sizes are respected
  - ik_group_restoreonbadlintol: the IK world state is left untouched if the linear precision is not reached
  - ik_group_restoreonbadangtol: the IK world state is left untouched if the angular precision is not reached
  - ik_group_avoidlimits: joint limits are actively avoided
  - ik_group_stoponlimithit: calculation ends when a joint limit is hit

#### Return values:

true in case of success.

#### See also:

[ikSetGroupFlags](#ikSetGroupFlags), [ikGetGroupHandle](#ikGetGroupHandle)

### <a name="ikGetGroupHandle" id="ikGetGroupHandle"></a> ikGetGroupHandle

Retrieves the handle of an IK group based on its name.

#### Synopsis:

```cpp
bool ikGetGroupHandle(const char* ikGroupName,int* ikGroupHandle)
```

#### Parameters:

- **ikGroupName** (const char*): the name of the IK group.
- **ikGroupHandle** (int*): the returned IK group handle.

#### Return values:

true in case of success.

#### See also:

[ikDoesGroupExist](#ikDoesGroupExist), [ikCreateGroup](#ikCreateGroup)

### <a name="ikGetGroupJointLimitHits" id="ikGetGroupJointLimitHits"></a> ikGetGroupJointLimitHits

Checks which joints of an IK group hit a limit last time that IK group was handled

#### Synopsis:

```cpp
bool ikGetGroupJointLimitHits(int ikGroupHandle,std::vector<int>* jointHandles,std::vector<double>* underOrOvershots)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **jointHandles** (std::vector&lt;int&gt;*): the joint handles that that had joint limits under- or overshot. Can be null
- **underOrOvershots** (std::vector&lt;double&gt;*): the corresponding under- or overshots, for each of the jointHandles. Can be null

#### Return values:

true in case of success.

#### See also:

[ikSetGroupFlags](#ikSetGroupFlags)

### <a name="ikGetGroupJoints" id="ikGetGroupJoints"></a> ikGetGroupJoints

Returns the joint handles involved in the IK group calculation, i.e. one handle per Jacobian column (except with revolute joints that have 3 corresponding Jacobian columns)

#### Synopsis:

```cpp
bool ikGetGroupJoints(int ikGroupHandle,std::vector<int>* jointHandles)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **jointHandles** (std::vector&lt;int&gt;*): the joint handles

#### Return values:

true in case of success.

### <a name="ikGetJointDependency" id="ikGetJointDependency"></a> ikGetJointDependency

Retrieves information about a possible joint dependency.

#### Synopsis:

```cpp
bool ikGetJointDependency(int jointHandle,int* dependencyJointHandle,double* offset,double* mult)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **dependencyJointHandle** (int*): the handle of the dependency joint, in return.
- **offset** (double*): the offset, in return. We have joint linear/angular position = dependency joint linear/angular position * mult + offset.
- **mult** (double*): the multiplication factor, in return. We have joint linear/angular position = dependency joint linear/angular position * mult + offset.

#### Return values:

true in case of success.

#### See also:

[ikSetJointDependency](#ikSetJointDependency), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetJointInterval" id="ikGetJointInterval"></a> ikGetJointInterval

Retrieves the joint limits.

#### Synopsis:

```cpp
bool ikGetJointInterval(int jointHandle,bool* cyclic,double* intervalMinAndRange)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **cyclic** (bool*): whether the joint is cyclic (has no limits).
- **intervalMinAndRange** (double*): a pointer to two values: the joint lower limit, and the joint range (i.e. joint upper limit = joint lower limit + joint range)

#### Return values:

true in case of success.

#### See also:

[ikSetJointInterval](#ikSetJointInterval), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetJointLimitMargin" id="ikGetJointLimitMargin"></a> ikGetJointLimitMargin

Retrieves the limit margin of a joint, i.e. the threshold that will be used to counteract on joint limit violation during IK resolution, if the appropriate IK group flag was set.

#### Synopsis:

```cpp
bool ikGetJointLimitMargin(int jointHandle,double* margin)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **margin** (double*): the margin, in return.

#### Return values:

true in case of success.

#### See also:

[ikSetJointLimitMargin](#ikSetJointLimitMargin), [ikGetGroupFlags](#ikGetGroupFlags)

### <a name="ikGetJointMatrix" id="ikGetJointMatrix"></a> ikGetJointMatrix

Retrieves the intrinsic transformation matrix of a joint.

#### Synopsis:

```cpp
bool ikGetJointMatrix(int jointHandle,C4X4Matrix* matrix)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **matrix** (C4X4Matrix*): the transformation matrix, in return.

#### Return values:

true in case of success.

#### See also:

[ikSetSphericalJointMatrix](#ikSetSphericalJointMatrix), [ikGetJointPosition](#ikGetJointPosition), [ikGetJointTransformation](#ikGetJointTransformation), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetJointMaxStepSize" id="ikGetJointMaxStepSize"></a> ikGetJointMaxStepSize

Retrieves the maximum step size of a joint.

#### Synopsis:

```cpp
bool ikGetJointMaxStepSize(int jointHandle,double* maxStepSize)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **maxStepSize** (double*): the maximum step size, in return.

#### Return values:

true in case of success.

#### See also:

[ikSetJointMaxStepSize](#ikSetJointMaxStepSize), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetJointMode" id="ikGetJointMode"></a> ikGetJointMode

Retrieves the joint mode.

#### Synopsis:

```cpp
bool ikGetJointMode(int jointHandle,int* mode)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **mode** (int*): the joint mode, in return. Possible values are: ik_jointmode_passive, ik_jointmode_ik

#### Return values:

true in case of success.

#### See also:

[ikSetJointMode](#ikSetJointMode), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetJointPosition" id="ikGetJointPosition"></a> ikGetJointPosition

Retrieves the position (linear or angular) of a joint.

#### Synopsis:

```cpp
bool ikGetJointPosition(int jointHandle,double* position)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **position** (double*): the position, in return.

#### Return values:

true in case of success.

#### See also:

[ikSetJointPosition](#ikSetJointPosition), [ikGetJointMatrix](#ikGetJointMatrix), [ikGetJointTransformation](#ikGetJointTransformation), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetJointScrewLead" id="ikGetJointScrewLead"></a> ikGetJointScrewLead

Retrieves the screw lead of a revolute joint.

#### Synopsis:

```cpp
bool ikGetJointScrewLead(int jointHandle,double* lead)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **lead** (double*): the screw lead of the joint, in return. A lead value of zero represents a revolute joint, a value different from zero represents a screw.

#### Return values:

true in case of success.

#### See also:

[ikSetJointScrewLead](#ikSetJointScrewLead), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetJointTransformation" id="ikGetJointTransformation"></a> ikGetJointTransformation

Retrieves the intrinsic transformation of a joint.

#### Synopsis:

```cpp
bool ikGetJointTransformation(int jointHandle,CPose* transf)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **transf** (CPose*): the transformation, in return.

#### Return values:

true in case of success.

#### See also:

[ikSetSphericalJointQuaternion](#ikSetSphericalJointQuaternion), [ikGetJointPosition](#ikGetJointPosition), [ikGetJointMatrix](#ikGetJointMatrix), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetJointType" id="ikGetJointType"></a> ikGetJointType

Retrieves the joint type.

#### Synopsis:

```cpp
bool ikGetJointType(int jointHandle,int* jointType)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **jointType** (int*): the joint type, in return. Possible values are: ik_jointtype_revolute, ik_jointtype_prismatic and ik_jointtype_spherical.

#### Return values:

true in case of success.

#### See also:

[ikCreateJoint](#ikCreateJoint)

### <a name="ikGetJointWeight" id="ikGetJointWeight"></a> ikGetJointWeight

Retrieves the IK weight of a joint, i.e. the weight it has during IK resolution.

#### Synopsis:

```cpp
bool ikGetJointWeight(int jointHandle,double* ikWeight)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **ikWeight** (double*): the IK weight, in return.

#### Return values:

true in case of success.

#### See also:

[ikSetJointWeight](#ikSetJointWeight), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetLastError" id="ikGetLastError"></a> ikGetLastError

Retrieves and clears the last error string.

#### Synopsis:

```cpp
std::string ikGetLastError()
```

#### Parameters:

No params

#### Return values:

The error string.

### <a name="ikGetLinkedDummy" id="ikGetLinkedDummy"></a> ikGetLinkedDummy

Deprecated. Use [ikGetTargetDummy](#ikGetTargetDummy) instead

#### Synopsis:

```cpp
bool ikGetLinkedDummy(int dummyHandle,int* linkedDummyHandle)
```

#### Parameters:

- **dummyHandle** (int): the handle of the dummy object.
- **linkedDummyHandle** (int*): the handle of the linked dummy object, in return. Is -1 if no dummy object is linked to this one.

#### Return values:

true in case of success.

#### See also:

[ikSetLinkedDummy](#ikSetLinkedDummy), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetObjectHandle" id="ikGetObjectHandle"></a> ikGetObjectHandle

Retrieves the handle of an object based on its name.

#### Synopsis:

```cpp
bool ikGetObjectHandle(const char* objectName,int* objectHandle)
```

#### Parameters:

- **objectName** (const char*): the name of the object.
- **objectHandle** (int*): the returned object handle.

#### Return values:

true in case of success.

#### See also:

[ikDoesObjectExist](#ikDoesObjectExist), [ikCreateDummy](#ikCreateDummy), [ikCreateJoint](#ikCreateJoint)

### <a name="ikGetObjectMatrix" id="ikGetObjectMatrix"></a> ikGetObjectMatrix

Retrieves the transformation matrix of an object. If the object is a joint object, the matrix does not include the joint's intrinsic transformation.

#### Synopsis:

```cpp
bool ikGetObjectMatrix(int objectHandle,int relativeToObjectHandle,C4X4Matrix* matrix)
```

#### Parameters:

- **objectHandle** (int): the handle of the object.
- **relativeToObjectHandle** (int): the handle of an object relative to which we want the matrix expressed. Otherwise, specify ik_handle_world if you want the absolute matrix, or ik_handle_parent if you want the matrix relative to the parent object.
- **matrix** (C4X4Matrix*): a pointer to 12 values representing the transformation matrix (the last row of the 4x4 matrix (0,0,0,1) is omitted)

#### Return values:

true in case of success.

#### See also:

[ikSetObjectMatrix](#ikSetObjectMatrix), [ikGetObjectTransformation](#ikGetObjectTransformation), [ikGetJointTransformation](#ikGetJointTransformation), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetObjectParent" id="ikGetObjectParent"></a> ikGetObjectParent

Retrieves an object's parent handle.

#### Synopsis:

```cpp
bool ikGetObjectParent(int objectHandle,int* parentObjectHandle)
```

#### Parameters:

- **objectHandle** (int): the handle of the object.
- **parentObjectHandle** (int*): the returned handle of the parent, or -1 if the object has no parent.

#### Return values:

true in case of success.

#### See also:

[ikSetObjectParent](#ikSetObjectParent), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetObjects" id="ikGetObjects"></a> ikGetObjects

Allows to loop through all objects in the environment.

#### Synopsis:

```cpp
bool ikGetObjects(size_t index,int* objectHandle=nullptr,std::string* objectName=nullptr,bool* isJoint=nullptr,int* jointType=nullptr)
```

#### Parameters:

- **index** (size_t): the zero-based index. Start at 0, and increment until the return value is false, in order to loop through all objects in the environment.
- **objectHandle** (int*, default: nullptr): the handle of the object, in return.
- **objectName** (std::string*, default: nullptr): the name of the object, in return.
- **isJoint** (bool*, default: nullptr): whether the object is a joint, in return.
- **jointType** (int*, default: nullptr): the type of joint, in return, if the object at the specified index is a joint. Possible values are ik_jointtype_revolute, ik_jointtype_prismatic or ik_jointtype_spherical.

#### Return values:

false in case of an error, or if no object exists at the specified index.

#### See also:

[ikGetObjectHandle](#ikGetObjectHandle), [ikDoesObjectExist](#ikDoesObjectExist)

### <a name="ikGetObjectTransformation" id="ikGetObjectTransformation"></a> ikGetObjectTransformation

Retrieves the transformation (position and quaternion) of an object. If the object is a joint object, the transformation does not include the joint's intrinsic transformation.

#### Synopsis:

```cpp
bool ikGetObjectTransformation(int objectHandle,int relativeToObjectHandle,CPose* transf)
```

#### Parameters:

- **objectHandle** (int): the handle of the object.
- **relativeToObjectHandle** (int): the handle of an object relative to which we want the transformation expressed. Otherwise, specify ik_handle_world if you want the absolute transformation, or ik_handle_parent if you want the transformation relative to the parent object.
- **transf** (CPose*): a pointer to the transformation that will be returned.

#### Return values:

true in case of success.

#### See also:

[ikSetObjectTransformation](#ikSetObjectTransformation), [ikGetObjectMatrix](#ikGetObjectMatrix), [ikGetJointTransformation](#ikGetJointTransformation), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikGetObjectType" id="ikGetObjectType"></a> ikGetObjectType

Retrieves the type of an object.

#### Synopsis:

```cpp
bool ikGetObjectType(int objectHandle,int* objectType)
```

#### Parameters:

- **objectHandle** (int): the handle of the object.
- **objectType** (int*): the returned type of the object.

#### Return values:

true in case of success.

### <a name="ikGetTargetDummy" id="ikGetTargetDummy"></a> ikGetTargetDummy

Retrieves the handle of the target dummy associated with a tip dummy

#### Synopsis:

```cpp
bool ikGetTargetDummy(int tipDummyHandle,int* targetDummyHandle)
```

#### Parameters:

- **tipDummyHandle** (int): the handle of the tip dummy
- **targetDummyHandle** (int*): the handle of the target dummy, in return. Is -1 if no target is associated with the tip dummy

#### Return values:

true in case of success.

#### See also:

[ikSetTargetDummy](#ikSetTargetDummy), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikHandleGroups" id="ikHandleGroups"></a> ikHandleGroups

Handles (i.e. computes/resolves) one or several IK groups

#### Synopsis:

```cpp
bool ikHandleGroups(const std::vector* ikGroupHandles,int* result=nullptr,double* precision,int(*cb)(const int*,double*,const int*,const int*,const int*,const int*,double*,double*,double*)=nullptr)
```

#### Parameters:

- **ikGroupHandles** (const std::vector*): the handles of the IK groups. If one handle is specified, then a simple IK calculation is performed. If more than one handle is specified, then a projected IK computation is performed, where each group calculation result is projected into the previous group's null space, i.e. dq=dq0+NP0*(dq1+NP1*(dq2+NP2*(...))), where dqi is the result of i-th task's IK (i.e. typically J#*e), and NPi is the i-th task's I-J#*J
- **result** (int*, default: nullptr): the bit-coded execution result, in return. Possible values are ik_calc_notperformed, ik_calc_cannotinvert, ik_calc_notwithintolerance, ik_calc_stepstoobig, ik_calc_limithit. Can be nullptr
- **precision** (double*): a pointer to 2 values, receiving the largest linear and angular distance between all tip-target pairs. Can be nullptr
- **cb** (int(*cb)(const int*,double*,const int*,const int*,const int*,const, default: nullptr): an optional callback address that allows to inspect and manipulate the generated jacobians and error vector:
  - int jacobianCallback(const int jacobianSize[2],double* jacobian,const int* rowConstraints,const int* rowIkElements,const int* colHandles,const int* colStages,double* errorVector,double* qVector,double* jacobianPseudoinverse)
  - The return value is bit-coded: bit0=qVector is provided, bit1=jacobianPseudoinverse is provided. jacobian and errorVector can be modified, and qVector can be set (set also return bit0 in that case). If jacobianPseudoinverse is provided (set also return bit1 in that case), then if qVector is not provided, qVector will be computed simply like: qVector=jacobianPseudoinverse*errorVector. If qVector and jacobianPseudoinverse are not provided, then internal calculations are used.

#### Return values:

true in case of execution success.

#### See also:

[ikComputeJacobian](#ikComputeJacobian), [ikFindConfig](#ikFindConfig)

### <a name="ikLoad" id="ikLoad"></a> ikLoad

Loads kinematic content previously exported with [ikSave](#ikSave). Make sure that the environment is empty before calling this function.

#### Synopsis:

```cpp
bool ikLoad(const unsigned char* data,size_t dataLength)
```

#### Parameters:

- **data** (const unsigned char*): a pointer to a buffer with the kinematic content.
- **dataLength** (size_t): the size of the kinematic content buffer.

#### Return values:

true in case of success.

#### See also:

[ikSave](#ikSave), [ikCreateEnvironment](#ikCreateEnvironment), [ikEraseEnvironment](#ikEraseEnvironment)

### <a name="ikReleaseBuffer" id="ikReleaseBuffer"></a> ikReleaseBuffer

Releases a buffer allocated by the specific API functions.

#### Synopsis:

```cpp
void ikReleaseBuffer(void* buffer)
```

#### Parameters:

- **buffer** (void*): the buffer to release.

#### Return values:

No return values

### <a name="ikSave" id="ikSave"></a> ikSave

Saves the kinematic content in the current environment.

#### Synopsis:

```cpp
unsigned char* ikSave(size_t* dataLength)
```

#### Parameters:

- **dataLength** (size_t*): the size of the kinematic content buffer.

#### Return values:

a pointer to a buffer with the kinematic content. The user is in charge of releasing the buffer with [ikReleaseBuffer](#ikReleaseBuffer)

#### See also:

[ikLoad](#ikLoad)

### <a name="ikSetElementBase" id="ikSetElementBase"></a> ikSetElementBase

Sets the base object of an IK element.

#### Synopsis:

```cpp
bool ikSetElementBase(int ikGroupHandle,int ikElementHandle,int baseHandle,int constraintsBaseHandle=-1)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **baseHandle** (int): the handle of the base object, or -1 if the world is the base.
- **constraintsBaseHandle** (int, default: -1): the handle of the constraints base object, relative to which the constraints are specified. Set to -1 to have the constraints relative to the base object.

#### Return values:

true in case of success.

#### See also:

[ikGetElementBase](#ikGetElementBase), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetElementConstraints" id="ikSetElementConstraints"></a> ikSetElementConstraints

Sets the constraints of an IK element.

#### Synopsis:

```cpp
bool ikSetElementConstraints(int ikGroupHandle,int ikElementHandle,int constraints)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **constraints** (int): the constraints. Combine following: ik_constraint_x, ik_constraint_y, ik_constraint_z, ik_constraint_alpha_beta, ik_constraint_gamma (ik_constraint_gamma should only be set if ik_constraint_alpha_beta is also set). For convenience we also have ik_constraint_position=ik_constraint_x|ik_constraint_y|ik_constraint_z, ik_constraint_orientation=ik_constraint_alpha_beta|ik_constraint_gamma, and ik_constraint_pose=ik_constraint_position|ik_constraint_orientation.

#### Return values:

true in case of success.

#### See also:

[ikGetElementConstraints](#ikGetElementConstraints), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetElementFlags" id="ikSetElementFlags"></a> ikSetElementFlags

Sets various flags for an IK element.

#### Synopsis:

```cpp
bool ikSetElementFlags(int ikGroupHandle,int ikElementHandle,int flags)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **flags** (int): bit-coded flags: bit0 set (1): the enabled state of the ik element.

#### Return values:

true in case of success.

#### See also:

[ikGetElementFlags](#ikGetElementFlags), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetElementPrecision" id="ikSetElementPrecision"></a> ikSetElementPrecision

Sets the desired precision of an IK element.

#### Synopsis:

```cpp
bool ikSetElementPrecision(int ikGroupHandle,int ikElementHandle,double linearPrecision,double angularPrecision)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **linearPrecision** (double): the required linear precision.
- **angularPrecision** (double): the required angular precision.

#### Return values:

true in case of success.

#### See also:

[ikGetElementPrecision](#ikGetElementPrecision), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetElementWeights" id="ikSetElementWeights"></a> ikSetElementWeights

Sets the desired linear and angular resolution weights of an IK element.

#### Synopsis:

```cpp
bool ikSetElementWeights(int ikGroupHandle,int ikElementHandle,double linearWeight,double angularWeight,double elementWeight)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **ikElementHandle** (int): the IK element handle, or X, where X=handleOfTipDummy|ik_handleflag_tipdummy (if several IK elements with the same tip dummy exist, then the first encountered will be selected).
- **linearWeight** (double): the desired linear resolution weight.
- **angularWeight** (double): the desired angular resolution weight.
- **elementWeight** (double): the desired overall weight for that IK element.

#### Return values:

true in case of success.

#### See also:

[ikGetElementWeights](#ikGetElementWeights), [ikGetGroupHandle](#ikGetGroupHandle), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetGroupCalculation" id="ikSetGroupCalculation"></a> ikSetGroupCalculation

Sets calculation properties for an IK group.

#### Synopsis:

```cpp
bool ikSetGroupCalculation(int ikGroupHandle,int method,double damping,int maxIterations)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **method** (int): the resolution method. Possible values are ik_method_pseudo_inverse (features a tiny bit of hard-coded damping), ik_method_undamped_pseudo_inverse, ik_method_damped_least_squares and ik_method_jacobian_transpose.
- **damping** (double): the damping, in case the resolution method is ik_method_damped_least_squares.
- **maxIterations** (int): the maximum number of iterations.

#### Return values:

true in case of success.

#### See also:

[ikGetGroupCalculation](#ikGetGroupCalculation), [ikGetGroupHandle](#ikGetGroupHandle)

### <a name="ikSetGroupFlags" id="ikSetGroupFlags"></a> ikSetGroupFlags

Sets flags of an IK group.

#### Synopsis:

```cpp
bool ikSetGroupFlags(int ikGroupHandle,int flags)
```

#### Parameters:

- **ikGroupHandle** (int): the handle of the IK group.
- **flags** (int): the flags of the IK group bit-coded:
  - ik_group_enabled: the group's enabled state
  - ik_group_ignoremaxsteps: the joints' max step sizes are ignored. Otherwise, when detected, an interpolation factor (originally at 1.0) is successively divided by 2.0 until the max. step sizes are respected
  - ik_group_restoreonbadlintol: the IK world state is left untouched if the linear precision is not reached
  - ik_group_restoreonbadangtol: the IK world state is left untouched if the angular precision is not reached
  - ik_group_avoidlimits: joint limits are actively avoided
  - ik_group_stoponlimithit: calculation ends when a joint limit is hit

#### Return values:

true in case of success.

#### See also:

[ikGetGroupFlags](#ikGetGroupFlags), [ikGetGroupHandle](#ikGetGroupHandle)

### <a name="ikSetJointDependency" id="ikSetJointDependency"></a> ikSetJointDependency

Sets information about a possible dependent joint.

#### Synopsis:

```cpp
bool ikSetJointDependency(int jointHandle,int dependencyJointHandle,double offset=0.0,double mult=1.0,double(*cb)(int ikEnv,int slaveJoint,double masterPos)=nullptr)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **dependencyJointHandle** (int): the handle of the joint, this joint is dependent of. -1 to disable.
- **offset** (double, default: 0.0): the offset. We have joint linear/angular position = dependency joint linear/angular position * mult + offset
- **mult** (double, default: 1.0): the multiplication factor. We have joint linear/angular position = dependency joint linear/angular position * mult + offset
- **cb** (double(*cb)(int ikEnv,int slaveJoint,double, default: nullptr): an optional callback address that allows to provide a custom dependency function:
  - double dependencyCallback(int ikEnv,int slaveJointHandle,double masterJointPosition)

#### Return values:

true in case of success.

#### See also:

[ikGetJointDependency](#ikGetJointDependency), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetJointInterval" id="ikSetJointInterval"></a> ikSetJointInterval

Sets the joint limits.

#### Synopsis:

```cpp
bool ikSetJointInterval(int jointHandle,bool cyclic,const double* intervalMinAndRange=nullptr)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **cyclic** (bool): whether the joint is cyclic (has no limits). Only revolute joints can be cyclic.
- **intervalMinAndRange** (const double*, default: nullptr): a pointer to two values: the joint lower limit, and the joint range (i.e. joint upper limit = joint lower limit + joint range)

#### Return values:

true in case of success.

#### See also:

[ikGetJointInterval](#ikGetJointInterval), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetJointLimitMargin" id="ikSetJointLimitMargin"></a> ikSetJointLimitMargin

Sets the limit margin of a joint, i.e. the the threshold that will be used to counteract on joint limit violation during IK resolution, if the appropriate IK group flag was set.

#### Synopsis:

```cpp
bool ikSetJointLimitMargin(int jointHandle,double margin)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **margin** (double): the margin.

#### Return values:

true in case of success.

#### See also:

[ikGetJointLimitMargin](#ikGetJointLimitMargin), [ikGetGroupFlags](#ikGetGroupFlags)

### <a name="ikSetJointMaxStepSize" id="ikSetJointMaxStepSize"></a> ikSetJointMaxStepSize

Sets the maximum step size of a joint.

#### Synopsis:

```cpp
bool ikSetJointMaxStepSize(int jointHandle,double maxStepSize)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **maxStepSize** (double): the maximum step size.

#### Return values:

true in case of success.

#### See also:

[ikGetJointMaxStepSize](#ikGetJointMaxStepSize), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetJointMode" id="ikSetJointMode"></a> ikSetJointMode

Sets the joint mode.

#### Synopsis:

```cpp
bool ikSetJointMode(int jointHandle,int jointMode)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **jointMode** (int): the joint mode. Allowed values are: ik_jointmode_passive, ik_jointmode_ik

#### Return values:

true in case of success.

#### See also:

[ikGetJointMode](#ikGetJointMode), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetJointPosition" id="ikSetJointPosition"></a> ikSetJointPosition

Sets the position (linear or angular) of a joint.

#### Synopsis:

```cpp
bool ikSetJointPosition(int jointHandle,double position)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **position** (double): the position.

#### Return values:

true in case of success.

#### See also:

[ikGetJointPosition](#ikGetJointPosition), [ikSetSphericalJointMatrix](#ikSetSphericalJointMatrix), [ikSetSphericalJointQuaternion](#ikSetSphericalJointQuaternion), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetJointScrewLead" id="ikSetJointScrewLead"></a> ikSetJointScrewLead

Sets the screw lead, in case of a revolute joint.

#### Synopsis:

```cpp
bool ikSetJointScrewLead(int jointHandle,double lead)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **lead** (double): the screw lead of the joint. A lead value of zero represents a revolute joint, a value different from zero represents a screw.

#### Return values:

true in case of success.

#### See also:

[ikGetJointScrewLead](#ikGetJointScrewLead), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetJointWeight" id="ikSetJointWeight"></a> ikSetJointWeight

Sets the IK weight of a joint, i.e. the weight it has during IK resolution.

#### Synopsis:

```cpp
bool ikSetJointWeight(int jointHandle,double ikWeight)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **ikWeight** (double): the IK weight.

#### Return values:

true in case of success.

#### See also:

[ikGetJointWeight](#ikGetJointWeight), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetLinkedDummy" id="ikSetLinkedDummy"></a> ikSetLinkedDummy

Deprecated. Use [ikSetTargetDummy instead](#ikSetTargetDummy)

#### Synopsis:

```cpp
bool ikSetLinkedDummy(int dummyHandle,int linkedDummyHandle)
```

#### Parameters:

- **dummyHandle** (int): the handle of the dummy object.
- **linkedDummyHandle** (int): the handle of the dummy object to be linked, or -1 to detach this dummy object from a linked dummy object.

#### Return values:

true in case of success.

#### See also:

[ikGetLinkedDummy](#ikGetLinkedDummy), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetObjectMatrix" id="ikSetObjectMatrix"></a> ikSetObjectMatrix

Sets the transformation matrix of an object. If the object is a joint object, the matrix does not include the joint's intrinsic transformation.

#### Synopsis:

```cpp
bool ikSetObjectMatrix(int objectHandle,int relativeToObjectHandle,const C4X4Matrix* matrix)
```

#### Parameters:

- **objectHandle** (int): the handle of the object.
- **relativeToObjectHandle** (int): the handle of an object relative to which the matrix is expressed. Otherwise, specify ik_handle_world if you specify the absolute matrix, or ik_handle_parent if you specify the matrix relative to the parent object.
- **matrix** (const C4X4Matrix*): the transformation matrix

#### Return values:

true in case of success.

#### See also:

[ikGetObjectMatrix](#ikGetObjectMatrix), [ikSetObjectTransformation](#ikSetObjectTransformation), [ikSetJointPosition](#ikSetJointPosition), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetObjectParent" id="ikSetObjectParent"></a> ikSetObjectParent

Sets the parent of an object.

#### Synopsis:

```cpp
bool ikSetObjectParent(int objectHandle,int parentObjectHandle,bool keepInPlace)
```

#### Parameters:

- **objectHandle** (int): the handle of the object.
- **parentObjectHandle** (int): the desired parent object, Set -1 for no parent.
- **keepInPlace** (bool): if true, the object will stay in place, otherwise, it will keep its local transformation.

#### Return values:

true in case of success.

#### See also:

[ikGetObjectParent](#ikGetObjectParent), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetObjectTransformation" id="ikSetObjectTransformation"></a> ikSetObjectTransformation

Sets the transformation (position and quaternion) of an object. If the object is a joint object, the transformation does not include the joint's intrinsic transformation.

#### Synopsis:

```cpp
bool ikSetObjectTransformation(int objectHandle,int relativeToObjectHandle,const CPose* transf)
```

#### Parameters:

- **objectHandle** (int): the handle of the object.
- **relativeToObjectHandle** (int): the handle of an object relative to which the transformation is expressed. Otherwise, specify ik_handle_world if you specify the absolute transformation, or ik_handle_parent if you specify the transformation relative to the parent object.
- **transf** (const CPose*): the transformation.

#### Return values:

true in case of success.

#### See also:

[ikGetObjectTransformation](#ikGetObjectTransformation), [ikSetObjectMatrix](#ikSetObjectMatrix), [ikSetJointPosition](#ikSetJointPosition), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetSphericalJointMatrix" id="ikSetSphericalJointMatrix"></a> ikSetSphericalJointMatrix

Sets the rotation transformation matrix of a spherical joint.

#### Synopsis:

```cpp
bool ikSetSphericalJointMatrix(int jointHandle,const C3X3Matrix* rotMatrix)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **rotMatrix** (const C3X3Matrix*): the rotation transformation matrix to apply.

#### Return values:

true in case of success.

#### See also:

[ikGetJointMatrix](#ikGetJointMatrix), [ikSetJointPosition](#ikSetJointPosition), [ikSetSphericalJointQuaternion](#ikSetSphericalJointQuaternion), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetSphericalJointQuaternion" id="ikSetSphericalJointQuaternion"></a> ikSetSphericalJointQuaternion

Sets the rotation transformation of a spherical joint.

#### Synopsis:

```cpp
bool ikSetSphericalJointQuaternion(int jointHandle,const CQuaternion* quaternion)
```

#### Parameters:

- **jointHandle** (int): the handle of the joint.
- **quaternion** (const CQuaternion*): the rotation transformation to apply.

#### Return values:

true in case of success.

#### See also:

[ikGetJointTransformation](#ikGetJointTransformation), [ikSetSphericalJointMatrix](#ikSetSphericalJointMatrix), [ikSetJointPosition](#ikSetJointPosition), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSetTargetDummy" id="ikSetTargetDummy"></a> ikSetTargetDummy

Associates a tip dummy with a target dummy, or removes that association. If the tip dummy is already associated with another target dummy, then first remove that association before setting another one

#### Synopsis:

```cpp
bool ikSetTargetDummy(int tipDummyHandle,int targetDummyHandle)
```

#### Parameters:

- **tipDummyHandle** (int): the handle of the tip dummy
- **targetDummyHandle** (int): the handle of the dummy to become the target of the tip, or -1 to remove an existing target association from the tip dummy

#### Return values:

true in case of success.

#### See also:

[ikGetTargetDummy](#ikGetTargetDummy), [ikGetObjectHandle](#ikGetObjectHandle)

### <a name="ikSwitchEnvironment" id="ikSwitchEnvironment"></a> ikSwitchEnvironment

Switches to another environment and all function calls will be directed to that environment.

#### Synopsis:

```cpp
bool ikSwitchEnvironment(int handle,bool allowAlsoProtectedEnvironment=false)
```

#### Parameters:

- **handle** (int): handle of the environment to switch to.
- **allowAlsoProtectedEnvironment** (bool, default: false): set to false.

#### Return values:

true in case of success.

#### See also:

[ikCreateEnvironment](#ikCreateEnvironment), [ikEraseEnvironment](#ikEraseEnvironment)
