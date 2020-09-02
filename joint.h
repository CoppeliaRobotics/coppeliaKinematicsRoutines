#pragma once

#include "ik.h"
#include "sceneObject.h"
#include <vector>

class CJoint : public CSceneObject
{
public:
    CJoint();
    CJoint(int jointType);
    virtual ~CJoint();

    bool announceSceneObjectWillBeErased(int objectHandle);
    void announceIkGroupWillBeErased(int ikGroupHandle);
    void performSceneObjectLoadingMapping(const std::vector<int>* map);
    void serialize(CSerialization& ar);

    CSceneObject* copyYourself() const;
    simReal getPosition(bool tempVals=false) const;
    void setPosition(simReal parameter,bool tempVals=false);

    void initializeParametersForIK(simReal angularJointLimitationThreshold);
    size_t getDoFs() const;
    void getLocalTransformationExPart1(C7Vector& mTr,size_t index) const;
    simReal getTempParameterEx(size_t index) const;
    void setTempParameterEx(simReal parameter,size_t index);
    void applyTempParametersEx();
    int getTempSphericalJointLimitations() const;

    simReal getScrewPitch() const;
    void setScrewPitch(simReal p);
    void setSphericalTransformation(const C4Vector& tr);
    C4Vector getSphericalTransformation() const;
    int getJointType() const;
    simReal getPositionIntervalMin() const;
    void setPositionIntervalMin(simReal m);
    simReal getPositionIntervalRange() const;
    void setPositionIntervalRange(simReal r);

    bool getPositionIsCyclic() const;
    void setPositionIsCyclic(bool c);

    simReal getIkWeight() const;
    void setIkWeight(simReal newWeight);

    simReal getMaxStepSize() const;
    void setMaxStepSize(simReal stepS) ;

    void _rectifyDependentJoints();

    void setJointMode(int theMode);
    int getJointMode() const;

    int getDependencyJointHandle() const;
    simReal getDependencyJointMult() const;
    simReal getDependencyJointAdd() const;
    void setDependencyJointHandle(int jointHandle);
    void setDependencyJointMult(simReal m);
    void setDependencyJointAdd(simReal off);

    std::vector<CJoint*> dependentJoints;

protected:
    int _jointType;
    C4Vector _sphericalTransformation;
    bool _positionIsCyclic;
    simReal _screwPitch;
    simReal _jointMinPosition;
    simReal _jointPositionRange;

    simReal _jointPosition;

    simReal _maxStepSize;

    simReal _ikWeight;

    int _jointMode;
    int _dependencyJointHandle;
    simReal _dependencyJointMult;
    simReal _dependencyJointAdd;

    simReal _jointPosition_tempForIK;
    simReal _sphericalTransformation_euler1TempForIK;
    simReal _sphericalTransformation_euler2TempForIK;
    simReal _sphericalTransformation_euler3TempForIK;
    int _sphericalTransformation_eulerLockTempForIK; // bit-coded, bit0--> _sphericalTransformation_euler1TempForIK, bit1--> _sphericalTransformation_euler2TempForIK, etc.
};
