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
    simReal getPosition() const;
    void setPosition(simReal parameter,const CJoint* masterJoint=nullptr);

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
    C4Vector _sphericalTransformation; // spherical joints don't have a range anymore since 22.10.22
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
};
