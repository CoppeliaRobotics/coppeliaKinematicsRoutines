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
    double getPosition() const;
    void setPosition(double parameter,const CJoint* masterJoint=nullptr);

    double getScrewPitch() const;
    void setScrewPitch(double p);
    void setSphericalTransformation(const C4Vector& tr);
    C4Vector getSphericalTransformation() const;
    int getJointType() const;
    double getPositionIntervalMin() const;
    void setPositionIntervalMin(double m);
    double getPositionIntervalRange() const;
    void setPositionIntervalRange(double r);

    bool getPositionIsCyclic() const;
    void setPositionIsCyclic(bool c);

    double getIkWeight() const;
    void setIkWeight(double newWeight);

    double getLimitMargin() const;
    void setLimitMargin(double newMargin);

    double getMaxStepSize() const;
    void setMaxStepSize(double stepS) ;

    void setJointMode(int theMode);
    int getJointMode() const;

    int getDependencyJointHandle() const;
    double getDependencyJointMult() const;
    double getDependencyJointAdd() const;
    void setDependencyJointHandle(int jointHandle);
    void setDependencyJointMult(double m);
    void setDependencyJointAdd(double off);

    std::vector<CJoint*> dependentJoints;

protected:
    int _jointType;
    C4Vector _sphericalTransformation; // spherical joints don't have a range anymore since 22.10.22
    bool _positionIsCyclic;
    double _screwPitch;
    double _jointMinPosition;
    double _jointPositionRange;

    double _jointPosition;

    double _maxStepSize;

    double _ikWeight;
    double _limitMargin;

    int _jointMode;
    int _dependencyJointHandle;
    double _dependencyJointMult;
    double _dependencyJointAdd;
};
