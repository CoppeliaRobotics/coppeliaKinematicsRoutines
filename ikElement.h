#pragma once

#include "ik.h"
#include "serialization.h"
#include <vector>
#include "4X4Matrix.h"
#include "MMatrix.h"

class CSceneObject;

class CikElement
{
public:
    CikElement();
    CikElement(int theTooltip);
    virtual ~CikElement();

    CikElement* copyYourself() const;
    bool announceSceneObjectWillBeErased(int objectHandle);
    void performSceneObjectLoadingMapping(const std::vector<int>* map);
    void serialize(CSerialization& ar);

    void setIkElementHandle(int handle);
    int getIkElementHandle() const;
    int getTipHandle() const;
    int getTargetHandle() const;
    int getBaseHandle() const;
    void setBaseHandle(int newBaseHandle);
    int getAltBaseHandleForConstraints() const;
    void setAltBaseHandleForConstraints(int newAltBaseHandle);

    void setRelatedJointsToPassiveMode();
    bool getIsActive() const;
    void setIsActive(bool isActive);

    simReal getMinLinearPrecision() const;
    void setMinLinearPrecision(simReal precision);
    simReal getMinAngularPrecision() const;
    void setMinAngularPrecision(simReal precision);
    simReal getPositionWeight() const;
    void setPositionWeight(simReal weight);
    simReal getOrientationWeight() const;
    void setOrientationWeight(simReal weight);
    int getConstraints() const;
    void setConstraints(int constraints);

    void isWithinTolerance(bool& position,bool& orientation) const;
    void getTipTargetDistance(simReal& linDist,simReal& angDist) const;
    static bool getJacobian(CMatrix& jacob,CMatrix& errVect,const simReal weights[2],const int tipBaseAltBase[3],int constraints,simReal interpolationFactor,std::vector<int>* equTypes,std::vector<int>* jHandles,std::vector<int>* jDofIndex);
    void prepareEquations(simReal interpolationFactor);

    CMatrix jacobian;
    CMatrix errorVector;
    std::vector<int> equationTypes;
    std::vector<int> jointHandles;
    std::vector<int> jointDofIndex;

private:
    static CMatrix _getNakedJacobian(const CSceneObject* tip,const CSceneObject* base,const CSceneObject* constrBase,int constraints,std::vector<int>* jHandles,std::vector<int>* jDofIndex);


    int _ikElementHandle;
    int _tipHandle;
    int _baseHandle;
    int _altBaseHandleForConstraints;
    int _constraints;
    bool _isActive;
    simReal _positionWeight;
    simReal _orientationWeight;
    simReal _minAngularPrecision;
    simReal _minLinearPrecision;
};
