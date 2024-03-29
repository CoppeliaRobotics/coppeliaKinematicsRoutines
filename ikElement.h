#pragma once

#include "ik.h"
#include "serialization.h"
#include <vector>
#include <simMath/4X4Matrix.h>
#include <simMath/mXnMatrix.h>

class CSceneObject;

class CikElement
{
public:
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

    void setRelatedJointsToPassiveMode_old();
    bool getIsActive() const;
    void setIsActive(bool isActive);
    bool getIsValid() const;

    void getWeights(double w[3]) const;
    void setWeights(const double w[3]);
    void getPrecisions(double p[2]) const;
    void setPrecisions(const double p[2]);
    int getConstraints() const;
    void setConstraints(int constraints);

    void getTipTargetDistance(double& linDist,double& angDist) const;
    static bool getJacobian(CMatrix& jacob,CMatrix& errVect,int ttip,int tbase,int constraints,int altBase,double interpolationFactor,std::vector<int>* equTypes,std::vector<int>* jHandles,std::vector<int>* jDofIndex,double backCompatibility=1.0);
    void prepareRawJacobian(double interpolationFactor);

    CMatrix jacobian;
    CMatrix errorVector;
    std::vector<int> equationTypes;
    std::vector<int> jointHandles;
    std::vector<int> jointDofIndex;

private:
    static CMatrix _getNakedJacobian(const CSceneObject* tip,const CSceneObject* target,const CSceneObject* base,const CSceneObject* constrBase,int constraints,double interpolationFactor,std::vector<int>* jHandles,std::vector<int>* jDofIndex,double backCompatibility=1.0);

    int _ikElementHandle;
    int _tipHandle;
    int _baseHandle;
    int _altBaseHandleForConstraints;
    int _constraints;
    bool _isActive;
    double _weights[3]; // linear, angular, and element weights
    double _precisions[2]; // linear and angular precisions
    double _backCompatibility;
};
