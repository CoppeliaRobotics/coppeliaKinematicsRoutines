#pragma once

#include "ik.h"
#include "serialization.h"
#include <vector>
#include "4X4Matrix.h"
#include "MMatrix.h"

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

    void isWithinTolerance(bool& position,bool& orientation,bool useTempValues) const;
    void getDistances(simReal& linDist,simReal& angDist,bool useTempValues) const;
    void prepareEquations(simReal interpolationFactor);
    void clearIkEquations();

    CMatrix matrix;
    CMatrix matrix_correctJacobian;
    CMatrix errorVector;

    std::vector<int> jointHandles_tipToBase;
    std::vector<size_t> jointStages_tipToBase;

private:
    void _getMatrixError(const C7Vector& frame1,const C7Vector& frame2,simReal& linError,simReal& angError) const;

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
