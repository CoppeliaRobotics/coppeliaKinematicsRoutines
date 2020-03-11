#pragma once

#include "ik.h"
#include "ikElement.h"
#include "sceneObject.h"
#include "joint.h"
#include "dummy.h"

class CikGroup  
{
public:
    CikGroup();
    virtual ~CikGroup();

    bool announceSceneObjectWillBeErased(int objectHandle);
    bool announceIkGroupWillBeErased(int ikGroupHandle);
    void performObjectLoadingMapping(std::vector<int>* map);
    void serialize(CSerialization& ar);

    CikElement* getIkElement(int ikElementID) const;
    CikElement* getIkElementWithTooltipID(int tooltipID) const;
    void removeIkElement(int elementID);

    void setExplicitHandling(bool explicitHandl);
    bool getExplicitHandling() const;
    void setAllInvolvedJointsToPassiveMode();

    int getObjectID() const;
    std::string getObjectName() const;
    void setObjectName(std::string newName);
    void setMaxIterations(int maxIter);
    int getMaxIterations() const;
    bool getActive() const;
    void setDlsFactor(simReal theFactor);
    simReal getDlsFactor() const;
    void setCalculationMethod(int theMethod);
    int getCalculationMethod() const;
    bool getRestoreIfPositionNotReached() const;
    bool getRestoreIfOrientationNotReached() const;

    int getDoOnFailOrSuccessOf() const;
    bool getDoOnFail() const;
    bool getDoOnPerformed() const;
    void setConstraints(int constr);
    int getConstraints() const;
    void setJointLimitWeight(simReal weight);
    simReal getJointLimitWeight() const;

    simReal getJointTreshholdAngular() const;
    simReal getJointTreshholdLinear() const;
    void setJointTreshholdAngular(simReal t);
    void setJointTreshholdLinear(simReal t);
    int computeGroupIk(bool forInternalFunctionality);
    void getAllActiveJoints(std::vector<CJoint*>& jointList) const;
    void getTipAndTargetLists(std::vector<CDummy*>& tipList,std::vector<CDummy*>& targetList) const;

    bool getIgnoreMaxStepSizes() const;
    void setIgnoreMaxStepSizes(bool ignore);
    void resetCalculationResult();
    void setCalculationResult(int res);
    int getCalculationResult() const;
    void setCorrectJointLimits(bool c);
    bool getCorrectJointLimits() const;

    void setActive(bool isActive);

    simReal* getLastJacobianData(size_t matrixSize[2]) const;
    simReal getLastManipulabilityValue(bool& ok) const;
    simReal getDeterminant(const CMatrix& m,const std::vector<size_t>* activeRows,const std::vector<size_t>* activeColumns) const;
    bool computeOnlyJacobian(int options);

    // Variables which need to be serialized and copied:
    std::vector<CikElement*> ikElements;

private:
    void _resetTemporaryParameters();
    void _applyTemporaryParameters();

    int performOnePass(std::vector<CikElement*>* validElements,bool& limitOrAvoidanceNeedMoreCalculation,simReal interpolFact,bool forInternalFunctionality);
    bool performOnePass_jacobianOnly(std::vector<CikElement*>* validElements,int options);

    // Variables which need to be serialized and copied:
    int objectID;
    std::string objectName;
    int maxIterations;
    bool active;
    bool _correctJointLimits;
    simReal dlsFactor;
    int calculationMethod;
    bool restoreIfPositionNotReached;
    bool restoreIfOrientationNotReached;
    int doOnFailOrSuccessOf; // group identifier which success/fail will be evaluated
    bool doOnFail;
    bool doOnPerformed;
    int constraints; // only sim_avoidance_constraint is valid!
    simReal jointLimitWeight;
    simReal jointTreshholdAngular;    // in radian
    simReal jointTreshholdLinear;     // in meter

    bool ignoreMaxStepSizes;
    int _calculationResult;

    CMatrix* _lastJacobian;

    bool _explicitHandling;
};
