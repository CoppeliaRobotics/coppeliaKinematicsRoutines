#pragma once

#include "ik.h"
#include "ikElement.h"
#include "sceneObject.h"
#include "joint.h"
#include "dummy.h"
#include <map>

class CikGroup  
{
public:
    CikGroup();
    virtual ~CikGroup();

    bool announceSceneObjectWillBeErased(int objectHandle);
    bool announceIkGroupWillBeErased(int ikGroupHandle);
    void performObjectLoadingMapping(std::vector<int>* map);
    void serialize(CSerialization& ar);

    CikGroup* copyYourself() const;
    int addIkElement(CikElement* ikElement);
    CikElement* getIkElement(int elementHandle) const;
    CikElement* getIkElementWithTooltipHandle(int tooltipHandle) const;
    void removeIkElement(int elementHandle);
    size_t getIkElementCount() const;
    CikElement* getIkElementFromIndex(size_t index) const;


    void setExplicitHandling(bool explicitHandl);
    bool getExplicitHandling() const;
    void setAllInvolvedJointsToPassiveMode();

    int getObjectHandle() const;
    void setObjectHandle(int handle);
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
    void setRestoreIfPositionNotReached(bool restore);
    bool getRestoreIfOrientationNotReached() const;
    void setRestoreIfOrientationNotReached(bool restore);

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
    int computeGroupIk(bool forInternalFunctionality,int(*cb)(const int*,simReal*,const int*,const int*,const int*,const int*,simReal*,simReal*));
    void getAllActiveJoints(std::vector<CJoint*>& jointList) const;
    void getTipAndTargetLists(std::vector<CDummy*>& tipList,std::vector<CDummy*>& targetList) const;

    bool getIgnoreMaxStepSizes() const;
    void setIgnoreMaxStepSizes(bool ignore);
    void resetCalculationResult();
    void setCalculationResult(int res);
    int getCalculationResult() const;
    void setCorrectJointLimits(bool c);
    bool getCorrectJointLimits() const;
    bool getFailOnJointLimits() const;
    void setFailOnJointLimits(bool fail);
    bool getJointLimitHits(std::vector<int>* jointHandles,std::vector<simReal>* underOrOvershots) const;
    bool getForbidOvershoot() const;
    void setForbidOvershoot(bool forbid);


    void setActive(bool isActive);

    const simReal* getLastJacobianData(size_t matrixSize[2]) const;
    simReal getLastManipulabilityValue(bool& ok) const;
    simReal getDeterminant(const CMatrix& m,const std::vector<size_t>* activeRows,const std::vector<size_t>* activeColumns) const;
    bool computeOnlyJacobian(int options);



private:
    // Variables which need to be serialized and copied:
    std::vector<CikElement*> _ikElements;

    void _resetTemporaryParameters();
    void _applyTemporaryParameters();

    int performOnePass(std::vector<CikElement*>* validElements,bool& limitOrAvoidanceNeedMoreCalculation,simReal interpolFact,bool forInternalFunctionality,int(*cb)(const int*,simReal*,const int*,const int*,const int*,const int*,simReal*,simReal*));
    bool performOnePass_jacobianOnly(std::vector<CikElement*>* validElements,int options);

    // Variables which need to be serialized and copied:
    int objectHandle;
    std::string objectName;
    int maxIterations;
    bool active;
    bool _correctJointLimits;
    simReal dlsFactor;
    int calculationMethod;
    bool restoreIfPositionNotReached;
    bool restoreIfOrientationNotReached;
    bool _failOnJointLimits;
    bool _forbidOvershoot;
    std::map<int,simReal> _jointLimitHits;
    int doOnFailOrSuccessOf; // group identifier which success/fail will be evaluated
    bool doOnFail;
    bool doOnPerformed;
    int constraints; // only sim_avoidance_constraint is valid!
    simReal jointLimitWeight;
    simReal jointTreshholdAngular;    // in radian
    simReal jointTreshholdLinear;     // in meter

    bool ignoreMaxStepSizes;
    int _calculationResult;

    CMatrix _lastJacobian;

    bool _explicitHandling;
};
