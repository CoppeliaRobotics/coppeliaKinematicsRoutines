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


    int getObjectHandle() const;
    void setObjectHandle(int handle);
    std::string getObjectName() const;
    void setObjectName(std::string newName);
    void setMaxIterations(int maxIter);
    int getMaxIterations() const;
    void setDlsFactor(double theFactor);
    double getDlsFactor() const;
    void setCalculationMethod(int theMethod);
    int getCalculationMethod() const;

    void setOptions(int options);
    int getOptions() const;

    int computeGroupIk(bool forInternalFunctionality,bool(*cb)(const int*,std::vector<double>*,const int*,const int*,const int*,const int*,std::vector<double>*,double*));

    bool getJointLimitHits(std::vector<int>* jointHandles,std::vector<double>* underOrOvershots) const;

    const double* getLastJacobianData_old(size_t matrixSize[2]);
    double getLastManipulabilityValue_old(bool& ok) const;
    double getDeterminant_old(const CMatrix& m,const std::vector<size_t>* activeRows,const std::vector<size_t>* activeColumns) const;
    bool computeOnlyJacobian_old(int options);
    bool getExplicitHandling_old() const;
    void setAllInvolvedJointsToPassiveMode_old();


private:
    int _performOnePass(std::vector<CikElement*>* validElements,bool& limitOrAvoidanceNeedMoreCalculation,double interpolFact,bool forInternalFunctionality,bool computeOnlyJacobian,bool(*cb)(const int*,std::vector<double>*,const int*,const int*,const int*,const int*,std::vector<double>*,double*));

    std::vector<CikElement*> _ikElements;
    int _objectHandle;
    std::string _objectName;
    int _maxIterations;
    double _dlsFactor;
    int _calculationMethod;
    std::map<int,double> _jointLimitHits;
    int _options; // bits set: 0=disabled,1=don'tignoreMaxStepSize,2=restoreIfPosNotReached,3=restoreIfOrientationNotReached,4=failOnJointLimitHit,5=forbidOvershoot,6=ignoreJointDependencies,7=doJointLimitCorrections

    CMatrix _lastJacobian;
    CMatrix _lastJacobian_flipped; // for backw. compatibility. Cols are from tip to base
    std::vector<int> _jointHandles; // going through the Jacobian cols
    std::vector<int> _jointDofIndex; // going through the Jacobian cols
    std::vector<int> _elementHandles; // going through the Jacobian rows
    std::vector<int> _equationType; // going through the Jacobian rows. 0-2: x,y,z, 3-5: alpha,beta,gamma, 6=jointLimits, 7=jointDependency

    bool _explicitHandling_old;
};
