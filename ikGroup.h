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

    bool getValidElements(std::vector<CikElement*>& el);
    void prepareRawJacobians(std::vector<CikElement*>& el,double interpolFact);
    bool computeGroupIk(CMatrix& jacobian,CMatrix& errorVect);
    void getJointHandles(std::vector<int>& handles);

    void clearJointLimitHits();
    bool getJointLimitHits(std::vector<int>* jointHandles,std::vector<double>* underOrOvershots) const;

    const double* getLastJacobianData_old(size_t matrixSize[2]);
    double getLastManipulabilityValue_old(bool& ok) const;
    double getDeterminant_old(const CMatrix& m,const std::vector<size_t>* activeRows,const std::vector<size_t>* activeColumns) const;
    bool computeOnlyJacobian_old();
    bool getExplicitHandling_old() const;
    void setAllInvolvedJointsToPassiveMode_old();

    bool selectJoints(std::vector<CikElement*>* validElements,std::vector<CJoint*>* allJoints,std::vector<int>* allJointDofIndices);
    int computeDq(std::vector<CikElement*>* validElements,bool nakedJacobianOnly,bool(*cb)(const int*,std::vector<double>*,const int*,const int*,const int*,const int*,std::vector<double>*,double*));
    int checkDq(const CMatrix* dq,double* maxStepFact);
    void applyDq(const CMatrix* dq);

    const CMatrix& getNakedJacobian() const;
    const CMatrix& getJacobian() const;
    const CMatrix& getInvJacobian() const;
    const CMatrix& getDq() const;

    static CMatrix pinv(const CMatrix& J,const CMatrix& E,CMatrix* Jinv);

private:
    std::vector<CikElement*> _ikElements;
    int _objectHandle;
    std::string _objectName;
    int _maxIterations;
    double _dlsFactor;
    int _calculationMethod;
    std::map<int,double> _jointLimitHits;
    int _options; // ik_group_enabled and similar

    CMatrix _nakedJacobian;
    CMatrix _jacobian;
    CMatrix _jacobianPseudoinv;
    CMatrix _E;
    CMatrix _dQ;
    std::vector<CJoint*> _joints; // going through the Jacobian cols
    std::vector<int> _jointHandles; // going through the Jacobian cols
    std::vector<int> _jointDofIndex; // going through the Jacobian cols

    bool _explicitHandling_old;
    CMatrix _lastJacobian_flipped; // for backw. compatibility. Cols are from tip to base
};
