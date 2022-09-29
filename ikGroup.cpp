#include "ikGroup.h"
#include "ikRoutines.h"
#include "environment.h"
#include <algorithm>


CikGroup::CikGroup()
{
    objectHandle=2030003;
    maxIterations=3;
    active=true;
    ignoreMaxStepSizes=true;
    _failOnJointLimits=false;
    _lastJacobian=nullptr;
    _explicitHandling=false;
    dlsFactor=simReal(0.1);
    calculationMethod=ik_method_pseudo_inverse;
    restoreIfPositionNotReached=false;
    restoreIfOrientationNotReached=false;
    doOnFailOrSuccessOf=-1;
    doOnFail=true;
    doOnPerformed=true;
    constraints=0;
    jointLimitWeight=1.0;
    jointTreshholdAngular=simReal(2.0)*degToRad;
    jointTreshholdLinear=simReal(0.001);
    _calculationResult=ik_result_not_performed;
    _correctJointLimits=false;
}

CikGroup::~CikGroup()
{
    while (_ikElements.size()!=0)
        removeIkElement(_ikElements[0]->getIkElementHandle());
    delete _lastJacobian;
}

void CikGroup::performObjectLoadingMapping(std::vector<int>* map)
{
    for (size_t i=0;i<_ikElements.size();i++)
        _ikElements[i]->performSceneObjectLoadingMapping(map);
}

void CikGroup::setExplicitHandling(bool explicitHandl)
{
    _explicitHandling=explicitHandl;
}

bool CikGroup::getExplicitHandling() const
{
    return(_explicitHandling);
}

void CikGroup::setAllInvolvedJointsToPassiveMode()
{
    for (size_t i=0;i<_ikElements.size();i++)
        _ikElements[i]->setRelatedJointsToPassiveMode();
}

void CikGroup::setObjectName(std::string newName)
{
    objectName=newName;
}

void CikGroup::setActive(bool isActive)
{
    active=isActive;
}

void CikGroup::resetCalculationResult()
{
    _calculationResult=ik_result_not_performed;
}

void CikGroup::setCorrectJointLimits(bool c)
{
    _correctJointLimits=c;
}

bool CikGroup::getCorrectJointLimits() const
{
    return(_correctJointLimits);
}

void CikGroup::setCalculationResult(int res)
{
    _calculationResult=res;
}

int CikGroup::getCalculationResult() const
{
    return(_calculationResult);
}

void CikGroup::setJointLimitWeight(simReal weight)
{
    jointLimitWeight=weight;
}
simReal CikGroup::getJointLimitWeight() const
{
    return(jointLimitWeight);
}

simReal CikGroup::getJointTreshholdAngular() const
{
    return(jointTreshholdAngular);
}
simReal CikGroup::getJointTreshholdLinear() const
{
    return(jointTreshholdLinear);
}

void CikGroup::setJointTreshholdAngular(simReal t)
{
    jointTreshholdLinear=t;
}

void CikGroup::setJointTreshholdLinear(simReal t)
{
    jointTreshholdAngular=t;
}

void CikGroup::setConstraints(int constr)
{
    constraints=constr;
}

int CikGroup::getConstraints() const
{
    return(constraints);
}

void CikGroup::setMaxIterations(int maxIter)
{
    maxIterations=maxIter;
}
int CikGroup::getMaxIterations() const
{
    return(maxIterations);
}

bool CikGroup::getActive() const
{
    return(active);
}
void CikGroup::setDlsFactor(simReal theFactor)
{
    dlsFactor=theFactor;
}
simReal CikGroup::getDlsFactor() const
{
    return(dlsFactor);
}
void CikGroup::setCalculationMethod(int theMethod)
{
    if ( (theMethod==ik_method_pseudo_inverse)||(theMethod==ik_method_damped_least_squares)||
        (theMethod==ik_method_jacobian_transpose)||(theMethod==ik_method_undamped_pseudo_inverse) )
    {
        calculationMethod=theMethod;
    }
}
int CikGroup::getCalculationMethod() const
{
    return(calculationMethod);
}

bool CikGroup::getRestoreIfPositionNotReached() const
{
    return(restoreIfPositionNotReached);
}

void CikGroup::setRestoreIfPositionNotReached(bool restore)
{
    restoreIfPositionNotReached=restore;
}

bool CikGroup::getRestoreIfOrientationNotReached() const
{
    return(restoreIfOrientationNotReached);
}

void CikGroup::setRestoreIfOrientationNotReached(bool restore)
{
    restoreIfOrientationNotReached=restore;
}

int CikGroup::getDoOnFailOrSuccessOf() const
{
    return(doOnFailOrSuccessOf);
}
bool CikGroup::getDoOnFail() const
{
    return(doOnFail);
}
bool CikGroup::getDoOnPerformed() const
{
    return(doOnPerformed);
}
void CikGroup::removeIkElement(int elementHandle)
{
    for (size_t i=0;i<_ikElements.size();i++)
    {
        if (_ikElements[i]->getIkElementHandle()==elementHandle)
        {
            delete _ikElements[i];
            _ikElements.erase(_ikElements.begin()+i);
            break;
        }
    }
}

size_t CikGroup::getIkElementCount() const
{
    return(_ikElements.size());
}

CikElement* CikGroup::getIkElementFromIndex(size_t index) const
{
    CikElement* retVal=nullptr;
    if (index<_ikElements.size())
        retVal=_ikElements[index];
    return(retVal);
}

CikGroup* CikGroup::copyYourself() const
{
    CikGroup* duplicate=new CikGroup();

    for (size_t i=0;i<_ikElements.size();i++)
        duplicate->_ikElements.push_back(_ikElements[i]->copyYourself());

    duplicate->objectHandle=objectHandle;
    duplicate->objectName=objectName;
    duplicate->maxIterations=maxIterations;
    duplicate->active=active;
    duplicate->_correctJointLimits=_correctJointLimits;
    duplicate->dlsFactor=dlsFactor;
    duplicate->calculationMethod=calculationMethod;
    duplicate->restoreIfPositionNotReached=restoreIfPositionNotReached;
    duplicate->restoreIfOrientationNotReached=restoreIfOrientationNotReached;
    duplicate->doOnFailOrSuccessOf=doOnFailOrSuccessOf;
    duplicate->doOnFail=doOnFail;
    duplicate->doOnPerformed=doOnPerformed;
    duplicate->constraints=constraints;
    duplicate->jointLimitWeight=jointLimitWeight;
    duplicate->jointTreshholdAngular=jointTreshholdAngular;
    duplicate->jointTreshholdLinear=jointTreshholdLinear;
    duplicate->_failOnJointLimits=_failOnJointLimits;
    duplicate->ignoreMaxStepSizes=ignoreMaxStepSizes;
    duplicate->_calculationResult=_calculationResult;
    duplicate->_explicitHandling=_explicitHandling;
    duplicate->_lastJacobian=nullptr;
    if (_lastJacobian!=nullptr)
        duplicate->_lastJacobian=new CMatrix(_lastJacobian[0]);

    return(duplicate);
}

int CikGroup::addIkElement(CikElement* ikElement)
{
    int retVal=0;
    while (getIkElement(retVal)!=nullptr)
        retVal++;
    ikElement->setIkElementHandle(retVal);
    _ikElements.push_back(ikElement);
    return(retVal);
}

CikElement* CikGroup::getIkElement(int elementHandle) const
{
    for (size_t i=0;i<_ikElements.size();i++)
    {
        if (_ikElements[i]->getIkElementHandle()==elementHandle)
            return(_ikElements[i]);
    }
    return(nullptr);
}

CikElement* CikGroup::getIkElementWithTooltipHandle(int tooltipHandle) const
{ 
    if (tooltipHandle==-1)
        return(nullptr);
    for (size_t i=0;i<_ikElements.size();i++)
    {
        if (_ikElements[i]->getTipHandle()==tooltipHandle)
            return(_ikElements[i]);
    }
    return(nullptr);
}

int CikGroup::getObjectHandle() const
{
    return(objectHandle);
}

void CikGroup::setObjectHandle(int handle)
{
    objectHandle=handle;
}

std::string CikGroup::getObjectName() const
{
    return(objectName);
}

bool CikGroup::announceSceneObjectWillBeErased(int objectHandle)
{ // Return value true means that this object should be destroyed
    size_t i=0;
    while (i<_ikElements.size())
    {
        if (_ikElements[i]->announceSceneObjectWillBeErased(objectHandle))
        {
            removeIkElement(_ikElements[i]->getIkElementHandle());
            i=0; // ordering may have changed
        }
        else
            i++;
    }
    return(_ikElements.size()==0);
}

bool CikGroup::announceIkGroupWillBeErased(int ikGroupHandle)
{ // Return value true means that this avoidance object should be destroyed
    if (doOnFailOrSuccessOf==ikGroupHandle)
    {
        doOnFailOrSuccessOf=-1;
        active=false;
    }
    return(false);
}

bool CikGroup::getIgnoreMaxStepSizes() const
{
    return(ignoreMaxStepSizes);
}

void CikGroup::setIgnoreMaxStepSizes(bool ignore)
{
    ignoreMaxStepSizes=ignore;
}

bool CikGroup::getFailOnJointLimits() const
{
    return(_failOnJointLimits);
}

void CikGroup::setFailOnJointLimits(bool fail)
{
    _failOnJointLimits=fail;
}

bool CikGroup::getJointLimitHits(std::vector<int>* jointHandles,std::vector<simReal>* underOrOvershots) const
{
    for (auto it=_jointLimitHits.begin();it!=_jointLimitHits.end();++it)
    {
        if (jointHandles!=nullptr)
            jointHandles->push_back(it->first);
        if (underOrOvershots!=nullptr)
            underOrOvershots->push_back(it->second);
    }
    return(!_jointLimitHits.empty());
}

void CikGroup::getAllActiveJoints(std::vector<CJoint*>& jointList) const
{ // Retrieves all active joints in this group. Ordering is random!
    for (size_t elNb=0;elNb<_ikElements.size();elNb++)
    {
        CikElement* element=_ikElements[elNb];
        CDummy* tooltip=CEnvironment::currentEnvironment->objectContainer->getDummy(element->getTipHandle());
        CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(element->getBaseHandle());
        bool valid=true;
        if (!element->getIsActive())
            valid=false;
        if (tooltip==nullptr)
            valid=false;
        // We check that tooltip is parented with base and has at least one joint in-between:
        if (valid)
        {
            valid=false;
            bool jointPresent=false;
            bool baseOk=false;
            CSceneObject* iterat=tooltip;
            while ( (iterat!=base)&&(iterat!=nullptr) )
            {
                iterat=iterat->getParentObject();
                if (iterat==base)
                {
                    baseOk=true;
                    if (jointPresent)
                        valid=true;
                }
                if ( (iterat!=base)&&(iterat!=nullptr)&&(iterat->getObjectType()==ik_objecttype_joint) )
                    jointPresent=true;
            }
            if (!valid)
            {
                element->setIsActive(false); // This element has an error
                if (!baseOk)
                    element->setBaseHandle(-1); // The base was illegal!
            }
        }
        if (valid)
        { // We add all joint between tooltip and base which are not yet present:
            CSceneObject* iterat=tooltip;
            while ( (iterat!=base)&&(iterat!=nullptr) )
            {
                if (iterat->getObjectType()==ik_objecttype_joint)
                {
                    if (std::find(jointList.begin(),jointList.end(),iterat)==jointList.end())
                        jointList.push_back(static_cast<CJoint*>(iterat));
                }
                iterat=iterat->getParentObject();
            }
        }
    }
}

void CikGroup::getTipAndTargetLists(std::vector<CDummy*>& tipList,std::vector<CDummy*>& targetList) const
{
    for (size_t elNb=0;elNb<_ikElements.size();elNb++)
    {
        CikElement* element=_ikElements[elNb];
        CDummy* tooltip=CEnvironment::currentEnvironment->objectContainer->getDummy(element->getTipHandle());
        CDummy* target=CEnvironment::currentEnvironment->objectContainer->getDummy(element->getTargetHandle());
        CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(element->getBaseHandle());
        bool valid=true;
        if (!element->getIsActive())
            valid=false;
        if (tooltip==nullptr)
            valid=false;
        if (target==nullptr)
            valid=false;
        // We check that tooltip is parented with base and has at least one joint in-between:
        if (valid)
        {
            valid=false;
            bool jointPresent=false;
            bool baseOk=false;
            CSceneObject* iterat=tooltip;
            while ( (iterat!=base)&&(iterat!=nullptr) )
            {
                iterat=iterat->getParentObject();
                if (iterat==base)
                {
                    baseOk=true;
                    if (jointPresent)
                        valid=true;
                }
                if ( (iterat!=base)&&(iterat!=nullptr)&&(iterat->getObjectType()==ik_objecttype_joint) )
                    jointPresent=true;
            }
            if (!valid)
            {
                element->setIsActive(false); // This element has an error
                if (!baseOk)
                    element->setBaseHandle(-1); // The base was illegal!
            }
        }
        if (valid)
        { 
            tipList.push_back(tooltip);
            targetList.push_back(target);
        }
    }
}

int CikGroup::computeGroupIk(bool forInternalFunctionality)
{ // Return value is one of following: ik_result_not_performed, ik_result_success, ik_result_fail
    _jointLimitHits.clear();
    if (!active)
        return(ik_result_not_performed); // That group is not active!
    if (!forInternalFunctionality)
    {
        if (doOnFailOrSuccessOf!=-1)
        { // Conditional execution part:
            CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(doOnFailOrSuccessOf);
            if (it!=nullptr)
            {
                if (doOnPerformed)
                {
                    if (it->getCalculationResult()==ik_result_not_performed)
                        return(ik_result_not_performed);
                    if (it->getCalculationResult()==ik_result_success)
                    {
                        if (doOnFail)
                            return(ik_result_not_performed);
                    }
                    else
                    {
                        if (!doOnFail)
                            return(ik_result_not_performed);
                    }
                }
                else
                {
                    if (it->getCalculationResult()!=ik_result_not_performed)
                        return(ik_result_not_performed);
                }
            }
        }
    }

    // Now we prepare a vector with all valid and active elements:
    std::vector<CikElement*> validElements;
    validElements.reserve(_ikElements.size());
    validElements.clear();

    for (size_t elNb=0;elNb<_ikElements.size();elNb++)
    {
        CikElement* element=_ikElements[elNb];
        CDummy* tooltip=CEnvironment::currentEnvironment->objectContainer->getDummy(element->getTipHandle());
        CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(element->getBaseHandle());
        bool valid=true;
        if (!element->getIsActive())
            valid=false;
        if (tooltip==nullptr)
            valid=false; // should normally never happen!
        // We check that tooltip is parented with base and has at least one joint in-between:
        if (valid)
        {
            valid=false;
            bool jointPresent=false;
            bool baseOk=false;
            CSceneObject* iterat=tooltip;
            while ( (iterat!=base)&&(iterat!=nullptr) )
            {
                iterat=iterat->getParentObject();
                if (iterat==base)
                {
                    baseOk=true;
                    if (jointPresent)
                        valid=true;
                }
                if ( (iterat!=base)&&(iterat!=nullptr)&&(iterat->getObjectType()==ik_objecttype_joint) )
                { 
                    if ( ((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_ik)||((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_reserved_previously_ikdependent)||((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_dependent) )
                        jointPresent=true;
                }
            }
            if (!valid)
            {
                element->setIsActive(false); // This element has an error
                if (!baseOk)
                    element->setBaseHandle(-1); // The base was illegal!
            }
        }
        if (valid)
            validElements.push_back(element);
    }
    // Now validElements contains all valid elements we have to use in the following computation!
    if (validElements.size()==0)
        return(ik_result_fail); // Error!

    _resetTemporaryParameters();

    // Here we have the main iteration loop:
    simReal interpolFact=1.0; // We first try to solve in one step
    int successNumber=0;
    bool limitOrAvoidanceNeedMoreCalculation;
    bool leaveNow=false;
    bool errorOccured=false;
    for (int iterationNb=0;iterationNb<maxIterations;iterationNb++)
    {
        // Here we prepare all element equations:
        for (size_t elNb=0;elNb<validElements.size();elNb++)
        {
            CikElement* element=validElements[elNb];
            element->prepareEquations(interpolFact);
        }

        int res=performOnePass(&validElements,limitOrAvoidanceNeedMoreCalculation,interpolFact,forInternalFunctionality);
        if (res==-1)
        {
            errorOccured=true;
            break;
        }
        if (res==1)
        { // Joint variations within tolerance
            successNumber++;

            // We check if all IK elements are under the required precision and
            // that there are not active joint limitation or avoidance equations
            bool posAndOrAreOk=true;
            for (size_t elNb=0;elNb<validElements.size();elNb++)
            {
                CikElement* element=validElements[elNb];
                bool posit,orient;
                element->isWithinTolerance(posit,orient,true);
                if (!(posit&&orient))
                {
                    posAndOrAreOk=false;
                    break;
                }
            }
            if (posAndOrAreOk&&(!limitOrAvoidanceNeedMoreCalculation))
                leaveNow=true; // Everything is fine, we can leave here
        }
        else
        { // Joint variations not within tolerance
            successNumber=0;
            interpolFact=interpolFact/simReal(2.0);
            _resetTemporaryParameters();
        }

        // Here we remove all element equations (free memory)
        for (size_t elNb=0;elNb<validElements.size();elNb++)
        {
            CikElement* element=validElements[elNb];
            element->clearIkEquations();
        }
        if (leaveNow)
            break;
    }
    int returnValue=ik_result_success;
    if (errorOccured)
        returnValue=ik_result_fail;
    bool setNewValues=(!errorOccured);
    for (size_t elNb=0;elNb<validElements.size();elNb++)
    {
        CikElement* element=validElements[elNb];
        bool posit,orient;
        element->isWithinTolerance(posit,orient,true);
        if ( (!posit)||(!orient) )
        {
            returnValue=ik_result_fail;
            if ( (restoreIfPositionNotReached&&(!posit))||
                (restoreIfOrientationNotReached&&(!orient)) )
                setNewValues=false;
        }
    }

    // We set all joint parameters:
    if (setNewValues)
        _applyTemporaryParameters();
    return(returnValue);
}

void CikGroup::_resetTemporaryParameters()
{
    // We prepare all joint temporary parameters:
    for (size_t jc=0;jc<CEnvironment::currentEnvironment->objectContainer->jointList.size();jc++)
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(CEnvironment::currentEnvironment->objectContainer->jointList[jc]);
        it->setPosition(it->getPosition(),true);
        it->initializeParametersForIK(getJointTreshholdAngular());
    }
}

void CikGroup::_applyTemporaryParameters()
{
    // Joints:
    for (size_t jc=0;jc<CEnvironment::currentEnvironment->objectContainer->jointList.size();jc++)
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(CEnvironment::currentEnvironment->objectContainer->jointList[jc]);
        it->setPosition(it->getPosition(true),false);
        it->applyTempParametersEx();
    }
}

int CikGroup::performOnePass(std::vector<CikElement*>* validElements,bool& limitOrAvoidanceNeedMoreCalculation,simReal interpolFact,bool forInternalFunctionality)
{   // Return value -1 means that an error occured or joint limits were hit (with appropriate flag) --> keep old configuration
    // Return value 0 means that the max. angular or linear variation were overshot
    // Return value 1 means everything went ok
    // In that case the joints temp. values are not actualized. Another pass is needed
    // Here we have the multi-ik solving algorithm:
    //********************************************************************************
    limitOrAvoidanceNeedMoreCalculation=false;
    // We prepare a vector of all used joints and a counter for the number of rows:
    std::vector<CJoint*> allJoints;
    std::vector<size_t> allJointStages;
    size_t numberOfRows=0;
    for (size_t elNb=0;elNb<validElements->size();elNb++)
    {
        CikElement* element=validElements->at(elNb);
        numberOfRows+=element->matrix->rows;
        for (size_t i=0;i<element->rowJointHandles->size();i++)
        {
            int current=element->rowJointHandles->at(i);
            size_t currentStage=element->rowJointStages->at(i);
            // We check if that joint is already present:
            bool present=false;
            for (size_t j=0;j<allJoints.size();j++)
            {
                if ( (allJoints[j]->getObjectHandle()==current)&&(allJointStages[j]==currentStage) )
                {
                    present=true;
                    break;
                }
            }
            if (!present)
            {
                allJoints.push_back(CEnvironment::currentEnvironment->objectContainer->getJoint(current));
                allJointStages.push_back(currentStage);
            }
        }
    }
    //---------------------------------------------------------------------------

    // Now we prepare the joint limitation part:
    //---------------------------------------------------------------------------
    std::vector<simReal> limitationError;
    std::vector<size_t> limitationIndex;
    std::vector<simReal> limitationValue;
    if (_correctJointLimits)
    {
        for (size_t jointCounter=0;jointCounter<allJoints.size();jointCounter++)
        {
            CJoint* it=allJoints[jointCounter];
            size_t stage=allJointStages[jointCounter];
            simReal minVal=it->getPositionIntervalMin();
            simReal range=it->getPositionIntervalRange();
            simReal value=it->getPosition(true);
            if (it->getJointType()==ik_jointtype_revolute)
            { // We have to handle a revolute joint
                if (!it->getPositionIsCyclic())
                { // Limitation applies only if joint is not cyclic!
                    simReal distFromMin=value-jointTreshholdAngular-minVal;
                    simReal distFromMax=value+jointTreshholdAngular-(minVal+range);
                    simReal eq=simZero;
                    simReal activate=simReal(-10.0);
                    if (distFromMin<simZero)
                    {
                        activate=simOne; // We correct in the positive direction
                        eq=-distFromMin;
                    }
                    if (distFromMax>simZero)
                    {
                        activate=-simOne; // We correct in the negative direction
                        eq=distFromMax;
                    }
                    if (activate>simReal(-5.0))
                    { // We have to activate a joint limitation equation!
                        // If we are over the treshhold of more than 5%:
                        // (important in case target and tooltip are within tolerance)
                        if (eq>jointTreshholdAngular*simReal(0.05))
                            limitOrAvoidanceNeedMoreCalculation=true;
                        // First the error part:
                        limitationError.push_back(eq*jointLimitWeight);
                        // Now the matrix part:
                        limitationIndex.push_back(jointCounter);
                        limitationValue.push_back(activate);
                    }
                }
            }
            if (it->getJointType()==ik_jointtype_prismatic)
            { // We have to handle a prismatic joint:
                simReal distFromMin=value-(minVal+jointTreshholdLinear);
                simReal distFromMax=value-(minVal+range-jointTreshholdLinear);
                simReal eq=simZero;
                simReal activate=simReal(-10.0);
                if ( (distFromMin<simZero)&&(fabs(distFromMin)<fabs(distFromMax)) )
                {
                    activate=simOne; // We correct in the positive direction
                    eq=-distFromMin;
                }
                if ( (distFromMax>simZero)&&(fabs(distFromMax)<fabs(distFromMin)) )
                {
                    activate=-simOne; // We correct in the negative direction
                    eq=distFromMax;
                }
                if (activate>simReal(-5.0))
                { // We have to activate a joint limitation equation!
                    // If we are over the treshhold of more than 5%:
                    // (important in case target and tooltip are within tolerance)
                    if (eq>jointTreshholdLinear*simReal(0.05))
                        limitOrAvoidanceNeedMoreCalculation=true;
                    // First the error part:
                    limitationError.push_back(eq*jointLimitWeight);
                    // Now the matrix part:
                    limitationIndex.push_back(jointCounter);
                    limitationValue.push_back(activate);
                }
            }
            if (it->getJointType()==ik_jointtype_spherical)
            { // We have to handle a spherical joint
                if ( (it->getTempSphericalJointLimitations()!=0)&&(stage==1) ) // Joint limitation configuration was activated!
                {
                    simReal v=it->getTempParameterEx(stage);
                    simReal distFromMax=v+jointTreshholdAngular-(minVal+range);
                    simReal eq=simZero;
                    simReal activate=simReal(-10.0);
                    if (distFromMax>simZero)
                    {
                        activate=-simOne; // We correct in the negative direction
                        eq=distFromMax;
                    }
                    if (activate>simReal(-5.0))
                    { // We have to activate a joint limitation equation!
                        // If we are over the treshhold of more than 5%:
                        // (important in case target and tooltip are within tolerance)
                        if (eq>jointTreshholdAngular*simReal(0.05))
                            limitOrAvoidanceNeedMoreCalculation=true;
                        // First the error part:
                        limitationError.push_back(eq*jointLimitWeight);
                        // Now the matrix part:
                        limitationIndex.push_back(jointCounter);
                        limitationValue.push_back(activate);
                    }
                }
            }
        }
    }
    numberOfRows+=limitationError.size();
    //---------------------------------------------------------------------------

    // Now we prepare the individual joint constraints part:
    //---------------------------------------------------------------------------
    for (size_t i=0;i<allJoints.size();i++)
    {
        if (allJoints[i]->getJointType()!=ik_jointtype_spherical)
        {
            if ( (allJoints[i]->getJointMode()==ik_jointmode_reserved_previously_ikdependent)||(allJoints[i]->getJointMode()==ik_jointmode_dependent) )
                numberOfRows++;
        }
    }
    //---------------------------------------------------------------------------

    // We prepare the main matrix and the main error vector.
    CMatrix mainMatrix(numberOfRows,allJoints.size());
    CMatrix mainMatrix_correctJacobian(numberOfRows,allJoints.size());
    // We have to zero it first:
    mainMatrix.clear();
    mainMatrix_correctJacobian.clear();
    CMatrix mainErrorVector(numberOfRows,1);
    
    // Now we fill in the main matrix and the main error vector:
    size_t currentRow=0;
    for (size_t elNb=0;elNb<validElements->size();elNb++)
    {
        CikElement* element=validElements->at(elNb);
        for (size_t i=0;i<element->errorVector->rows;i++)
        { // We go through the rows:
            // We first set the error part:
            mainErrorVector(currentRow,0)=(*element->errorVector)(i,0);
            // Now we set the delta-parts:
            for (size_t j=0;j<element->matrix->cols;j++)
            { // We go through the columns:
                // We search for the right entry
                int jointHandle=element->rowJointHandles->at(j);
                size_t stage=element->rowJointStages->at(j);
                size_t index=0;
                while ( (allJoints[index]->getObjectHandle()!=jointHandle)||(allJointStages[index]!=stage) )
                    index++;
                mainMatrix(currentRow,index)=(*element->matrix)(i,j);
                mainMatrix_correctJacobian(currentRow,index)=(*element->matrix_correctJacobian)(i,j);
            }
            currentRow++;
        }
    }
    // Now we add the joint limitation equations:
    for (size_t i=0;i<limitationError.size();i++)
    { // We go through the rows:
        mainErrorVector(currentRow,0)=limitationError[i];
        // Now we set the delta-part:
        mainMatrix(currentRow,limitationIndex[i])=limitationValue[i];
        mainMatrix_correctJacobian(currentRow,limitationIndex[i])=limitationValue[i];
        currentRow++;
    }

    // Now we prepare the individual joint constraints part:
    //---------------------------------------------------------------------------
    for (size_t i=0;i<allJoints.size();i++)
    {
        if ( ((allJoints[i]->getJointMode()==ik_jointmode_dependent)||(allJoints[i]->getJointMode()==ik_jointmode_reserved_previously_ikdependent))&&(allJoints[i]->getJointType()!=ik_jointtype_spherical) )
        {
            int dependenceHandle=allJoints[i]->getDependencyJointHandle();
            if (dependenceHandle!=-1)
            {
                bool found=false;
                size_t depJointIndex;
                for (depJointIndex=0;depJointIndex<allJoints.size();depJointIndex++)
                {
                    if (allJoints[depJointIndex]->getObjectHandle()==dependenceHandle)
                    {
                        found=true;
                        break;
                    }
                }
                if (found)
                {
                    simReal coeff=allJoints[i]->getDependencyJointMult();
                    simReal fact=allJoints[i]->getDependencyJointAdd();
                    mainErrorVector(currentRow,0)=((allJoints[i]->getPosition(true)-fact)-
                                    coeff*allJoints[depJointIndex]->getPosition(true))*interpolFact;
                    mainMatrix(currentRow,i)=-simOne;
                    mainMatrix(currentRow,depJointIndex)=coeff;
                    mainMatrix_correctJacobian(currentRow,i)=-simOne;
                    mainMatrix_correctJacobian(currentRow,depJointIndex)=coeff;
                }
                else
                {   // joint of dependenceHandle is not part of this group calculation:
                    // therefore we take its current value --> WRONG! Since all temp params are initialized!
                    CJoint* dependentJoint=CEnvironment::currentEnvironment->objectContainer->getJoint(dependenceHandle);
                    if (dependentJoint!=nullptr)
                    {
                        simReal coeff=allJoints[i]->getDependencyJointMult();
                        simReal fact=allJoints[i]->getDependencyJointAdd();
                        mainErrorVector(currentRow,0)=((allJoints[i]->getPosition(true)-fact)-
                                        coeff*dependentJoint->getPosition(true))*interpolFact;
                        mainMatrix(currentRow,i)=-simOne;
                        mainMatrix_correctJacobian(currentRow,i)=-simOne;
                    }
                }
            }
            else
            {               
                mainErrorVector(currentRow,0)=interpolFact*(allJoints[i]->getPosition(true)-allJoints[i]->getDependencyJointAdd());
                mainMatrix(currentRow,i)=-simOne;
                mainMatrix_correctJacobian(currentRow,i)=-simOne;
            }
            currentRow++;
        }
    }
    //---------------------------------------------------------------------------

    // We take the joint weights into account here (part1):
    for (size_t i=0;i<mainMatrix.rows;i++)
    {
        for (size_t j=0;j<allJoints.size();j++)
        {
            simReal coeff=allJoints[j]->getIkWeight();
            if (coeff>=simZero)
                coeff=sqrt(coeff);
            else
                coeff=-sqrt(-coeff);
            mainMatrix(i,j)=mainMatrix(i,j)*coeff;
            mainMatrix_correctJacobian(i,j)=mainMatrix_correctJacobian(i,j)*coeff;
        }
    }

    // Now we just have to solve:
    size_t doF=mainMatrix.cols;
    size_t eqNumb=mainMatrix.rows;
    CMatrix solution(doF,1);

    if (!forInternalFunctionality)
    {
        delete _lastJacobian;
        _lastJacobian=new CMatrix(mainMatrix_correctJacobian);
    }

    int calcMethod=calculationMethod;
    simReal dampingFact=dlsFactor;

    if (calcMethod==ik_method_undamped_pseudo_inverse)
    {
        CMatrix JT(mainMatrix);
        JT.transpose();
        CMatrix pseudoJ(doF,eqNumb);
        CMatrix JJTInv(mainMatrix*JT);
        if (!JJTInv.inverse())
            return(-1);
        pseudoJ=JT*JJTInv;
        solution=pseudoJ*mainErrorVector;
    }
    if (calcMethod==ik_method_pseudo_inverse)
    {
        calcMethod=ik_method_damped_least_squares;
        dampingFact=0.000001;
    }
    if (calcMethod==ik_method_damped_least_squares)
    {
        CMatrix JT(mainMatrix);
        JT.transpose();
        CMatrix DLSJ(doF,eqNumb);
        CMatrix JJTInv(mainMatrix*JT);
        CMatrix ID(mainMatrix.rows,mainMatrix.rows);
        ID.setIdentity();
        ID/=simReal(1.0)/(dampingFact*dampingFact);
        JJTInv+=ID;
        if (!JJTInv.inverse())
            return(-1);
        DLSJ=JT*JJTInv;
        solution=DLSJ*mainErrorVector;
    }
    if (calcMethod==ik_method_jacobian_transpose)
    {
        CMatrix JT(mainMatrix);
        JT.transpose();
        solution=JT*mainErrorVector;
    }

    // We take the joint weights into account here (part2):
    for (size_t i=0;i<doF;i++)
    {
        CJoint* it=allJoints[i];
        simReal coeff=sqrt(fabs(it->getIkWeight()));
        solution(i,0)=solution(i,0)*coeff;
    }

    // We check if some variations are too big:
    if (!ignoreMaxStepSizes)
    {
        for (size_t i=0;i<doF;i++)
        {
            CJoint* it=allJoints[i];
            if (it->getJointType()!=ik_jointtype_prismatic)
                solution(i,0)=atan2(sin(solution(i,0)),cos(solution(i,0)));
            if (fabs(solution(i,0))>it->getMaxStepSize())
                return(0);
        }
    }

    // Check which joints hit a joint limit:
    for (size_t i=0;i<doF;i++)
    {
        CJoint* it=allJoints[i];
        if ( (it->getJointType()!=ik_jointtype_spherical)&&(!it->getPositionIsCyclic()) )
        {
            simReal nv=it->getPosition(true)+solution(i,0);
            if ( (solution(i,0)>0.0)&&(nv>it->getPositionIntervalMin()+it->getPositionIntervalRange()) )
                _jointLimitHits[it->getObjectHandle()]=nv-(it->getPositionIntervalMin()+it->getPositionIntervalRange());
            if ( (solution(i,0)<0.0)&&(nv<it->getPositionIntervalMin()) )
                _jointLimitHits[it->getObjectHandle()]=nv-it->getPositionIntervalMin();
        }
    }

    if ( _failOnJointLimits&&(!_jointLimitHits.empty()) )
        return(-1);

    // Now we set the computed values
    for (size_t i=0;i<doF;i++)
    {
        CJoint* it=allJoints[i];
        size_t stage=allJointStages[i];
        if (it->getJointType()!=ik_jointtype_spherical)
            it->setPosition(it->getPosition(true)+solution(i,0),true);
        else
            it->setTempParameterEx(it->getTempParameterEx(stage)+solution(i,0),stage);
    }
    return(1);
}

bool CikGroup::computeOnlyJacobian(int options)
{
    // Now we prepare a vector with all valid and active elements:
    std::vector<CikElement*> validElements;

    for (size_t elNb=0;elNb<_ikElements.size();elNb++)
    {
        CikElement* element=_ikElements[elNb];
        CDummy* tooltip=CEnvironment::currentEnvironment->objectContainer->getDummy(element->getTipHandle());
        CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(element->getBaseHandle());
        bool valid=true;
        if (!element->getIsActive())
            valid=false;
        if (tooltip==nullptr)
            valid=false; // should normally never happen!
        // We check that tooltip is parented with base and has at least one joint in-between:
        if (valid)
        {
            valid=false;
            bool jointPresent=false;
            bool baseOk=false;
            CSceneObject* iterat=tooltip;
            while ( (iterat!=base)&&(iterat!=nullptr) )
            {
                iterat=iterat->getParentObject();
                if (iterat==base)
                {
                    baseOk=true;
                    if (jointPresent)
                        valid=true;
                }
                if ( (iterat!=base)&&(iterat!=nullptr)&&(iterat->getObjectType()==ik_objecttype_joint) )
                {
                    if ( ((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_ik)||((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_reserved_previously_ikdependent)||((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_dependent) )
                        jointPresent=true;
                }
            }
        }
        if (valid)
            validElements.push_back(element);
    }

    // Now validElements contains all valid elements we have to use in the following computation!
    if (validElements.size()==0)
        return(false); // error

    _resetTemporaryParameters();

    // Here we prepare all element equations:
    for (size_t elNb=0;elNb<validElements.size();elNb++)
    {
        CikElement* element=validElements[elNb];
        element->prepareEquations(1.0);
    }
    return(performOnePass_jacobianOnly(&validElements,options));
}

bool CikGroup::performOnePass_jacobianOnly(std::vector<CikElement*>* validElements,int options)
{
    // We prepare a vector of all used joints and a counter for the number of rows:
    std::vector<CJoint*> allJoints;
    std::vector<size_t> allJointStages;
    size_t numberOfRows=0;
    for (size_t elNb=0;elNb<validElements->size();elNb++)
    {
        CikElement* element=validElements->at(elNb);
        numberOfRows+=element->matrix->rows;
        for (size_t i=0;i<element->rowJointHandles->size();i++)
        {
            int current=element->rowJointHandles->at(i);
            size_t currentStage=element->rowJointStages->at(i);
            // We check if that joint is already present:
            bool present=false;
            for (size_t j=0;j<allJoints.size();j++)
            {
                if ( (allJoints[j]->getObjectHandle()==current)&&(allJointStages[j]==currentStage) )
                {
                    present=true;
                    break;
                }
            }
            if (!present)
            {
                allJoints.push_back(CEnvironment::currentEnvironment->objectContainer->getJoint(current));
                allJointStages.push_back(currentStage);
            }
        }
    }

    // Now we prepare the individual joint constraints part:
    for (size_t i=0;i<allJoints.size();i++)
    {
        if (allJoints[i]->getJointType()!=ik_jointtype_spherical)
        {
            if ( (allJoints[i]->getJointMode()==ik_jointmode_reserved_previously_ikdependent)||(allJoints[i]->getJointMode()==ik_jointmode_dependent) )
                numberOfRows++;
        }
    }

    // We prepare the main matrix and the main error vector.
    CMatrix mainMatrix(numberOfRows,allJoints.size());
    CMatrix mainMatrix_correctJacobian(numberOfRows,allJoints.size());
    // We have to zero it first:
    mainMatrix.clear();
    mainMatrix_correctJacobian.clear();
    CMatrix mainErrorVector(numberOfRows,1);

    // Now we fill in the main matrix and the main error vector:
    size_t currentRow=0;
    for (size_t elNb=0;elNb<validElements->size();elNb++)
    {
        CikElement* element=validElements->at(elNb);
        for (size_t i=0;i<element->errorVector->rows;i++)
        { // We go through the rows:
            // We first set the error part:
            mainErrorVector(currentRow,0)=(*element->errorVector)(i,0);
            // Now we set the delta-parts:
            for (size_t j=0;j<element->matrix->cols;j++)
            { // We go through the columns:
                // We search for the right entry
                int jointHandle=element->rowJointHandles->at(j);
                size_t stage=element->rowJointStages->at(j);
                size_t index=0;
                while ( (allJoints[index]->getObjectHandle()!=jointHandle)||(allJointStages[index]!=stage) )
                    index++;
                mainMatrix(currentRow,index)=(*element->matrix)(i,j);
                mainMatrix_correctJacobian(currentRow,index)=(*element->matrix_correctJacobian)(i,j);
            }
            currentRow++;
        }
    }

    // Now we prepare the individual joint constraints part:
    //---------------------------------------------------------------------------
    for (size_t i=0;i<allJoints.size();i++)
    {
        if ( ((allJoints[i]->getJointMode()==ik_jointmode_dependent)||(allJoints[i]->getJointMode()==ik_jointmode_reserved_previously_ikdependent))&&(allJoints[i]->getJointType()!=ik_jointtype_spherical) )
        {
            int dependenceHandle=allJoints[i]->getDependencyJointHandle();
            if (dependenceHandle!=-1)
            {
                bool found=false;
                size_t depJointIndex;
                for (depJointIndex=0;depJointIndex<allJoints.size();depJointIndex++)
                {
                    if (allJoints[depJointIndex]->getObjectHandle()==dependenceHandle)
                    {
                        found=true;
                        break;
                    }
                }
                if (found)
                {
                    simReal coeff=allJoints[i]->getDependencyJointMult();
                    simReal fact=allJoints[i]->getDependencyJointAdd();
                    mainErrorVector(currentRow,0)=((allJoints[i]->getPosition(true)-fact)-
                                    coeff*allJoints[depJointIndex]->getPosition(true));
                    mainMatrix(currentRow,i)=-simOne;
                    mainMatrix(currentRow,depJointIndex)=coeff;
                    mainMatrix_correctJacobian(currentRow,i)=-simOne;
                    mainMatrix_correctJacobian(currentRow,depJointIndex)=coeff;
                }
                else
                {   // joint of dependenceHandle is not part of this group calculation:
                    // therefore we take its current value --> WRONG! Since all temp params are initialized!
                    CJoint* dependentJoint=CEnvironment::currentEnvironment->objectContainer->getJoint(dependenceHandle);
                    if (dependentJoint!=nullptr)
                    {
                        simReal coeff=allJoints[i]->getDependencyJointMult();
                        simReal fact=allJoints[i]->getDependencyJointAdd();
                        mainErrorVector(currentRow,0)=((allJoints[i]->getPosition(true)-fact)-
                                        coeff*dependentJoint->getPosition(true));
                        mainMatrix(currentRow,i)=-simOne;
                        mainMatrix_correctJacobian(currentRow,i)=-simOne;
                    }
                }
            }
            else
            {
                mainErrorVector(currentRow,0)=(allJoints[i]->getPosition(true)-allJoints[i]->getDependencyJointAdd());
                mainMatrix(currentRow,i)=-simOne;
                mainMatrix_correctJacobian(currentRow,i)=-simOne;
            }
            currentRow++;
        }
    }

    if ((options&1)!=0)
    { // We take the joint weights into account here (part1):
        for (size_t i=0;i<mainMatrix.rows;i++)
        {
            for (size_t j=0;j<allJoints.size();j++)
            {
                simReal coeff=allJoints[j]->getIkWeight();
                if (coeff>=simZero)
                    coeff=sqrt(coeff);
                else
                    coeff=-sqrt(-coeff);
                mainMatrix(i,j)=mainMatrix(i,j)*coeff;
                mainMatrix_correctJacobian(i,j)=mainMatrix_correctJacobian(i,j)*coeff;
            }
        }
    }

    delete _lastJacobian;
    _lastJacobian=new CMatrix(mainMatrix_correctJacobian);

    return(true);
}

simReal*  CikGroup::getLastJacobianData(size_t matrixSize[2]) const
{
    if (_lastJacobian==nullptr)
        return(nullptr);
    matrixSize[0]=_lastJacobian->cols;
    matrixSize[1]=_lastJacobian->rows;
    return(_lastJacobian->data);
}


simReal  CikGroup::getLastManipulabilityValue(bool& ok) const
{
    simReal retVal=0.0;
    if (_lastJacobian==nullptr)
        ok=false;
    else
    {
        ok=true;
        CMatrix JT(_lastJacobian[0]);
        JT.transpose();
        CMatrix JJT(_lastJacobian[0]*JT);
        retVal=sqrt(getDeterminant(JJT,nullptr,nullptr));
    }
    return(retVal);
}

simReal CikGroup::getDeterminant(const CMatrix& m,const std::vector<size_t>* activeRows,const std::vector<size_t>* activeColumns) const
{ // activeRows and activeColumns are nullptr by default (--> all rows and columns are active)
    // Routine is recursive! (i.e. Laplace expansion, which is not efficient for large matrices!)
    if (activeRows==nullptr)
    { // First call goes here:
        std::vector<size_t> actR;
        std::vector<size_t> actC;
        for (size_t i=0;i<m.cols;i++)
        {
            actR.push_back(i);
            actC.push_back(i);
        }
        return(getDeterminant(m,&actR,&actC));
    }

    // If we arrived here, we have to compute the determinant of the sub-matrix obtained
    // by removing all rows and columns not listed in activeRows, respectively activeColumns
    if (activeRows->size()==2)
    { // We compute this directly, we have a two-by-two matrix:
        simReal retVal=simZero;
        retVal+=m(activeRows->at(0),activeColumns->at(0))*m(activeRows->at(1),activeColumns->at(1));
        retVal-=m(activeRows->at(0),activeColumns->at(1))*m(activeRows->at(1),activeColumns->at(0));
        return(retVal);
    }

    if (activeRows->size()==3)
    { // We compute this directly, we have a three-by-three matrix:
        simReal retVal=simZero;
        retVal+=m(activeRows->at(0),activeColumns->at(0)) * ( (m(activeRows->at(1),activeColumns->at(1))*m(activeRows->at(2),activeColumns->at(2))) - (m(activeRows->at(1),activeColumns->at(2))*m(activeRows->at(2),activeColumns->at(1))) );
        retVal-=m(activeRows->at(0),activeColumns->at(1)) * ( (m(activeRows->at(1),activeColumns->at(0))*m(activeRows->at(2),activeColumns->at(2))) - (m(activeRows->at(1),activeColumns->at(2))*m(activeRows->at(2),activeColumns->at(0))) );
        retVal+=m(activeRows->at(0),activeColumns->at(2)) * ( (m(activeRows->at(1),activeColumns->at(0))*m(activeRows->at(2),activeColumns->at(1))) - (m(activeRows->at(1),activeColumns->at(1))*m(activeRows->at(2),activeColumns->at(0))) );
        return(retVal);
    }

    // The general routine
    std::vector<size_t> actR;
    std::vector<size_t> actC;
    simReal retVal=simZero;

    for (size_t colInd=1;colInd<activeColumns->size();colInd++)
        actC.push_back(activeColumns->at(colInd));
    for (size_t rowInd=0;rowInd<activeRows->size();rowInd++)
    {
        actR.clear();
        size_t i=activeRows->at(rowInd);
        for (size_t rowInd2=0;rowInd2<activeRows->size();rowInd2++)
        {
            size_t j=activeRows->at(rowInd2);
            if (j!=i)
                actR.push_back(j);
        }
        retVal+=m(i,activeColumns->at(0))*getDeterminant(m,&actR,&actC)*pow(-simOne,simReal(rowInd+2)); // was rowInd+1 until 3.1.3 rev2.
    }
    return(retVal);
}

void CikGroup::serialize(CSerialization &ar)
{
    if (ar.isWriting())
    {
        ar.writeInt(objectHandle);
        ar.writeString(objectName.c_str());
        ar.writeInt(maxIterations);
        ar.writeInt(constraints);
        ar.writeFloat(float(jointLimitWeight));
        ar.writeFloat(float(jointTreshholdAngular));
        ar.writeFloat(float(jointTreshholdLinear));
        ar.writeFloat(float(dlsFactor));
        ar.writeInt(calculationMethod);
        ar.writeInt(doOnFailOrSuccessOf);

        unsigned char nothing=0;
        SIM_SET_CLEAR_BIT(nothing,0,active);
        SIM_SET_CLEAR_BIT(nothing,1,restoreIfPositionNotReached);
        SIM_SET_CLEAR_BIT(nothing,2,restoreIfOrientationNotReached);
        SIM_SET_CLEAR_BIT(nothing,3,doOnFail);
        SIM_SET_CLEAR_BIT(nothing,4,doOnPerformed);
        SIM_SET_CLEAR_BIT(nothing,5,!ignoreMaxStepSizes);
        SIM_SET_CLEAR_BIT(nothing,6,_explicitHandling);
        SIM_SET_CLEAR_BIT(nothing,7,_failOnJointLimits);
        ar.writeByte(nothing);

        nothing=0;
        SIM_SET_CLEAR_BIT(nothing,0,_correctJointLimits);
        ar.writeByte(nothing);

        ar.writeInt(int(_ikElements.size()));
        for (size_t i=0;i<_ikElements.size();i++)
            _ikElements[i]->serialize(ar);
    }
    else
    {
        while (_ikElements.size()!=0)
            removeIkElement(_ikElements[0]->getIkElementHandle());

        objectHandle=ar.readInt();

        objectName=ar.readString();

        maxIterations=ar.readInt();

        constraints=ar.readInt();

        jointLimitWeight=simReal(ar.readFloat());

        jointTreshholdAngular=simReal(ar.readFloat());
        jointTreshholdLinear=simReal(ar.readFloat());

        dlsFactor=simReal(ar.readFloat());

        calculationMethod=ar.readInt();

        doOnFailOrSuccessOf=ar.readInt();

        unsigned char nothing=ar.readByte();
        active=SIM_IS_BIT_SET(nothing,0);
        restoreIfPositionNotReached=SIM_IS_BIT_SET(nothing,1);
        restoreIfOrientationNotReached=SIM_IS_BIT_SET(nothing,2);
        doOnFail=SIM_IS_BIT_SET(nothing,3);
        doOnPerformed=SIM_IS_BIT_SET(nothing,4);
        ignoreMaxStepSizes=!SIM_IS_BIT_SET(nothing,5);
        _explicitHandling=SIM_IS_BIT_SET(nothing,6);
        _failOnJointLimits=SIM_IS_BIT_SET(nothing,7);

        nothing=ar.readByte();
        _correctJointLimits=SIM_IS_BIT_SET(nothing,0);

        int el=ar.readInt();
        for (int i=0;i<el;i++)
        {
            CikElement* it=new CikElement(-1);
            it->serialize(ar);
            _ikElements.push_back(it);
        }
    }
}
