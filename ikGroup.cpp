#include "ikGroup.h"
#include "environment.h"
#include <algorithm>


CikGroup::CikGroup()
{
    objectHandle=2030003;
    maxIterations=3;
    active=true;
    ignoreMaxStepSizes=true;
    _failOnJointLimits=false;
    _forbidOvershoot=false;
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
    duplicate->_forbidOvershoot=_forbidOvershoot;
    duplicate->ignoreMaxStepSizes=ignoreMaxStepSizes;
    duplicate->_calculationResult=_calculationResult;
    duplicate->_explicitHandling=_explicitHandling;
    duplicate->_lastJacobian.set(_lastJacobian);

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

bool CikGroup::getForbidOvershoot() const
{
    return(_forbidOvershoot);
}

void CikGroup::setForbidOvershoot(bool forbid)
{
    _forbidOvershoot=forbid;
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

int CikGroup::computeGroupIk(bool forInternalFunctionality,int(*cb)(const int*,simReal*,const int*,const int*,const int*,const int*,simReal*,simReal*))
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

        int res=performOnePass(&validElements,limitOrAvoidanceNeedMoreCalculation,interpolFact,forInternalFunctionality,false,cb);
        if (res==-1)
        { // an error occured during resolution, or a joint limit was hit (and we didn't allow joint limits to be hit)
            errorOccured=true;
            leaveNow=true;
        }
        if (res==0)
        { // Joint variations not within tolerance. Restart from the beginning
            interpolFact=interpolFact/simReal(2.0);
            _resetTemporaryParameters();
        }
        if (res==1)
        { // Joint variations within tolerance
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
        if (leaveNow)
            break;
    }
    int returnValue=ik_result_fail;;
    if (!errorOccured)
        returnValue=ik_result_success;
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

int CikGroup::performOnePass(std::vector<CikElement*>* validElements,bool& limitOrAvoidanceNeedMoreCalculation,simReal interpolFact,bool forInternalFunctionality,bool computeOnlyJacobian,int(*cb)(const int*,simReal*,const int*,const int*,const int*,const int*,simReal*,simReal*))
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
    std::vector<int> callback_jointHandles;
    std::vector<int> callback_jointStages;
    for (size_t elNb=0;elNb<validElements->size();elNb++)
    {
        CikElement* element=validElements->at(elNb);
        for (size_t i=0;i<element->jointHandles.size();i++)
        {
            int current=element->jointHandles[i];
            size_t currentStage=element->jointStages[i];
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
                callback_jointHandles.push_back(current);
                callback_jointStages.push_back(int(currentStage));
            }
        }
    }
    //---------------------------------------------------------------------------

    std::vector<int> callback_elementHandles;
    std::vector<int> callback_rowType;
    CMatrix mainJacobian(0,allJoints.size());
    CMatrix mainErrorVector(0,1);

    for (size_t elNb=0;elNb<validElements->size();elNb++)
    { // position and orientation constraints for all IK elements in that group:
        CikElement* element=validElements->at(elNb);
        for (size_t i=0;i<element->errorVector.rows;i++)
        { // We go through the rows:
            int rows=mainJacobian.rows+1;
            int currentRow=rows-1;
            mainJacobian.resize(rows,allJoints.size(),0.0);
            mainErrorVector.resize(rows,1,0.0);
            callback_elementHandles.push_back(element->getIkElementHandle());
            callback_rowType.push_back(element->rowConstraints[i]);
            mainErrorVector(currentRow,0)=element->errorVector(i,0);
            // Now we set the delta-parts:
            for (size_t j=0;j<element->jacobian.cols;j++)
            { // We go through the columns:
                // We search for the right entry
                int jointHandle=element->jointHandles[j];
                size_t stage=element->jointStages[j];
                size_t index=0;
                while ( (allJoints[index]->getObjectHandle()!=jointHandle)||(allJointStages[index]!=stage) )
                    index++;
                mainJacobian(currentRow,index)=element->jacobian(i,j);
            }
        }
    }

    if (_correctJointLimits)
    { // handle joint limits by counter-acting:
        for (size_t jointCounter=0;jointCounter<allJoints.size();jointCounter++)
        {
            CJoint* it=allJoints[jointCounter];
            size_t stage=allJointStages[jointCounter];
            simReal minVal=it->getPositionIntervalMin();
            simReal range=it->getPositionIntervalRange();
            simReal jointThreshold=jointTreshholdAngular;
            if (it->getJointType()==ik_jointtype_prismatic)
                jointThreshold=jointTreshholdLinear;
            simReal value=it->getPosition(true);
            if (it->getJointType()==ik_jointtype_spherical)
                value=it->getTempParameterEx(stage);
            simReal distFromMin=value-jointThreshold-minVal;
            simReal distFromMax=value+jointThreshold-minVal-range;
            simReal eq=simZero;
            simReal activate=simReal(-10.0);
            if (it->getJointType()==ik_jointtype_revolute)
            {
                if (!it->getPositionIsCyclic())
                {
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
                }
            }
            if (it->getJointType()==ik_jointtype_prismatic)
            {
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
            }
            if (it->getJointType()==ik_jointtype_spherical)
            {
                if ( (it->getTempSphericalJointLimitations()!=0)&&(stage==1) ) // Joint limitation configuration was activated!
                {
                    if (distFromMax>simZero)
                    {
                        activate=-simOne; // We correct in the negative direction
                        eq=distFromMax;
                    }
                }
            }
            if (activate>simReal(-5.0))
            { // We have to activate a joint limitation equation
                // If we are over the treshhold of more than 5%:
                // (important in case target and tooltip are within tolerance)
                if (eq>jointThreshold*simReal(0.05))
                    limitOrAvoidanceNeedMoreCalculation=true;

                int rows=mainJacobian.rows+1;
                int currentRow=rows-1;
                mainJacobian.resize(rows,allJoints.size(),0.0);
                mainErrorVector.resize(rows,1,0.0);
                callback_elementHandles.push_back(-1);
                callback_rowType.push_back(6);
                mainJacobian(currentRow,jointCounter)=activate;
                mainErrorVector(currentRow,0)=eq*jointLimitWeight;
            }
        }
    }

    for (size_t i=0;i<allJoints.size();i++)
    { // handle joint dependencies:
        if ( ((allJoints[i]->getJointMode()==ik_jointmode_dependent)||(allJoints[i]->getJointMode()==ik_jointmode_reserved_previously_ikdependent))&&(allJoints[i]->getJointType()!=ik_jointtype_spherical) )
        {
            int rows=mainJacobian.rows+1;
            int currentRow=rows-1;
            mainJacobian.resize(rows,allJoints.size(),0.0);
            mainErrorVector.resize(rows,1,0.0);
            callback_elementHandles.push_back(-1);
            callback_rowType.push_back(7);

            int dependenceHandle=allJoints[i]->getDependencyJointHandle();
            if (dependenceHandle!=-1)
            {
                simReal coeff=allJoints[i]->getDependencyJointMult();
                simReal fact=allJoints[i]->getDependencyJointAdd();
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
                    mainJacobian(currentRow,i)=-simOne;
                    mainJacobian(currentRow,depJointIndex)=coeff;
                    mainErrorVector(currentRow,0)=((allJoints[i]->getPosition(true)-fact)-
                                    coeff*allJoints[depJointIndex]->getPosition(true))*interpolFact;
                }
                else
                {   // joint of dependenceHandle is not part of this group calculation:
                    // therefore we take its current value --> WRONG! Since all temp params are initialized!
                    CJoint* dependentJoint=CEnvironment::currentEnvironment->objectContainer->getJoint(dependenceHandle);
                    if (dependentJoint!=nullptr)
                    {
                        mainJacobian(currentRow,i)=-simOne;
                        mainErrorVector(currentRow,0)=((allJoints[i]->getPosition(true)-fact)-
                                        coeff*dependentJoint->getPosition(true))*interpolFact;
                    }
                }
            }
            else
            {  // there is no dependent counterpart joint. We keep the joint fixed:
                mainJacobian(currentRow,i)=-simOne;
                mainErrorVector(currentRow,0)=interpolFact*(allJoints[i]->getPosition(true)-allJoints[i]->getDependencyJointAdd());
            }
        }
    }
    _lastJacobian.set(mainJacobian);
    if (computeOnlyJacobian)
        return(1);

    // Now we just have to solve:
    size_t doF=mainJacobian.cols;
    size_t eqNumb=mainJacobian.rows;
    CMatrix solution(doF,1);
    bool computeHere=true;
    if (cb)
    {
        int js[2]={int(mainJacobian.rows),int(mainJacobian.cols)};
        int res=cb(js,mainJacobian.data.data(),callback_rowType.data(),callback_elementHandles.data(),callback_jointHandles.data(),callback_jointStages.data(),mainErrorVector.data.data(),solution.data.data());
        if (res==2)
            computeHere=false;
        else
        {
            // We take the joint weights into account here (part1):
            for (size_t i=0;i<mainJacobian.rows;i++)
            {
                for (size_t j=0;j<mainJacobian.cols;j++)
                {
                    simReal coeff=allJoints[j]->getIkWeight();
                    if (coeff>=simZero)
                        coeff=sqrt(coeff);
                    else
                        coeff=-sqrt(-coeff);
                    mainJacobian(i,j)=mainJacobian(i,j)*coeff;
                }
            }

        }
    }

    if (computeHere)
    {
        if (!forInternalFunctionality)
            _lastJacobian.set(mainJacobian);

        int calcMethod=calculationMethod;
        simReal dampingFact=dlsFactor;

        if (calcMethod==ik_method_undamped_pseudo_inverse)
        {
            CMatrix JT(mainJacobian);
            JT.transpose();
            CMatrix pseudoJ(doF,eqNumb);
            CMatrix JJTInv(mainJacobian*JT);
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
            CMatrix JT(mainJacobian);
            JT.transpose();
            CMatrix DLSJ(doF,eqNumb);
            CMatrix JJTInv(mainJacobian*JT);
            CMatrix ID(mainJacobian.rows,mainJacobian.rows);
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
            CMatrix JT(mainJacobian);
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


    // Measure tip-target distances before applying new values:
    std::vector<simReal> distBefore;
    simReal overshootDamp=1.0;
    if (_forbidOvershoot)
    {
        overshootDamp=0.75; // for now hard-coded
        for (size_t elNb=0;elNb<validElements->size();elNb++)
        {
            CikElement* element=validElements->at(elNb);
            simReal lin,ang;
            element->getDistances(lin,ang,true);
            distBefore.push_back(lin);
            distBefore.push_back(ang);
        }
    }

    // Set the computed values
    for (size_t i=0;i<doF;i++)
    {
        CJoint* it=allJoints[i];
        size_t stage=allJointStages[i];
        if (it->getJointType()!=ik_jointtype_spherical)
            it->setPosition(it->getPosition(true)+solution(i,0)*overshootDamp,true);
        else
            it->setTempParameterEx(it->getTempParameterEx(stage)+solution(i,0)*overshootDamp,stage);
    }

    if (_forbidOvershoot)
    {
        for (size_t elNb=0;elNb<validElements->size();elNb++)
        {
            CikElement* element=validElements->at(elNb);
            simReal lin,ang;
            element->getDistances(lin,ang,true);
            bool linOvershoot=(distBefore[2*elNb+0]<lin)&&(lin>0.01);
            bool angOvershoot=(distBefore[2*elNb+1]<ang)&&(ang>1.0*piValD2/180.0);
            if ( linOvershoot&&(angOvershoot||((element->getConstraints()&ik_constraint_orientation)==0)) )
                return(-1); // we overshot
            if ( angOvershoot&&(linOvershoot||((element->getConstraints()&ik_constraint_position)==0)) )
                return(-1); // we overshot
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
    bool dummy=false;
    return(performOnePass(&validElements,dummy,1.0,false,true,nullptr));
}

const simReal*  CikGroup::getLastJacobianData(size_t matrixSize[2]) const
{
    if (_lastJacobian.data.size()==0)
        return(nullptr);
    matrixSize[0]=_lastJacobian.cols; // actually inverted, but let's leave it
    matrixSize[1]=_lastJacobian.rows;
    return(_lastJacobian.data.data());
}


simReal  CikGroup::getLastManipulabilityValue(bool& ok) const
{
    simReal retVal=0.0;
    if (_lastJacobian.data.size()==0)
        ok=false;
    else
    {
        ok=true;
        CMatrix JT(_lastJacobian);
        JT.transpose();
        CMatrix JJT(_lastJacobian*JT);
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
        SIM_SET_CLEAR_BIT(nothing,1,_forbidOvershoot);
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
        _forbidOvershoot=SIM_IS_BIT_SET(nothing,1);

        int el=ar.readInt();
        for (int i=0;i<el;i++)
        {
            CikElement* it=new CikElement(-1);
            it->serialize(ar);
            _ikElements.push_back(it);
        }
    }
}
