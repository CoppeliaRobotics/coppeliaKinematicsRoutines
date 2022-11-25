#include "ikGroup.h"
#include "environment.h"
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/QR>

CikGroup::CikGroup()
{
    _objectHandle=2030003;
    _maxIterations=3;
    _explicitHandling_old=true;
    _dlsFactor=0.1;
    _calculationMethod=ik_method_pseudo_inverse;

    _options=ik_group_enabled|ik_group_ignoremaxsteps;
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

bool CikGroup::getExplicitHandling_old() const
{
    return(_explicitHandling_old);
}

void CikGroup::setAllInvolvedJointsToPassiveMode_old()
{
    for (size_t i=0;i<_ikElements.size();i++)
        _ikElements[i]->setRelatedJointsToPassiveMode_old();
}

void CikGroup::setObjectName(std::string newName)
{
    _objectName=newName;
}

void CikGroup::setOptions(int options)
{
    _options=options;
}

int CikGroup::getOptions() const
{
    return(_options);
}

void CikGroup::setMaxIterations(int maxIter)
{
    _maxIterations=maxIter;
}
int CikGroup::getMaxIterations() const
{
    return(_maxIterations);
}

void CikGroup::setDlsFactor(double theFactor)
{
    _dlsFactor=theFactor;
}
double CikGroup::getDlsFactor() const
{
    return(_dlsFactor);
}
void CikGroup::setCalculationMethod(int theMethod)
{
    if ( (theMethod==ik_method_pseudo_inverse)||(theMethod==ik_method_damped_least_squares)||
        (theMethod==ik_method_jacobian_transpose)||(theMethod==ik_method_undamped_pseudo_inverse) )
    {
        _calculationMethod=theMethod;
    }
}
int CikGroup::getCalculationMethod() const
{
    return(_calculationMethod);
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

    duplicate->_objectHandle=_objectHandle;
    duplicate->_objectName=_objectName;
    duplicate->_maxIterations=_maxIterations;
    duplicate->_dlsFactor=_dlsFactor;
    duplicate->_calculationMethod=_calculationMethod;
    duplicate->_options=_options;
    duplicate->_explicitHandling_old=_explicitHandling_old;
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
    return(_objectHandle);
}

void CikGroup::setObjectHandle(int handle)
{
    _objectHandle=handle;
}

std::string CikGroup::getObjectName() const
{
    return(_objectName);
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
    return(false);
}

bool CikGroup::getJointLimitHits(std::vector<int>* jointHandles,std::vector<double>* underOrOvershots) const
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

void CikGroup::getJointHandles(std::vector<int>& handles)
{ // returns a joint for each Jacobian column (spherical joints will however have a single entry, even if 3 columns in the Jacobian)
    handles.clear();
    // prepare a vector with all valid and active elements:
    std::vector<CikElement*> validElements;
    for (size_t elNb=0;elNb<_ikElements.size();elNb++)
    {
        CikElement* element=_ikElements[elNb];
        CDummy* tooltip=CEnvironment::currentEnvironment->objectContainer->getDummy(element->getTipHandle());
        CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(element->getBaseHandle());
        if ( element->getIsActive()&&(tooltip!=nullptr) )
        {
            bool valid=false;
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
                    if ((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_ik)
                        jointPresent=true;
                }
            }
            if (valid)
                validElements.push_back(element);
        }
    }
    if (validElements.size()!=0)
    {
        for (size_t elNb=0;elNb<validElements.size();elNb++)
        {
            CikElement* element=validElements[elNb];
            element->prepareEquations(1.0);
        }
        _performOnePass(&validElements,nullptr,true,2,nullptr);
        for (size_t i=0;i<_jointHandles.size();i++)
        {
            if (_jointDofIndex[i]==0)
                handles.push_back(_jointHandles[i]);
        }
    }
}

int CikGroup::computeGroupIk(double precision[2],bool forInternalFunctionality,bool(*cb)(const int*,std::vector<double>*,const int*,const int*,const int*,const int*,std::vector<double>*,double*))
{ // Return value is one ik_resultinfo...-value
    if (precision!=nullptr)
    {
        precision[0]=0.0;
        precision[1]=0.0;
    }

    _jointLimitHits.clear();
    if ((_options&ik_group_enabled)==0)
        return(ik_calc_notperformed|ik_calc_notwithintolerance); // group is disabled!

    // prepare a vector with all valid and active elements:
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
                    if ((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_ik)
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
        return(ik_calc_notperformed|ik_calc_notwithintolerance);

    std::vector<int> memorizedConf_handles;
    std::vector<double> memorizedConf_vals;
    CEnvironment::currentEnvironment->objectContainer->memorizeJointConfig(memorizedConf_handles,memorizedConf_vals);

    // Here we have the main iteration loop:
    double interpolFact=1.0; // We first try to solve in one step
    int cumulResultInfo=0;
    bool withinPosition,withinOrientation;
    for (int iterationNb=0;iterationNb<_maxIterations;iterationNb++)
    {
        // Here we prepare all element equations:
        for (size_t elNb=0;elNb<validElements.size();elNb++)
        {
            CikElement* element=validElements[elNb];
            element->prepareEquations(interpolFact);
        }

        std::vector<double> memorizedConfPass_vals;
        CEnvironment::currentEnvironment->objectContainer->memorizeJointConfig(memorizedConf_handles,memorizedConfPass_vals);

        double maxStepFact;
        int resultInfo=_performOnePass(&validElements,&maxStepFact,forInternalFunctionality,0,cb);
        cumulResultInfo=cumulResultInfo|resultInfo;

        if ( ((_options&ik_group_ignoremaxsteps)==0)&&((resultInfo&ik_calc_stepstoobig)!=0) )
        { // Joint variations not within tolerance. Retry by interpolating:
            interpolFact=interpolFact/(maxStepFact*1.1); // 10% tolerance
            CEnvironment::currentEnvironment->objectContainer->restoreJointConfig(memorizedConf_handles,memorizedConfPass_vals);
            iterationNb--; // redo this pass
        }
        else
        {
            if ( (maxStepFact<0.8)&&(interpolFact<1.0) )
            { // Joint variations are not too large: try to return to a non-interpolated target:
                interpolFact=interpolFact/(maxStepFact*1.1); // 10% tolerance
                if (interpolFact>1.0)
                    interpolFact=1.0;
            }
        }

        // We check if all IK elements are under the required precision
        withinPosition=true;
        withinOrientation=true;;
        for (size_t elNb=0;elNb<validElements.size();elNb++)
        {
            CikElement* element=validElements[elNb];
            double lp,ap;
            element->getTipTargetDistance(lp,ap);
            double pr[2];
            element->getPrecisions(pr);
            if (lp>pr[0])
                withinPosition=false;
            if (ap>pr[1])
                withinOrientation=false;
            if (precision!=nullptr)
            {
                if (lp>precision[0])
                    precision[0]=lp;
                if (ap>precision[1])
                    precision[1]=ap;
            }
        }
        if ( (!withinPosition)||(!withinOrientation) )
            cumulResultInfo=cumulResultInfo|ik_calc_notwithintolerance;
        if ( (cumulResultInfo&ik_calc_cannotinvert)!=0)
            break; // unrecoverable error
        if ( ((cumulResultInfo&ik_calc_limithit)!=0)&&((_options&ik_group_stoponlimithit)!=0) )
            break;
        if (withinPosition&&withinOrientation)
        {
            cumulResultInfo=cumulResultInfo&(~ik_calc_notwithintolerance);
            break;
        }
    }

    bool restoreConfig=false;
    if ( (cumulResultInfo&ik_calc_cannotinvert)!=0 )
        restoreConfig=true;
    if ( (cumulResultInfo&ik_calc_notwithintolerance)!=0 ) // notwithintolerance cannot always be considered as an error and results are applied by default
        restoreConfig=( (((_options&ik_group_restoreonbadlintol)!=0)&&(!withinPosition))||(((_options&ik_group_restoreonbadangtol)!=0)&&(!withinOrientation)) );
    if (restoreConfig)
        CEnvironment::currentEnvironment->objectContainer->restoreJointConfig(memorizedConf_handles,memorizedConf_vals);

    return(cumulResultInfo);
}

int CikGroup::_performOnePass(std::vector<CikElement*>* validElements,double* maxStepFact,bool forInternalFunctionality,int operation,bool(*cb)(const int*,std::vector<double>*,const int*,const int*,const int*,const int*,std::vector<double>*,double*))
{   // Return value: one of ik_resultinfo...-values
    // operation: 0=compute a pass, 1=compute Jacobian only, 2=get involved joint handles only
    if (maxStepFact!=nullptr)
        maxStepFact[0]=0.0;
    // We prepare a vector of all used joints and a counter for the number of rows:
    std::vector<CJoint*> allJoints;
    _jointHandles.clear(); // going through the Jacobian cols
    _jointDofIndex.clear();
    for (size_t elNb=0;elNb<validElements->size();elNb++)
    {
        CikElement* element=validElements->at(elNb);
        for (size_t i=0;i<element->jointHandles.size();i++)
        {
            int current=element->jointHandles[i];
            int currentDofIndex=element->jointDofIndex[i];
            // We check if that joint is already present:
            bool present=false;
            for (size_t j=0;j<allJoints.size();j++)
            {
                if ( (allJoints[j]->getObjectHandle()==current)&&(_jointDofIndex[j]==currentDofIndex) )
                {
                    present=true;
                    break;
                }
            }
            if (!present)
            {
                allJoints.push_back(CEnvironment::currentEnvironment->objectContainer->getJoint(current));
                _jointHandles.push_back(current);
                _jointDofIndex.push_back(currentDofIndex);
            }
        }
    }
    if (operation==2)
        return(0); // get involved joints only

    std::vector<int> _elementHandles; // going through the Jacobian rows
    std::vector<CikElement*> _elements; // going through the Jacobian rows
    std::vector<int> _equationType; // going through the Jacobian rows. 0-2: x,y,z, 3-5: alpha,beta,gamma, 6=jointLimits
    CMatrix mainJacobian(0,allJoints.size());
    CMatrix mainErrorVector(0,1);

    for (size_t elNb=0;elNb<validElements->size();elNb++)
    { // position and orientation constraints for all IK elements in that group:
        CikElement* element=validElements->at(elNb);
        for (size_t i=0;i<element->errorVector.rows;i++)
        { // We go through the rows:
            size_t rows=mainJacobian.rows+1;
            size_t currentRow=rows-1;
            mainJacobian.resize(rows,allJoints.size(),0.0);
            mainErrorVector.resize(rows,1,0.0);
            _elementHandles.push_back(element->getIkElementHandle());
            _elements.push_back(element);
            _equationType.push_back(element->equationTypes[i]);
            mainErrorVector(currentRow,0)=element->errorVector(i,0);
            // Now we set the delta-parts:
            for (size_t j=0;j<element->jacobian.cols;j++)
            { // We go through the columns:
                // We search for the right entry
                int jointHandle=element->jointHandles[j];
                int dofIndex=element->jointDofIndex[j];
                size_t index=0;
                while ( (allJoints[index]->getObjectHandle()!=jointHandle)||(_jointDofIndex[index]!=dofIndex) )
                    index++;
                mainJacobian(currentRow,index)=element->jacobian(i,j);
            }
        }
    }

    if ((_options&ik_group_avoidlimits)!=0)
    { // handle joint limits by counter-acting:
        for (size_t jointCounter=0;jointCounter<allJoints.size();jointCounter++)
        {
            CJoint* it=allJoints[jointCounter];
            double minVal=it->getPositionIntervalMin();
            double range=it->getPositionIntervalRange();
            double limitMargin=it->getLimitMargin();
            double value=it->getPosition();
            double distFromMin=value-limitMargin-minVal;
            double distFromMax=value+limitMargin-minVal-range;
            double eq=0.0;
            double activate=-10.0;
            if (it->getJointType()==ik_jointtype_revolute)
            {
                if (!it->getPositionIsCyclic())
                {
                    if (distFromMin<0.0)
                    {
                        activate=1.0; // We correct in the positive direction
                        eq=-distFromMin;
                    }
                    if (distFromMax>0.0)
                    {
                        activate=-1.0; // We correct in the negative direction
                        eq=distFromMax;
                    }
                }
            }
            if (it->getJointType()==ik_jointtype_prismatic)
            {
                if ( (distFromMin<0.0)&&(fabs(distFromMin)<fabs(distFromMax)) )
                {
                    activate=1.0; // We correct in the positive direction
                    eq=-distFromMin;
                }
                if ( (distFromMax>0.0)&&(fabs(distFromMax)<fabs(distFromMin)) )
                {
                    activate=-1.0; // We correct in the negative direction
                    eq=distFromMax;
                }
            }
            if (activate>-5.0)
            { // We have to activate a joint limitation equation
                // If we are over the treshhold of more than 5%:
                // (important in case target and tooltip are within tolerance)

                int rows=mainJacobian.rows+1;
                int currentRow=rows-1;
                mainJacobian.resize(rows,allJoints.size(),0.0);
                mainErrorVector.resize(rows,1,0.0);
                _elementHandles.push_back(-1);
                _equationType.push_back(6);
                mainJacobian(currentRow,jointCounter)=activate;
                mainErrorVector(currentRow,0)=eq;
            }
        }
    }

    _lastJacobian.set(mainJacobian);
    if (operation==1)
        return(0); // compute jacobian only

    // Now we just have to solve:
    CMatrix solution(mainJacobian.cols,1);
    bool computeHere=true;
    if (cb)
    {
        int js[2]={int(mainJacobian.rows),int(mainJacobian.cols)};
        computeHere=!cb(js,&mainJacobian.data,_equationType.data(),_elementHandles.data(),_jointHandles.data(),_jointDofIndex.data(),&mainErrorVector.data,solution.data.data());
        mainJacobian.rows=mainJacobian.data.size()/mainJacobian.cols;
        mainErrorVector.rows=mainJacobian.rows;
        _equationType.resize(mainJacobian.rows,8);
    }

    if (computeHere)
    {
        if (!forInternalFunctionality)
            _lastJacobian.set(mainJacobian);

        // position/orientation weights, and element weights:
        for (size_t i=0;i<_elements.size();i++)
        {
            if (_equationType[i]<=5)
            {
                double w[3];
                _elements[i]->getWeights(w);
                if (_equationType[i]<=2)
                    mainErrorVector(i,0)*=w[0]*w[2];
                else
                    mainErrorVector(i,0)*=w[1]*w[2];
            }
        }

        bool useJointWeights=false;
        for (size_t j=0;j<allJoints.size();j++)
        {
            if (allJoints[j]->getIkWeight()!=1.0)
            {
                useJointWeights=true;
                break;
            }
        }

        int calcMethod=_calculationMethod;
        double dampingFact=_dlsFactor;

        if (calcMethod==ik_method_undamped_pseudo_inverse)
        {
            calcMethod=ik_method_damped_least_squares;
            dampingFact=0.0;
        }
        if (calcMethod==ik_method_pseudo_inverse)
        {
            calcMethod=ik_method_damped_least_squares;
            dampingFact=0.000001;
        }
        if (calcMethod==ik_method_damped_least_squares)
        { // pseudo inverse with damping and joint weights: inv(W)*transp(J)*inv(J*inv(W)*transp(J)+damp*damp*I)
            CMatrix Idamp(mainJacobian.rows,mainJacobian.rows);
            Idamp.clear();
            for (size_t i=0;i<Idamp.rows;i++)
                Idamp(i,i)=dampingFact*dampingFact;
            CMatrix JT(mainJacobian);
            JT.transpose();
            if (useJointWeights)
            {
                CMatrix Winv(mainJacobian.cols,mainJacobian.cols);
                Winv.clear();
                for (size_t i=0;i<allJoints.size();i++)
                    Winv(i,i)=allJoints[i]->getIkWeight();
                solution=Winv*JT*pseudoInverse(mainJacobian*Winv*JT+Idamp)*mainErrorVector;
            }
            else
                solution=JT*pseudoInverse(mainJacobian*JT+Idamp)*mainErrorVector;
        }
        if (calcMethod==ik_method_jacobian_transpose)
        {
            CMatrix JT(mainJacobian);
            JT.transpose();
            solution=JT*mainErrorVector;
        }
    }

    int retVal=0;

    // We check if some variations are too big:
    for (size_t i=0;i<mainJacobian.cols;i++)
    {
        CJoint* it=allJoints[i];
        if (it->getJointType()!=ik_jointtype_prismatic)
            solution(i,0)=atan2(sin(solution(i,0)),cos(solution(i,0)));
        if (fabs(solution(i,0))>it->getMaxStepSize())
            retVal=retVal|ik_calc_stepstoobig;
        if (maxStepFact!=nullptr)
        {
            if (it->getMaxStepSize()>0.0)
            {
                double v=fabs(solution(i,0))/it->getMaxStepSize();
                if (v>maxStepFact[0])
                    maxStepFact[0]=v;
            }
        }
    }

    // Measure tip-target distances before applying new values:
    std::vector<double> distBefore;
    for (size_t elNb=0;elNb<validElements->size();elNb++)
    {
        CikElement* element=validElements->at(elNb);
        double lin,ang;
        element->getTipTargetDistance(lin,ang);
        distBefore.push_back(lin);
        distBefore.push_back(ang);
    }

    // Set the computed values
    for (size_t i=0;i<mainJacobian.cols;i++)
    {
        CJoint* it=allJoints[i];
        if (it->getJointType()==ik_jointtype_spherical)
        {
            C4Vector tr;
            tr.setEulerAngles(solution(i,0),solution(i+1,0),solution(i+2,0));
            it->setSphericalTransformation(it->getSphericalTransformation()*tr);
            i+=2;
        }
        else
            it->setPosition(it->getPosition()+solution(i,0));
    }

    for (size_t elNb=0;elNb<validElements->size();elNb++)
    {
        CikElement* element=validElements->at(elNb);
        double lin,ang;
        element->getTipTargetDistance(lin,ang);
        bool linOvershoot=(lin>0.001)&&(lin>distBefore[2*elNb+0]*1.1); // tolerate 10% overshoot
        bool angOvershoot=(ang>0.1*piValD2/180.0)&&(ang>distBefore[2*elNb+1]*1.1); // tolerate 10% overshoot
        if (linOvershoot||angOvershoot)
        {
            retVal=retVal|ik_calc_movingaway; // we overshot
            break;
        }
    }

    // Check which joints hit a joint limit:
    for (size_t i=0;i<mainJacobian.cols;i++)
    {
        CJoint* it=allJoints[i];
        if ( (it->getJointType()!=ik_jointtype_spherical)&&(!it->getPositionIsCyclic()) )
        {
            double nv=it->getPosition()+solution(i,0);
            if ( (solution(i,0)>0.0)&&(nv>it->getPositionIntervalMin()+it->getPositionIntervalRange()) )
                _jointLimitHits[it->getObjectHandle()]=nv-(it->getPositionIntervalMin()+it->getPositionIntervalRange());
            if ( (solution(i,0)<0.0)&&(nv<it->getPositionIntervalMin()) )
                _jointLimitHits[it->getObjectHandle()]=nv-it->getPositionIntervalMin();
        }
    }
    if (!_jointLimitHits.empty())
        retVal=retVal|ik_calc_limithit;

    return(retVal);
}

CMatrix CikGroup::pseudoInverse(const CMatrix& m)
{
    Eigen::MatrixXd m2(m.rows,m.cols);
    for (size_t i=0;i<m.rows;i++)
    {
        for (size_t j=0;j<m.cols;j++)
            m2(i,j)=m(i,j);
    }
    auto t=m2.completeOrthogonalDecomposition();
    Eigen::MatrixXd pseudoJ=t.pseudoInverse();
    CMatrix mOut(m.rows,m.rows);
    for (size_t i=0;i<m.rows;i++)
    {
        for (size_t j=0;j<m.rows;j++)
            mOut(i,j)=pseudoJ(i,j);
    }
    return(mOut);
}

bool CikGroup::computeOnlyJacobian_old(int options)
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
                    if ((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_ik)
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

    std::vector<int> memorizedConf_handles;
    std::vector<double> memorizedConf_vals;
    CEnvironment::currentEnvironment->objectContainer->memorizeJointConfig(memorizedConf_handles,memorizedConf_vals);

    // Here we prepare all element equations:
    for (size_t elNb=0;elNb<validElements.size();elNb++)
    {
        CikElement* element=validElements[elNb];
        element->prepareEquations(1.0);
    }
    int retVal=_performOnePass(&validElements,nullptr,false,true,nullptr);
    CEnvironment::currentEnvironment->objectContainer->restoreJointConfig(memorizedConf_handles,memorizedConf_vals);
    return(retVal!=ik_calc_cannotinvert);
}

const double*  CikGroup::getLastJacobianData_old(size_t matrixSize[2])
{ // back compat. function. Deprecated
    if (_lastJacobian.data.size()==0)
        return(nullptr);
    matrixSize[0]=_lastJacobian.cols;
    matrixSize[1]=_lastJacobian.rows;
    _lastJacobian_flipped=_lastJacobian;
    for (size_t i=0;i<_lastJacobian.rows;i++)
    {
        size_t jj=0;
        for (size_t j=_lastJacobian.cols;j>0;j--)
        {
            size_t cnt=size_t(_jointDofIndex[j-1]+1);
            j=j-cnt+1;
            for (size_t k=0;k<cnt;k++)
                _lastJacobian_flipped(i,jj++)=_lastJacobian(i,j-1+k);
        }
    }
    return(_lastJacobian_flipped.data.data());
}


double  CikGroup::getLastManipulabilityValue_old(bool& ok) const
{
    double retVal=0.0;
    if (_lastJacobian.data.size()==0)
        ok=false;
    else
    {
        ok=true;
        CMatrix JT(_lastJacobian);
        JT.transpose();
        CMatrix JJT(_lastJacobian*JT);
        retVal=sqrt(getDeterminant_old(JJT,nullptr,nullptr));
    }
    return(retVal);
}

double CikGroup::getDeterminant_old(const CMatrix& m,const std::vector<size_t>* activeRows,const std::vector<size_t>* activeColumns) const
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
        return(getDeterminant_old(m,&actR,&actC));
    }

    // If we arrived here, we have to compute the determinant of the sub-matrix obtained
    // by removing all rows and columns not listed in activeRows, respectively activeColumns
    if (activeRows->size()==2)
    { // We compute this directly, we have a two-by-two matrix:
        double retVal=0.0;
        retVal+=m(activeRows->at(0),activeColumns->at(0))*m(activeRows->at(1),activeColumns->at(1));
        retVal-=m(activeRows->at(0),activeColumns->at(1))*m(activeRows->at(1),activeColumns->at(0));
        return(retVal);
    }

    if (activeRows->size()==3)
    { // We compute this directly, we have a three-by-three matrix:
        double retVal=0.0;
        retVal+=m(activeRows->at(0),activeColumns->at(0)) * ( (m(activeRows->at(1),activeColumns->at(1))*m(activeRows->at(2),activeColumns->at(2))) - (m(activeRows->at(1),activeColumns->at(2))*m(activeRows->at(2),activeColumns->at(1))) );
        retVal-=m(activeRows->at(0),activeColumns->at(1)) * ( (m(activeRows->at(1),activeColumns->at(0))*m(activeRows->at(2),activeColumns->at(2))) - (m(activeRows->at(1),activeColumns->at(2))*m(activeRows->at(2),activeColumns->at(0))) );
        retVal+=m(activeRows->at(0),activeColumns->at(2)) * ( (m(activeRows->at(1),activeColumns->at(0))*m(activeRows->at(2),activeColumns->at(1))) - (m(activeRows->at(1),activeColumns->at(1))*m(activeRows->at(2),activeColumns->at(0))) );
        return(retVal);
    }

    // The general routine
    std::vector<size_t> actR;
    std::vector<size_t> actC;
    double retVal=0.0;

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
        retVal+=m(i,activeColumns->at(0))*getDeterminant_old(m,&actR,&actC)*pow(-1.0,rowInd+2); // was rowInd+1 until 3.1.3 rev2.
    }
    return(retVal);
}

void CikGroup::serialize(CSerialization &ar)
{
    if (ar.isWriting())
    {
        ar.writeInt(_objectHandle);
        ar.writeString(_objectName.c_str());
        ar.writeInt(_maxIterations);
        ar.writeInt(_options);
        ar.writeFloat(float(1.0)); // prev. jointLimitWeight
        ar.writeFloat(float(5.0*degToRad));
        ar.writeFloat(float(0.01));
        ar.writeFloat(float(_dlsFactor));
        ar.writeInt(_calculationMethod);
        ar.writeInt(-1);

        unsigned char nothing=0;
        // SIM_SET_CLEAR_BIT(nothing,0,active);
        //SIM_SET_CLEAR_BIT(nothing,1,restoreIfPositionNotReached);
        //SIM_SET_CLEAR_BIT(nothing,2,restoreIfOrientationNotReached);
        //SIM_SET_CLEAR_BIT(nothing,3,doOnFail);
        //SIM_SET_CLEAR_BIT(nothing,4,doOnPerformed);
        // SIM_SET_CLEAR_BIT(nothing,5,!ignoreMaxStepSizes);
        SIM_SET_CLEAR_BIT(nothing,6,_explicitHandling_old);
        //SIM_SET_CLEAR_BIT(nothing,7,_failOnJointLimits);
        ar.writeByte(nothing);

        nothing=0;
        //SIM_SET_CLEAR_BIT(nothing,0,_correctJointLimits);
        //SIM_SET_CLEAR_BIT(nothing,1,_forbidOvershoot);
        ar.writeByte(nothing);

        ar.writeInt(int(_ikElements.size()));
        for (size_t i=0;i<_ikElements.size();i++)
            _ikElements[i]->serialize(ar);
    }
    else
    {
        while (_ikElements.size()!=0)
            removeIkElement(_ikElements[0]->getIkElementHandle());

        _objectHandle=ar.readInt();

        _objectName=ar.readString();

        _maxIterations=ar.readInt();

        _options=ar.readInt();

        ar.readFloat(); // prev. jointLimitWeight

        ar.readFloat(); // prev. _jointTreshholdAngular
        ar.readFloat(); // prev. _jointTreshholdLinear

        _dlsFactor=double(ar.readFloat());

        _calculationMethod=ar.readInt();

        ar.readInt(); // previously doOnFailOrSuccessOf

        unsigned char nothing=ar.readByte();
        // active=SIM_IS_BIT_SET(nothing,0);
        //restoreIfPositionNotReached=SIM_IS_BIT_SET(nothing,1);
        //restoreIfOrientationNotReached=SIM_IS_BIT_SET(nothing,2);
        //doOnFail=SIM_IS_BIT_SET(nothing,3);
        //doOnPerformed=SIM_IS_BIT_SET(nothing,4);
        // ignoreMaxStepSizes=!SIM_IS_BIT_SET(nothing,5);
        _explicitHandling_old=SIM_IS_BIT_SET(nothing,6);
        //_failOnJointLimits=SIM_IS_BIT_SET(nothing,7);

        nothing=ar.readByte();
        //_correctJointLimits=SIM_IS_BIT_SET(nothing,0);
        //_forbidOvershoot=SIM_IS_BIT_SET(nothing,1);

        int el=ar.readInt();
        for (int i=0;i<el;i++)
        {
            CikElement* it=new CikElement(-1);
            it->serialize(ar);
            _ikElements.push_back(it);
        }
    }
}
