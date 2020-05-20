#include "ikElement.h"
#include "ikRoutines.h"
#include "environment.h"

CikElement::CikElement(int theTooltip)
{
    _tipHandle=theTooltip;
    _baseHandle=-1;
    _altBaseHandleForConstraints=-1;
    _isActive=true;
    _constraints=(ik_constraint_x|ik_constraint_y|ik_constraint_z);
    _minAngularPrecision=simReal(0.1)*degToRad;
    _minLinearPrecision=simReal(0.0005);
    _positionWeight=1.0;
    _orientationWeight=1.0;
    _ikElementHandle=-1;
    matrix=nullptr;
    matrix_correctJacobian=nullptr;
    errorVector=nullptr;
    rowJointHandles=nullptr;
    rowJointStages=nullptr;
}

CikElement::~CikElement()
{
    clearIkEquations();
}

bool CikElement::announceSceneObjectWillBeErased(int objectHandle)
{
    bool retVal=( (_baseHandle==objectHandle)||(_altBaseHandleForConstraints==objectHandle)||(_tipHandle==objectHandle) );
    return(retVal);
}

void CikElement::performSceneObjectLoadingMapping(const std::vector<int>* map)
{
    _tipHandle=_getLoadingMapping(map,_tipHandle);
    _baseHandle=_getLoadingMapping(map,_baseHandle);
    _altBaseHandleForConstraints=_getLoadingMapping(map,_altBaseHandleForConstraints);
}

void CikElement::serialize(CSerialization& ar)
{
    _ikElementHandle=ar.readInt();
    _tipHandle=ar.readInt();
    _baseHandle=ar.readInt();
    _altBaseHandleForConstraints=ar.readInt();
    _minAngularPrecision=simReal(ar.readFloat());
    _minLinearPrecision=simReal(ar.readFloat());
    _constraints=ar.readInt();
    _positionWeight=simReal(ar.readFloat());
    _orientationWeight=simReal(ar.readFloat());
    _isActive=(ar.readByte()&1);
}

int CikElement::getIkElementHandle() const
{
    return(_ikElementHandle);
}

void CikElement::setIkElementHandle(int handle)
{
    _ikElementHandle=handle;
}

int CikElement::getTipHandle() const
{
    return(_tipHandle);
}

int CikElement::getTargetHandle() const
{
    int retVal=-1;
    CDummy* tip=CEnvironment::currentEnvironment->objectContainer->getDummy(_tipHandle);
    if (tip!=nullptr)
    {
        int linkedDummyHandle=tip->getLinkedDummyHandle();
        if (tip->getLinkType()==ik_linktype_ik_tip_target)
            retVal=linkedDummyHandle;
    }
    return(retVal);
}

int CikElement::getBaseHandle() const
{
    return(_baseHandle);
}

void CikElement::setBaseHandle(int newBaseHandle)
{
    _baseHandle=newBaseHandle;
}

int CikElement::getAltBaseHandleForConstraints() const
{
    return(_altBaseHandleForConstraints);
}

void CikElement::setAltBaseHandleForConstraints(int newAltBaseHandle)
{
    _altBaseHandleForConstraints=newAltBaseHandle;
}

void CikElement::setRelatedJointsToPassiveMode()
{
    CSceneObject* it=CEnvironment::currentEnvironment->objectContainer->getDummy(_tipHandle);
    if (it!=nullptr)
    {
        CSceneObject* baseObj=CEnvironment::currentEnvironment->objectContainer->getObject(_baseHandle);
        it=it->getParentObject();
        while ( (it!=nullptr)&&(it!=baseObj) )
        {
            if (it->getObjectType()==ik_objecttype_joint)
                (static_cast<CJoint*>(it))->setJointMode(ik_jointmode_passive);
            it=it->getParentObject();
        }
    }
}

bool CikElement::getIsActive() const
{
    return(_isActive);
}

void CikElement::setIsActive(bool isActive)
{
    _isActive=isActive;
}

simReal CikElement::getMinLinearPrecision() const
{
    return(_minLinearPrecision);
}

void CikElement::setMinLinearPrecision(simReal precision)
{
    _minLinearPrecision=precision;
}

simReal CikElement::getMinAngularPrecision() const
{
    return(_minAngularPrecision);
}

void CikElement::setMinAngularPrecision(simReal precision)
{
    _minAngularPrecision=precision;
}

simReal CikElement::getPositionWeight() const
{
    return(_positionWeight);
}

void CikElement::setPositionWeight(simReal weight)
{
    _positionWeight=weight;
}

simReal CikElement::getOrientationWeight() const
{
    return(_orientationWeight);
}

void CikElement::setOrientationWeight(simReal weight)
{
    _orientationWeight=weight;
}

int CikElement::getConstraints() const
{
    return(_constraints);
}

void CikElement::setConstraints(int constraints)
{
    _constraints=constraints;
}

void CikElement::isWithinTolerance(bool& position,bool& orientation,bool useTempValues) const
{
    position=true;
    orientation=true;
    CDummy* targetObject=CEnvironment::currentEnvironment->objectContainer->getDummy(getTargetHandle());
    if (targetObject!=nullptr)
    {
        C7Vector targetTr(targetObject->getCumulativeTransformationPart1(useTempValues));
        CDummy* tooltipObject=CEnvironment::currentEnvironment->objectContainer->getDummy(_tipHandle);
        C7Vector tooltipTr(tooltipObject->getCumulativeTransformationPart1(useTempValues));
        C7Vector baseTrInv(C7Vector::identityTransformation);
        CDummy* baseObject=CEnvironment::currentEnvironment->objectContainer->getDummy(_baseHandle);
        if (baseObject!=nullptr)
            baseTrInv=baseObject->getCumulativeTransformationPart1(useTempValues).getInverse();
        CDummy* altBaseObject=CEnvironment::currentEnvironment->objectContainer->getDummy(_altBaseHandleForConstraints);
        if (altBaseObject!=nullptr)
            baseTrInv=altBaseObject->getCumulativeTransformationPart1(useTempValues).getInverse();
        tooltipTr=baseTrInv*tooltipTr;
        targetTr=baseTrInv*targetTr;
        simReal linAndAngErrors[2];
        _getMatrixError(targetTr.getMatrix(),tooltipTr.getMatrix(),linAndAngErrors);
        if ( (_constraints&(ik_constraint_x|ik_constraint_y|ik_constraint_z))!=0 )
        {
            if (_minLinearPrecision<linAndAngErrors[0])
                position=false;
        }
        if ( (_constraints&(ik_constraint_alpha_beta|ik_constraint_gamma))!=0 )
        {
            if (_minAngularPrecision<linAndAngErrors[1])
                orientation=false;
        }
    }
}

void CikElement::prepareEquations(simReal interpolationFactor)
{
    CDummy* targetObject=CEnvironment::currentEnvironment->objectContainer->getDummy(getTargetHandle());
    rowJointHandles=new std::vector<int>;
    rowJointStages=new std::vector<size_t>;
    C4X4Matrix m;
    CMatrix* jacobian=CIkRoutines::getJacobian(this,m,rowJointHandles,rowJointStages);
    C7Vector oldFrame(m);
    C7Vector oldFrameInv(oldFrame.getInverse());
    size_t equationNumber=0;
    size_t doF=jacobian->cols;
    C7Vector currentFrame;
    if (targetObject!=nullptr)
    {
        CSceneObject* baseObject=CEnvironment::currentEnvironment->objectContainer->getObject(_baseHandle);
        C7Vector baseTrInv(C7Vector::identityTransformation);
        if (baseObject!=nullptr)
            baseTrInv=baseObject->getCumulativeTransformation(true).getInverse();
        CSceneObject* altBaseObject=CEnvironment::currentEnvironment->objectContainer->getObject(_altBaseHandleForConstraints);
        if (altBaseObject!=nullptr)
            baseTrInv=altBaseObject->getCumulativeTransformation(true).getInverse();
        C7Vector targetTr=targetObject->getCumulativeTransformationPart1(true);
        targetTr=baseTrInv*targetTr;
        currentFrame.buildInterpolation(oldFrame,targetTr,interpolationFactor);
        if ((_constraints&ik_constraint_x)!=0)
            equationNumber++;
        if ((_constraints&ik_constraint_y)!=0)
            equationNumber++;
        if ((_constraints&ik_constraint_z)!=0)
            equationNumber++;
        if ((_constraints&ik_constraint_alpha_beta)!=0)
            equationNumber+=2;
        if ((_constraints&ik_constraint_gamma)!=0)
            equationNumber++;
    }
    matrix=new CMatrix(equationNumber,doF);
    matrix_correctJacobian=new CMatrix(equationNumber,doF);
    errorVector=new CMatrix(equationNumber,1);
    if (targetObject!=nullptr)
    {
        size_t pos=0;
        if ((_constraints&ik_constraint_x)!=0)
        {
            for (size_t i=0;i<doF;i++)
            {
                (*matrix)(pos,i)=(*jacobian)(0,i);
                (*matrix_correctJacobian)(pos,i)=(*jacobian)(0,i);
            }
            (*errorVector)(pos,0)=(currentFrame.X(0)-oldFrame.X(0))*_positionWeight;
            pos++;
        }
        if ((_constraints&ik_constraint_y)!=0)
        {
            for (size_t i=0;i<doF;i++)
            {
                (*matrix)(pos,i)=(*jacobian)(1,i);
                (*matrix_correctJacobian)(pos,i)=(*jacobian)(1,i);
            }
            (*errorVector)(pos,0)=(currentFrame.X(1)-oldFrame.X(1))*_positionWeight;
            pos++;
        }
        if ((_constraints&ik_constraint_z)!=0)
        {
            for (size_t i=0;i<doF;i++)
            {
                (*matrix)(pos,i)=(*jacobian)(2,i);
                (*matrix_correctJacobian)(pos,i)=(*jacobian)(2,i);
            }
            (*errorVector)(pos,0)=(currentFrame.X(2)-oldFrame.X(2))*_positionWeight;
            pos++;
        }
        if ( ((_constraints&ik_constraint_alpha_beta)!=0)&&((_constraints&ik_constraint_gamma)!=0) )
        { // full orientation constr.
            for (size_t i=0;i<doF;i++)
            {
                (*matrix)(pos,i)=(*jacobian)(3,i);
                (*matrix)(pos+1,i)=(*jacobian)(4,i);
                (*matrix)(pos+2,i)=(*jacobian)(5,i);
                (*matrix_correctJacobian)(pos,i)=(*jacobian)(3,i)*IK_DIVISION_FACTOR;
                (*matrix_correctJacobian)(pos+1,i)=(*jacobian)(4,i)*IK_DIVISION_FACTOR;
                (*matrix_correctJacobian)(pos+2,i)=(*jacobian)(5,i)*IK_DIVISION_FACTOR;
            }
            C4X4Matrix diff(oldFrameInv*currentFrame);
            C3Vector euler(diff.M.getEulerAngles());
            (*errorVector)(pos,0)=euler(0)*_orientationWeight/IK_DIVISION_FACTOR;
            (*errorVector)(pos+1,0)=euler(1)*_orientationWeight/IK_DIVISION_FACTOR;
            (*errorVector)(pos+2,0)=euler(2)*_orientationWeight/IK_DIVISION_FACTOR;
            pos=pos+3;
        }
        else
        {
            if ((_constraints&ik_constraint_alpha_beta)!=0)
            {
                for (size_t i=0;i<doF;i++)
                {
                    (*matrix)(pos,i)=(*jacobian)(3,i);
                    (*matrix)(pos+1,i)=(*jacobian)(4,i);
                    (*matrix_correctJacobian)(pos,i)=(*jacobian)(3,i)*IK_DIVISION_FACTOR;
                    (*matrix_correctJacobian)(pos+1,i)=(*jacobian)(4,i)*IK_DIVISION_FACTOR;
                }
                C4X4Matrix diff(oldFrameInv*currentFrame);
                C3Vector euler(diff.M.getEulerAngles());
                (*errorVector)(pos,0)=euler(0)*_orientationWeight/IK_DIVISION_FACTOR;
                (*errorVector)(pos+1,0)=euler(1)*_orientationWeight/IK_DIVISION_FACTOR;
                pos=pos+2;
            }
            if ((_constraints&ik_constraint_gamma)!=0)
            { // sim_gamma_constraint can also exist without ik_constraint_alpha_beta
                for (size_t i=0;i<doF;i++)
                {
                    (*matrix)(pos,i)=(*jacobian)(5,i);
                    (*matrix_correctJacobian)(pos,i)=(*jacobian)(5,i)*IK_DIVISION_FACTOR;
                }
                C4X4Matrix diff(oldFrameInv*currentFrame);
                C3Vector euler(diff.M.getEulerAngles());
                (*errorVector)(pos,0)=euler(2)*_orientationWeight/IK_DIVISION_FACTOR;
                pos++;
            }
        }
    }
    delete jacobian;
}

void CikElement::clearIkEquations()
{
    delete matrix;
    matrix=nullptr;
    delete matrix_correctJacobian;
    matrix_correctJacobian=nullptr;
    delete errorVector;
    errorVector=nullptr;
    delete rowJointHandles;
    rowJointHandles=nullptr;
    delete rowJointStages;
    rowJointStages=nullptr;
}

void CikElement::_getMatrixError(const C4X4Matrix& frame1,const C4X4Matrix& frame2,simReal linAndAngErrors[2]) const
{
    // Linear:
    simReal constr[3]={simZero,simZero,simZero};
    if ( (_constraints&ik_constraint_x)!=0 )
        constr[0]=simOne;
    if ( (_constraints&ik_constraint_y)!=0 )
        constr[1]=simOne;
    if ( (_constraints&ik_constraint_z)!=0 )
        constr[2]=simOne;
    C3Vector displ(frame2.X-frame1.X);
    linAndAngErrors[0]=sqrt(displ(0)*displ(0)*constr[0]+displ(1)*displ(1)*constr[1]+displ(2)*displ(2)*constr[2]);

    // Angular:
    if ( (_constraints&ik_constraint_gamma)!=0 )
    { // means implicitely also ik_constraint_alpha_beta constraint
        simReal x=frame1.M.axis[0]*frame2.M.axis[0];
        if (x<-simOne)
            x=-simOne;
        if (x>simOne)
            x=simOne;
        simReal z=frame1.M.axis[2]*frame2.M.axis[2];
        if (z<-simOne)
            z=-simOne;
        if (z>simOne)
            z=simOne;
        linAndAngErrors[1]=fabs(CMath::robustAcos(x));
        simReal v=fabs(CMath::robustAcos(z));
        if (v>linAndAngErrors[1])
            linAndAngErrors[1]=v;
    }
    else if ( (_constraints&ik_constraint_alpha_beta)!=0 )
    { // Free around Z
        simReal z=frame1.M.axis[2]*frame2.M.axis[2];
        if (z<-simOne)
            z=-simOne;
        if (z>simOne)
            z=simOne;
        linAndAngErrors[1]=fabs(CMath::robustAcos(z));
    }
    else
        linAndAngErrors[1]=simZero; // No ang. constraints
}

