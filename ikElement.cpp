#include "ikElement.h"
#include "ikRoutines.h"
#include "environment.h"

CikElement::CikElement()
{
}

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
}

CikElement::~CikElement()
{
    clearIkEquations();
}

CikElement* CikElement::copyYourself() const
{
    CikElement* duplicate=new CikElement();

    duplicate->_ikElementHandle=_ikElementHandle;
    duplicate->_tipHandle=_tipHandle;
    duplicate->_baseHandle=_baseHandle;
    duplicate->_altBaseHandleForConstraints=_altBaseHandleForConstraints;
    duplicate->_constraints=_constraints;
    duplicate->_isActive=_isActive;
    duplicate->_positionWeight=_positionWeight;
    duplicate->_orientationWeight=_orientationWeight;
    duplicate->_minAngularPrecision=_minAngularPrecision;
    duplicate->_minLinearPrecision=_minLinearPrecision;
    duplicate->jointHandles_tipToBase.assign(jointHandles_tipToBase.begin(),jointHandles_tipToBase.end());
    duplicate->jointStages_tipToBase.assign(jointStages_tipToBase.begin(),jointStages_tipToBase.end());
    duplicate->rowConstraints.assign(rowConstraints.begin(),rowConstraints.end());
    duplicate->matrix.set(matrix);
    duplicate->matrix_correctJacobian.set(matrix_correctJacobian);
    duplicate->errorVector.set(errorVector);

    return(duplicate);
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
    if (ar.isWriting())
    {
        ar.writeInt(_ikElementHandle);
        ar.writeInt(_tipHandle);
        ar.writeInt(_baseHandle);
        ar.writeInt(_altBaseHandleForConstraints);
        ar.writeFloat(float(_minAngularPrecision));
        ar.writeFloat(float(_minLinearPrecision));
        ar.writeInt(_constraints);
        ar.writeFloat(float(_positionWeight));
        ar.writeFloat(float(_orientationWeight));
        unsigned char nothing=0;
        nothing=nothing+1*_isActive;
        ar.writeByte(nothing);
    }
    else
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

void CikElement::getDistances(simReal& linDist,simReal& angDist,bool useTempValues) const
{ // returns the tip-target lin./ang. distances, taking into account the constraint settings
    linDist=simZero;
    angDist=simZero;
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
        _getMatrixError(targetTr,tooltipTr,linDist,angDist);
    }
}

void CikElement::isWithinTolerance(bool& position,bool& orientation,bool useTempValues) const
{
    position=true;
    orientation=true;
    simReal linDist,angDist;
    getDistances(linDist,angDist,useTempValues);
    if ( (_constraints&(ik_constraint_x|ik_constraint_y|ik_constraint_z))!=0 )
    {
        if (_minLinearPrecision<linDist)
            position=false;
    }
    if ( (_constraints&(ik_constraint_alpha_beta|ik_constraint_gamma))!=0 )
    {
        if (_minAngularPrecision<angDist)
            orientation=false;
    }
}

void CikElement::prepareEquations(simReal interpolationFactor)
{
    CDummy* targetObject=CEnvironment::currentEnvironment->objectContainer->getDummy(getTargetHandle());
    jointHandles_tipToBase.clear();
    jointStages_tipToBase.clear();
    C4X4Matrix m;
    CMatrix jacobian=CIkRoutines::getJacobian(this,m,&jointHandles_tipToBase,&jointStages_tipToBase);
    if (jacobian.data.size()==0)
        return; // should normally not happen

    C7Vector oldFrame(m);
    C7Vector oldFrameInv(oldFrame.getInverse());
    size_t equationNumber=0;
    size_t doF=jacobian.cols;
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
    matrix.resize(equationNumber,doF,0.0);
    matrix_correctJacobian.resize(equationNumber,doF,0.0);
    errorVector.resize(equationNumber,1,0.0);
    rowConstraints.clear();
    if (targetObject!=nullptr)
    {
        size_t pos=0;
        if ((_constraints&ik_constraint_x)!=0)
        {
            for (size_t i=0;i<doF;i++)
            {
                matrix(pos,i)=jacobian(0,i);
                matrix_correctJacobian(pos,i)=jacobian(0,i);
            }
            errorVector(pos,0)=(currentFrame.X(0)-oldFrame.X(0))*_positionWeight;
            rowConstraints.push_back(0);
            pos++;
        }
        if ((_constraints&ik_constraint_y)!=0)
        {
            for (size_t i=0;i<doF;i++)
            {
                matrix(pos,i)=jacobian(1,i);
                matrix_correctJacobian(pos,i)=jacobian(1,i);
            }
            errorVector(pos,0)=(currentFrame.X(1)-oldFrame.X(1))*_positionWeight;
            rowConstraints.push_back(1);
            pos++;
        }
        if ((_constraints&ik_constraint_z)!=0)
        {
            for (size_t i=0;i<doF;i++)
            {
                matrix(pos,i)=jacobian(2,i);
                matrix_correctJacobian(pos,i)=jacobian(2,i);
            }
            errorVector(pos,0)=(currentFrame.X(2)-oldFrame.X(2))*_positionWeight;
            rowConstraints.push_back(2);
            pos++;
        }
        if ( ((_constraints&ik_constraint_alpha_beta)!=0)&&((_constraints&ik_constraint_gamma)!=0) )
        { // full orientation constr.
            for (size_t i=0;i<doF;i++)
            {
                matrix(pos,i)=jacobian(3,i);
                matrix(pos+1,i)=jacobian(4,i);
                matrix(pos+2,i)=jacobian(5,i);
                matrix_correctJacobian(pos,i)=jacobian(3,i)*IK_DIVISION_FACTOR;
                matrix_correctJacobian(pos+1,i)=jacobian(4,i)*IK_DIVISION_FACTOR;
                matrix_correctJacobian(pos+2,i)=jacobian(5,i)*IK_DIVISION_FACTOR;
            }

            C4Vector q;
            q.buildInterpolation(oldFrame.Q,currentFrame.Q,1.0/IK_DIVISION_FACTOR);
            C3X3Matrix diff(oldFrame.Q.getInverse()*q);
            C3Vector euler(diff.getEulerAngles());
            euler=euler*1.0;
            errorVector(pos,0)=euler(0)*_orientationWeight;
            errorVector(pos+1,0)=euler(1)*_orientationWeight;
            errorVector(pos+2,0)=euler(2)*_orientationWeight;
            rowConstraints.push_back(3);
            rowConstraints.push_back(4);
            rowConstraints.push_back(5);
//            C4X4Matrix diff(oldFrameInv*currentFrame);
//            C3Vector euler(diff.M.getEulerAngles());
//            errorVector(pos,0)=euler(0)*_orientationWeight/IK_DIVISION_FACTOR;
//            errorVector(pos+1,0)=euler(1)*_orientationWeight/IK_DIVISION_FACTOR;
//            errorVector(pos+2,0)=euler(2)*_orientationWeight/IK_DIVISION_FACTOR;
            pos=pos+3;
        }
        else
        {
            if ((_constraints&ik_constraint_alpha_beta)!=0)
            {
                for (size_t i=0;i<doF;i++)
                {
                    matrix(pos,i)=jacobian(3,i);
                    matrix(pos+1,i)=jacobian(4,i);
                    matrix_correctJacobian(pos,i)=jacobian(3,i)*IK_DIVISION_FACTOR;
                    matrix_correctJacobian(pos+1,i)=jacobian(4,i)*IK_DIVISION_FACTOR;
                }
                C4Vector q;
                q.buildInterpolation(oldFrame.Q,currentFrame.Q,1.0/IK_DIVISION_FACTOR);
                C3X3Matrix diff(oldFrame.Q.getInverse()*q);
                C3Vector euler(diff.getEulerAngles());
                euler=euler*1.0;
                errorVector(pos,0)=euler(0)*_orientationWeight;
                errorVector(pos+1,0)=euler(1)*_orientationWeight;
                rowConstraints.push_back(3);
                rowConstraints.push_back(4);
//                C4X4Matrix diff(oldFrameInv*currentFrame);
//                C3Vector euler(diff.M.getEulerAngles());
//                errorVector(pos,0)=euler(0)*_orientationWeight/IK_DIVISION_FACTOR;
//                errorVector(pos+1,0)=euler(1)*_orientationWeight/IK_DIVISION_FACTOR;
                pos=pos+2;
            }
            if ((_constraints&ik_constraint_gamma)!=0)
            { // sim_gamma_constraint can also exist without ik_constraint_alpha_beta, e.g. when working in 2D
                for (size_t i=0;i<doF;i++)
                {
                    matrix(pos,i)=jacobian(5,i);
                    matrix_correctJacobian(pos,i)=jacobian(5,i)*IK_DIVISION_FACTOR;
                }
                C4Vector q;
                q.buildInterpolation(oldFrame.Q,currentFrame.Q,1.0/IK_DIVISION_FACTOR);
                C3X3Matrix diff(oldFrame.Q.getInverse()*q);
                C3Vector euler(diff.getEulerAngles());
                euler=euler*1.0;
                errorVector(pos,0)=euler(2)*_orientationWeight;
                rowConstraints.push_back(5);
//                C4X4Matrix diff(oldFrameInv*currentFrame);
//                C3Vector euler(diff.M.getEulerAngles());
//                errorVector(pos,0)=euler(2)*_orientationWeight/IK_DIVISION_FACTOR;
                pos++;
            }
        }
    }
}

void CikElement::clearIkEquations()
{
    jointHandles_tipToBase.clear();
    jointStages_tipToBase.clear();
}

void CikElement::_getMatrixError(const C7Vector& frame1,const C7Vector& frame2,simReal& linError,simReal& angError) const
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
    linError=sqrt(displ(0)*displ(0)*constr[0]+displ(1)*displ(1)*constr[1]+displ(2)*displ(2)*constr[2]);

    // Angular:
    if ( ((_constraints&ik_constraint_alpha_beta)!=0)&&((_constraints&ik_constraint_gamma)!=0) )
    {
        C4Vector aa((frame1.getInverse()*frame2).Q.getAngleAndAxis());
        angError=aa(0);
    }
    else if ( (_constraints&ik_constraint_alpha_beta)!=0 )
    { // Free around Z
        simReal z=frame1.getMatrix().M.axis[2]*frame2.getMatrix().M.axis[2];
        if (z<-simOne)
            z=-simOne;
        if (z>simOne)
            z=simOne;
        angError=fabs(CMath::robustAcos(z));
    }
    else if ( (_constraints&ik_constraint_gamma)!=0 )
    { // gamma constraint can exist also without alpha/beta constraint, e.g. in 2D
        C3Vector e((frame1.getInverse()*frame2).Q.getEulerAngles());
        angError=fabs(e(2));
    }
    else
        angError=simZero; // No ang. constraints
}

