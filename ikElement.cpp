#include "ikElement.h"
#include "environment.h"
#include "MyMath.h"

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
    duplicate->jacobian.set(jacobian);
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
    jointHandles_tipToBase.clear();
    jointStages_tipToBase.clear();
    rowConstraints.clear();
    CDummy* tip=CEnvironment::currentEnvironment->objectContainer->getDummy(_tipHandle);
    CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(_baseHandle);
    if ( (tip==nullptr)||((base!=nullptr)&&(!tip->isObjectAffiliatedWith(base))) )
        return;
    CSceneObject* constrBase=base;
    CSceneObject* altBase=CEnvironment::currentEnvironment->objectContainer->getObject(_altBaseHandleForConstraints);
    if (altBase!=nullptr)
        constrBase=altBase;
    jacobian=_getJacobian(tip,base,constrBase,&jointHandles_tipToBase,&jointStages_tipToBase);
    matrix_correctJacobian=jacobian;
    if ( (jacobian.rows==0)||(jacobian.cols==0) )
        return;
    errorVector.resize(jacobian.rows,1,0.0);

    CDummy* target=CEnvironment::currentEnvironment->objectContainer->getDummy(getTargetHandle());
    if (target==nullptr)
        return;
    C7Vector constrBaseTrInverse;
    constrBaseTrInverse.setIdentity();
    if (constrBase!=nullptr)
        constrBaseTrInverse=constrBase->getCumulativeTransformation(true).getInverse();
    C7Vector tipRelConstrBase(constrBaseTrInverse*tip->getCumulativeTransformationPart1(true));
    C7Vector targetRelConstrBase(constrBaseTrInverse*target->getCumulativeTransformationPart1(true));
    C7Vector interpolTargetRelConstrBase;
    simReal dq=0.1; // not that relevant apparently
    interpolTargetRelConstrBase.buildInterpolation(tipRelConstrBase,targetRelConstrBase,interpolationFactor*dq);
    C7Vector dx(tipRelConstrBase.getInverse()*interpolTargetRelConstrBase);
    size_t rowIndex=0;
    if ((_constraints&ik_constraint_x)!=0)
    {
        errorVector(rowIndex++,0)=dx(0)*_positionWeight/dq;
        rowConstraints.push_back(0);
    }
    if ((_constraints&ik_constraint_y)!=0)
    {
        errorVector(rowIndex++,0)=dx(1)*_positionWeight/dq;
        rowConstraints.push_back(1);
    }
    if ((_constraints&ik_constraint_z)!=0)
    {
        errorVector(rowIndex++,0)=dx(2)*_positionWeight/dq;
        rowConstraints.push_back(2);
    }
    if ((_constraints&(ik_constraint_alpha_beta|ik_constraint_gamma))!=0)
    {
        C3Vector euler(dx.Q.getEulerAngles());
        if ((_constraints&ik_constraint_alpha_beta)!=0)
        {
            errorVector(rowIndex++,0)=euler(0)/dq;
            rowConstraints.push_back(3);
            errorVector(rowIndex++,0)=euler(1)/dq;
            rowConstraints.push_back(4);
        }
        if ((_constraints&ik_constraint_gamma)!=0)
        {
            errorVector(rowIndex++,0)=euler(2)/dq;
            rowConstraints.push_back(5);
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

CMatrix CikElement::_getJacobian(const CSceneObject* tip,const CSceneObject* base,const CSceneObject* constrBase,std::vector<int>* jointHandles_tipToBase,std::vector<size_t>* jointStages_tipToBase) const
{
    size_t rows=0;
    if ((_constraints&ik_constraint_x)!=0)
        rows++;
    if ((_constraints&ik_constraint_y)!=0)
        rows++;
    if ((_constraints&ik_constraint_z)!=0)
        rows++;
    if ((_constraints&ik_constraint_alpha_beta)!=0)
        rows+=2;
    if ((_constraints&ik_constraint_gamma)!=0)
        rows++;

    CMatrix jacobian(rows,0);
    CSceneObject* object=tip->getParentObject();
    C7Vector constrBaseTrInverse;
    constrBaseTrInverse.setIdentity();
    if (constrBase!=nullptr)
        constrBaseTrInverse=constrBase->getCumulativeTransformation(true).getInverse();
    C7Vector tipRelConstrBase_inv((constrBaseTrInverse*tip->getCumulativeTransformationPart1(true)).getInverse());
    int dofIndex=0;
    while (object!=base)
    {
        if (object->getObjectType()==ik_objecttype_joint)
        {
            CJoint* joint=(CJoint*)object;
            if ( (joint->getJointMode()==ik_jointmode_ik)||(joint->getJointMode()==ik_jointmode_reserved_previously_ikdependent)||(joint->getJointMode()==ik_jointmode_dependent) )
            {
                jointHandles_tipToBase->push_back(joint->getObjectHandle());
                jointStages_tipToBase->push_back(dofIndex);
                int colIndex=jacobian.cols;
                jacobian.resize(rows,jacobian.cols+1,0.0);
                C7Vector jbabs(joint->getCumulativeTransformation(true));
                C7Vector jb(constrBaseTrInverse*jbabs);
                C7Vector x(jbabs.getInverse()*tip->getCumulativeTransformationPart1(true));
                C7Vector dj;
                dj.setIdentity();
                simReal dq=0.01; // starts breaking at dj>0.05 or at dj<0.01
                if (joint->getJointType()==ik_jointtype_prismatic)
                    dj(2)=dq;
                if (joint->getJointType()==ik_jointtype_revolute)
                {
                    dj(2)=joint->getScrewPitch()*dq;
                    dj.Q.setAngleAndAxis(dq,C3Vector(0.0,0.0,1.0));
                }
                if (joint->getJointType()==ik_jointtype_spherical)
                {
                    if (dofIndex==0)
                        dj.Q.setAngleAndAxis(dq,C3Vector(1.0,0.0,0.0));
                    if (dofIndex==1)
                        dj.Q.setAngleAndAxis(dq,C3Vector(0.0,1.0,0.0));
                    if (dofIndex==2)
                        dj.Q.setAngleAndAxis(dq,C3Vector(0.0,0.0,1.0));
                    dofIndex++;
                }
                C7Vector tipRelConstrBase_changed(jb*dj*x);
                C7Vector dx(tipRelConstrBase_inv*tipRelConstrBase_changed);
                size_t rowIndex=0;
                if ((_constraints&ik_constraint_x)!=0)
                    jacobian(rowIndex++,colIndex)=dx(0)/dq;
                if ((_constraints&ik_constraint_y)!=0)
                    jacobian(rowIndex++,colIndex)=dx(1)/dq;
                if ((_constraints&ik_constraint_z)!=0)
                    jacobian(rowIndex++,colIndex)=dx(2)/dq;
                if ((_constraints&(ik_constraint_alpha_beta|ik_constraint_gamma))!=0)
                {
                    C3Vector euler(dx.Q.getEulerAngles());
                    if ((_constraints&ik_constraint_alpha_beta)!=0)
                    {
                        jacobian(rowIndex++,colIndex)=euler(0)/dq;
                        jacobian(rowIndex++,colIndex)=euler(1)/dq;
                    }
                    if ((_constraints&ik_constraint_gamma)!=0)
                        jacobian(rowIndex++,colIndex)=euler(2)/dq;
                }
            }
        }
        if (dofIndex==3)
            dofIndex=0;
        if (dofIndex==0)
            object=object->getParentObject();
    }
    return(jacobian);
}
