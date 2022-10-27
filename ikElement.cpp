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
    duplicate->jointHandles.assign(jointHandles.begin(),jointHandles.end());
    duplicate->jointDofIndex.assign(jointDofIndex.begin(),jointDofIndex.end());
    duplicate->equationTypes.assign(equationTypes.begin(),equationTypes.end());
    duplicate->jacobian.set(jacobian);
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

void CikElement::getDistances(simReal& linDist,simReal& angDist) const
{ // returns the tip-target lin./ang. distances, taking into account the constraint settings
    linDist=simZero;
    angDist=simZero;
    CDummy* targetObject=CEnvironment::currentEnvironment->objectContainer->getDummy(getTargetHandle());
    if (targetObject!=nullptr)
    {
        C7Vector targetTr(targetObject->getCumulativeTransformationPart1());
        CDummy* tooltipObject=CEnvironment::currentEnvironment->objectContainer->getDummy(_tipHandle);
        C7Vector tooltipTr(tooltipObject->getCumulativeTransformationPart1());
        C7Vector baseTrInv(C7Vector::identityTransformation);
        CDummy* baseObject=CEnvironment::currentEnvironment->objectContainer->getDummy(_baseHandle);
        if (baseObject!=nullptr)
            baseTrInv=baseObject->getCumulativeTransformationPart1().getInverse();
        CDummy* altBaseObject=CEnvironment::currentEnvironment->objectContainer->getDummy(_altBaseHandleForConstraints);
        if (altBaseObject!=nullptr)
            baseTrInv=altBaseObject->getCumulativeTransformationPart1().getInverse();
        tooltipTr=baseTrInv*tooltipTr;
        targetTr=baseTrInv*targetTr;
        _getMatrixError(targetTr,tooltipTr,linDist,angDist);
    }
}

void CikElement::isWithinTolerance(bool& position,bool& orientation) const
{
    position=true;
    orientation=true;
    simReal linDist,angDist;
    getDistances(linDist,angDist);
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
    int tipBaseAltBase[3]={_tipHandle,_baseHandle,_altBaseHandleForConstraints};
    simReal weights[2]={_positionWeight,_orientationWeight};
    getJacobian(jacobian,errorVector,weights,tipBaseAltBase,_constraints,interpolationFactor,&equationTypes,&jointHandles,&jointDofIndex);
}

bool CikElement::getJacobian(CMatrix& jacob,CMatrix& errVect,const simReal weights[2],const int tipBaseAltBase[3],int constraints,simReal interpolationFactor,std::vector<int>* equTypes,std::vector<int>* jHandles,std::vector<int>* jDofIndex)
{ // equTypes, jHandles and jDofIndex can be nullptr
    if (jHandles!=nullptr)
        jHandles->clear();
    if (jDofIndex!=nullptr)
        jDofIndex->clear();
    if (equTypes!=nullptr)
        equTypes->clear();
    errVect.resize(0,1,0.0);
    CDummy* tip=CEnvironment::currentEnvironment->objectContainer->getDummy(tipBaseAltBase[0]);
    C7Vector tr(tip->getCumulativeTransformation());

    CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(tipBaseAltBase[1]);
    if ( (tip==nullptr)||((base!=nullptr)&&(!tip->isObjectAffiliatedWith(base))) )
        return(false);
    CSceneObject* constrBase=base;
    CSceneObject* altBase=CEnvironment::currentEnvironment->objectContainer->getObject(tipBaseAltBase[2]);
    if (altBase!=nullptr)
        constrBase=altBase;
    std::vector<simReal> mem;
    jacob=_getNakedJacobian(tip,base,constrBase,constraints,jHandles,jDofIndex);
    bool retVal=false;
    if ( (jacob.rows!=0)&&(jacob.cols!=0) )
    {
        retVal=true;
        CDummy* target=nullptr;
        if (tip->getLinkType()==ik_linktype_ik_tip_target)
            target=CEnvironment::currentEnvironment->objectContainer->getDummy(tip->getLinkedDummyHandle());
        if (target!=nullptr)
        {
            errVect.resize(jacob.rows,1,0.0);
            C7Vector constrBaseTrInverse;
            constrBaseTrInverse.setIdentity();
            if (constrBase!=nullptr)
                constrBaseTrInverse=constrBase->getCumulativeTransformation().getInverse();
            C7Vector tipTrRel(constrBaseTrInverse*tip->getCumulativeTransformationPart1());
            C7Vector targetTrRel(constrBaseTrInverse*target->getCumulativeTransformationPart1());
            if ((constraints&(ik_constraint_alpha_beta|ik_constraint_gamma))==ik_constraint_alpha_beta)
            { // We need to reorient the target around its z-axis to "ressemble" most the tip orientation
                C3Vector tipXaxisProj(targetTrRel.Q.getInverse()*tipTrRel.Q.getMatrix().axis[0]);
                simReal angle=tipXaxisProj.getAngle(C3Vector::unitXVector);
                if (fabs(angle)>0.001)
                {
                    if (tipXaxisProj(1)<0.0)
                        angle=-angle;
                    C4Vector q(angle,C3Vector::unitZVector);
                    targetTrRel.Q*=q;
                }
            }
            if ((constraints&(ik_constraint_alpha_beta|ik_constraint_gamma))==ik_constraint_gamma)
            { // We need to reorient the target with a rotation that brings both z-axes together
                C3Vector tipZaxis(tipTrRel.Q.getMatrix().axis[2]);
                C3Vector targetZaxis(targetTrRel.Q.getMatrix().axis[2]);
                simReal angle=targetZaxis.getAngle(tipZaxis);
                if (angle>0.001)
                {
                    C4Vector q(angle,(targetZaxis^tipZaxis).getNormalized());
                    targetTrRel.Q=q*targetTrRel.Q;
                }
            }
            C7Vector interpolTargetRel;
            simReal dq=0.01; // not that relevant apparently
            interpolTargetRel.buildInterpolation(tipTrRel,targetTrRel,interpolationFactor*dq);

            C4Vector dx_q(tipTrRel.Q.getInverse()*interpolTargetRel.Q);
            C4Vector aaxis(dx_q.getAngleAndAxis()); // angle and axis relative to tipTrRel
            C3Vector axisRel=tipTrRel.Q*C3Vector(aaxis(1),aaxis(2),aaxis(3)); // axis relative to constrBase
            C4Vector dx_q2(aaxis(0),axisRel);
            C3Vector euler(dx_q2.getEulerAngles());
            C3Vector dx_x(interpolTargetRel.X-tipTrRel.X);

            size_t rowIndex=0;
            if ((constraints&ik_constraint_x)!=0)
            {
                errVect(rowIndex++,0)=dx_x(0)*weights[0]/dq;
                if (equTypes!=nullptr)
                    equTypes->push_back(0);
            }
            if ((constraints&ik_constraint_y)!=0)
            {
                errVect(rowIndex++,0)=dx_x(1)*weights[0]/dq;
                if (equTypes!=nullptr)
                    equTypes->push_back(1);
            }
            if ((constraints&ik_constraint_z)!=0)
            {
                errVect(rowIndex++,0)=dx_x(2)*weights[0]/dq;
                if (equTypes!=nullptr)
                    equTypes->push_back(2);
            }
            if ((constraints&(ik_constraint_alpha_beta|ik_constraint_gamma))!=0)
            {
                if ((constraints&ik_constraint_alpha_beta)!=0)
                {
                    errVect(rowIndex++,0)=euler(0)*weights[1]/dq;
                    errVect(rowIndex++,0)=euler(1)*weights[1]/dq;
                    if (equTypes!=nullptr)
                    {
                        equTypes->push_back(3);
                        equTypes->push_back(4);
                    }
                }
                if ((constraints&ik_constraint_gamma)!=0)
                {
                    errVect(rowIndex++,0)=euler(2)*weights[1]/dq;
                    if (equTypes!=nullptr)
                        equTypes->push_back(5);
                }
            }
        }
    }

    return(retVal);
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

CMatrix CikElement::_getNakedJacobian(const CSceneObject* tip,const CSceneObject* base,const CSceneObject* constrBase,int constraints,std::vector<int>* jHandles,std::vector<int>* jDofIndex)
{ // jHandles and jDofIndex can be nullptr
    size_t rows=0;
    if ((constraints&ik_constraint_x)!=0)
        rows++;
    if ((constraints&ik_constraint_y)!=0)
        rows++;
    if ((constraints&ik_constraint_z)!=0)
        rows++;
    if ((constraints&ik_constraint_alpha_beta)!=0)
        rows+=2;
    if ((constraints&ik_constraint_gamma)!=0)
        rows++;

    std::vector<CJoint*> joints;
    CSceneObject* object=tip->getParentObject();
    while (object!=base)
    {
        if (object->getObjectType()==ik_objecttype_joint)
        {
            CJoint* joint=(CJoint*)object;
            if ( (joint->getJointMode()==ik_jointmode_ik)||(joint->getJointMode()==ik_jointmode_reserved_previously_ikdependent)||(joint->getJointMode()==ik_jointmode_dependent) )
                joints.insert(joints.begin(),joint);
        }
        object=object->getParentObject();
    }

    CMatrix jacobian(rows,0);
    C7Vector constrBaseTrInverse;
    constrBaseTrInverse.setIdentity();
    if (constrBase!=nullptr)
        constrBaseTrInverse=constrBase->getCumulativeTransformation().getInverse();
    C7Vector tipRelConstrBase(constrBaseTrInverse*tip->getCumulativeTransformationPart1());

    int dofIndex=0;
    size_t jointIndex=0;
    while (jointIndex<joints.size())
    {
        CJoint* joint=joints[jointIndex];
        if (jHandles!=nullptr)
            jHandles->push_back(joint->getObjectHandle());
        if (jDofIndex!=nullptr)
            jDofIndex->push_back(dofIndex);
        int colIndex=int(jacobian.cols);
        jacobian.resize(rows,jacobian.cols+1,0.0);
        C7Vector jbabs(joint->getCumulativeTransformation());
        C7Vector jb(constrBaseTrInverse*jbabs);
        C7Vector x(jbabs.getInverse()*tip->getCumulativeTransformationPart1());
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

        C4Vector dx_q(tipRelConstrBase.Q.getInverse()*tipRelConstrBase_changed.Q);
        C4Vector aaxis(dx_q.getAngleAndAxis()); // angle and axis relative to tipRelConstrBase
        C3Vector axisRel=tipRelConstrBase.Q*C3Vector(aaxis(1),aaxis(2),aaxis(3)); // axis relative to constrBase
        C4Vector dx_q2(aaxis(0),axisRel);
        C3Vector euler(dx_q2.getEulerAngles());
        C3Vector dx_x(tipRelConstrBase_changed.X-tipRelConstrBase.X);

        size_t rowIndex=0;
        if ((constraints&ik_constraint_x)!=0)
            jacobian(rowIndex++,colIndex)=dx_x(0)/dq;
        if ((constraints&ik_constraint_y)!=0)
            jacobian(rowIndex++,colIndex)=dx_x(1)/dq;
        if ((constraints&ik_constraint_z)!=0)
            jacobian(rowIndex++,colIndex)=dx_x(2)/dq;
        if ((constraints&(ik_constraint_alpha_beta|ik_constraint_gamma))!=0)
        {
            if ((constraints&ik_constraint_alpha_beta)!=0)
            {
                jacobian(rowIndex++,colIndex)=euler(0)/dq;
                jacobian(rowIndex++,colIndex)=euler(1)/dq;
            }
            if ((constraints&ik_constraint_gamma)!=0)
                jacobian(rowIndex++,colIndex)=euler(2)/dq;
        }

        if (dofIndex==3)
            dofIndex=0;
        if (dofIndex==0)
            jointIndex++;
    }
    return(jacobian);
}
