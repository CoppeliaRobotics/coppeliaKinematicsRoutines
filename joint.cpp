#include "joint.h"
#include "environment.h"

CJoint::CJoint(int jointType)
{
    _objectType=ik_objecttype_joint;
    _jointMode=ik_jointmode_ik;
    _objectName="joint";
    _jointPosition=simZero;
    _screwPitch=simZero;
    _sphericalTransformation.setIdentity();
    _ikWeight=simOne;
    _dependencyJointHandle=-1;
    _dependencyJointMult=simOne;
    _dependencyJointAdd=simZero;
    _jointType=jointType;
    if (jointType==ik_jointtype_revolute)
    {
        _positionIsCyclic=true;
        _jointPositionRange=piValTimes2;
        _jointMinPosition=-piValue;
        _maxStepSize=simReal(10.0)*degToRad;
    }
    if (jointType==ik_jointtype_prismatic)
    {
        _positionIsCyclic=false;
        _jointPositionRange=simOne;
        _jointMinPosition=simReal(-0.5);
        _maxStepSize=simReal(0.1);
    }
    if (jointType==ik_jointtype_spherical)
    {
        _positionIsCyclic=true;
        _jointPositionRange=piValue;
        _jointMinPosition=simZero;
        _maxStepSize=simReal(10.0)*degToRad;
    }
}

CJoint::~CJoint()
{

}

void CJoint::performSceneObjectLoadingMapping(const std::vector<int>* map)
{
    performSceneObjectLoadingMappingMain(map);
    _dependencyJointHandle=_getLoadingMapping(map,_dependencyJointHandle);
}

void CJoint::setJointMode(int theMode)
{
    _jointMode=theMode;
    if ( (theMode!=ik_jointmode_dependent)&&(theMode!=ik_jointmode_reserved_previously_ikdependent) )
    {
        bool d=(_dependencyJointHandle!=-1);
        _dependencyJointHandle=-1;
        if (d)
            CEnvironment::currentEnvironment->objectContainer->actualizeObjectInformation();
    }
    setPosition(getPosition());
    setSphericalTransformation(getSphericalTransformation());
}

void CJoint::_rectifyDependentJoints()
{
    for (size_t i=0;i<dependentJoints.size();i++)
    {
        if (dependentJoints[i]->getJointMode()==ik_jointmode_dependent)
            dependentJoints[i]->setPosition(simZero,false);
    }
}

int CJoint::getJointMode() const
{
    return(_jointMode);
}

int CJoint::getDependencyJointHandle() const
{
    return(_dependencyJointHandle);
}

simReal CJoint::getDependencyJointMult() const
{
    return(_dependencyJointMult);
}

simReal CJoint::getDependencyJointAdd() const
{
    return(_dependencyJointAdd);
}

void CJoint::setDependencyJointHandle(int jointHandle)
{
    _dependencyJointHandle=jointHandle;
    if (jointHandle!=-1)
    {
        // Illegal loop check:
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        CJoint* iterat=it;
        while (iterat->getDependencyJointHandle()!=-1)
        {
            if (iterat->getJointMode()!=_jointMode)
                break;
            int joint=iterat->getDependencyJointHandle();
            if (joint==getObjectHandle())
            {
                iterat->setDependencyJointHandle(-1);
                break;
            }
            iterat=CEnvironment::currentEnvironment->objectContainer->getJoint(joint);
        }
        CEnvironment::currentEnvironment->objectContainer->actualizeObjectInformation();
        setPosition(getPosition());
    }
    else
    {
        _dependencyJointHandle=-1;
        CEnvironment::currentEnvironment->objectContainer->actualizeObjectInformation();
    }
}

void CJoint::setDependencyJointMult(simReal m)
{
    if (_jointType!=ik_jointtype_spherical)
    {
        _dependencyJointMult=m;
        setPosition(getPosition());
    }
}

void CJoint::setDependencyJointAdd(simReal off)
{
    if (_jointType!=ik_jointtype_spherical)
    {
        _dependencyJointAdd=off;
        setPosition(getPosition());
    }
}

int CJoint::getJointType() const
{
    return(_jointType);
}

simReal CJoint::getScrewPitch() const
{
    return(_screwPitch);
}

void CJoint::setScrewPitch(simReal p)
{
    if (_jointType==ik_jointtype_revolute)
    {
        if (_jointMode!=ik_jointmode_force)
            _screwPitch=p;
    }
}

void CJoint::setSphericalTransformation(const C4Vector& tr)
{
    C4Vector transf(tr);
    if (_jointPositionRange<piValue*simReal(0.99))
    {
        C3X3Matrix theTr(transf);
        C3Vector zReset(simZero,simZero,simOne);
        simReal angle=zReset.getAngle(theTr.axis[2]);
        if (angle>_jointPositionRange)
        {
            C3Vector rotAxis((theTr.axis[2]^zReset).getNormalized());
            C4Vector rot(angle-_jointPositionRange,rotAxis);
            transf=rot*transf;
        }
    }
    _sphericalTransformation=transf;
}

C4Vector CJoint::getSphericalTransformation() const
{
    return(_sphericalTransformation);
}

void CJoint::setMaxStepSize(simReal stepS)
{
    _maxStepSize=stepS;
}

simReal CJoint::getMaxStepSize() const
{
    return(_maxStepSize);
}

simReal CJoint::getPosition(bool tempVals) const
{
    if (tempVals)
        return(_jointPosition_tempForIK); 
    return(_jointPosition);
}

simReal CJoint::getIkWeight() const
{
    return(_ikWeight);
}

void CJoint::setIkWeight(simReal newWeight)
{
    _ikWeight=newWeight;
}

void CJoint::setPosition(simReal parameter,bool tempVals)
{
    if (_positionIsCyclic)
        parameter=atan2(sin(parameter),cos(parameter));
    else
    {
        if (parameter>(getPositionIntervalMin()+getPositionIntervalRange()))
            parameter=getPositionIntervalMin()+getPositionIntervalRange();
        if (parameter<getPositionIntervalMin())
            parameter=getPositionIntervalMin();
    }
    if (tempVals)
        _jointPosition_tempForIK=parameter;
    else
        _jointPosition=parameter;

    if (_jointMode==ik_jointmode_dependent)
    {
        simReal linked=simZero;
        if (_dependencyJointHandle!=-1)
        {
            CJoint* anAct=CEnvironment::currentEnvironment->objectContainer->getJoint(_dependencyJointHandle);
            if (anAct!=nullptr)
                linked=_dependencyJointMult*anAct->getPosition(tempVals);
        }
        if (tempVals)
            _jointPosition_tempForIK=linked+_dependencyJointAdd;
        else
            _jointPosition=linked+_dependencyJointAdd;
    }
    _rectifyDependentJoints();
}

simReal CJoint::getPositionIntervalMin() const
{ 
    return(_jointMinPosition); 
}

void CJoint::setPositionIntervalMin(simReal m)
{
    _jointMinPosition=m;
    setSphericalTransformation(getSphericalTransformation());
    setPosition(getPosition());
}

simReal CJoint::getPositionIntervalRange() const
{ 
    return(_jointPositionRange); 
}

void CJoint::setPositionIntervalRange(simReal r)
{
    _jointPositionRange=r;
    setSphericalTransformation(getSphericalTransformation());
    setPosition(getPosition());
}

bool CJoint::getPositionIsCyclic() const
{
    if (_jointType==ik_jointtype_prismatic)
        return(false);
    return(_positionIsCyclic);
}

void CJoint::setPositionIsCyclic(bool c)
{
    if (!c)
        _positionIsCyclic=c;
    else
    {
        if (getJointType()==ik_jointtype_revolute)
        {
            _screwPitch=simZero;
            _jointMinPosition=-piValue;
            _jointPositionRange=piValTimes2;
            _positionIsCyclic=c;
        }
    }
}

void CJoint::initializeParametersForIK(simReal angularJointLimitationThreshold)
{
    if (_jointType!=ik_jointtype_spherical)
        _jointPosition_tempForIK=_jointPosition;
    else
    {
        // 1. Do we need to prepare the thing for the joint limitation?
        _sphericalTransformation_eulerLockTempForIK=0;
        C3X3Matrix m(_sphericalTransformation);
        simReal angle=C3Vector::unitZVector.getAngle(m.axis[2]);
        if ( (_jointPositionRange<simReal(179.9)*degToRad)&&(angle>simOne*degToRad) )
        {
            if ((_jointPositionRange-angularJointLimitationThreshold)/simTwo<angle)
            { // We have to activate the second type of spherical joint (with joint limitation (IK pass dependent))
                _sphericalTransformation_eulerLockTempForIK=2;
                C3Vector n(m.axis[2]);
                n(2)=simZero;
                n.normalize();
                C3Vector y((C3Vector::unitZVector^n).getNormalized());
                simReal angle2=C3Vector::unitXVector.getAngle(y);
                C3Vector zz(C3Vector::unitXVector^y);
                if (zz(2)<simZero)
                    angle2=-angle2;
                _jointPosition_tempForIK=simZero; // Not really needed!
                _sphericalTransformation_euler1TempForIK=angle2;
                _sphericalTransformation_euler2TempForIK=angle;
                simReal angle3=m.axis[0].getAngle(y);
                C3Vector nz(y^m.axis[0]);
                if (nz*m.axis[2]<simZero)
                    angle3=-angle3;
                _sphericalTransformation_euler3TempForIK=angle3;
            }
        }
        if (_sphericalTransformation_eulerLockTempForIK==0)
        { // No joint limitations for the IK (in this IK pass)
            _jointPosition_tempForIK=simZero; // Not really needed!
            _sphericalTransformation_euler1TempForIK=simZero;
            _sphericalTransformation_euler2TempForIK=simZero;
            _sphericalTransformation_euler3TempForIK=simZero;
        }
    }
}

size_t CJoint::getDoFs() const
{
    if (_jointType!=ik_jointtype_spherical)
        return(1);
    return(3);
}

int CJoint::getTempSphericalJointLimitations() const
{
    return(_sphericalTransformation_eulerLockTempForIK);
}

void CJoint::getLocalTransformationExPart1(C7Vector& mTr,size_t index) const
{ // Used for Jacobian calculation with spherical joints
    if (_sphericalTransformation_eulerLockTempForIK==0)
    { // Spherical joint limitations are not activated in the IK algorithm (but if we come close to the limit, it might get activated in next pass!)
        if (index==0)
        { 
            mTr.setIdentity();
            mTr.Q.setEulerAngles(simZero,piValD2,simZero);
            C7Vector tr2(getLocalTransformation());
            mTr=tr2*mTr;
        }
        if (index==1)
        {
            mTr.setIdentity();
            mTr.Q.setEulerAngles(-piValD2,simZero,-piValD2);
        }
        if (index==2)
        {
            mTr.setIdentity();
            mTr.Q.setEulerAngles(piValD2,simZero,simZero);
        }
    }
    else
    {
        if (index==0)
        {
            mTr=getLocalTransformationPart1();
        }
        if (index==1)
        {
            mTr.setIdentity();
            mTr.Q.setEulerAngles(simZero,piValD2,simZero);
        }
        if (index==2)
        {
            mTr.setIdentity();
            mTr.Q.setEulerAngles(simZero,-piValD2,simZero);
        }
    }
}

simReal CJoint::getTempParameterEx(size_t index) const
{
    if (index==0)
        return(_sphericalTransformation_euler1TempForIK);
    if (index==1)
        return(_sphericalTransformation_euler2TempForIK);
    if (index==2)
        return(_sphericalTransformation_euler3TempForIK);
    return(simZero);
}

void CJoint::setTempParameterEx(simReal parameter,size_t index)
{
    if (index==0)
        _sphericalTransformation_euler1TempForIK=parameter;
    if (index==1)
        _sphericalTransformation_euler2TempForIK=parameter;
    if (index==2)
        _sphericalTransformation_euler3TempForIK=parameter;

    if (_sphericalTransformation_eulerLockTempForIK==0)
    { // Spherical joint limitations are not activated in the IK algorithm (but if we come close to the limit, it might get activated in next pass!)
        C4Vector saved(_sphericalTransformation);
        applyTempParametersEx();
        C4Vector tr(saved.getInverse()*_sphericalTransformation);
        C3Vector euler(tr.getEulerAngles());
        _sphericalTransformation_euler1TempForIK=euler(0);
        _sphericalTransformation_euler2TempForIK=euler(1);
        _sphericalTransformation_euler3TempForIK=euler(2);
        _sphericalTransformation=saved;
    }
    else
    { // Spherical joint limitations are activated in the IK algorithm
        C4Vector saved(_sphericalTransformation);
        applyTempParametersEx();

        C3X3Matrix m(_sphericalTransformation);

        simReal angle=C3Vector::unitZVector.getAngle(m.axis[2]);
        if (angle>simReal(0.01)*degToRad)
        {
            C3Vector n(m.axis[2]);
            n(2)=simZero;
            n.normalize();
            C3Vector y((C3Vector::unitZVector^n).getNormalized());
            simReal angle2=C3Vector::unitXVector.getAngle(y);
            C3Vector zz(C3Vector::unitXVector^y);
            if (zz(2)<simZero)
                angle2=-angle2;
            _sphericalTransformation_euler1TempForIK=angle2;
            _sphericalTransformation_euler2TempForIK=angle;
            simReal angle3=m.axis[0].getAngle(y);
            C3Vector nz(y^m.axis[0]);
            if (nz*m.axis[2]<simZero)
                angle3=-angle3;
            _sphericalTransformation_euler3TempForIK=angle3;
        }
        else
        { // This is a rare case and should never happen if the spherical joint limitation is not too small!
            simReal angle=C3Vector::unitXVector.getAngle(m.axis[0]);
            if ((C3Vector::unitXVector^m.axis[0])(2)<simZero)
                angle=-angle;
            _sphericalTransformation_euler1TempForIK=angle;
            _sphericalTransformation_euler2TempForIK=simZero;
            _sphericalTransformation_euler3TempForIK=simZero;
        }
        _sphericalTransformation=saved;
    }
}

void CJoint::applyTempParametersEx()
{
    if (_jointType==ik_jointtype_spherical)
    {
        C7Vector tr1(getLocalTransformationPart1(true));
        C7Vector tr2(getLocalTransformation(true));
        setSphericalTransformation(tr1.Q.getInverse()*tr2.Q);
    }
}

bool CJoint::announceSceneObjectWillBeErased(int objectHandle)
{
    announceSceneObjectWillBeErasedMain(objectHandle);
    if (_dependencyJointHandle==objectHandle)
    {
        _dependencyJointHandle=-1;
    }
    for (size_t i=0;i<dependentJoints.size();i++)
    {
        if (dependentJoints[i]->getObjectHandle()==objectHandle)
            dependentJoints.erase(dependentJoints.begin()+i);
    }
    return(false);
}

void CJoint::announceIkGroupWillBeErased(int ikGroupHandle)
{
    announceIkGroupWillBeErasedMain(ikGroupHandle);
}

void CJoint::serialize(CSerialization& ar)
{
    serializeMain(ar);
    _jointType=ar.readInt();
    _screwPitch=simReal(ar.readFloat());
    _sphericalTransformation(0)=simReal(ar.readFloat());
    _sphericalTransformation(1)=simReal(ar.readFloat());
    _sphericalTransformation(2)=simReal(ar.readFloat());
    _sphericalTransformation(3)=simReal(ar.readFloat());
    unsigned char dummy=ar.readByte();
    _positionIsCyclic=SIM_IS_BIT_SET(dummy,0);
    _jointMinPosition=simReal(ar.readFloat());
    _jointPositionRange=simReal(ar.readFloat());
    _jointPosition=simReal(ar.readFloat());
    _maxStepSize=simReal(ar.readFloat());
    _ikWeight=simReal(ar.readFloat());
    _jointMode=ar.readInt();
    _dependencyJointHandle=ar.readInt();
    _dependencyJointMult=simReal(ar.readFloat());
    _dependencyJointAdd=simReal(ar.readFloat());
}
