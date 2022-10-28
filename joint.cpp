#include "joint.h"
#include "environment.h"

CJoint::CJoint()
{
    _objectType=ik_objecttype_joint;
}

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
    if (theMode!=_jointMode)
        _jointMode=theMode;
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
    if (_dependencyJointHandle!=jointHandle)
    {
        _dependencyJointHandle=jointHandle;
        if (jointHandle!=-1)
        {
            // Illegal loop check:
            CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(_dependencyJointHandle);
            CJoint* iterat=it;
            while (iterat->getDependencyJointHandle()!=-1)
            {
                int joint=iterat->getDependencyJointHandle();
                if (joint==_objectHandle)
                {
                    iterat->setDependencyJointHandle(-1);
                    break;
                }
                iterat=CEnvironment::currentEnvironment->objectContainer->getJoint(joint);
            }
            setPosition(getPosition());
        }
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
        _screwPitch=p;
}

void CJoint::setSphericalTransformation(const C4Vector& tr)
{
        _sphericalTransformation=tr;
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

CSceneObject* CJoint::copyYourself() const
{
    CJoint* duplicate=(CJoint*)CSceneObject::copyYourself();

    duplicate->_jointType=_jointType;
    duplicate->_sphericalTransformation=_sphericalTransformation;
    duplicate->_positionIsCyclic=_positionIsCyclic;
    duplicate->_screwPitch=_screwPitch;
    duplicate->_jointMinPosition=_jointMinPosition;
    duplicate->_jointPositionRange=_jointPositionRange;
    duplicate->_jointPosition=_jointPosition;
    duplicate->_maxStepSize=_maxStepSize;
    duplicate->_ikWeight=_ikWeight;
    duplicate->_jointMode=_jointMode;
    duplicate->_dependencyJointHandle=_dependencyJointHandle;
    duplicate->_dependencyJointMult=_dependencyJointMult;
    duplicate->_dependencyJointAdd=_dependencyJointAdd;

    return(duplicate);
}

simReal CJoint::getPosition() const
{
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

void CJoint::setPosition(simReal parameter,const CJoint* masterJoint/*=nullptr*/)
{
    if (masterJoint!=nullptr)
    {
        if (_dependencyJointHandle==masterJoint->getObjectHandle())
        {
            _jointPosition=_dependencyJointAdd+_dependencyJointMult*masterJoint->getPosition();
            for (size_t i=0;i<dependentJoints.size();i++)
                dependentJoints[i]->setPosition(simZero,this);
        }
    }
    else
    {
        if (_dependencyJointHandle==-1)
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
            _jointPosition=parameter;

            for (size_t i=0;i<dependentJoints.size();i++)
                dependentJoints[i]->setPosition(simZero,this);
        }
    }

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

bool CJoint::announceSceneObjectWillBeErased(int objectHandle)
{
    announceSceneObjectWillBeErasedMain(objectHandle);
    if (_dependencyJointHandle==objectHandle)
        _dependencyJointHandle=-1;
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
    if (ar.isWriting())
    {
        ar.writeInt(_jointType);
        ar.writeFloat(float(_screwPitch));

        ar.writeFloat(float(_sphericalTransformation(0)));
        ar.writeFloat(float(_sphericalTransformation(1)));
        ar.writeFloat(float(_sphericalTransformation(2)));
        ar.writeFloat(float(_sphericalTransformation(3)));

        unsigned char dummy=0;
        SIM_SET_CLEAR_BIT(dummy,0,_positionIsCyclic);
        ar.writeByte(dummy);

        ar.writeFloat(float(_jointMinPosition));
        ar.writeFloat(float(_jointPositionRange));
        ar.writeFloat(float(_jointPosition));
        ar.writeFloat(float(_maxStepSize));
        ar.writeFloat(float(_ikWeight));
        ar.writeInt(_jointMode);
        ar.writeInt(_dependencyJointHandle);
        ar.writeFloat(float(_dependencyJointMult));
        ar.writeFloat(float(_dependencyJointAdd));
    }
    else
    {
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
}
