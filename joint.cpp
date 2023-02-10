#include "joint.h"
#include "environment.h"

CJoint::CJoint()
{
    _objectType=ik_objecttype_joint;
    _dependencyJointCallback=nullptr;
}

CJoint::CJoint(int jointType)
{
    _objectType=ik_objecttype_joint;
    _dependencyJointCallback=nullptr;
    _jointMode=ik_jointmode_ik;
    _objectName="joint";
    _jointPosition=0.0;
    _screwLead=0.0;
    _sphericalTransformation.setIdentity();
    _ikWeight=1.0;
    _dependencyJointHandle=-1;
    _dependencyJointMult=1.0;
    _dependencyJointAdd=0.0;
    _jointType=jointType;
    if (jointType==ik_jointtype_revolute)
    {
        _positionIsCyclic=true;
        _jointPositionRange=piValT2;
        _jointMinPosition=-piValue;
        _maxStepSize=10.0*degToRad;
        _limitMargin=5.0*degToRad;
    }
    if (jointType==ik_jointtype_prismatic)
    {
        _positionIsCyclic=false;
        _jointPositionRange=1.0;
        _jointMinPosition=-0.5;
        _maxStepSize=0.1;
        _limitMargin=0.01;
    }
    if (jointType==ik_jointtype_spherical)
    {
        _positionIsCyclic=true;
        _jointPositionRange=piValue;
        _jointMinPosition=0.0;
        _maxStepSize=10.0*degToRad;
        _limitMargin=0.0;
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

double CJoint::getDependencyJointMult() const
{
    return(_dependencyJointMult);
}

double CJoint::getDependencyJointAdd() const
{
    return(_dependencyJointAdd);
}

void CJoint::updateSelfAsSlave()
{
    if (_dependencyJointHandle!=-1)
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(_dependencyJointHandle);
        it->updateSelfAsSlave();
    }
    else
        setPosition(getPosition());
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
            updateSelfAsSlave();
        }
        else
            setPosition(getPosition());
        CEnvironment::currentEnvironment->objectContainer->actualizeObjectInformation();
    }
}

void CJoint::setDependencyJointMult(double m)
{
    if (_jointType!=ik_jointtype_spherical)
    {
        _dependencyJointMult=m;
        if (_dependencyJointHandle!=-1)
            updateSelfAsSlave();
    }
}

void CJoint::setDependencyJointAdd(double off)
{
    if (_jointType!=ik_jointtype_spherical)
    {
        _dependencyJointAdd=off;
        if (_dependencyJointHandle!=-1)
            updateSelfAsSlave();
    }
}

void CJoint::setDependencyJointCallback(double(*cb)(int ikEnv,int slaveJoint,double masterPos))
{
    if (_jointType!=ik_jointtype_spherical)
    {
        _dependencyJointCallback=cb;
        if (_dependencyJointHandle!=-1)
            updateSelfAsSlave();
    }
}

int CJoint::getJointType() const
{
    return(_jointType);
}

double CJoint::getScrewLead() const
{
    return(_screwLead);
}

void CJoint::setScrewLead(double p)
{
    if (_jointType==ik_jointtype_revolute)
        _screwLead=p;
}

void CJoint::setSphericalTransformation(const C4Vector& tr)
{
        _sphericalTransformation=tr;
}

C4Vector CJoint::getSphericalTransformation() const
{
    return(_sphericalTransformation);
}

void CJoint::setMaxStepSize(double stepS)
{
    _maxStepSize=stepS;
}

double CJoint::getMaxStepSize() const
{
    return(_maxStepSize);
}

CSceneObject* CJoint::copyYourself() const
{
    CJoint* duplicate=(CJoint*)CSceneObject::copyYourself();

    duplicate->_jointType=_jointType;
    duplicate->_sphericalTransformation=_sphericalTransformation;
    duplicate->_positionIsCyclic=_positionIsCyclic;
    duplicate->_screwLead=_screwLead;
    duplicate->_jointMinPosition=_jointMinPosition;
    duplicate->_jointPositionRange=_jointPositionRange;
    duplicate->_jointPosition=_jointPosition;
    duplicate->_maxStepSize=_maxStepSize;
    duplicate->_ikWeight=_ikWeight;
    duplicate->_limitMargin=_limitMargin;
    duplicate->_jointMode=_jointMode;
    duplicate->_dependencyJointHandle=_dependencyJointHandle;
    duplicate->_dependencyJointMult=_dependencyJointMult;
    duplicate->_dependencyJointAdd=_dependencyJointAdd;

    return(duplicate);
}

double CJoint::getPosition() const
{
    return(_jointPosition);
}

double CJoint::getIkWeight() const
{
    return(_ikWeight);
}

void CJoint::setIkWeight(double newWeight)
{
    if (newWeight<0.00001)
        newWeight=0.00001;
    _ikWeight=newWeight;
}

double CJoint::getLimitMargin() const
{
    return(_limitMargin);
}

void CJoint::setLimitMargin(double newMargin)
{
    _limitMargin=newMargin;
}

void CJoint::setPosition(double parameter,const CJoint* masterJoint/*=nullptr*/)
{
    if (masterJoint!=nullptr)
    {
        if (_dependencyJointHandle==masterJoint->getObjectHandle())
        {
            if (_dependencyJointCallback==nullptr)
                _jointPosition=_dependencyJointAdd+_dependencyJointMult*masterJoint->getPosition();
            else
                _jointPosition=_dependencyJointCallback(CEnvironment::currentEnvironment->getHandle(),_objectHandle,masterJoint->getPosition());
            for (size_t i=0;i<dependentJoints.size();i++)
                dependentJoints[i]->setPosition(0.0,this);
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
                dependentJoints[i]->setPosition(0.0,this);
        }
    }

}

double CJoint::getPositionIntervalMin() const
{ 
    return(_jointMinPosition); 
}

void CJoint::setPositionIntervalMin(double m)
{
    _jointMinPosition=m;
    setSphericalTransformation(getSphericalTransformation());
    setPosition(getPosition());
}

double CJoint::getPositionIntervalRange() const
{ 
    return(_jointPositionRange); 
}

void CJoint::setPositionIntervalRange(double r)
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
            _screwLead=0.0;
            _jointMinPosition=-piValue;
            _jointPositionRange=piValT2;
            _positionIsCyclic=c;
        }
    }
}

bool CJoint::announceSceneObjectWillBeErased(int objectHandle)
{
    announceSceneObjectWillBeErasedMain(objectHandle);
    if (_dependencyJointHandle==objectHandle)
    {
        _dependencyJointHandle=-1;
        setPosition(getPosition()); // we need to take the joint limits into account again
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
    if (ar.isWriting())
    {
        ar.writeInt(_jointType);
        ar.writeFloat(float(_screwLead/piValT2));

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
        _screwLead=double(ar.readFloat())*piValT2;
        _sphericalTransformation(0)=double(ar.readFloat());
        _sphericalTransformation(1)=double(ar.readFloat());
        _sphericalTransformation(2)=double(ar.readFloat());
        _sphericalTransformation(3)=double(ar.readFloat());
        unsigned char dummy=ar.readByte();
        _positionIsCyclic=SIM_IS_BIT_SET(dummy,0);
        _jointMinPosition=double(ar.readFloat());
        _jointPositionRange=double(ar.readFloat());
        _jointPosition=double(ar.readFloat());
        _maxStepSize=double(ar.readFloat());
        _ikWeight=double(ar.readFloat());
        _jointMode=ar.readInt();
        _dependencyJointHandle=ar.readInt();
        _dependencyJointMult=double(ar.readFloat());
        _dependencyJointAdd=double(ar.readFloat());
    }
}
