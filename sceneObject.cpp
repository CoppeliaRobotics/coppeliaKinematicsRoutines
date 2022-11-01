#include "sceneObject.h"
#include "environment.h"

CSceneObject::CSceneObject()
{
    _objectHandle=0;
    _parentObject=nullptr;
    _parentObjectHandle=-1;
    _transformation.setIdentity();
}

CSceneObject::~CSceneObject()
{
}

void CSceneObject::performSceneObjectLoadingMapping(const std::vector<int>* map)
{
}

void CSceneObject::performSceneObjectLoadingMappingMain(const std::vector<int>* map)
{
    int newParentHandle=_getLoadingMapping(map,_parentObjectHandle);
    setParentObject(CEnvironment::currentEnvironment->objectContainer->getObject(newParentHandle));
}

std::string CSceneObject::getObjectName() const
{
    return(_objectName);
}

void CSceneObject::setObjectName(std::string newName)
{ 
    _objectName=newName;
}

bool CSceneObject::announceSceneObjectWillBeErased(int objectHandle)
{
    return(false);
}

void CSceneObject::announceIkGroupWillBeErased(int ikGroupHandle)
{
}

void CSceneObject::announceSceneObjectWillBeErasedMain(int objectHandle)
{ 
    if (getParentObject()!=nullptr)
    {
        if (getParentObject()->getObjectHandle()==objectHandle)
            CEnvironment::currentEnvironment->objectContainer->makeObjectChildOf(this,getParentObject()->getParentObject());
    }
}

void CSceneObject::announceIkGroupWillBeErasedMain(int ikGroupHandle)
{
}

CSceneObject* CSceneObject::copyYourself() const
{
    CSceneObject* duplicate=nullptr;
    if (_objectType==ik_objecttype_dummy)
        duplicate=new CDummy();
    if (_objectType==ik_objecttype_joint)
        duplicate=new CJoint();

    duplicate->_objectHandle=_objectHandle;
    duplicate->_objectName=_objectName;
    duplicate->_transformation=_transformation;
    duplicate->_objectType=_objectType;
    duplicate->_parentObjectHandle=_parentObjectHandle;

    duplicate->_parentObject=nullptr;

    return(duplicate);
}

C7Vector CSceneObject::getParentCumulativeTransformation() const
{
    if (getParentObject()==nullptr)
        return(C7Vector::identityTransformation);
    else
        return(getParentObject()->getCumulativeTransformation());
}

C7Vector CSceneObject::getCumulativeTransformation() const
{
    if (getParentObject()==nullptr)
        return(getLocalTransformation());
    else
        return(getParentCumulativeTransformation()*getLocalTransformation());
}

C7Vector CSceneObject::getCumulativeTransformationPart1() const
{
    if (getObjectType()==ik_objecttype_joint)
    {
        if (getParentObject()==nullptr)
            return(getLocalTransformationPart1());
        else
            return(getParentCumulativeTransformation()*getLocalTransformationPart1());
    }
    else
        return(getCumulativeTransformation());
}

C7Vector CSceneObject::getLocalTransformation() const
{
    C7Vector retVal=_transformation;
    if (getObjectType()==ik_objecttype_joint)
    {
        const CJoint* it=dynamic_cast<const CJoint*>(this);
        C7Vector jointTr;
        jointTr.setIdentity();
        double val=it->getPosition();
        if (it->getJointType()==ik_jointtype_revolute)
        {
            jointTr.Q.setAngleAndAxis(val,C3Vector(0.0,0.0,1.0));
            jointTr.X(2)=val*it->getScrewPitch();
        }
        if (it->getJointType()==ik_jointtype_prismatic)
            jointTr.X(2)=val;
        if (it->getJointType()==ik_jointtype_spherical)
            jointTr.Q=it->getSphericalTransformation();
        retVal=_transformation*jointTr;
    }
    return(retVal);
}

C7Vector CSceneObject::getLocalTransformationPart1() const
{
    return(_transformation);
}

void CSceneObject::setLocalTransformation(const C7Vector& v)
{
    _transformation=v;
}

void CSceneObject::setLocalTransformation(const C4Vector& q)
{
    _transformation.Q=q;
}

void CSceneObject::setLocalTransformation(const C3Vector& x)
{
    _transformation.X=x;
}

int CSceneObject::getObjectHandle() const
{
    return(_objectHandle);
}

void CSceneObject::setObjectHandle(int newHandle)
{
    _objectHandle=newHandle;
}

bool CSceneObject::isObjectAffiliatedWith(const CSceneObject* theObject) const
{
    bool retVal=false;
    if (getParentObject()!=nullptr)
    {
        if (getParentObject()==theObject)
            retVal=true;
        else
            retVal=getParentObject()->isObjectAffiliatedWith(theObject);
    }
    return(retVal);
}

void CSceneObject::setParentObject(CSceneObject* newParentObject,bool updateContainer/*=true*/)
{ // also used during fast environment duplicate operation
    if (newParentObject!=this)
    {
        _parentObject=newParentObject;
        if (updateContainer)
            CEnvironment::currentEnvironment->objectContainer->actualizeObjectInformation();
    }
}

CSceneObject* CSceneObject::getParentObject() const
{
    return(_parentObject);
}

int CSceneObject::getParentObjectHandle() const
{
    return(_parentObjectHandle);
}

int CSceneObject::getObjectType() const
{
    return(_objectType);
}

void CSceneObject::serialize(CSerialization& ar)
{
}

void CSceneObject::serializeMain(CSerialization& ar)
{
    if (ar.isWriting())
    {
        C7Vector tr=getLocalTransformationPart1();
        ar.writeFloat(float(tr.Q(0)));
        ar.writeFloat(float(tr.Q(1)));
        ar.writeFloat(float(tr.Q(2)));
        ar.writeFloat(float(tr.Q(3)));
        ar.writeFloat(float(tr.X(0)));
        ar.writeFloat(float(tr.X(1)));
        ar.writeFloat(float(tr.X(2)));

        int parentID=-1;
        if (getParentObject()!=nullptr)
            parentID=getParentObject()->getObjectHandle();
        ar.writeInt(_objectHandle);
        ar.writeInt(parentID);
        ar.writeString(_objectName.c_str());
    }
    else
    {
        _transformation.Q(0)=double(ar.readFloat());
        _transformation.Q(1)=double(ar.readFloat());
        _transformation.Q(2)=double(ar.readFloat());
        _transformation.Q(3)=double(ar.readFloat());
        _transformation.X(0)=double(ar.readFloat());
        _transformation.X(1)=double(ar.readFloat());
        _transformation.X(2)=double(ar.readFloat());
        _objectHandle=ar.readInt();
        _parentObjectHandle=ar.readInt();
        _objectName=ar.readString().c_str();
    }
}
