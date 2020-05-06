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

C7Vector CSceneObject::getParentCumulativeTransformation(bool tempVals) const
{
    if (getParentObject()==nullptr)
        return(C7Vector::identityTransformation);
    else
        return(getParentObject()->getCumulativeTransformation(tempVals));
}

C7Vector CSceneObject::getCumulativeTransformation(bool tempVals) const
{
    if (getParentObject()==nullptr)
        return(getLocalTransformation(tempVals));
    else
        return(getParentCumulativeTransformation(tempVals)*getLocalTransformation(tempVals));
}

C7Vector CSceneObject::getCumulativeTransformationPart1(bool tempVals) const
{
    if (getObjectType()==ik_objecttype_joint)
    {
        if (getParentObject()==nullptr)
            return(getLocalTransformationPart1(tempVals));
        else
            return(getParentCumulativeTransformation(tempVals)*getLocalTransformationPart1(tempVals));
    }
    else
        return(getCumulativeTransformation(tempVals));
}

C7Vector CSceneObject::getLocalTransformation(bool tempVals) const
{
    if (getObjectType()==ik_objecttype_joint)
    {
        const CJoint* it=dynamic_cast<const CJoint*>(this);
        C7Vector jointTr;
        jointTr.setIdentity();
        simReal val;
        val=it->getPosition(tempVals);
        if (it->getJointType()==ik_jointtype_revolute)
        {
            jointTr.Q.setAngleAndAxis(val,C3Vector(simZero,simZero,simOne));
            jointTr.X(2)=val*it->getScrewPitch();
        }
        if (it->getJointType()==ik_jointtype_prismatic)
            jointTr.X(2)=val;
        if (it->getJointType()==ik_jointtype_spherical)
        {
            if (tempVals)
            {
                if (it->getTempSphericalJointLimitations()==0)
                { // Used by the IK routine when away from joint limitations
                    jointTr.Q.setEulerAngles(simZero,simZero,it->getTempParameterEx(2));
                    C4Vector q2;
                    q2.setEulerAngles(piValD2,simZero,simZero);
                    jointTr.Q=q2*jointTr.Q;

                    q2.setEulerAngles(simZero,simZero,it->getTempParameterEx(1));
                    jointTr.Q=q2*jointTr.Q;
                    q2.setEulerAngles(-piValD2,simZero,-piValD2);
                    jointTr.Q=q2*jointTr.Q;

                    q2.setEulerAngles(simZero,simZero,it->getTempParameterEx(0));
                    jointTr.Q=q2*jointTr.Q;
                    q2.setEulerAngles(simZero,piValD2,simZero);
                    jointTr.Q=q2*jointTr.Q;
                    q2=it->getSphericalTransformation();
                    jointTr.Q=q2*jointTr.Q;
                }
                else
                { // Used by the IK routine when close to joint limitations
                    jointTr.Q.setEulerAngles(simZero,simZero,it->getTempParameterEx(2));
                    C4Vector q2;
                    q2.setEulerAngles(simZero,-piValD2,simZero);
                    jointTr.Q=q2*jointTr.Q;

                    q2.setEulerAngles(simZero,simZero,it->getTempParameterEx(1));
                    jointTr.Q=q2*jointTr.Q;
                    q2.setEulerAngles(simZero,piValD2,simZero);
                    jointTr.Q=q2*jointTr.Q;

                    q2.setEulerAngles(simZero,simZero,it->getTempParameterEx(0));
                    jointTr.Q=q2*jointTr.Q;
                }
            }
            else
                jointTr.Q=it->getSphericalTransformation();
        }
        return(_transformation*jointTr);
    }
    else
        return(_transformation);
}

C7Vector CSceneObject::getLocalTransformationPart1(bool tempVals) const
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

void CSceneObject::setParentObject(CSceneObject* newParentObject)
{
    if (newParentObject!=this)
    {
        _parentObject=newParentObject;
        CEnvironment::currentEnvironment->objectContainer->actualizeObjectInformation();
    }
}

CSceneObject* CSceneObject::getParentObject() const
{
    return(_parentObject);
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
    _transformation.Q(0)=simReal(ar.readFloat());
    _transformation.Q(1)=simReal(ar.readFloat());
    _transformation.Q(2)=simReal(ar.readFloat());
    _transformation.Q(3)=simReal(ar.readFloat());
    _transformation.X(0)=simReal(ar.readFloat());
    _transformation.X(1)=simReal(ar.readFloat());
    _transformation.X(2)=simReal(ar.readFloat());
    _objectHandle=ar.readInt();
    _parentObjectHandle=ar.readInt();
    _objectName=ar.readString().c_str();
}
