#include "objectContainer.h"
#include "environment.h"

CObjectContainer::CObjectContainer()
{
    _nextObjectHandle=0;
    newSceneProcedure();
}

CObjectContainer::~CObjectContainer()
{
    removeAllObjects();
}

void CObjectContainer::newSceneProcedure()
{
    removeAllObjects();
}

void CObjectContainer::removeAllObjects()
{

    while (objectList.size()!=0)
    {
        CSceneObject* it=getObject(objectList[0]);
        if (it!=nullptr)
            eraseObject(it);
    }
    // The above loop destroys all 3DObjects, and normally automatically
    // all resources linked to them. So we don't have to destroy the resources

    objectList.clear();

    _objectIndex.clear();


    jointList.clear();
    dummyList.clear();
    orphanList.clear();
}

bool CObjectContainer::makeObjectChildOf(CSceneObject* childObject,CSceneObject* parentObject)
{   // This will trigger an actualization (important! orphanList needs also an update and other things too)
    if (childObject==nullptr)
        return(false);
    // Check if the child has already his desired parent (so that we don't have to call the actualization (heavy and will also refresh all dialogs) (added on 2009/12/15)
    if (childObject->getParentObject()==parentObject)
        return(true);
    if (parentObject==nullptr)
    {
        C7Vector oldAbsoluteTransf(childObject->getCumulativeTransformationPart1());
        childObject->setParentObject(nullptr);
        childObject->setLocalTransformation(oldAbsoluteTransf);
        actualizeObjectInformation();
        return(true);
    }
    // Illegal loop checking:
    if (parentObject->isObjectAffiliatedWith(childObject))
        return(false);
    C7Vector oldAbsoluteTransf(childObject->getCumulativeTransformationPart1());
    C7Vector parentInverse(parentObject->getCumulativeTransformation().getInverse());
    childObject->setLocalTransformation(parentInverse*oldAbsoluteTransf);
    childObject->setParentObject(parentObject);
    actualizeObjectInformation();
    return(true);
}

void CObjectContainer::setAbsoluteConfiguration(int objectHandle,const C7Vector& v,bool keepChildrenInPlace)
{
    CSceneObject* it=getObject(objectHandle);
    if (it!=nullptr)
    {
        C7Vector childPreTr(it->getLocalTransformation());
        it->setLocalTransformation(it->getParentCumulativeTransformation().getInverse()*v);
        if (keepChildrenInPlace)
        {
            childPreTr=it->getLocalTransformation().getInverse()*childPreTr;
            for (size_t i=0;i<it->childList.size();i++)
                it->childList[i]->setLocalTransformation(childPreTr*it->childList[i]->getLocalTransformationPart1());
        }
    }
}

void CObjectContainer::_updateJointDependencies()
{
    for (size_t i=0;i<jointList.size();i++)
    {
        CJoint* it=getJoint(jointList[i]);
        it->dependentJoints.clear();
        for (size_t j=0;j<jointList.size();j++)
        {
            CJoint* it2=getJoint(jointList[j]);
            if (it2->getDependencyJointHandle()==it->getObjectHandle())
                it->dependentJoints.push_back(it2);
        }
    }
}

void CObjectContainer::actualizeObjectInformation()
{
    for (size_t i=0;i<objectList.size();i++)
    {
        CSceneObject* it=_objectIndex[size_t(objectList[i])];
        it->childList.clear();
    }
    for (size_t i=0;i<objectList.size();i++)
    {
        CSceneObject* it=_objectIndex[size_t(objectList[i])];
        CSceneObject* parent=it->getParentObject();
        if (parent!=nullptr)
            parent->childList.push_back(it);
    }

    jointList.clear();
    dummyList.clear();
    orphanList.clear();

    for (size_t i=0;i<objectList.size();i++)
    {
        if (_objectIndex[size_t(objectList[i])]->getObjectType()==ik_objecttype_joint)
            jointList.push_back(objectList[i]);
        if (_objectIndex[size_t(objectList[i])]->getObjectType()==ik_objecttype_dummy)
            dummyList.push_back(objectList[i]);
        if (_objectIndex[size_t(objectList[i])]->getParentObject()==nullptr)
            orphanList.push_back(objectList[i]);
    }
    _updateJointDependencies();
}

CObjectContainer* CObjectContainer::copyYourself() const
{
    CObjectContainer* duplicate=new CObjectContainer();
    duplicate->_objectIndex.resize(_objectIndex.size());
    for (size_t i=0;i<_objectIndex.size();i++)
    {
        if (_objectIndex[i]==nullptr)
            duplicate->_objectIndex[i]=nullptr;
        else
            duplicate->_objectIndex[i]=_objectIndex[i]->copyYourself();
    }
    duplicate->_nextObjectHandle=_nextObjectHandle;
    duplicate->orphanList.assign(orphanList.begin(),orphanList.end());
    duplicate->objectList.assign(objectList.begin(),objectList.end());
    duplicate->jointList.assign(jointList.begin(),jointList.end());
    duplicate->dummyList.assign(dummyList.begin(),dummyList.end());

    // Fix parenting info and joint dependencies lists:
    for (size_t i=0;i<_objectIndex.size();i++)
    {
        if (_objectIndex[i]!=nullptr)
        {
            CSceneObject* obj=duplicate->_objectIndex[i];
            if (_objectIndex[i]->getParentObject()!=nullptr)
            {
                CSceneObject* p=duplicate->getObject(_objectIndex[i]->getParentObject()->getObjectHandle());
                obj->setParentObject(p,false);
                p->childList.push_back(obj);
            }
            if (obj->getObjectType()==ik_objecttype_joint)
            {
                CJoint* duplicateJoint=(CJoint*)obj;
                CJoint* originalJoint=(CJoint*)_objectIndex[i];
                for (size_t j=0;j<originalJoint->dependentJoints.size();j++)
                    duplicateJoint->dependentJoints.push_back(duplicate->getJoint(originalJoint->dependentJoints[j]->getObjectHandle()));
            }
        }
    }

    return(duplicate);
}

int CObjectContainer::getObjectHandle(const std::string& objectName) const
{
    for (size_t i=0;i<objectList.size();i++)
    {
        if (_objectIndex[size_t(objectList[i])]->getObjectName().compare(objectName)==0)
            return(objectList[i]);
    }
    return(-1);
}

int CObjectContainer::getHighestObjectHandle() const
{
    int highest=-1;
    for (size_t i=0;i<objectList.size();i++)
    {
        if (objectList[i]>highest)
            highest=objectList[i];
    }
    return(highest);
}

CSceneObject* CObjectContainer::getObject(int objectHandle) const
{
    if ( (objectHandle>=0)&&(objectHandle<int(_objectIndex.size())) )
        return(_objectIndex[size_t(objectHandle)]);
    return(nullptr);
}

CDummy* CObjectContainer::getDummy(int objectHandle) const
{
    CSceneObject* it=getObject(objectHandle);
    if (it!=nullptr)
    {
        if (it->getObjectType()==ik_objecttype_dummy)
            return(static_cast<CDummy*>(it));
    }
    return(nullptr);
}

CJoint* CObjectContainer::getJoint(int objectHandle) const
{
    CSceneObject* it=getObject(objectHandle);
    if (it!=nullptr)
    {
        if (it->getObjectType()==ik_objecttype_joint)
            return(static_cast<CJoint*>(it));
    }
    return(nullptr);
}

CSceneObject* CObjectContainer::getObject(const std::string& name) const
{
    return(getObject(getObjectHandle(name)));
}

CSceneObject* CObjectContainer::getObjectFromIndex(size_t index) const
{
    CSceneObject* retVal=nullptr;
    if (index<objectList.size())
        retVal=_objectIndex[objectList[index]];
    return(retVal);
}

int CObjectContainer::createDummy(const char* objectName)
{
    CDummy* it=new CDummy();
    if (objectName!=nullptr)
        it->setObjectName(objectName);
    addObjectToScene(it,false);
    return(it->getObjectHandle());
}

int CObjectContainer::createJoint(const char* objectName,int jointType)
{
    CJoint* it=new CJoint(jointType);
    if (objectName!=nullptr)
        it->setObjectName(objectName);
    addObjectToScene(it,false);
    return(it->getObjectHandle());
}

bool CObjectContainer::eraseObject(CSceneObject* it)
{
    if (it==nullptr)
        return(false);

    // We announce the object will be erased:
    announceObjectWillBeErased(it->getObjectHandle()); // this may trigger other "eraseObject" calls (not really, since we don't have versatiles anymore!)
    // We remove the object from the object list
    size_t i;
    for (i=0;i<objectList.size();i++)
    {
        if (objectList[i]==it->getObjectHandle())
            break;
    }
    objectList.erase(objectList.begin()+i);
    // Now remove the object from the index
    _objectIndex[size_t(it->getObjectHandle())]=nullptr;
    delete it;
    actualizeObjectInformation();

    return(true);
}

//------------------ Object destruction announcement -------------------------
void CObjectContainer::announceObjectWillBeErased(int objectHandle)
{
    for (int i=0;i<int(objectList.size());i++)
    {
        CSceneObject* it=getObject(objectList[size_t(i)]);
        if (it->getObjectHandle()!=objectHandle)
        {
            if (it->announceSceneObjectWillBeErased(objectHandle))
            { // We should never enter here since one obj destruction cannot trigger another obj destruction (anymore, no more versatiles!) 
                eraseObject(it);
                i=-1; // ordering might have changed!
            }
        }
    }
    CEnvironment::currentEnvironment->ikGroupContainer->announceSceneObjectWillBeErased(objectHandle);
}

void CObjectContainer::announceIkGroupWillBeErased(int ikGroupHandle)
{
    for (size_t i=0;i<objectList.size();i++)
        getObject(objectList[i])->announceIkGroupWillBeErased(ikGroupHandle); // this never triggers 3DObject destruction!
    CEnvironment::currentEnvironment->ikGroupContainer->announceIkGroupWillBeErased(ikGroupHandle); // This will never trigger an Ik group destruction
}

void CObjectContainer::importExportKinematicsData(CSerialization& ar)
{
    if (ar.isWriting())
    {
        ar.writeInt(1); // 0 is/was from CoppeliaSim, 1 is from here!

        ar.writeInt(int(_objectIndex.size()));
        for (size_t i=0;i<_objectIndex.size();i++)
        {
            CSceneObject* it=getObjectFromIndex(i);
            if (it!=nullptr)
            {
                ar.writeInt(it->getObjectType());
                it->serialize(ar);
            }
            else
                ar.writeInt(-1);
        }

        ar.writeInt(_nextObjectHandle);

        ar.writeInt(int(orphanList.size()));
        for (size_t i=0;i<orphanList.size();i++)
            ar.writeInt(orphanList[i]);
        ar.writeInt(int(objectList.size()));
        for (size_t i=0;i<objectList.size();i++)
            ar.writeInt(objectList[i]);
        ar.writeInt(int(jointList.size()));
        for (size_t i=0;i<jointList.size();i++)
            ar.writeInt(jointList[i]);
        ar.writeInt(int(dummyList.size()));
        for (size_t i=0;i<dummyList.size();i++)
            ar.writeInt(dummyList[i]);

        ar.writeInt(int(CEnvironment::currentEnvironment->ikGroupContainer->ikGroups.size()));
        for (size_t i=0;i<CEnvironment::currentEnvironment->ikGroupContainer->ikGroups.size();i++)
            CEnvironment::currentEnvironment->ikGroupContainer->ikGroups[i]->serialize(ar);
    }
    else
    {
        removeAllObjects();
        CEnvironment::currentEnvironment->ikGroupContainer->removeAllIkGroups(); // just in case

        int versionNumber=ar.readInt(); // this is the ext IK serialization version. Not forward nor backward compatible!

        if (versionNumber==1)
        {
            int cnt=ar.readInt();
            for (int i=0;i<cnt;i++)
            {
                int objType=ar.readInt();
                if (objType==-1)
                    _objectIndex.push_back(nullptr);
                else if (objType==ik_objecttype_joint)
                {
                    CJoint* joint=new CJoint(ik_jointtype_revolute);
                    joint->serialize(ar);
                    _objectIndex.push_back(joint);
                }
                else
                {
                    CDummy* dum=new CDummy();
                    dum->serialize(ar);
                    _objectIndex.push_back(dum);
                }
            }
            _nextObjectHandle=ar.readInt();
            cnt=ar.readInt();
            for (int i=0;i<cnt;i++)
            {
                int v=ar.readInt();
                orphanList.push_back(v);
            }
            cnt=ar.readInt();
            for (int i=0;i<cnt;i++)
            {
                int v=ar.readInt();
                objectList.push_back(v);
            }
            cnt=ar.readInt();
            for (int i=0;i<cnt;i++)
            {
                int v=ar.readInt();
                jointList.push_back(v);
            }
            cnt=ar.readInt();
            for (int i=0;i<cnt;i++)
            {
                int v=ar.readInt();
                dummyList.push_back(v);
            }

            for (size_t i=0;i<objectList.size();i++)
            {
                CSceneObject* obj=getObject(objectList[i]);
                int p=obj->getParentObjectHandle();
                if (p>=0)
                    obj->setParentObject(getObject(p),false);
            }

            _updateJointDependencies();

            int ikGroupCnt=ar.readInt();

            for (int i=0;i<ikGroupCnt;i++)
            {
                CikGroup* it=new CikGroup();
                it->serialize(ar);
                CEnvironment::currentEnvironment->ikGroupContainer->addIkGroup(it,true);
            }
        }
        else
        {
            int objCnt=ar.readInt();

            std::vector<int> objectMapping;
            for (int i=0;i<objCnt;i++)
            {
                int objType=ar.readInt();

                CSceneObject* it;

                if (objType==ik_objecttype_joint)
                {
                    CJoint* joint=new CJoint(ik_jointtype_revolute);
                    joint->serialize(ar);
                    it=joint;
                }
                else
                {
                    CDummy* dum=new CDummy();
                    dum->serialize(ar);
                    it=dum;
                }
                objectMapping.push_back(it->getObjectHandle());
                addObjectToScene(it,false);
                objectMapping.push_back(it->getObjectHandle());
            }

            for (size_t i=0;i<objectList.size();i++)
            {
                CSceneObject* it=getObject(objectList[i]);
                it->performSceneObjectLoadingMapping(&objectMapping);
            }

            int ikGroupCnt=ar.readInt();

            for (int i=0;i<ikGroupCnt;i++)
            {
                CikGroup* it=new CikGroup();
                it->serialize(ar);
                CEnvironment::currentEnvironment->ikGroupContainer->addIkGroup(it,true);
            }

            for (size_t i=0;i<CEnvironment::currentEnvironment->ikGroupContainer->ikGroups.size();i++)
            {
                CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->ikGroups[i];
                it->performObjectLoadingMapping(&objectMapping);
            }
        }
    }
}

void CObjectContainer::addObjectToScene(CSceneObject* newObject,bool keepAllCurrentSettings)
{
    if (keepAllCurrentSettings)
        _objectIndex.push_back(newObject);
    else
    {
        std::string newObjName=newObject->getObjectName();
        if (newObjName.size()==0)
        {
            newObjName="0";
            newObject->setObjectName(newObjName);
        }
        while (getObject(newObjName)!=nullptr)
        {
            char aChar=newObjName[newObjName.length()-1];
            std::string oldNumber("");
            while ( (aChar>='0')&&(aChar<='9') )
            {
                oldNumber.insert(oldNumber.begin(),aChar);
                newObjName.erase(newObjName.end()-1);
                if (newObjName.length()!=0)
                    aChar=newObjName[newObjName.length()-1];
                else
                    aChar='a';
            }
            if (oldNumber.length()==0)
                newObjName=newObjName+"0";
            else
                newObjName=newObjName+std::to_string(atoi(oldNumber.c_str())+1);
        }

        // Give the object a new handle
        int handle=-1;
        for (size_t i=0;i<_objectIndex.size();i++)
        {
            if (_objectIndex[i]==nullptr)
            {
                _objectIndex[i]=newObject;
                handle=int(i);
                break;
            }
        }
        if (handle==-1)
        {
            handle=int(_objectIndex.size());
            _objectIndex.push_back(newObject);
        }

        // set the new handle to the object:
        newObject->setObjectHandle(handle);
        objectList.push_back(handle);

        // Actualize the object information
        actualizeObjectInformation();
    }
}

void CObjectContainer::memorizeJointConfig(std::vector<int>& jointHandles,std::vector<double>& jointValues)
{
    jointHandles.clear();
    jointValues.clear();
    for (size_t i=0;i<jointList.size();i++)
    {
        jointHandles.push_back(jointList[i]);
        CJoint* it=getJoint(jointList[i]);
        if (it->getJointType()==ik_jointtype_spherical)
        { // we memorize the 4 quaternion vals
            C4Vector q(it->getSphericalTransformation());
            for (size_t j=0;j<4;j++)
                jointValues.push_back(q(j));
        }
        else
            jointValues.push_back(it->getPosition());
    }
}

void CObjectContainer::restoreJointConfig(const std::vector<int>& jointHandles,const std::vector<double>& jointValues)
{
    size_t ind=0;
    for (size_t i=0;i<jointHandles.size();i++)
    {
        CJoint* it=getJoint(jointHandles[i]);
        if (it!=nullptr)
        {
            if (it->getJointType()==ik_jointtype_spherical)
            { // we memorize the 4 quaternion vals
                C4Vector q;
                for (size_t j=0;j<4;j++)
                    q(j)=jointValues[ind++];
                it->setSphericalTransformation(q);
            }
            else
                it->setPosition(jointValues[ind++]);
        }
        else
            ind++; // should not happen
    }
}
