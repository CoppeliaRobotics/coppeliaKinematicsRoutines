#pragma once

#include "ik.h"
#include "sceneObject.h"
#include "joint.h"
#include "ikGroup.h"

class CObjectContainer
{
public:
    CObjectContainer();
    virtual ~CObjectContainer();

    void newSceneProcedure();
    void removeAllObjects();
    void actualizeObjectInformation();

    CObjectContainer* copyYourself() const;
    int getObjectHandle(const std::string& objectName) const;
    CSceneObject* getObject(int objectHandle) const;
    CDummy* getDummy(int objectHandle) const;
    CJoint* getJoint(int objectHandle) const;
    CSceneObject* getObject(const std::string& name) const;
    CSceneObject* getObjectFromIndex(size_t index) const;

    int createDummy(const char* objectName);
    int createJoint(const char* objectName,int jointType);

    bool makeObjectChildOf(CSceneObject* childObject,CSceneObject* parentObject);
    void setAbsoluteConfiguration(int objectHandle,const C7Vector& v,bool keepChildrenInPlace);

    int getHighestObjectHandle() const;
    bool eraseObject(CSceneObject* it);

    void announceObjectWillBeErased(int objectHandle);
    void announceIkGroupWillBeErased(int ikGroupHandle);

    std::vector<CSceneObject*> _objectIndex;

    int _nextObjectHandle;

    std::vector<int> orphanList;
    std::vector<int> objectList;
    std::vector<int> jointList;
    std::vector<int> dummyList;

    void importExportKinematicsData(CSerialization& ar);
    void addObjectToScene(CSceneObject* newObject,bool keepAllCurrentSettings);

    void memorizeJointConfig(std::vector<int>& jointHandles,std::vector<double>& jointValues);
    void restoreJointConfig(const std::vector<int>& jointHandles,const std::vector<double>& jointValues);

private:
    void _updateJointDependencies();

};

