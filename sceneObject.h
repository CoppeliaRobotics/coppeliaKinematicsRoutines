#pragma once

#include "ik.h"
#include <vector>
#include "serialization.h"
#include "3Vector.h"
#include "4Vector.h"
#include "7Vector.h"

class CSceneObject
{
public:
    CSceneObject();
    virtual ~CSceneObject();

    virtual bool announceSceneObjectWillBeErased(int objectHandle);
    virtual void announceIkGroupWillBeErased(int ikGroupHandle);
    void announceSceneObjectWillBeErasedMain(int objectHandle);
    void announceIkGroupWillBeErasedMain(int ikGroupHandle);
    virtual void performSceneObjectLoadingMapping(const std::vector<int>* map);
    void performSceneObjectLoadingMappingMain(const std::vector<int>* map);
    virtual void serialize(CSerialization& ar);
    void serializeMain(CSerialization& ar);

    virtual CSceneObject* copyYourself() const;
    C7Vector getParentCumulativeTransformation() const;
    C7Vector getCumulativeTransformation() const;
    C7Vector getLocalTransformation() const;
    C7Vector getCumulativeTransformationPart1() const;
    C7Vector getLocalTransformationPart1() const;

    void setLocalTransformation(const C7Vector& v);
    void setLocalTransformation(const C4Vector& q);
    void setLocalTransformation(const C3Vector& x);

    int getObjectHandle() const;
    void setObjectHandle(int newHandle);
    void setObjectName(std::string newName);
    std::string getObjectName() const;
    CSceneObject* getParentObject() const;
    int getParentObjectHandle() const;
    bool isObjectAffiliatedWith(const CSceneObject* theObject) const;
    void setParentObject(CSceneObject* newParentObject,bool updateContainer=true);
    int getObjectType() const;

    std::vector<CSceneObject*> childList;

protected:
    int _objectHandle;
    std::string _objectName;
    C7Vector _transformation;
    CSceneObject* _parentObject;
    int _parentObjectHandle;
    int _objectType;
};
