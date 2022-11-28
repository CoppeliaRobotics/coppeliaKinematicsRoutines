#pragma once

#include "ik.h"
#include "sceneObject.h"
#include <vector>

class CDummy : public CSceneObject
{
public:
    CDummy();
    virtual ~CDummy();

    bool announceSceneObjectWillBeErased(int objectHandle);
    void announceIkGroupWillBeErased(int ikGroupHandle);
    void performSceneObjectLoadingMapping(const std::vector<int>* map);
    void serialize(CSerialization& ar);

    CSceneObject* copyYourself() const;
    int getTargetDummyHandle() const;
    void setTargetDummyHandle(int theHandle);

    int getLinkedDummyHandle_old() const;
    void setLinkedDummyHandle_old(int theHandle,bool setDirectly);

protected:
    int _targetDummyHandle;
    int _linkedDummyHandle_old; // for backward compatibility
};
