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
    int getLinkedDummyHandle() const;
    void setLinkedDummyHandle(int theHandle,bool setDirectly);

protected:
    int _linkedDummyHandle;
};
