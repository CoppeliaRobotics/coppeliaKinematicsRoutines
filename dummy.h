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

    int getLinkedDummyHandle() const;
    void setLinkedDummyHandle(int theHandle,bool setDirectly);
    int getLinkType() const;
    void setLinkType(int theLinkType,bool setDirectly);

protected:
    int _linkedDummyHandle;
    int _linkType;
};
