#include "dummy.h"
#include "environment.h"

CDummy::CDummy()
{
    _objectType=ik_objecttype_dummy;
    _objectName="dummy";
    _linkedDummyHandle=-1;
}

CDummy::~CDummy()
{
}

bool CDummy::announceSceneObjectWillBeErased(int objectHandle)
{
    announceSceneObjectWillBeErasedMain(objectHandle);
    if (_linkedDummyHandle==objectHandle)
        setLinkedDummyHandle(-1,false);
    return(false);
}

void CDummy::announceIkGroupWillBeErased(int ikGroupHandle)
{
    announceIkGroupWillBeErasedMain(ikGroupHandle);
}

void CDummy::performSceneObjectLoadingMapping(const std::vector<int>* map)
{
    performSceneObjectLoadingMappingMain(map);
    _linkedDummyHandle=_getLoadingMapping(map,_linkedDummyHandle);
}

void CDummy::serialize(CSerialization& ar)
{
    serializeMain(ar);
    if (ar.isWriting())
    {
        ar.writeInt(_linkedDummyHandle);
        ar.writeInt(_linkedDummyHandle); // previously link type
    }
    else
    {
        _linkedDummyHandle=ar.readInt();
        ar.readInt(); // previously link type
    }
}

CSceneObject* CDummy::copyYourself() const
{
    CDummy* duplicate=(CDummy*)CSceneObject::copyYourself();

    duplicate->_linkedDummyHandle=_linkedDummyHandle;

    return(duplicate);
}

int CDummy::getLinkedDummyHandle() const
{
    return(_linkedDummyHandle);
}

void CDummy::setLinkedDummyHandle(int theHandle,bool setDirectly)
{
    if (_linkedDummyHandle!=theHandle)
    {
        if (setDirectly)
            _linkedDummyHandle=theHandle;
        else
        {
            CDummy* oldLinkedDummy=CEnvironment::currentEnvironment->objectContainer->getDummy(_linkedDummyHandle);
            if (oldLinkedDummy!=nullptr)
                oldLinkedDummy->setLinkedDummyHandle(-1,true);
            CDummy* newLinkedDummy=CEnvironment::currentEnvironment->objectContainer->getDummy(theHandle);
            if (newLinkedDummy!=nullptr)
            {
                newLinkedDummy->setLinkedDummyHandle(-1,false);
                _linkedDummyHandle=theHandle;
                newLinkedDummy->setLinkedDummyHandle(_objectHandle,true);
            }
            else
                _linkedDummyHandle=-1;
        }
    }
}

