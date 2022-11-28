#include "dummy.h"
#include "environment.h"

CDummy::CDummy()
{
    _objectType=ik_objecttype_dummy;
    _objectName="dummy";
    _targetDummyHandle=-1;
    _linkedDummyHandle_old=-1; // backward compatibility
}

CDummy::~CDummy()
{
}

bool CDummy::announceSceneObjectWillBeErased(int objectHandle)
{
    announceSceneObjectWillBeErasedMain(objectHandle);
    if (_targetDummyHandle==objectHandle)
        _targetDummyHandle=-1;
    // backward compatibility:
    if (_linkedDummyHandle_old==objectHandle)
        setLinkedDummyHandle_old(-1,false);
    return(false);
}

void CDummy::announceIkGroupWillBeErased(int ikGroupHandle)
{
    announceIkGroupWillBeErasedMain(ikGroupHandle);
}

void CDummy::performSceneObjectLoadingMapping(const std::vector<int>* map)
{
    performSceneObjectLoadingMappingMain(map);
    _linkedDummyHandle_old=_getLoadingMapping(map,_linkedDummyHandle_old); // backward compatibility
    _targetDummyHandle=_getLoadingMapping(map,_targetDummyHandle);
}

void CDummy::serialize(CSerialization& ar)
{
    serializeMain(ar);
    if (ar.isWriting())
    {
        ar.writeInt(_targetDummyHandle);
        ar.writeInt(_targetDummyHandle); // previously link type
    }
    else
    {
        _targetDummyHandle=ar.readInt();
        ar.readInt(); // previously link type
    }
}

CSceneObject* CDummy::copyYourself() const
{
    CDummy* duplicate=(CDummy*)CSceneObject::copyYourself();

    duplicate->_linkedDummyHandle_old=_linkedDummyHandle_old; // backward compatibility
    duplicate->_targetDummyHandle=_targetDummyHandle;

    return(duplicate);
}

int CDummy::getTargetDummyHandle() const
{
    return(_targetDummyHandle);
}

void CDummy::setTargetDummyHandle(int theHandle)
{
    if (_targetDummyHandle!=theHandle)
        _targetDummyHandle=theHandle;
}

int CDummy::getLinkedDummyHandle_old() const
{ // backward compatibility
    return(_linkedDummyHandle_old);
}

void CDummy::setLinkedDummyHandle_old(int theHandle,bool setDirectly)
{ // backward compatibility
    if (_linkedDummyHandle_old!=theHandle)
    {
        if (setDirectly)
            _linkedDummyHandle_old=theHandle;
        else
        {
            CDummy* oldLinkedDummy=CEnvironment::currentEnvironment->objectContainer->getDummy(_linkedDummyHandle_old);
            if (oldLinkedDummy!=nullptr)
                oldLinkedDummy->setLinkedDummyHandle_old(-1,true);
            CDummy* newLinkedDummy=CEnvironment::currentEnvironment->objectContainer->getDummy(theHandle);
            if (newLinkedDummy!=nullptr)
            {
                newLinkedDummy->setLinkedDummyHandle_old(-1,false);
                _linkedDummyHandle_old=theHandle;
                newLinkedDummy->setLinkedDummyHandle_old(_objectHandle,true);
            }
            else
                _linkedDummyHandle_old=-1;
        }
    }
}

