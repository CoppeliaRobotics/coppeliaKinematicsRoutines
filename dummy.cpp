#include "dummy.h"
#include "environment.h"

CDummy::CDummy()
{
    _objectType=ik_objecttype_dummy;
    _objectName="dummy";
    _linkedDummyHandle=-1;
    _linkType=ik_linktype_ik_tip_target;
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
    _linkedDummyHandle=ar.readInt();
    _linkType=ar.readInt();
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
                if ( (_linkType==ik_linktype_ik_tip_target)||(_linkType==ik_linktype_gcs_loop_closure)||(_linkType==ik_linktype_dynamics_loop_closure)||(_linkType==ik_linktype_dynamics_force_constraint) )
                    newLinkedDummy->setLinkType(_linkType,true);
                // Deprecated:
                if (_linkType==ik_linktype_gcs_tip)
                    newLinkedDummy->setLinkType(ik_linktype_gcs_target,true);
                if (_linkType==ik_linktype_gcs_target)
                    newLinkedDummy->setLinkType(ik_linktype_gcs_tip,true);
            }
            else
                _linkedDummyHandle=-1;
        }
    }
}

int CDummy::getLinkType() const
{
    return(_linkType);
}

void CDummy::setLinkType(int theLinkType,bool setDirectly)
{
    _linkType=theLinkType;
    if ( (_linkedDummyHandle!=-1)&&(!setDirectly) )
    {
        CDummy* linkedDummy=CEnvironment::currentEnvironment->objectContainer->getDummy(_linkedDummyHandle);
        if (linkedDummy!=nullptr)
        {
            if ( (theLinkType==ik_linktype_ik_tip_target)||(theLinkType==ik_linktype_gcs_loop_closure)||(theLinkType==ik_linktype_dynamics_loop_closure)||(theLinkType==ik_linktype_dynamics_force_constraint) )
                linkedDummy->setLinkType(theLinkType,true);
            // Deprecated:
            if (theLinkType==ik_linktype_gcs_tip)
                linkedDummy->setLinkType(ik_linktype_gcs_target,true);
            if (theLinkType==ik_linktype_gcs_target)
                linkedDummy->setLinkType(ik_linktype_gcs_tip,true);
        }
    }
}
