#include "simConst.h"
#include "dummy.h"
#include "app.h"


CDummy::CDummy()
{
    _objectType=sim_object_dummy_type;
    _objectName="dummy";
    _linkedDummyHandle=-1;
    _linkType=sim_dummy_linktype_ik_tip_target;
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
            CDummy* oldLinkedDummy=App::currentInstance->objectContainer->getDummy(_linkedDummyHandle);
            if (oldLinkedDummy!=nullptr)
                oldLinkedDummy->setLinkedDummyHandle(-1,true);
            CDummy* newLinkedDummy=App::currentInstance->objectContainer->getDummy(theHandle);
            if (newLinkedDummy!=nullptr)
            {
                newLinkedDummy->setLinkedDummyHandle(-1,false);
                _linkedDummyHandle=theHandle;
                newLinkedDummy->setLinkedDummyHandle(_objectHandle,true);
                if ( (_linkType==sim_dummy_linktype_ik_tip_target)||(_linkType==sim_dummy_linktype_gcs_loop_closure)||(_linkType==sim_dummy_linktype_dynamics_loop_closure)||(_linkType==sim_dummy_linktype_dynamics_force_constraint) )
                    newLinkedDummy->setLinkType(_linkType,true);
                // Deprecated:
                if (_linkType==sim_dummy_linktype_gcs_tip)
                    newLinkedDummy->setLinkType(sim_dummy_linktype_gcs_target,true);
                if (_linkType==sim_dummy_linktype_gcs_target)
                    newLinkedDummy->setLinkType(sim_dummy_linktype_gcs_tip,true);
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
        CDummy* linkedDummy=App::currentInstance->objectContainer->getDummy(_linkedDummyHandle);
        if (linkedDummy!=nullptr)
        {
            if ( (theLinkType==sim_dummy_linktype_ik_tip_target)||(theLinkType==sim_dummy_linktype_gcs_loop_closure)||(theLinkType==sim_dummy_linktype_dynamics_loop_closure)||(theLinkType==sim_dummy_linktype_dynamics_force_constraint) )
                linkedDummy->setLinkType(theLinkType,true);
            // Deprecated:
            if (theLinkType==sim_dummy_linktype_gcs_tip)
                linkedDummy->setLinkType(sim_dummy_linktype_gcs_target,true);
            if (theLinkType==sim_dummy_linktype_gcs_target)
                linkedDummy->setLinkType(sim_dummy_linktype_gcs_tip,true);
        }
    }
}
