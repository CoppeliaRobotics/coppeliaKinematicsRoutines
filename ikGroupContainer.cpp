#include "ikGroupContainer.h"
#include "ikRoutines.h"
#include "app.h"
#include "simConst.h"


CIkGroupContainer::CIkGroupContainer()
{
}

CIkGroupContainer::~CIkGroupContainer()
{
    removeAllIkGroups();
}

CikGroup* CIkGroupContainer::getIkGroup(int groupID) const
{
     for (size_t i=0;i<ikGroups.size();i++)
     {
        if (ikGroups[i]->getObjectID()==groupID)
            return(ikGroups[i]);
     }
     return(nullptr);
}

CikGroup* CIkGroupContainer::getIkGroup(std::string groupName) const
{
    for (size_t i=0;i<ikGroups.size();i++)
    {
        if (ikGroups[i]->getObjectName().compare(groupName)==0)
            return(ikGroups[i]);
    }
    return(nullptr);
}

void CIkGroupContainer::removeIkGroup(int ikGroupHandle)
{
    App::currentInstance->objectContainer->announceIkGroupWillBeErased(ikGroupHandle);
    for (size_t i=0;i<ikGroups.size();i++)
    {
        if (ikGroups[i]->getObjectID()==ikGroupHandle)
        {
            delete ikGroups[i];
            ikGroups.erase(ikGroups.begin()+i);
            return;
        }
    }
}

void CIkGroupContainer::removeAllIkGroups()
{
    while (ikGroups.size()!=0)
        removeIkGroup(ikGroups[0]->getObjectID());
}

void CIkGroupContainer::announceSceneObjectWillBeErased(int objectHandle)
{
    size_t i=0;
    while (i<ikGroups.size())
    {
        if (ikGroups[i]->announceSceneObjectWillBeErased(objectHandle))
        { // This ik group has to be erased:
            removeIkGroup(ikGroups[i]->getObjectID()); // This will call announceIkGroupWillBeErased!
            i=0; // order may have changed!
        }
        else
            i++;
    }
}

void CIkGroupContainer::resetCalculationResults()
{
    for (size_t i=0;i<ikGroups.size();i++)
        ikGroups[i]->resetCalculationResult();
}

void CIkGroupContainer::announceIkGroupWillBeErased(int ikGroupHandle)
{ // Never called from copy buffer!
    size_t i=0;
    while (i<ikGroups.size())
    {
        if (ikGroups[i]->announceIkGroupWillBeErased(ikGroupHandle))
        { // This ik group has to be erased (normally never happens)
            removeIkGroup(ikGroups[i]->getObjectID()); // This will call announceIkGroupWillBeErased!
            i=0; // ordering may have changed!
        }
        else
            i++;
    }
}

int CIkGroupContainer::computeAllIkGroups(bool exceptExplicitHandling)
{
    int performedCount=0;
    {
        for (size_t i=0;i<ikGroups.size();i++)
        {
            if ((!exceptExplicitHandling)||(!ikGroups[i]->getExplicitHandling()))
            {
                int res=0;
                res=ikGroups[i]->computeGroupIk(false);
                ikGroups[i]->setCalculationResult(res);
                if (res!=sim_ikresult_not_performed)
                    performedCount++;
            }
        }
    }
    return(performedCount);
}

void CIkGroupContainer::addIkGroup(CikGroup* anIkGroup)
{ // Be careful! We don't check if the group is valid!!
    ikGroups.push_back(anIkGroup);
}
