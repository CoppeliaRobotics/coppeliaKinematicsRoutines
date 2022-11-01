#include "ikGroupContainer.h"
#include "environment.h"

CIkGroupContainer::CIkGroupContainer()
{
}

CIkGroupContainer::~CIkGroupContainer()
{
    removeAllIkGroups();
}

CIkGroupContainer* CIkGroupContainer::copyYourself() const
{
    CIkGroupContainer* duplicate=new CIkGroupContainer();

    for (size_t i=0;i<ikGroups.size();i++)
        duplicate->ikGroups.push_back(ikGroups[i]->copyYourself());

    return(duplicate);
}

CikGroup* CIkGroupContainer::getIkGroup(int groupHandle) const
{
     for (size_t i=0;i<ikGroups.size();i++)
     {
        if (ikGroups[i]->getObjectHandle()==groupHandle)
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
    CEnvironment::currentEnvironment->objectContainer->announceIkGroupWillBeErased(ikGroupHandle);
    for (size_t i=0;i<ikGroups.size();i++)
    {
        if (ikGroups[i]->getObjectHandle()==ikGroupHandle)
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
        removeIkGroup(ikGroups[0]->getObjectHandle());
}

void CIkGroupContainer::announceSceneObjectWillBeErased(int objectHandle)
{
    size_t i=0;
    while (i<ikGroups.size())
    {
        if (ikGroups[i]->announceSceneObjectWillBeErased(objectHandle))
        { // This ik group has to be erased:
            removeIkGroup(ikGroups[i]->getObjectHandle()); // This will call announceIkGroupWillBeErased!
            i=0; // order may have changed!
        }
        else
            i++;
    }
}

void CIkGroupContainer::announceIkGroupWillBeErased(int ikGroupHandle)
{ // Never called from copy buffer!
    size_t i=0;
    while (i<ikGroups.size())
    {
        if (ikGroups[i]->announceIkGroupWillBeErased(ikGroupHandle))
        { // This ik group has to be erased (normally never happens)
            removeIkGroup(ikGroups[i]->getObjectHandle()); // This will call announceIkGroupWillBeErased!
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
            if ((!exceptExplicitHandling)||(!ikGroups[i]->getExplicitHandling_old()))
            {
                int res=0;
                res=ikGroups[i]->computeGroupIk(false,nullptr);
                if (res!=ik_result_not_performed)
                    performedCount++;
            }
        }
    }
    return(performedCount);
}

void CIkGroupContainer::addIkGroup(CikGroup* anIkGroup,bool keepCurrentHandle)
{ // Be careful! We don't check if the group is valid!!
    if (!keepCurrentHandle)
    {
        int newHandle=2030003;
        while (getIkGroup(newHandle)!=nullptr)
            newHandle++;
        anIkGroup->setObjectHandle(newHandle);
    }
    ikGroups.push_back(anIkGroup);
}
