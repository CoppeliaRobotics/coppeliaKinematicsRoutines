#include "app.h"

App* App::currentInstance=nullptr;
int App::currentInstanceHandle=0;
int App::_nextInstanceHandle=1;
std::vector<App*> App::_allInstances;
std::vector<int> App::_allInstanceHandles;

App::App(bool protectedEnv)
{
    objectContainer=new CObjectContainer();
    ikGroupContainer=new CIkGroupContainer();
    protectedEnvironment=protectedEnv;
    _objectContainers.push_back(objectContainer);
    _ikGroupContainers.push_back(ikGroupContainer);
    _protectedEnvironments.push_back(protectedEnvironment);
}

App::~App()
{
    while (_objectContainers.size()!=0)
    {
        size_t index=_objectContainers.size()-1;
        ikGroupContainer=_ikGroupContainers[index];
        objectContainer=_objectContainers[index];
        protectedEnvironment=_protectedEnvironments[index];
        ikGroupContainer->removeAllIkGroups();
        objectContainer->removeAllObjects();
        delete objectContainer;
        objectContainer=nullptr;
        delete ikGroupContainer;
        ikGroupContainer=nullptr;
        _objectContainers.erase(_objectContainers.begin()+index);
        _ikGroupContainers.erase(_ikGroupContainers.begin()+index);
        _protectedEnvironments.erase(_protectedEnvironments.begin()+index);
    }
}

int App::addInstance(App* inst)
{
    currentInstanceHandle=_nextInstanceHandle;
    currentInstance=inst;
    _allInstanceHandles.push_back(currentInstanceHandle);
    _allInstances.push_back(inst);
    _nextInstanceHandle++;
    return(currentInstanceHandle);
}

bool App::switchToInstance(int handle,bool alsoProtectedEnv)
{
    if (currentInstanceHandle!=handle)
    {
        for (size_t i=0;i<_allInstanceHandles.size();i++)
        {
            if (_allInstanceHandles[i]==handle)
            {
                if ( alsoProtectedEnv||(!_allInstances[i]->protectedEnvironment) )
                {
                    currentInstanceHandle=handle;
                    currentInstance=_allInstances[i];
                    return(true);
                }
            }
        }
        return(false);
    }
    return(alsoProtectedEnv||(!currentInstance->protectedEnvironment));
}

int App::killInstance(int handle)
{
    for (size_t i=0;i<_allInstanceHandles.size();i++)
    {
        if (_allInstanceHandles[i]==handle)
        {
            _allInstanceHandles.erase(_allInstanceHandles.begin()+i);
            _allInstances.erase(_allInstances.begin()+i);
            if (currentInstanceHandle==handle)
            {
                currentInstanceHandle=0;
                currentInstance=nullptr;
                if (_allInstanceHandles.size()>0)
                {
                    currentInstanceHandle=_allInstanceHandles[0];
                    currentInstance=_allInstances[0];
                }
                return(currentInstanceHandle);
            }
        }
    }
    return(-1);
}
