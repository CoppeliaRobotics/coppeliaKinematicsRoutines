#include "environment.h"

CEnvironment* CEnvironment::currentEnvironment=nullptr;
int CEnvironment::_nextHandle=0;
std::vector<CEnvironment*> CEnvironment::_allEnvironments;

CEnvironment::CEnvironment()
{
    objectContainer=nullptr;
    ikGroupContainer=nullptr;
    _protected=false;
}

CEnvironment::CEnvironment(bool protectedEnv)
{
    objectContainer=new CObjectContainer();
    ikGroupContainer=new CIkGroupContainer();
    _protected=protectedEnv;
}

CEnvironment::~CEnvironment()
{
    ikGroupContainer->removeAllIkGroups();
    objectContainer->removeAllObjects();
    delete ikGroupContainer;
    delete objectContainer;
}

int CEnvironment::getHandle() const
{
    return(_handle);
}

bool CEnvironment::isProtected() const
{
    return(_protected);
}

CEnvironment* CEnvironment::copyYourself() const
{
    CEnvironment* duplicate=new CEnvironment();
    duplicate->ikGroupContainer=ikGroupContainer->copyYourself();
    duplicate->objectContainer=objectContainer->copyYourself();
    return(duplicate);
}

int CEnvironment::addEnvironment(CEnvironment* env)
{
    currentEnvironment=env;
    _allEnvironments.push_back(env);
    currentEnvironment->_handle=_nextHandle;
    _nextHandle++;
    return(currentEnvironment->_handle);
}

bool CEnvironment::switchToEnvironment(int handle,bool alsoProtectedEnv)
{
    bool retVal=false;
    if (handle==-1)
    { // tries to switch to the first protected environment, for debug purposes
        for (size_t i=0;i<_allEnvironments.size();i++)
        {
            if (_allEnvironments[i]->_protected)
            {
                currentEnvironment=_allEnvironments[i];
                return(true);
            }
        }
    }
    else
    {
        if ( (handle>=0)&&(currentEnvironment!=nullptr) )
        {
            if (currentEnvironment->_handle!=handle)
            {
                for (size_t i=0;i<_allEnvironments.size();i++)
                {
                    if (_allEnvironments[i]->_handle==handle)
                    {
                        if ( alsoProtectedEnv||(!_allEnvironments[i]->_protected) )
                        {
                            currentEnvironment=_allEnvironments[i];
                            return(true);
                        }
                    }
                }
                return(false);
            }
            retVal=(alsoProtectedEnv||(!currentEnvironment->_protected));
        }
    }
    return(retVal);
}

int CEnvironment::killEnvironment(int handle)
{
    int retVal=-1;
    for (size_t i=0;i<_allEnvironments.size();i++)
    {
        if (_allEnvironments[i]->_handle==handle)
        {
            delete currentEnvironment;
            _allEnvironments.erase(_allEnvironments.begin()+i);
            if (_allEnvironments.size()>0)
            {
                currentEnvironment=_allEnvironments[0];
                retVal=currentEnvironment->_handle;
            }
            else
                currentEnvironment=nullptr;
            break;
        }
    }
    return(retVal);
}
