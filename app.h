#pragma once

#include "objectContainer.h"
#include "ikGroupContainer.h"

class App
{
public:
    App(bool protectedEnv);
    virtual ~App();

    static int addInstance(App* inst);
    static bool switchToInstance(int handle,bool alsoProtectedEnv);
    static int killInstance(int handle);

    CIkGroupContainer* ikGroupContainer;
    CObjectContainer* objectContainer;
    bool protectedEnvironment;

    static App* currentInstance;
    static int currentInstanceHandle;

private:
    std::vector<CIkGroupContainer*> _ikGroupContainers;
    std::vector<CObjectContainer*> _objectContainers;
    std::vector<bool> _protectedEnvironments;

    static int _nextInstanceHandle;
    static std::vector<App*> _allInstances;
    static std::vector<int> _allInstanceHandles;
};

