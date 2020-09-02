#pragma once

#include "objectContainer.h"
#include "ikGroupContainer.h"

class CEnvironment
{
public:
    CEnvironment();
    CEnvironment(bool protectedEnv);
    virtual ~CEnvironment();

    static int addEnvironment(CEnvironment* env);
    static bool switchToEnvironment(int handle,bool alsoProtectedEnv);
    static int killEnvironment(int handle);

    int getHandle() const;
    bool isProtected() const;
    CEnvironment* copyYourself() const;
    CIkGroupContainer* ikGroupContainer;
    CObjectContainer* objectContainer;

    static CEnvironment* currentEnvironment;

private:
    bool _protected;
    int _handle;

    static int _nextHandle;
    static std::vector<CEnvironment*> _allEnvironments;
};

