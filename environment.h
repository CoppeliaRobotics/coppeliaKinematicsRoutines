#pragma once

#include "objectContainer.h"
#include "ikGroupContainer.h"

class CEnvironment
{
public:
    CEnvironment();
    CEnvironment(int flags);
    virtual ~CEnvironment();

    static int addEnvironment(CEnvironment* env);
    static bool switchToEnvironment(int handle,bool alsoProtectedEnv);
    static int killEnvironment(int handle);

    int getHandle() const;
    bool isProtected() const;
    int getFlags() const;
    void setFlags(int f);
    CEnvironment* copyYourself() const;
    CIkGroupContainer* ikGroupContainer;
    CObjectContainer* objectContainer;

    static CEnvironment* currentEnvironment;

private:
    int _handle;
    int _flags; // bit0: protected (GUI-IK back-compat.), bit1: back-compatibility, with Jacob./errorV correction with a fact. 0.01

    static int _nextHandle;
    static std::vector<CEnvironment*> _allEnvironments;
};

