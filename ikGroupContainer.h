#pragma once

#include "ik.h"
#include <vector>
#include "ikGroup.h"

class CIkGroupContainer
{
public:
    CIkGroupContainer();
    virtual ~CIkGroupContainer();

    CIkGroupContainer* copyYourself() const;
    CikGroup* getIkGroup(int groupHandle) const;
    CikGroup* getIkGroup(std::string groupName) const;
    void addIkGroup(CikGroup* anIkGroup,bool keepCurrentHandle);
    void removeIkGroup(int ikGroupHandle);
    void removeAllIkGroups();
    void announceSceneObjectWillBeErased(int objectHandle);
    void announceIkGroupWillBeErased(int ikGroupHandle);
    int computeAllIkGroups(bool exceptExplicitHandling);
    int computeIk(int groupHandle,double precision[2],bool(*cb)(const int*,std::vector<double>*,const int*,const int*,const int*,const int*,std::vector<double>*,double*));
    int computeIk(const std::vector<int>& groupHandles,double precision[2],bool(*cb)(const int*,std::vector<double>*,const int*,const int*,const int*,const int*,std::vector<double>*,double*));


    std::vector<CikGroup*> ikGroups;
};
