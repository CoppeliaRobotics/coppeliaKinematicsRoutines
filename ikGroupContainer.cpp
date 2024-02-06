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
                std::vector<int> gr;
                gr.push_back(ikGroups[i]->getObjectHandle());
                if ( (computeIk(gr,nullptr,nullptr)&ik_calc_notperformed)==0 )
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

int CIkGroupContainer::computeIk(int groupHandle,double precision[2],int(*cb)(const int*,double*,const int*,const int*,const int*,const int*,double*,double*,double*,int,int))
{ // Return value is one ik_resultinfo...-value
    std::vector<int> gr;
    gr.push_back(groupHandle);
    return(computeIk(gr,precision,cb));
}

int CIkGroupContainer::computeIk(const std::vector<int>& groupHandles,double precision[2],int(*cb)(const int*,double*,const int*,const int*,const int*,const int*,double*,double*,double*,int,int))
{ // Return value is one ik_resultinfo...-value
    if (precision!=nullptr)
    {
        precision[0]=0.0;
        precision[1]=0.0;
    }
    std::vector<CikGroup*> ikGroups;
    std::vector<std::vector<CikElement*>> ikElements;
    for (size_t i=0;i<groupHandles.size();i++)
    {
        CikGroup* ikGroup=getIkGroup(groupHandles[i]);
        if ( (ikGroup!=nullptr)&&((ikGroup->getOptions()&ik_group_enabled)!=0) )
        {
            ikGroup->clearJointLimitHits();
            ikGroups.push_back(ikGroup);
            std::vector<CikElement*> els;
            if (!ikGroup->getValidElements(els))
                return(ik_calc_notperformed|ik_calc_notwithintolerance);
            ikElements.push_back(els);
        }
    }
    if (ikGroups.size()==0)
        return(ik_calc_notperformed|ik_calc_notwithintolerance);
    CikGroup* firstIkGroup=ikGroups[0];

    std::vector<int> memorizedConf_handles;
    std::vector<double> memorizedConf_vals;
    CEnvironment::currentEnvironment->objectContainer->memorizeJointConfig(memorizedConf_handles,memorizedConf_vals);

    // Here we have the main iteration loop:
    double interpolFact=1.0; // We first try to solve in one step
    int cumulResultInfo=0;
    bool withinPosition,withinOrientation;
    for (int iterationNb=0;iterationNb<ikGroups[0]->getMaxIterations();iterationNb++)
    {
        std::vector<CJoint*> allJoints;
        std::vector<int> allJointDofIndices;
        for (size_t i=0;i<ikGroups.size();i++)
        {
            CikGroup* ikGroup=ikGroups[i];
            ikGroup->prepareRawJacobians(ikElements[i],interpolFact);
            ikGroup->selectJoints(&ikElements[i],&allJoints,&allJointDofIndices);
        }
        for (size_t i=0;i<ikGroups.size();i++)
        {
            CikGroup* ikGroup=ikGroups[i];
            if (ikGroups.size()>1)
                ikGroup->selectJoints(&ikElements[i],&allJoints,&allJointDofIndices); // needs to be called a second time
            int r=ikGroup->computeDq(&ikElements[i],false,iterationNb,cb);
            if ( (r==ik_calc_cannotinvert)||(r==ik_calc_invalidcallbackdata) )
            {
                cumulResultInfo=cumulResultInfo|r|ik_calc_notwithintolerance;
                break;
            }
        }
        if ((cumulResultInfo&(ik_calc_cannotinvert|ik_calc_invalidcallbackdata))!=0)
            break;

        // "Assemble" result here:
        // ***************************
        CMatrix tdQ=ikGroups[ikGroups.size()-1]->getDq();
        for (int i=int(ikGroups.size())-2;i>=0;i--)
        {
            CikGroup* ikGroup=ikGroups[size_t(i)];
            const CMatrix dQ=ikGroup->getDq();
            const CMatrix J=ikGroup->getJacobian();
            const CMatrix Jinv=ikGroup->getInvJacobian();
            CMatrix I(allJoints.size(),allJoints.size());
            I.setIdentity();
            CMatrix NP=I-Jinv*J;
            tdQ=dQ+NP*tdQ;
        }
        // ***************************

        double maxStepFact=0.0;
        int resultInfo=firstIkGroup->checkDq(&tdQ,&maxStepFact);
        cumulResultInfo=cumulResultInfo|resultInfo;

        if ( ((firstIkGroup->getOptions()&ik_group_ignoremaxsteps)==0)&&((resultInfo&ik_calc_stepstoobig)!=0) )
        { // Joint variations not within tolerance. Retry by interpolating:
            interpolFact=interpolFact/(maxStepFact*1.1); // 10% tolerance
            iterationNb--; // redo this pass
        }
        else
        {
            firstIkGroup->applyDq(&tdQ);
            if ( (maxStepFact<0.8)&&(interpolFact<1.0) )
            { // Joint variations are not too large: try to return to a non-interpolated target:
                interpolFact=interpolFact/(maxStepFact*1.1); // 10% tolerance
                if (interpolFact>1.0)
                    interpolFact=1.0;
            }
        }

        // We check if all IK elements are under the required precision
        withinPosition=true;
        withinOrientation=true;;
        for (size_t i=0;i<ikElements.size();i++)
        {
            for (size_t j=0;j<ikElements[i].size();j++)
            {
                CikElement* element=ikElements[i][j];
                double lp,ap;
                element->getTipTargetDistance(lp,ap);
                double pr[2];
                element->getPrecisions(pr);
                if (lp>pr[0])
                    withinPosition=false;
                if (ap>pr[1])
                    withinOrientation=false;
            }
        }
        if ( (!withinPosition)||(!withinOrientation) )
            cumulResultInfo=cumulResultInfo|ik_calc_notwithintolerance;
        if ( ((cumulResultInfo&ik_calc_limithit)!=0)&&((firstIkGroup->getOptions()&ik_group_stoponlimithit)!=0) )
            break;
        if (withinPosition&&withinOrientation)
        {
            cumulResultInfo=cumulResultInfo&(~ik_calc_notwithintolerance);
            break;
        }
    }

    if (precision!=nullptr)
    {
        for (size_t i=0;i<ikElements.size();i++)
        {
            for (size_t j=0;j<ikElements[i].size();j++)
            {
                CikElement* element=ikElements[i][j];
                double lp,ap;
                element->getTipTargetDistance(lp,ap);
                if (lp>precision[0])
                    precision[0]=lp;
                if (ap>precision[1])
                    precision[1]=ap;
            }
        }
    }

    bool restoreConfig=(cumulResultInfo&ik_calc_cannotinvert)!=0;
    if ( (cumulResultInfo&ik_calc_notwithintolerance)!=0 ) // notwithintolerance cannot always be considered as an error (e.g. when we want slow convergence, etc.) and results are applied by default
        restoreConfig=( (((firstIkGroup->getOptions()&ik_group_restoreonbadlintol)!=0)&&(!withinPosition))||(((firstIkGroup->getOptions()&ik_group_restoreonbadangtol)!=0)&&(!withinOrientation)) );
    if (restoreConfig)
        CEnvironment::currentEnvironment->objectContainer->restoreJointConfig(memorizedConf_handles,memorizedConf_vals);

    return(cumulResultInfo);
}
