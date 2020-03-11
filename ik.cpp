#include "ik.h"
#include "app.h"
#include "simConst.h"

static std::string lastError;

std::string ikGetLastError()
{
    std::string le(lastError);
    lastError.clear();
    return(le);
}

bool hasLaunched()
{
    bool retVal=App::currentInstanceHandle>0;
    if (!retVal)
        lastError="Environment does not exist";
    return(retVal);
}

CikElement* getIkElementFromIndexOrTipFrame(const CikGroup* ikGroup,int ikElementIndex)
{
    CikElement* retVal=nullptr;
    bool fromHandle=(ikElementIndex&ik_handleflag_tipframe)!=0;
    if (fromHandle)
    {
        ikElementIndex-=ik_handleflag_tipframe;
        retVal=ikGroup->getIkElementWithTooltipID(ikElementIndex);
        if (retVal==nullptr)
            lastError="Invalid IK element tip frame handle";
    }
    else
    {
        if ( (ikElementIndex>=0)&&(size_t(ikElementIndex)<ikGroup->ikElements.size()) )
            retVal=ikGroup->ikElements[size_t(ikElementIndex)];
        else
            lastError="Invalid IK element index";
    }
    return(retVal);
}

bool ikCreateEnvironment(int* environmentHandle/*=nullptr*/,bool protectedEnvironment/*=false*/)
{
    App* newCt=new App(protectedEnvironment);
    int eh=App::addInstance(newCt);
    if (environmentHandle!=nullptr)
        environmentHandle[0]=eh;
    return(true);
}

bool ikEraseEnvironment(int* switchedEnvironmentHandle/*=nullptr*/)
{
    bool retVal=false;
    if (hasLaunched())
    {
        int eh=App::killInstance(App::currentInstanceHandle);
        if (switchedEnvironmentHandle!=nullptr)
            switchedEnvironmentHandle[0]=eh;
        retVal=true;
    }
    return(retVal);
}

void ikReleaseBuffer(void* buffer)
{
    delete[] static_cast<simReal*>(buffer);
}

bool ikSwitchEnvironment(int handle,bool allowAlsoProtectedEnvironment/*=false*/)
{
    bool retVal=false;
    if (hasLaunched())
    {
        if (App::switchToInstance(handle,allowAlsoProtectedEnvironment))
            retVal=true;
        else
            lastError="Invalid environment ID";
    }
    return(retVal);
}

bool ikLoad(const unsigned char* data,size_t dataLength)
{
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (App::currentInstance->objectContainer->objectList.size()==0)&&(App::currentInstance->ikGroupContainer->ikGroups.size()==0) )
        {
            if ((data!=nullptr)&&(dataLength!=0))
            {
                CSerialization ar(data,dataLength);
                App::currentInstance->objectContainer->importKinematicsData(ar);
                App::currentInstance->objectContainer->objectList.size();
                retVal=true;
            }
            else
                lastError="Invalid arguments";
        }
        else
            lastError="Environment must be empty";
    }
    return(retVal);
}

bool ikGetObjectHandle(const char* objectName,int* objectHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=App::currentInstance->objectContainer->getObject(objectName);
        if (it!=nullptr)
        {
            objectHandle[0]=it->getObjectHandle();
            retVal=true;
        }
        else
            lastError="Object does not exist";
    }
    return(retVal);
}

bool ikDoesObjectExist(const char* objectName)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=App::currentInstance->objectContainer->getObject(objectName);
        retVal=(it!=nullptr);
    }
    return(retVal);
}

bool ikGetObjectTransformation(int objectHandle,int relativeToObjectHandle,C7Vector* transf)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=App::currentInstance->objectContainer->getObject(objectHandle);
        if (it!=nullptr)
        {
            if (relativeToObjectHandle==sim_handle_parent)
            {
                relativeToObjectHandle=-1;
                CSceneObject* parent=it->getParentObject();
                if (parent!=nullptr)
                    relativeToObjectHandle=parent->getObjectHandle();
            }
            CSceneObject* relObj=App::currentInstance->objectContainer->getObject(relativeToObjectHandle);
            if ( (relativeToObjectHandle==-1)||(relObj!=nullptr) )
            {
                if (relativeToObjectHandle==-1)
                    transf[0]=it->getCumulativeTransformationPart1();
                else
                {
                    C7Vector relTr(relObj->getCumulativeTransformationPart1()); // added ..Part1 on 2010/06/14
                    transf[0]=relTr.getInverse()*it->getCumulativeTransformationPart1(); // Corrected bug on 2011/01/22: was getLocalTransformationPart1 before!!!
                }
                retVal=true;
            }
            else
                lastError="Invalid arguments";
        }
        else
            lastError="Invalid object handle";
    }
    return(retVal);
}

bool ikGetObjectMatrix(int objectHandle,int relativeToObjectHandle,C4X4Matrix* matrix)
{
    C7Vector transf;
    bool retVal=ikGetObjectTransformation(objectHandle,relativeToObjectHandle,&transf);
    if (retVal)
        matrix[0]=transf.getMatrix();
    return(retVal);
}

bool ikSetObjectTransformation(int objectHandle,int relativeToObjectHandle,const C7Vector* transf)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=App::currentInstance->objectContainer->getObject(objectHandle);
        if (it!=nullptr)
        {
            if (relativeToObjectHandle==sim_handle_parent)
            {
                relativeToObjectHandle=-1;
                CSceneObject* parent=it->getParentObject();
                if (parent!=nullptr)
                    relativeToObjectHandle=parent->getObjectHandle();
            }
            CSceneObject* relObj=App::currentInstance->objectContainer->getObject(relativeToObjectHandle);
            if ( (relativeToObjectHandle==-1)||(relObj!=nullptr) )
            {
                if (relativeToObjectHandle==-1)
                    App::currentInstance->objectContainer->setAbsoluteConfiguration(it->getObjectHandle(),transf[0],false);
                else
                {
                    C7Vector relTr(relObj->getCumulativeTransformationPart1());
                    C7Vector absTr(relTr*transf[0]);
                    App::currentInstance->objectContainer->setAbsoluteConfiguration(it->getObjectHandle(),absTr,false);
                }
                retVal=true;
            }
            else
                lastError="Invalid arguments";
        }
        else
            lastError="Invalid object handle";
    }
    return(retVal);
}

bool ikSetObjectMatrix(int objectHandle,int relativeToObjectHandle,const C4X4Matrix* matrix)
{
    C7Vector transf(matrix->getTransformation());
    return(ikSetObjectTransformation(objectHandle,relativeToObjectHandle,&transf));
}

bool ikGetJointPosition(int jointHandle,simReal* position)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if (it->getJointType()!=sim_joint_spherical_subtype)
            {
                position[0]=it->getPosition();
                retVal=true;
            }
            else
                lastError="Invalid call with spherical joint";
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikSetJointPosition(int jointHandle,simReal position)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if (it->getJointType()!=sim_joint_spherical_subtype)
            {
                it->setPosition(position);
                retVal=true;
            }
            else
                lastError="Invalid call with spherical joint";
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikDoesIkGroupExist(const char* ikGroupName)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupName);
        retVal=(it!=nullptr);
    }
    return(retVal);
}

bool ikGetIkGroupHandle(const char* ikGroupName,int* ikGroupHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupName);
        if (it!=nullptr)
        {
            ikGroupHandle[0]=it->getObjectID();
            retVal=true;
        }
        else
            lastError="IK group does not exist";
    }
    return(retVal);
}

bool ikCreateIkGroup(const char* ikGroupName/*=nullptr*/,int* ikGroupHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (ikGroupName==nullptr)||((strlen(ikGroupName)>0)&&(App::currentInstance->ikGroupContainer->getIkGroup(ikGroupName)==nullptr)) )
        {
            CikGroup* it=new CikGroup();
            if (ikGroupName!=nullptr)
                it->setObjectName(ikGroupName);
            it->setExplicitHandling(true);
            App::currentInstance->ikGroupContainer->addIkGroup(it);
            ikGroupHandle[0]=it->getObjectID();
            retVal=true;
        }
        else
            lastError="Invalid IK group name";
    }
    return(retVal);
}

bool ikAddIkElement(int ikGroupHandle,int tipHandle,int* ikElementIndex)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CDummy* dummy=App::currentInstance->objectContainer->getDummy(tipHandle);
            if (dummy!=nullptr)
            {
                CikElement* ikElement=new CikElement(tipHandle);
                ikGroup->ikElements.push_back(ikElement);
                ikElementIndex[0]=int(ikGroup->ikElements.size())-1;
                retVal=true;
            }
            else
                lastError="Invalid tip frame handle";
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikSetIkElementEnabled(int ikGroupHandle,int ikElementIndex,bool enabled)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                ikElement->setIsActive(enabled);
                retVal=true;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetIkElementEnabled(int ikGroupHandle,int ikElementIndex,bool* enabled)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                retVal=true;
                enabled[0]=ikElement->getIsActive();
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikSetIkElementBase(int ikGroupHandle,int ikElementIndex,int baseHandle,int constraintsBaseHandle/*=-1*/)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                CSceneObject* b1=App::currentInstance->objectContainer->getObject(baseHandle);
                if ( (b1!=nullptr)||(baseHandle==-1) )
                {
                    CSceneObject* b2=App::currentInstance->objectContainer->getObject(constraintsBaseHandle);
                    if ( (b2!=nullptr)||(constraintsBaseHandle==-1) )
                    {
                        if (ikElement->getBaseHandle()!=baseHandle)
                            ikElement->setBaseHandle(baseHandle);
                        if (ikElement->getAltBaseHandleForConstraints()!=constraintsBaseHandle)
                            ikElement->setAltBaseHandleForConstraints(constraintsBaseHandle);
                        retVal=true;
                    }
                    else
                        lastError="Invalid constraints base handle";
                }
                else
                    lastError="Invalid base handle";
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetIkElementBase(int ikGroupHandle,int ikElementIndex,int* baseHandle,int* constraintsBaseHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                baseHandle[0]=ikElement->getBaseHandle();
                constraintsBaseHandle[0]=ikElement->getAltBaseHandleForConstraints();
                retVal=true;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikSetIkElementConstraints(int ikGroupHandle,int ikElementIndex,int constraints)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                if (ikElement->getConstraints()!=constraints)
                    ikElement->setConstraints(constraints);
                retVal=true;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetIkElementConstraints(int ikGroupHandle,int ikElementIndex,int* constraints)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                constraints[0]=ikElement->getConstraints();
                retVal=true;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikSetIkElementPrecision(int ikGroupHandle,int ikElementIndex,simReal linearPrecision,simReal angularPrecision)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                if ( fabs(ikElement->getMinLinearPrecision()-linearPrecision)>simReal(0.0001) )
                    ikElement->setMinLinearPrecision(linearPrecision);
                if ( fabs(ikElement->getMinAngularPrecision()-angularPrecision)>simReal(0.01)*degToRad )
                    ikElement->setMinAngularPrecision(angularPrecision);
                retVal=true;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetIkElementPrecision(int ikGroupHandle,int ikElementIndex,simReal* linearPrecision,simReal* angularPrecision)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                linearPrecision[0]=ikElement->getMinLinearPrecision();
                angularPrecision[0]=ikElement->getMinAngularPrecision();
                retVal=true;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikSetIkElementWeights(int ikGroupHandle,int ikElementIndex,simReal linearWeight,simReal angularWeight)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                if ( fabs(ikElement->getPositionWeight()-linearWeight)>simReal(0.001) )
                    ikElement->setPositionWeight(linearWeight);
                if ( fabs(ikElement->getOrientationWeight()-angularWeight)>simReal(0.001) )
                    ikElement->setOrientationWeight(angularWeight);
                retVal=true;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetIkElementWeights(int ikGroupHandle,int ikElementIndex,simReal* linearWeight,simReal* angularWeight)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromIndexOrTipFrame(ikGroup,ikElementIndex);
            if (ikElement!=nullptr)
            {
                linearWeight[0]=ikElement->getPositionWeight();
                angularWeight[0]=ikElement->getOrientationWeight();
                retVal=true;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikComputeJacobian(int ikGroupHandle,int options,bool* success/*=nullptr*/)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            bool succ=it->computeOnlyJacobian(options);
            retVal=true;
            if (success!=nullptr)
                success[0]=succ;
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

simReal* ikGetJacobian(int ikGroupHandle,size_t* matrixSize)
{
    simReal* retVal=nullptr;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            simReal* b=it->getLastJacobianData(matrixSize);
            if (b!=nullptr)
            {
                retVal=new simReal[size_t(matrixSize[0]*matrixSize[1])];
                for (size_t i=0;i<size_t(matrixSize[0]*matrixSize[1]);i++)
                    retVal[i]=b[i];
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetManipulability(int ikGroupHandle,simReal* manip)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            simReal b=it->getLastManipulabilityValue(retVal);
            if (retVal)
                manip[0]=b;
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikHandleIkGroup(int ikGroupHandle,int* result/*=nullptr*/)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if ( (ikGroupHandle==sim_handle_all)||(ikGroupHandle==sim_handle_all_except_explicit)||(it!=nullptr) )
        {
            if (ikGroupHandle<0)
                retVal=App::currentInstance->ikGroupContainer->computeAllIkGroups(ikGroupHandle==sim_handle_all_except_explicit);
            else
            { // explicit handling
                if (it->getExplicitHandling())
                {
                    int res=it->computeGroupIk(false);
                    if (result!=nullptr)
                        result[0]=res;
                    retVal=true;
                }
                else
                    lastError="IK group cannot explicitely be handled";
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetJointTransformation(int jointHandle,C7Vector* transf)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            C7Vector trFull(it->getLocalTransformation());
            C7Vector trPart1(it->getLocalTransformationPart1());
            C7Vector tr(trPart1.getInverse()*trFull);
            transf[0]=tr;
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikGetJointMatrix(int jointHandle,C4X4Matrix* matrix)
{
    C7Vector transf;
    bool retVal=ikGetJointTransformation(jointHandle,&transf);
    if (retVal)
        matrix[0]=transf.getMatrix();
    return(retVal);
}

bool ikSetSphericalJointQuaternion(int jointHandle,const C4Vector* quaternion)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if (it->getJointType()==sim_joint_spherical_subtype)
            {
                it->setSphericalTransformation(quaternion[0]);
                retVal=true;
            }
            else
                lastError="Joint is not spherical";
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikSetSphericalJointMatrix(int jointHandle,const C3X3Matrix* rotMatrix)
{
    C4Vector q(rotMatrix->getQuaternion());
    return(ikSetSphericalJointQuaternion(jointHandle,&q));
}

bool ikGetObjectParent(int objectHandle,int* parentObjectHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=App::currentInstance->objectContainer->getObject(objectHandle);
        if (it!=nullptr)
        {
            parentObjectHandle[0]=-1;
            if (it->getParentObject()!=nullptr)
                parentObjectHandle[0]=it->getParentObject()->getObjectHandle();
            retVal=true;
        }
        else
            lastError="Invalid object handle";
    }
    return(retVal);
}

bool ikSetObjectParent(int objectHandle,int parentObjectHandle,bool keepInPlace)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=App::currentInstance->objectContainer->getObject(objectHandle);
        CSceneObject* parentIt=App::currentInstance->objectContainer->getObject(parentObjectHandle);
        if (it!=nullptr)
        {
            if ( (parentIt!=nullptr)||(parentObjectHandle==-1) )
            {
                if (keepInPlace)
                    App::currentInstance->objectContainer->makeObjectChildOf(it,parentIt);
                else
                    it->setParentObject(parentIt);
                retVal=true;
            }
            else
                lastError="Invalid parent object handle";
        }
        else
            lastError="Invalid object handle";
    }
    return(retVal);
}

bool ikCreateFrame(const char* frameName/*=nullptr*/,int* frameHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (frameName==nullptr)||((strlen(frameName)!=0)&&(App::currentInstance->objectContainer->getObject(frameName)==nullptr)) )
        {
            frameHandle[0]=App::currentInstance->objectContainer->createDummy(frameName);
            retVal=true;
        }
        else
            lastError="Invalid object name";
    }
    return(retVal);
}

bool ikSetLinkedFrame(int frameHandle,int linkedFrameHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CDummy* it=App::currentInstance->objectContainer->getDummy(frameHandle);
        if (it!=nullptr)
        {
            CDummy* it2=App::currentInstance->objectContainer->getDummy(linkedFrameHandle);
            if ( (it2!=nullptr)||(linkedFrameHandle==-1) )
            {
                if (it2==nullptr)
                    it->setLinkedDummyHandle(-1,false);
                else
                {
                    it->setLinkedDummyHandle(linkedFrameHandle,false);
                    it->setLinkType(sim_dummy_linktype_ik_tip_target,false);
                }
                retVal=true;
            }
            else
                lastError="Invalid linked frame handle";
        }
        else
            lastError="Invalid frame handle";
    }
    return(retVal);
}

bool ikGetLinkedFrame(int frameHandle,int* linkedFrameHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CDummy* it=App::currentInstance->objectContainer->getDummy(frameHandle);
        if (it!=nullptr)
        {
            linkedFrameHandle[0]=it->getLinkedDummyHandle();
            if (it->getLinkType()!=sim_dummy_linktype_ik_tip_target)
                linkedFrameHandle[0]=-1;
            retVal=true;
        }
        else
            lastError="Invalid frame handle";
    }
    return(retVal);
}

bool ikCreateJoint(const char* jointName/*=nullptr*/,int jointType,int* jointHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (jointName==nullptr)||((strlen(jointName)!=0)&&(App::currentInstance->objectContainer->getObject(jointName)==nullptr)) )
        {
            jointHandle[0]=App::currentInstance->objectContainer->createJoint(jointName,jointType);
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikSetJointMode(int jointHandle,int jointMode)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if (it->getJointMode()!=jointMode)
                it->setJointMode(jointMode);
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikGetJointMode(int jointHandle,int* mode)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            mode[0]=it->getJointMode();
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikSetJointInterval(int jointHandle,bool cyclic,const simReal* intervalMinAndRange/*=nullptr*/)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            simReal previousPos=it->getPosition();
            if (it->getPositionIsCyclic()!=cyclic)
                it->setPositionIsCyclic(cyclic);
            if (intervalMinAndRange!=nullptr)
            {
                if ( fabs(it->getPositionIntervalMin()-intervalMinAndRange[0])>simReal(0.00001) )
                    it->setPositionIntervalMin(intervalMinAndRange[0]);
                if ( fabs(it->getPositionIntervalRange()-intervalMinAndRange[1])>simReal(0.00001) )
                    it->setPositionIntervalRange(intervalMinAndRange[1]);
            }
            it->setPosition(previousPos);
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikSetJointScrewPitch(int jointHandle,simReal pitch)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if ( fabs(it->getScrewPitch()-pitch)>simReal(0.000001) )
                it->setScrewPitch(pitch);
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikSetJointIkWeight(int jointHandle,simReal ikWeight)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if ( fabs(it->getIkWeight()-ikWeight)>simReal(0.0001) )
                it->setIkWeight(ikWeight);
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikGetJointIkWeight(int jointHandle,simReal* ikWeight)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            ikWeight[0]=it->getIkWeight();
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikSetJointMaxStepSize(int jointHandle,simReal maxStepSize)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if ( fabs(it->getMaxStepSize()-maxStepSize)>simReal(0.00001) )
                it->setMaxStepSize(maxStepSize);
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikGetJointMaxStepSize(int jointHandle,simReal* maxStepSize)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            maxStepSize[0]=it->getMaxStepSize();
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikSetJointDependency(int jointHandle,int dependencyJointHandle,simReal offset/*=0.0*/,simReal mult/*=1.0*/)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            CJoint* it2=App::currentInstance->objectContainer->getJoint(dependencyJointHandle);
            if ( (it2!=nullptr)||(dependencyJointHandle==-1) )
            {
                bool ok=true;
                if (it->getDependencyJointHandle()!=dependencyJointHandle)
                    ok=it->setDependencyJointHandle(dependencyJointHandle);
                if (ok)
                {
                    if (dependencyJointHandle!=-1)
                    {
                        if ( fabs(it->getDependencyJointMult()-mult)>simReal(0.00001) )
                            it->setDependencyJointMult(mult);
                        if ( fabs(it->getDependencyJointAdd()-offset)>simReal(0.00001) )
                            it->setDependencyJointAdd(offset);
                    }
                    retVal=true;
                }
                else
                    lastError="Failed setting dependency joint";
            }
            else
                lastError="Invalid dependency joint handle";
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikGetJointDependency(int jointHandle,int* dependencyJointHandle,simReal* offset,simReal* mult)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            dependencyJointHandle[0]=it->getDependencyJointHandle();
            offset[0]=it->getDependencyJointAdd();
            mult[0]=it->getDependencyJointMult();
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikEraseObject(int objectHandle)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* obj=App::currentInstance->objectContainer->getObject(objectHandle);
        if (obj!=nullptr)
            retVal=App::currentInstance->objectContainer->eraseObject(obj);
        else
            lastError="Invalid object handle";
    }
    return(retVal);
}

bool ikGetJointInterval(int jointHandle,bool* cyclic,simReal* intervalMinAndRange)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            intervalMinAndRange[0]=it->getPositionIntervalMin();
            intervalMinAndRange[1]=it->getPositionIntervalRange();
            retVal=true;
            cyclic[0]=it->getPositionIsCyclic();
        }
        else
            lastError="Joint does not exist";
    }
    return(retVal);
}

bool ikGetJointScrewPitch(int jointHandle,simReal* pitch)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=App::currentInstance->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            pitch[0]=it->getScrewPitch();
            retVal=true;
        }
        else
            lastError="Joint does not exist";
    }
    return(retVal);
}

bool ikGetIkGroupCalculation(int ikGroupHandle,int* method,simReal* damping,int* maxIterations)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            retVal=true;
            method[0]=it->getCalculationMethod();
            damping[0]=it->getDlsFactor();
            maxIterations[0]=it->getMaxIterations();
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikSetIkGroupCalculation(int ikGroupHandle,int method,simReal damping,int maxIterations)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            if (it->getCalculationMethod()!=method)
                it->setCalculationMethod(method);
            if ( fabs(it->getDlsFactor()-damping)>simReal(0.0001) )
                it->setDlsFactor(damping);
            if (it->getMaxIterations()!=maxIterations)
                it->setMaxIterations(maxIterations);
            retVal=true;
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetIkGroupLimitThresholds(int ikGroupHandle,simReal* linearAndAngularThresholds)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            retVal=true;
            linearAndAngularThresholds[0]=it->getJointTreshholdLinear();
            linearAndAngularThresholds[1]=it->getJointTreshholdAngular();
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikSetIkGroupLimitThresholds(int ikGroupHandle,const simReal* linearAndAngularThresholds)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            if ( fabs(it->getJointTreshholdLinear()-linearAndAngularThresholds[0])>simReal(0.0001) )
                it->setJointTreshholdLinear(linearAndAngularThresholds[0]);
            if ( fabs(it->getJointTreshholdAngular()-linearAndAngularThresholds[1])>simReal(0.001)*degToRad )
                it->setJointTreshholdAngular(linearAndAngularThresholds[1]);
            retVal=true;
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}


bool ikSetIkGroupFlags(int ikGroupHandle,int flags)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            if (it->getActive()!=((flags&1)!=0))
                it->setActive((flags&1)!=0);
            if (it->getCorrectJointLimits()!=((flags&2)!=0))
                it->setCorrectJointLimits((flags&2)!=0);
            if (it->getIgnoreMaxStepSizes()!=((flags&4)!=0))
                it->setIgnoreMaxStepSizes((flags&4)!=0);
            retVal=false;
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetIkGroupFlags(int ikGroupHandle,int* flags)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            retVal=true;
            flags[0]=0;
            if (it->getActive())
                flags[0]|=1;
            if (it->getCorrectJointLimits())
                flags[0]|=2;
            if (it->getIgnoreMaxStepSizes())
                flags[0]|=4;
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

int ikGetConfigForTipPose(int ikGroupHandle,size_t jointCnt,const int* jointHandles,simReal thresholdDist,int maxIterations,simReal* retConfig,const simReal* metric/*=nullptr*/,bool(*validationCallback)(simReal*)/*=nullptr*/,const int* jointOptions/*=nullptr*/,const simReal* lowLimits/*=nullptr*/,const simReal* ranges/*=nullptr*/)
{
    int retVal=-1;
    std::vector<simReal> conf(jointCnt);
    if (hasLaunched())
    {
        CikGroup* ikGroup=App::currentInstance->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            const simReal _defaultMetric[4]={simOne,simOne,simOne,simReal(0.1)};
            const simReal* theMetric=_defaultMetric;
            if (metric!=nullptr)
                theMetric=metric;
            std::vector<CJoint*> joints;
            std::vector<simReal> minVals;
            std::vector<simReal> rangeVals;
            int err=0;
            for (size_t i=0;i<jointCnt;i++)
            {
                CJoint* aJoint=App::currentInstance->objectContainer->getJoint(jointHandles[i]);
                if (aJoint==nullptr)
                    err=1;
                else
                {
                    joints.push_back(aJoint);
                    if ( (lowLimits!=nullptr)&&(ranges!=nullptr) )
                    {
                        minVals.push_back(lowLimits[i]);
                        rangeVals.push_back(ranges[i]);
                    }
                    else
                    {
                        if (aJoint->getPositionIsCyclic())
                        {
                            minVals.push_back(-piValue);
                            rangeVals.push_back(piValTimes2);
                        }
                        else
                        {
                            minVals.push_back(aJoint->getPositionIntervalMin());
                            rangeVals.push_back(aJoint->getPositionIntervalRange());
                        }
                    }
                }
            }
            std::vector<CDummy*> tips;
            std::vector<CDummy*> targets;
            std::vector<CSceneObject*> bases;
            if (ikGroup!=nullptr)
            {
                if (ikGroup->ikElements.size()>0)
                {
                    for (size_t i=0;i<ikGroup->ikElements.size();i++)
                    {
                        CDummy* tip=App::currentInstance->objectContainer->getDummy(ikGroup->ikElements[i]->getTipHandle());
                        CDummy* target=App::currentInstance->objectContainer->getDummy(ikGroup->ikElements[i]->getTargetHandle());
                        CSceneObject* base=nullptr;
                        if (ikGroup->ikElements[i]->getAltBaseHandleForConstraints()!=-1)
                            base=App::currentInstance->objectContainer->getObject(ikGroup->ikElements[i]->getAltBaseHandleForConstraints());
                        else
                            base=App::currentInstance->objectContainer->getObject(ikGroup->ikElements[i]->getBaseHandle());
                        if ((tip==nullptr)||(target==nullptr))
                            err=2;
                        tips.push_back(tip);
                        targets.push_back(target);
                        bases.push_back(base);
                    }
                }
                else
                    err=3;
            }

            if (err==0)
            {
                retVal=0;
                // Save joint positions/modes (all of them, just in case)
                std::vector<CJoint*> sceneJoints;
                std::vector<simReal> initSceneJointValues;
                std::vector<int> initSceneJointModes;
                for (size_t i=0;i<App::currentInstance->objectContainer->jointList.size();i++)
                {
                    CJoint* aj=App::currentInstance->objectContainer->getJoint(App::currentInstance->objectContainer->jointList[i]);
                    sceneJoints.push_back(aj);
                    initSceneJointValues.push_back(aj->getPosition());
                    initSceneJointModes.push_back(aj->getJointMode());
                }

                ikGroup->setAllInvolvedJointsToPassiveMode();

                bool ikGroupWasActive=ikGroup->getActive();
                if (!ikGroupWasActive)
                    ikGroup->setActive(true);

                // It can happen that some IK elements get deactivated when the user provided wrong handles, so save the activation state:
                std::vector<bool> enabledElements;
                for (size_t i=0;i<ikGroup->ikElements.size();i++)
                    enabledElements.push_back(ikGroup->ikElements[i]->getIsActive());

                // Set the correct mode for the joints involved:
                for (size_t i=0;i<jointCnt;i++)
                {
                    if ( (jointOptions==nullptr)||((jointOptions[i]&1)==0) )
                        joints[i]->setJointMode(sim_jointmode_ik);
                    else
                        joints[i]->setJointMode(sim_jointmode_dependent);
                }

                // do the calculation:
                for (int iterationCnt=0;iterationCnt<maxIterations;iterationCnt++)
                {
                    // 1. Pick a random state:
                    for (size_t i=0;i<jointCnt;i++)
                        joints[i]->setPosition(minVals[i]+(rand()/simReal(RAND_MAX))*rangeVals[i]);

                    // 2. Check distances between tip and target pairs (there might be several pairs!):
                    simReal cumulatedDist=simZero;
                    for (size_t el=0;el<ikGroup->ikElements.size();el++)
                    {
                        C7Vector tipTr(tips[el]->getCumulativeTransformation());
                        C7Vector targetTr(targets[el]->getCumulativeTransformation());
                        C7Vector relTrInv(C7Vector::identityTransformation);
                        if (bases[el]!=nullptr)
                            relTrInv=bases[el]->getCumulativeTransformationPart1().getInverse();
                        tipTr=relTrInv*tipTr;
                        targetTr=relTrInv*targetTr;
                        C3Vector dx(tipTr.X-targetTr.X);
                        dx(0)*=theMetric[0];
                        dx(1)*=theMetric[1];
                        dx(2)*=theMetric[2];
                        simReal angle=tipTr.Q.getAngleBetweenQuaternions(targetTr.Q)*theMetric[3];
                        cumulatedDist+=sqrt(dx(0)*dx(0)+dx(1)*dx(1)+dx(2)*dx(2)+angle*angle);
                    }

                    // 3. If distance<=threshold, try to perform IK:
                    if (cumulatedDist<=thresholdDist)
                    {
                        if (sim_ikresult_success==ikGroup->computeGroupIk(true))
                        { // 3.1 We found a configuration that works!
                            // 3.2 Check joint limits:
                            bool limitsOk=true;
                            if ( (lowLimits!=nullptr)&&(ranges!=nullptr) )
                            {
                                for (size_t i=0;i<jointCnt;i++)
                                {
                                    if ( (joints[i]->getPosition()<minVals[i])||(joints[i]->getPosition()>minVals[i]+rangeVals[i]) )
                                        limitsOk=false;
                                }
                            }
                            // 3.3 Finally check if the callback accepts that configuration:
                            for (size_t i=0;i<jointCnt;i++)
                                conf[i]=joints[i]->getPosition();
                            if ( (validationCallback==nullptr)||validationCallback(&conf[0]) )
                            {
                                for (size_t i=0;i<jointCnt;i++)
                                    retConfig[i]=conf[i];
                                retVal=1;
                                break;
                            }
                        }
                    }
                }

                if (!ikGroupWasActive)
                    ikGroup->setActive(false);

                // Restore the IK element activation state:
                for (size_t i=0;i<ikGroup->ikElements.size();i++)
                    ikGroup->ikElements[i]->setIsActive(enabledElements[i]);

                // Restore joint positions/modes:
                for (size_t i=0;i<sceneJoints.size();i++)
                {
                    if (fabs(sceneJoints[i]->getPosition()-initSceneJointValues[i])>simReal(0.0001))
                        sceneJoints[i]->setPosition(initSceneJointValues[i]);
                    if (sceneJoints[i]->getJointMode()!=initSceneJointModes[i])
                        sceneJoints[i]->setJointMode(initSceneJointModes[i]);
                }
            }
            else
            {
                if (err==1)
                    lastError="Found invalid joint handle(s)";
                if (err==2)
                    lastError="Found ill-defined IK element(s)";
                if (err==3)
                    lastError="Ill-defined IK group";
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

int _getLoadingMapping(const std::vector<int>* map,int oldVal)
{
    for (size_t i=0;i<map->size()/2;i++)
    {
        if (oldVal==map->at(2*i+0))
            return(map->at(2*i+1));
    }
    return(-1);
}
