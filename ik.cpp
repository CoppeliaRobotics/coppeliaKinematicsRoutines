#include "ik.h"
#include "environment.h"
#ifdef _WIN32
    #include <Windows.h>
    #pragma message("Adding library: Winmm.lib")
    #pragma comment(lib,"Winmm.lib")
#else
    #include <sys/time.h>
#endif

static int debugLevel=0;
static std::string lastError;

int getTimeDiffInMs(int lastTime)
{
#ifdef _WIN32
    int retVal=int(timeGetTime()&0x03ffffff);
#else
    struct timeval tv;
    DWORD result=0;
    if (gettimeofday(&tv,NULL)==0)
        result=(tv.tv_sec*1000+tv.tv_usec/1000)&0x03ffffff;
    int retVal=int(result);
#endif
    if (lastTime!=-1)
    {
        if (retVal<lastTime)
            retVal+=0x03ffffff-lastTime;
        else
            retVal-=lastTime;
    }
    return(retVal);
}

void ikSetVerbosity(int level)
{ // 0=none, 1=errors, 2=warnings, 3=info, 4=debug, 5=trace
    debugLevel=level;
}

class debugInfo
{
    public:
    debugInfo(const char* funcName)
    {
        if (debugLevel>=1)
        {
            _lastErrorSaved=lastError;
            lastError.clear();
        }
        if (debugLevel>=5)
        {
            _funcName=funcName;
            _lastErrorSaved=lastError;
            lastError.clear();
            printf("Coppelia Kinematics Routines: trace: --> %s\n",funcName);
        }
    }
    virtual ~debugInfo()
    {
        if (debugLevel>=1)
        {
            if (lastError.size()>0)
                printf("Coppelia Kinematics Routines: error: %s\n",lastError.c_str());
            else
                lastError=_lastErrorSaved;
        }
        if (debugLevel>=5)
            printf("Coppelia Kinematics Routines: trace: <-- %s\n",_funcName.c_str());
    }
    std::string _funcName;
    std::string _lastErrorSaved;
};

std::string ikGetLastError()
{
    std::string le(lastError);
    lastError.clear();
    return(le);
}

bool hasLaunched()
{
    bool retVal=CEnvironment::currentEnvironment!=nullptr;
    if (!retVal)
        lastError="Environment does not exist";
    return(retVal);
}

CikElement* getIkElementFromHandleOrTipDummy(const CikGroup* ikGroup,int ikElementHandle)
{
    CikElement* retVal=nullptr;
    bool fromHandle=(ikElementHandle&ik_handleflag_tipdummy)!=0;
    if (fromHandle)
    {
        ikElementHandle-=ik_handleflag_tipdummy;
        retVal=ikGroup->getIkElementWithTooltipHandle(ikElementHandle);
        if (retVal==nullptr)
            lastError="Invalid IK element tip dummy handle";
    }
    else
    {
        retVal=ikGroup->getIkElement(ikElementHandle);
        if (retVal==nullptr)
            lastError="Invalid IK element handle";
    }
    return(retVal);
}

bool ikCreateEnvironment(int* environmentHandle/*=nullptr*/,bool protectedEnvironment/*=false*/)
{
    debugInfo inf(__FUNCTION__);
    CEnvironment* newCt=new CEnvironment(protectedEnvironment);
    int eh=CEnvironment::addEnvironment(newCt);
    if (environmentHandle!=nullptr)
        environmentHandle[0]=eh;
    return(true);
}

bool ikEraseEnvironment(int* switchedEnvironmentHandle/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        int eh=CEnvironment::killEnvironment(CEnvironment::currentEnvironment->getHandle());
        if (switchedEnvironmentHandle!=nullptr)
            switchedEnvironmentHandle[0]=eh;
        retVal=true;
    }
    return(retVal);
}

void ikReleaseBuffer(void* buffer)
{
    debugInfo inf(__FUNCTION__);
    delete[] static_cast<simReal*>(buffer);
}

bool ikSwitchEnvironment(int handle,bool allowAlsoProtectedEnvironment/*=false*/)
{
    bool retVal=false;
    if ( (CEnvironment::currentEnvironment!=nullptr)&&(CEnvironment::currentEnvironment->getHandle()==handle) )
        retVal=true;
    else
    { // no debug msg if we don't have to switch
        debugInfo inf(__FUNCTION__);
        if (hasLaunched())
        {
            if (CEnvironment::switchToEnvironment(handle,allowAlsoProtectedEnvironment))
                retVal=true;
            else
                lastError="Invalid environment handle";
        }
    }
    return(retVal);
}

bool ikLoad(const unsigned char* data,size_t dataLength)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (CEnvironment::currentEnvironment->objectContainer->objectList.size()==0)&&(CEnvironment::currentEnvironment->ikGroupContainer->ikGroups.size()==0) )
        {
            if ((data!=nullptr)&&(dataLength!=0))
            {
                CSerialization ar(data,dataLength);
                CEnvironment::currentEnvironment->objectContainer->importKinematicsData(ar);
                CEnvironment::currentEnvironment->objectContainer->objectList.size();
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

bool ikGetObjects(size_t index,int* objectHandle/*=nullptr*/,std::string* objectName/*=nullptr*/,bool* isJoint/*=nullptr*/,int* jointType/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=CEnvironment::currentEnvironment->objectContainer->getObjectFromIndex(index);
        if (it!=nullptr)
        {
            if (objectHandle!=nullptr)
                objectHandle[0]=it->getObjectHandle();
            if (objectName!=nullptr)
                objectName[0]=it->getObjectName();
            if (isJoint!=nullptr)
                isJoint[0]=(it->getObjectType()==ik_objecttype_joint);
            if (it->getObjectType()==ik_objecttype_joint)
            {
                if (jointType!=nullptr)
                    jointType[0]=((CJoint*)it)->getJointType();
            }
            retVal=true;
        }
    }
    return(retVal);
}

bool ikGetObjectHandle(const char* objectName,int* objectHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=CEnvironment::currentEnvironment->objectContainer->getObject(objectName);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=CEnvironment::currentEnvironment->objectContainer->getObject(objectName);
        retVal=(it!=nullptr);
    }
    return(retVal);
}

bool ikGetObjectTransformation(int objectHandle,int relativeToObjectHandle,C7Vector* transf)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=CEnvironment::currentEnvironment->objectContainer->getObject(objectHandle);
        if (it!=nullptr)
        {
            if (relativeToObjectHandle==ik_handle_parent)
            {
                relativeToObjectHandle=-1;
                CSceneObject* parent=it->getParentObject();
                if (parent!=nullptr)
                    relativeToObjectHandle=parent->getObjectHandle();
            }
            CSceneObject* relObj=CEnvironment::currentEnvironment->objectContainer->getObject(relativeToObjectHandle);
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
    debugInfo inf(__FUNCTION__);
    C7Vector transf;
    bool retVal=ikGetObjectTransformation(objectHandle,relativeToObjectHandle,&transf);
    if (retVal)
        matrix[0]=transf.getMatrix();
    return(retVal);
}

bool ikSetObjectTransformation(int objectHandle,int relativeToObjectHandle,const C7Vector* transf)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=CEnvironment::currentEnvironment->objectContainer->getObject(objectHandle);
        if (it!=nullptr)
        {
            if (relativeToObjectHandle==ik_handle_parent)
            {
                relativeToObjectHandle=-1;
                CSceneObject* parent=it->getParentObject();
                if (parent!=nullptr)
                    relativeToObjectHandle=parent->getObjectHandle();
            }
            CSceneObject* relObj=CEnvironment::currentEnvironment->objectContainer->getObject(relativeToObjectHandle);
            if ( (relativeToObjectHandle==-1)||(relObj!=nullptr) )
            {
                if (relativeToObjectHandle==-1)
                    CEnvironment::currentEnvironment->objectContainer->setAbsoluteConfiguration(it->getObjectHandle(),transf[0],false);
                else
                {
                    C7Vector relTr(relObj->getCumulativeTransformationPart1());
                    C7Vector absTr(relTr*transf[0]);
                    CEnvironment::currentEnvironment->objectContainer->setAbsoluteConfiguration(it->getObjectHandle(),absTr,false);
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
    debugInfo inf(__FUNCTION__);
    C7Vector transf(matrix->getTransformation());
    return(ikSetObjectTransformation(objectHandle,relativeToObjectHandle,&transf));
}

bool ikGetJointPosition(int jointHandle,simReal* position)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if (it->getJointType()!=ik_jointtype_spherical)
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if (it->getJointType()!=ik_jointtype_spherical)
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupName);
        retVal=(it!=nullptr);
    }
    return(retVal);
}

bool ikGetIkGroupHandle(const char* ikGroupName,int* ikGroupHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupName);
        if (it!=nullptr)
        {
            ikGroupHandle[0]=it->getObjectHandle();
            retVal=true;
        }
        else
            lastError="IK group does not exist";
    }
    return(retVal);
}

bool ikCreateIkGroup(const char* ikGroupName/*=nullptr*/,int* ikGroupHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (ikGroupName==nullptr)||((strlen(ikGroupName)>0)&&(CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupName)==nullptr)) )
        {
            CikGroup* it=new CikGroup();
            if (ikGroupName!=nullptr)
                it->setObjectName(ikGroupName);
            it->setExplicitHandling(true);
            CEnvironment::currentEnvironment->ikGroupContainer->addIkGroup(it,false);
            ikGroupHandle[0]=it->getObjectHandle();
            retVal=true;
        }
        else
            lastError="Invalid IK group name";
    }
    return(retVal);
}

bool ikEraseIkGroup(int ikGroupHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            retVal=true;
            CEnvironment::currentEnvironment->ikGroupContainer->removeIkGroup(ikGroupHandle);
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikAddIkElement(int ikGroupHandle,int tipHandle,int* ikElementHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CDummy* dummy=CEnvironment::currentEnvironment->objectContainer->getDummy(tipHandle);
            if (dummy!=nullptr)
            {
                CikElement* ikElement=new CikElement(tipHandle);
                ikElementHandle[0]=ikGroup->addIkElement(ikElement);
                retVal=true;
            }
            else
                lastError="Invalid tip dummy handle";
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikEraseIkElement(int ikGroupHandle,int ikElementHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
            if (ikElement!=nullptr)
            {
                retVal=true;
                ikGroup->removeIkElement(ikElement->getIkElementHandle());
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikSetIkElementFlags(int ikGroupHandle,int ikElementHandle,int flags)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
            if (ikElement!=nullptr)
            {
                if (ikElement->getIsActive()!=((flags&1)!=0))
                    ikElement->setIsActive((flags&1)!=0);
                retVal=true;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetIkElementFlags(int ikGroupHandle,int ikElementHandle,int* flags)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
            if (ikElement!=nullptr)
            {
                retVal=true;
                flags[0]=0;
                if (ikElement->getIsActive())
                    flags[0]|=1;
            }
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikSetIkElementBase(int ikGroupHandle,int ikElementHandle,int baseHandle,int constraintsBaseHandle/*=-1*/)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
            if (ikElement!=nullptr)
            {
                CSceneObject* b1=CEnvironment::currentEnvironment->objectContainer->getObject(baseHandle);
                if ( (b1!=nullptr)||(baseHandle==-1) )
                {
                    CSceneObject* b2=CEnvironment::currentEnvironment->objectContainer->getObject(constraintsBaseHandle);
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

bool ikGetIkElementBase(int ikGroupHandle,int ikElementHandle,int* baseHandle,int* constraintsBaseHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
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

bool ikSetIkElementConstraints(int ikGroupHandle,int ikElementHandle,int constraints)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
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

bool ikGetIkElementConstraints(int ikGroupHandle,int ikElementHandle,int* constraints)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
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

bool ikSetIkElementPrecision(int ikGroupHandle,int ikElementHandle,simReal linearPrecision,simReal angularPrecision)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
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

bool ikGetIkElementPrecision(int ikGroupHandle,int ikElementHandle,simReal* linearPrecision,simReal* angularPrecision)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
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

bool ikSetIkElementWeights(int ikGroupHandle,int ikElementHandle,simReal linearWeight,simReal angularWeight)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
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

bool ikGetIkElementWeights(int ikGroupHandle,int ikElementHandle,simReal* linearWeight,simReal* angularWeight)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
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
    debugInfo inf(__FUNCTION__);
    simReal* retVal=nullptr;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
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

bool ikHandleIkGroup(int ikGroupHandle/*=ik_handle_all*/,int* result/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if ( (ikGroupHandle==ik_handle_all)||(it!=nullptr) )
        {
            if (ikGroupHandle<0)
                retVal=CEnvironment::currentEnvironment->ikGroupContainer->computeAllIkGroups(false);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    C7Vector transf;
    bool retVal=ikGetJointTransformation(jointHandle,&transf);
    if (retVal)
        matrix[0]=transf.getMatrix();
    return(retVal);
}

bool ikSetSphericalJointQuaternion(int jointHandle,const C4Vector* quaternion)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if (it->getJointType()==ik_jointtype_spherical)
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
    debugInfo inf(__FUNCTION__);
    C4Vector q(rotMatrix->getQuaternion());
    return(ikSetSphericalJointQuaternion(jointHandle,&q));
}

bool ikGetObjectParent(int objectHandle,int* parentObjectHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=CEnvironment::currentEnvironment->objectContainer->getObject(objectHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* it=CEnvironment::currentEnvironment->objectContainer->getObject(objectHandle);
        CSceneObject* parentIt=CEnvironment::currentEnvironment->objectContainer->getObject(parentObjectHandle);
        if (it!=nullptr)
        {
            if ( (parentIt!=nullptr)||(parentObjectHandle==-1) )
            {
                if (keepInPlace)
                    CEnvironment::currentEnvironment->objectContainer->makeObjectChildOf(it,parentIt);
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

bool ikCreateDummy(const char* dummyName/*=nullptr*/,int* dummyHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (dummyName==nullptr)||((strlen(dummyName)!=0)&&(CEnvironment::currentEnvironment->objectContainer->getObject(dummyName)==nullptr)) )
        {
            dummyHandle[0]=CEnvironment::currentEnvironment->objectContainer->createDummy(dummyName);
            retVal=true;
        }
        else
            lastError="Invalid object name";
    }
    return(retVal);
}

bool ikSetLinkedDummy(int dummyHandle,int linkedDummyHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CDummy* it=CEnvironment::currentEnvironment->objectContainer->getDummy(dummyHandle);
        if (it!=nullptr)
        {
            CDummy* it2=CEnvironment::currentEnvironment->objectContainer->getDummy(linkedDummyHandle);
            if ( (it2!=nullptr)||(linkedDummyHandle==-1) )
            {
                if (it2==nullptr)
                    it->setLinkedDummyHandle(-1,false);
                else
                {
                    it->setLinkedDummyHandle(linkedDummyHandle,false);
                    it->setLinkType(ik_linktype_ik_tip_target,false);
                }
                retVal=true;
            }
            else
                lastError="Invalid linked dummy handle";
        }
        else
            lastError="Invalid dummy handle";
    }
    return(retVal);
}

bool ikGetLinkedDummy(int dummyHandle,int* linkedDummyHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CDummy* it=CEnvironment::currentEnvironment->objectContainer->getDummy(dummyHandle);
        if (it!=nullptr)
        {
            linkedDummyHandle[0]=it->getLinkedDummyHandle();
            if (it->getLinkType()!=ik_linktype_ik_tip_target)
                linkedDummyHandle[0]=-1;
            retVal=true;
        }
        else
            lastError="Invalid dummy handle";
    }
    return(retVal);
}

bool ikCreateJoint(const char* jointName/*=nullptr*/,int jointType,int* jointHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (jointName==nullptr)||((strlen(jointName)!=0)&&(CEnvironment::currentEnvironment->objectContainer->getObject(jointName)==nullptr)) )
        {
            jointHandle[0]=CEnvironment::currentEnvironment->objectContainer->createJoint(jointName,jointType);
            retVal=true;
        }
        else
            lastError="Invalid joint handle";
    }
    return(retVal);
}

bool ikSetJointMode(int jointHandle,int jointMode)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            CJoint* it2=CEnvironment::currentEnvironment->objectContainer->getJoint(dependencyJointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* obj=CEnvironment::currentEnvironment->objectContainer->getObject(objectHandle);
        if (obj!=nullptr)
            retVal=CEnvironment::currentEnvironment->objectContainer->eraseObject(obj);
        else
            lastError="Invalid object handle";
    }
    return(retVal);
}

bool ikGetJointInterval(int jointHandle,bool* cyclic,simReal* intervalMinAndRange)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
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

/*
bool ikGetIkGroupLimitThresholds(int ikGroupHandle,simReal* linearAndAngularThresholds)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
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
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
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
*/

bool ikSetIkGroupFlags(int ikGroupHandle,int flags)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            if (it->getActive()!=((flags&1)!=0))
                it->setActive((flags&1)!=0);
//            if (it->getCorrectJointLimits()!=((flags&2)!=0))
//                it->setCorrectJointLimits((flags&2)!=0);
            if (it->getIgnoreMaxStepSizes()!=((flags&2)!=0))
                it->setIgnoreMaxStepSizes((flags&2)!=0);
            if (it->getRestoreIfPositionNotReached()!=((flags&4)!=0))
                it->setRestoreIfPositionNotReached((flags&4)!=0);
            if (it->getRestoreIfOrientationNotReached()!=((flags&8)!=0))
                it->setRestoreIfOrientationNotReached((flags&8)!=0);
            retVal=true;
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

bool ikGetIkGroupFlags(int ikGroupHandle,int* flags)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            retVal=true;
            flags[0]=0;
            if (it->getActive())
                flags[0]=flags[0]|1;
//            if (it->getCorrectJointLimits())
//                flags[0]=flags[0]|2;
            if (it->getIgnoreMaxStepSizes())
                flags[0]=flags[0]|2;
            if (it->getRestoreIfPositionNotReached())
                flags[0]=flags[0]|4;
            if (it->getRestoreIfOrientationNotReached())
                flags[0]=flags[0]|8;
        }
        else
            lastError="Invalid IK group handle";
    }
    return(retVal);
}

int ikGetConfigForTipPose(int ikGroupHandle,size_t jointCnt,const int* jointHandles,simReal thresholdDist,int maxIterations,simReal* retConfig,const simReal* metric/*=nullptr*/,bool(*validationCallback)(simReal*)/*=nullptr*/,const int* jointOptions/*=nullptr*/,const simReal* lowLimits/*=nullptr*/,const simReal* ranges/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__);
    int retVal=-1;
    std::vector<simReal> conf(jointCnt);
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
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
                CJoint* aJoint=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandles[i]);
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
                if (ikGroup->getIkElementCount()>0)
                {
                    for (size_t i=0;i<ikGroup->getIkElementCount();i++)
                    {
                        CikElement* ikEl=ikGroup->getIkElementFromIndex(i);
                        CDummy* tip=CEnvironment::currentEnvironment->objectContainer->getDummy(ikEl->getTipHandle());
                        CDummy* target=CEnvironment::currentEnvironment->objectContainer->getDummy(ikEl->getTargetHandle());
                        CSceneObject* base=nullptr;
                        if (ikEl->getAltBaseHandleForConstraints()!=-1)
                            base=CEnvironment::currentEnvironment->objectContainer->getObject(ikEl->getAltBaseHandleForConstraints());
                        else
                            base=CEnvironment::currentEnvironment->objectContainer->getObject(ikEl->getBaseHandle());
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
                for (size_t i=0;i<CEnvironment::currentEnvironment->objectContainer->jointList.size();i++)
                {
                    CJoint* aj=CEnvironment::currentEnvironment->objectContainer->getJoint(CEnvironment::currentEnvironment->objectContainer->jointList[i]);
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
                for (size_t i=0;i<ikGroup->getIkElementCount();i++)
                    enabledElements.push_back(ikGroup->getIkElementFromIndex(i)->getIsActive());

                // Set the correct mode for the joints involved:
                for (size_t i=0;i<jointCnt;i++)
                {
                    if ( (jointOptions==nullptr)||((jointOptions[i]&1)==0) )
                        joints[i]->setJointMode(ik_jointmode_ik);
                    else
                        joints[i]->setJointMode(ik_jointmode_dependent);
                }

                // do the calculation:
                int startTime=0;
                int mi=999999999;
                if (maxIterations>=0)
                    mi=maxIterations;
                else
                    startTime=getTimeDiffInMs(-1);
                for (int iterationCnt=0;iterationCnt<mi;iterationCnt++)
                {
                    // 1. Pick a random state:
                    for (size_t i=0;i<jointCnt;i++)
                        joints[i]->setPosition(minVals[i]+(rand()/simReal(RAND_MAX))*rangeVals[i]);

                    // 2. Check distances between tip and target pairs (there might be several pairs!):
                    simReal cumulatedDist=simZero;
                    for (size_t el=0;el<ikGroup->getIkElementCount();el++)
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
                        if (ik_result_success==ikGroup->computeGroupIk(true))
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
                    if (maxIterations<0)
                    {
                        if (getTimeDiffInMs(startTime)>-maxIterations)
                            break;
                    }
                }

                if (!ikGroupWasActive)
                    ikGroup->setActive(false);

                // Restore the IK element activation state:
                for (size_t i=0;i<ikGroup->getIkElementCount();i++)
                    ikGroup->getIkElementFromIndex(i)->setIsActive(enabledElements[i]);

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
