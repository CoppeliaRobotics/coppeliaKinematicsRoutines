#include "ik.h"
#include "environment.h"
#include <stdio.h>
#include <string.h>
#ifdef _WIN32
    #include <Windows.h>
    #pragma message("Adding library: Winmm.lib")
    #pragma comment(lib,"Winmm.lib")
#else
    #include <sys/time.h>
#endif

static int _verbosityLevel=0;
static std::string _lastError;
static bool(*_logCallback)(int,const char*,const char*)=nullptr;

void _setLastError(const char* errStr,const char* substr1/*=nullptr*/,const char* substr2/*=nullptr*/)
{
    char buff[500];
    if (substr1!=nullptr)
    {
        if (substr2!=nullptr)
            snprintf(buff,sizeof(buff),errStr,substr1,substr2);
        else
            snprintf(buff,sizeof(buff),errStr,substr1);
    }
    else
        strcpy(buff,errStr);
   _lastError=buff;
}

void _setLastError(const char* errStr,int intVal1,int intVal2/*=-1*/)
{
    char buff[500];
    snprintf(buff,sizeof(buff),errStr,intVal1,intVal2);
    _lastError=buff;
}

int getTimeDiffInMs(int lastTime)
{
#ifdef _WIN32
    int retVal=int(timeGetTime()&0x03ffffff);
#else
    struct timeval tv;
    int result=0;
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

void ikSetLogCallback(bool(*logCallback)(int,const char*,const char*))
{
    _logCallback=logCallback;
}

void ikSetVerbosity(int level)
{ // verbosity is: 0=none, 1=errors, 2=warnings, 3=infos, 4=debug, 5=trace
    _verbosityLevel=level;
}

class debugInfo
{
    public:
    debugInfo(const char* funcName,const char* strArg=nullptr)
    {
        if (_logCallback!=nullptr)
        {
            if (_verbosityLevel>=1)
            {
                _lastErrorSaved=_lastError;
                _lastError.clear();
            }
            if (_verbosityLevel>=5)
            {
                _funcName=funcName;
                std::string msg("--> ");
                msg+=funcName;

                if (strArg!=nullptr)
                {
                    msg+=" (strArg: ";
                    msg+=strArg;
                    msg+=")";
                }
                _logCallback(5,_funcName.c_str(),msg.c_str());
            }
        }
    }
    debugInfo(const char* funcName,int intArg1,int intArg2=-123456789,int intArg3=-123456789,int intArg4=-123456789)
    {
        if (_logCallback!=nullptr)
        {
            if (_verbosityLevel>=1)
            {
                _lastErrorSaved=_lastError;
                _lastError.clear();
            }
            if (_verbosityLevel>=5)
            {
                _funcName=funcName;
                std::string msg("--> ");
                msg+=funcName;
                msg+=" (intArgs: ";
                msg+=std::to_string(intArg1);

                if (intArg2!=-123456789)
                {
                    msg+=", ";
                    msg+=std::to_string(intArg2);
                    if (intArg3!=-123456789)
                    {
                        msg+=", ";
                        msg+=std::to_string(intArg3);
                        if (intArg4!=-123456789)
                        {
                            msg+=", ";
                            msg+=std::to_string(intArg4);
                        }
                    }
                }
                msg+=")";
                _logCallback(5,_funcName.c_str(),msg.c_str());
            }
        }
    }
    virtual ~debugInfo()
    {
        if (_logCallback!=nullptr)
        {
            if (_verbosityLevel>=1)
            {
                if (_lastError.size()>0)
                {
                    if (_logCallback(1,_funcName.c_str(),_lastError.c_str()))
                        _lastError=_lastErrorSaved;
                }
                else
                    _lastError=_lastErrorSaved;
            }
            if (_verbosityLevel>=5)
            {
                std::string msg("<-- ");
                msg+=_funcName;
                _logCallback(5,_funcName.c_str(),msg.c_str());
            }
        }
    }
    void debugMsg(const char* msg,int intArg1=-123456789)
    {
        if (_logCallback!=nullptr)
        {
            std::string mm(msg);
            if (intArg1!=-123456789)
                mm+=std::to_string(intArg1);
            _logCallback(4,_funcName.c_str(),mm.c_str());
        }
    }


    std::string _funcName;
    std::string _lastErrorSaved;
};

std::string ikGetLastError()
{
    std::string le(_lastError);
    _lastError.clear();
    return(le);
}

bool hasLaunched()
{
    bool retVal=CEnvironment::currentEnvironment!=nullptr;
    if (!retVal)
        _setLastError("Environment does not exist");
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
            _setLastError("Invalid IK element tip dummy handle: %i",ikElementHandle);
    }
    else
    {
        retVal=ikGroup->getIkElement(ikElementHandle);
        if (retVal==nullptr)
            _setLastError("Invalid IK element handle: %i",ikElementHandle);
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

bool ikDuplicateEnvironment(int* duplicateEnvironmentHandle)
{
    debugInfo inf(__FUNCTION__);
    bool retVal=false;
    if (hasLaunched())
    {
        int currentEnvHandle=CEnvironment::currentEnvironment->getHandle();
        CEnvironment* duplicate=CEnvironment::currentEnvironment->copyYourself();
        duplicateEnvironmentHandle[0]=CEnvironment::addEnvironment(duplicate);
        CEnvironment::switchToEnvironment(currentEnvHandle,true);
        retVal=true;
    }
    return(retVal);
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
    delete[] static_cast<double*>(buffer);
}

bool ikSwitchEnvironment(int handle,bool allowAlsoProtectedEnvironment/*=false*/)
{
    bool retVal=false;
    if ( (CEnvironment::currentEnvironment!=nullptr)&&(CEnvironment::currentEnvironment->getHandle()==handle) )
        retVal=true;
    else
    { // no debug msg if we don't have to switch
        debugInfo inf(__FUNCTION__,handle);
        if (hasLaunched())
        {
            if (CEnvironment::switchToEnvironment(handle,allowAlsoProtectedEnvironment))
                retVal=true;
            else
                _setLastError("Invalid environment handle: %i",handle);
        }
    }
    return(retVal);
}

unsigned char* ikSave(size_t* dataLength)
{
    debugInfo inf(__FUNCTION__);
    unsigned char* retVal=nullptr;
    if (hasLaunched())
    {
        if (CEnvironment::currentEnvironment->objectContainer->objectList.size()!=0)
        {
            CSerialization ar;
            CEnvironment::currentEnvironment->objectContainer->importExportKinematicsData(ar);
            unsigned char* data=ar.getWriteBuffer(dataLength[0]);
            retVal=new unsigned char[dataLength[0]];
            for (size_t i=0;i<dataLength[0];i++)
                retVal[i]=data[i];
        }
        else
            _setLastError("Environment is empty");
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
                CEnvironment::currentEnvironment->objectContainer->importExportKinematicsData(ar);
                retVal=true;
            }
            else
                _setLastError("Invalid arguments");
        }
        else
            _setLastError("Environment must be empty");
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
    debugInfo inf(__FUNCTION__,objectName);
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
            _setLastError("Object does not exist: %s",objectName);
    }
    return(retVal);
}

bool ikDoesObjectExist(const char* objectName)
{
    debugInfo inf(__FUNCTION__,objectName);
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
    debugInfo inf(__FUNCTION__,objectHandle,relativeToObjectHandle);
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
                    C7Vector relTr(relObj->getCumulativeTransformation());
                    transf[0]=relTr.getInverse()*it->getCumulativeTransformationPart1();
                }
                retVal=true;
            }
            else
                _setLastError("Invalid arguments");
        }
        else
            _setLastError("Invalid object handle: %i",objectHandle);
    }
    return(retVal);
}

bool ikGetObjectMatrix(int objectHandle,int relativeToObjectHandle,C4X4Matrix* matrix)
{
    debugInfo inf(__FUNCTION__,objectHandle,relativeToObjectHandle);
    C7Vector transf;
    bool retVal=ikGetObjectTransformation(objectHandle,relativeToObjectHandle,&transf);
    if (retVal)
        matrix[0]=transf.getMatrix();
    return(retVal);
}

bool ikSetObjectTransformation(int objectHandle,int relativeToObjectHandle,const C7Vector* transf)
{
    debugInfo inf(__FUNCTION__,objectHandle,relativeToObjectHandle);
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
                    C7Vector relTr(relObj->getCumulativeTransformation());
                    C7Vector absTr(relTr*transf[0]);
                    CEnvironment::currentEnvironment->objectContainer->setAbsoluteConfiguration(it->getObjectHandle(),absTr,false);
                }
                retVal=true;
            }
            else
                _setLastError("Invalid arguments");
        }
        else
            _setLastError("Invalid object handle: %i",objectHandle);
    }
    return(retVal);
}

bool ikSetObjectMatrix(int objectHandle,int relativeToObjectHandle,const C4X4Matrix* matrix)
{
    debugInfo inf(__FUNCTION__,objectHandle,relativeToObjectHandle);
    C7Vector transf(matrix->getTransformation());
    return(ikSetObjectTransformation(objectHandle,relativeToObjectHandle,&transf));
}

bool ikGetJointPosition(int jointHandle,double* position)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
                _setLastError("Invalid call with spherical joint");
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikSetJointPosition(int jointHandle,double position)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
                _setLastError("Invalid call with spherical joint");
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikDoesIkGroupExist(const char* ikGroupName)
{
    debugInfo inf(__FUNCTION__,ikGroupName);
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
    debugInfo inf(__FUNCTION__,ikGroupName);
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
            _setLastError("IK group does not exist: %s",ikGroupName);
    }
    return(retVal);
}

bool ikCreateIkGroup(const char* ikGroupName/*=nullptr*/,int* ikGroupHandle)
{
    debugInfo inf(__FUNCTION__,ikGroupName);
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (ikGroupName==nullptr)||((strlen(ikGroupName)>0)&&(CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupName)==nullptr)) )
        {
            std::string dn;
            if (ikGroupName==nullptr)
            {
                int nb=0;
                while (true)
                {
                    dn="ikGroup";
                    dn+=std::to_string(nb);
                    if (CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(dn.c_str())==nullptr)
                        break;
                    nb++;
                }
            }
            else
                dn=ikGroupName;

            CikGroup* it=new CikGroup();
            it->setObjectName(dn.c_str());
            CEnvironment::currentEnvironment->ikGroupContainer->addIkGroup(it,false);
            ikGroupHandle[0]=it->getObjectHandle();
            retVal=true;
        }
        else
            _setLastError("Invalid IK group name: %s",ikGroupName);
    }
    inf.debugMsg("handle: ",ikGroupHandle[0]);
    return(retVal);
}

bool ikEraseIkGroup(int ikGroupHandle)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle);
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
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikAddIkElement(int ikGroupHandle,int tipHandle,int* ikElementHandle)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,tipHandle);
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
                _setLastError("Invalid tip dummy handle: %i",tipHandle);
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    inf.debugMsg("handle: ",ikElementHandle[0]);
    return(retVal);
}

bool ikEraseIkElement(int ikGroupHandle,int ikElementHandle)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle);
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
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikSetIkElementFlags(int ikGroupHandle,int ikElementHandle,int flags)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle);
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
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikGetIkElementFlags(int ikGroupHandle,int ikElementHandle,int* flags)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle);
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
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikSetIkElementBase(int ikGroupHandle,int ikElementHandle,int baseHandle,int constraintsBaseHandle/*=-1*/)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle,baseHandle,constraintsBaseHandle);
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
                        _setLastError("Invalid constraints base handle: %i",constraintsBaseHandle);
                }
                else
                    _setLastError("Invalid base handle: %i",baseHandle);
            }
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikGetIkElementBase(int ikGroupHandle,int ikElementHandle,int* baseHandle,int* constraintsBaseHandle)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle);
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
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikSetIkElementConstraints(int ikGroupHandle,int ikElementHandle,int constraints)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle,constraints);
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
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikGetIkElementConstraints(int ikGroupHandle,int ikElementHandle,int* constraints)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle);
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
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikSetIkElementPrecision(int ikGroupHandle,int ikElementHandle,double linearPrecision,double angularPrecision)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
            if (ikElement!=nullptr)
            {
                double p[2]={linearPrecision,angularPrecision};
                ikElement->setPrecisions(p);
                retVal=true;
            }
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikGetIkElementPrecision(int ikGroupHandle,int ikElementHandle,double* linearPrecision,double* angularPrecision)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
            if (ikElement!=nullptr)
            {
                double p[2];
                ikElement->getPrecisions(p);
                linearPrecision[0]=p[0];
                angularPrecision[0]=p[1];
                retVal=true;
            }
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikSetIkElementWeights(int ikGroupHandle,int ikElementHandle,double linearWeight,double angularWeight)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
            if (ikElement!=nullptr)
            {
                double w[2]={linearWeight,angularWeight};
                ikElement->setWeights(w);
                retVal=true;
            }
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikGetIkElementWeights(int ikGroupHandle,int ikElementHandle,double* linearWeight,double* angularWeight)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,ikElementHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            CikElement* ikElement=getIkElementFromHandleOrTipDummy(ikGroup,ikElementHandle);
            if (ikElement!=nullptr)
            {
                double w[2];
                ikElement->getWeights(w);
                linearWeight[0]=w[0];
                angularWeight[0]=w[1];
                retVal=true;
            }
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikComputeJacobian(int baseHandle,int jointHandle,int constraints,const C7Vector* tipPose,const C7Vector* targetPose,const C7Vector* taltBasePose,std::vector<double>* jacobian,std::vector<double>* errorVect)
{
    bool retVal=false;
    if (hasLaunched())
    {
        CMatrix jacob;
        CMatrix errVect;
        CDummy* tip=CEnvironment::currentEnvironment->objectContainer->getDummy(CEnvironment::currentEnvironment->objectContainer->createDummy(nullptr));
        CDummy* target=CEnvironment::currentEnvironment->objectContainer->getDummy(CEnvironment::currentEnvironment->objectContainer->createDummy(nullptr));
        tip->setLocalTransformation(*tipPose);
        target->setLocalTransformation(*targetPose);
        tip->setLinkedDummyHandle(target->getObjectHandle(),true);
        CJoint* tipJoint=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (tipJoint!=nullptr)
        {
            CEnvironment::currentEnvironment->objectContainer->makeObjectChildOf(tip,tipJoint);
            if (CikElement::getJacobian(jacob,errVect,tip->getObjectHandle(),baseHandle,constraints,taltBasePose,1.0,nullptr,nullptr,nullptr))
            {
                if (jacobian!=nullptr)
                    jacobian->assign(jacob.data.begin(),jacob.data.end());
                if (errorVect!=nullptr)
                    errorVect->assign(errVect.data.begin(),errVect.data.end());
                retVal=true;
            }
            else
                _setLastError("Failed getting the Jacobian. Are handles valid?");
        }
        else
            _setLastError("Failed getting the Jacobian. Are handles valid?");
        CEnvironment::currentEnvironment->objectContainer->eraseObject(target);
        CEnvironment::currentEnvironment->objectContainer->eraseObject(tip);
    }
    return(retVal);
}

bool ikComputeJacobian_old(int ikGroupHandle,int options,bool* success/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,options);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            bool succ=it->computeOnlyJacobian_old(options);
            retVal=true;
            if (success!=nullptr)
                success[0]=succ;
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

double* ikGetJacobian_old(int ikGroupHandle,size_t* matrixSize)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle);
    double* retVal=nullptr;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            const double* b=it->getLastJacobianData_old(matrixSize);
            if (b!=nullptr)
            {
                retVal=new double[size_t(matrixSize[0]*matrixSize[1])];
                for (size_t i=0;i<size_t(matrixSize[0]*matrixSize[1]);i++)
                    retVal[i]=b[i];
            }
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikGetManipulability_old(int ikGroupHandle,double* manip)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            double b=it->getLastManipulabilityValue_old(retVal);
            if (retVal)
                manip[0]=b;
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikHandleIkGroup(int ikGroupHandle/*=ik_handle_all*/,int* result/*=nullptr*/,bool(*cb)(const int*,std::vector<double>*,const int*,const int*,const int*,const int*,std::vector<double>*,double*)/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle);
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
                if (it->getExplicitHandling_old())
                {
                    int res=it->computeGroupIk(false,cb);
                    if (result!=nullptr)
                        result[0]=res;
                    retVal=true;
                }
                else
                    _setLastError("IK group cannot explicitely be handled");
            }
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikGetJointTransformation(int jointHandle,C7Vector* transf)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikGetJointMatrix(int jointHandle,C4X4Matrix* matrix)
{
    debugInfo inf(__FUNCTION__,jointHandle);
    C7Vector transf;
    bool retVal=ikGetJointTransformation(jointHandle,&transf);
    if (retVal)
        matrix[0]=transf.getMatrix();
    return(retVal);
}

bool ikSetSphericalJointQuaternion(int jointHandle,const C4Vector* quaternion)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
                _setLastError("Joint is not spherical");
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikSetSphericalJointMatrix(int jointHandle,const C3X3Matrix* rotMatrix)
{
    debugInfo inf(__FUNCTION__,jointHandle);
    C4Vector q(rotMatrix->getQuaternion());
    return(ikSetSphericalJointQuaternion(jointHandle,&q));
}

bool ikGetObjectParent(int objectHandle,int* parentObjectHandle)
{
    debugInfo inf(__FUNCTION__,objectHandle);
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
            _setLastError("Invalid object handle: %i",objectHandle);
    }
    return(retVal);
}

bool ikSetObjectParent(int objectHandle,int parentObjectHandle,bool keepInPlace)
{
    debugInfo inf(__FUNCTION__,objectHandle,parentObjectHandle);
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
                _setLastError("Invalid parent object handle: %i",parentObjectHandle);
        }
        else
            _setLastError("Invalid object handle: %i",objectHandle);
    }
    return(retVal);
}

bool ikCreateDummy(const char* dummyName/*=nullptr*/,int* dummyHandle)
{
    debugInfo inf(__FUNCTION__,dummyName);
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (dummyName==nullptr)||((strlen(dummyName)!=0)&&(CEnvironment::currentEnvironment->objectContainer->getObject(dummyName)==nullptr)) )
        {
            std::string dn;
            if (dummyName==nullptr)
            {
                int nb=0;
                while (true)
                {
                    dn="dummy";
                    dn+=std::to_string(nb);
                    if (CEnvironment::currentEnvironment->objectContainer->getObject(dn.c_str())==nullptr)
                        break;
                    nb++;
                }
            }
            else
                dn=dummyName;
            dummyHandle[0]=CEnvironment::currentEnvironment->objectContainer->createDummy(dn.c_str());
            retVal=true;
        }
        else
            _setLastError("Invalid object name: %s",dummyName);
    }
    inf.debugMsg("handle: ",dummyHandle[0]);
    return(retVal);
}

bool ikSetLinkedDummy(int dummyHandle,int linkedDummyHandle)
{
    debugInfo inf(__FUNCTION__,dummyHandle,linkedDummyHandle);
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
                    it->setLinkedDummyHandle(linkedDummyHandle,false);
                retVal=true;
            }
            else
                _setLastError("Invalid linked dummy handle: %i",linkedDummyHandle);
        }
        else
            _setLastError("Invalid dummy handle: %i",dummyHandle);
    }
    return(retVal);
}

bool ikGetLinkedDummy(int dummyHandle,int* linkedDummyHandle)
{
    debugInfo inf(__FUNCTION__,dummyHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CDummy* it=CEnvironment::currentEnvironment->objectContainer->getDummy(dummyHandle);
        if (it!=nullptr)
        {
            linkedDummyHandle[0]=it->getLinkedDummyHandle();
            retVal=true;
        }
        else
            _setLastError("Invalid dummy handle: %i",dummyHandle);
    }
    return(retVal);
}

bool ikCreateJoint(const char* jointName/*=nullptr*/,int jointType,int* jointHandle)
{
    debugInfo inf(__FUNCTION__,jointName);
    bool retVal=false;
    if (hasLaunched())
    {
        if ( (jointName==nullptr)||((strlen(jointName)!=0)&&(CEnvironment::currentEnvironment->objectContainer->getObject(jointName)==nullptr)) )
        {
            std::string dn;
            if (jointName==nullptr)
            {
                int nb=0;
                while (true)
                {
                    dn="joint";
                    dn+=std::to_string(nb);
                    if (CEnvironment::currentEnvironment->objectContainer->getObject(dn.c_str())==nullptr)
                        break;
                    nb++;
                }
            }
            else
                dn=jointName;
            jointHandle[0]=CEnvironment::currentEnvironment->objectContainer->createJoint(dn.c_str(),jointType);
            retVal=true;
        }
        else
            _setLastError("Invalid joint name: %s",jointName);
    }
    inf.debugMsg("handle: ",jointHandle[0]);
    return(retVal);
}

bool ikGetJointType(int jointHandle,int* theType)
{
    debugInfo inf(__FUNCTION__,jointHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            theType[0]=it->getJointType();
            retVal=true;
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikSetJointMode(int jointHandle,int jointMode)
{
    debugInfo inf(__FUNCTION__,jointHandle,jointMode);
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
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikGetJointMode(int jointHandle,int* mode)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikSetJointInterval(int jointHandle,bool cyclic,const double* intervalMinAndRange/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__,jointHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            double previousPos=it->getPosition();
            if (it->getPositionIsCyclic()!=cyclic)
                it->setPositionIsCyclic(cyclic);
            if (intervalMinAndRange!=nullptr)
            {
                if ( fabs(it->getPositionIntervalMin()-intervalMinAndRange[0])>0.00001 )
                    it->setPositionIntervalMin(intervalMinAndRange[0]);
                if ( fabs(it->getPositionIntervalRange()-intervalMinAndRange[1])>0.00001 )
                    it->setPositionIntervalRange(intervalMinAndRange[1]);
            }
            it->setPosition(previousPos);
            retVal=true;
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikSetJointScrewPitch(int jointHandle,double pitch)
{
    debugInfo inf(__FUNCTION__,jointHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if ( fabs(it->getScrewPitch()-pitch)>0.000001 )
                it->setScrewPitch(pitch);
            retVal=true;
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikSetJointIkWeight(int jointHandle,double ikWeight)
{
    debugInfo inf(__FUNCTION__,jointHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if ( fabs(it->getIkWeight()-ikWeight)>0.0001 )
                it->setIkWeight(ikWeight);
            retVal=true;
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikGetJointIkWeight(int jointHandle,double* ikWeight)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikSetJointLimitMargin(int jointHandle,double m)
{
    debugInfo inf(__FUNCTION__,jointHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            it->setLimitMargin(m);
            retVal=true;
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikGetJointLimitMargin(int jointHandle,double* m)
{
    debugInfo inf(__FUNCTION__,jointHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            m[0]=it->getLimitMargin();
            retVal=true;
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikSetJointMaxStepSize(int jointHandle,double maxStepSize)
{
    debugInfo inf(__FUNCTION__,jointHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            if ( fabs(it->getMaxStepSize()-maxStepSize)>0.00001 )
                it->setMaxStepSize(maxStepSize);
            retVal=true;
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikGetJointMaxStepSize(int jointHandle,double* maxStepSize)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikSetJointDependency(int jointHandle,int dependencyJointHandle,double offset/*=0.0*/,double mult/*=1.0*/,double(*cb)(int ikEnv,int slaveJoint,double masterPos)/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__,jointHandle,dependencyJointHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CJoint* it=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandle);
        if (it!=nullptr)
        {
            CJoint* it2=CEnvironment::currentEnvironment->objectContainer->getJoint(dependencyJointHandle);
            if ( (it2!=nullptr)||(dependencyJointHandle==-1) )
            {
                if (it->getDependencyJointHandle()!=dependencyJointHandle)
                    it->setDependencyJointHandle(dependencyJointHandle);
                if (dependencyJointHandle!=-1)
                {
                    it->setDependencyJointMult(mult);
                    it->setDependencyJointAdd(offset);
                    it->setDependencyJointCallback(cb);
                }
                retVal=true;
            }
            else
                _setLastError("Invalid dependency joint handle: %i",dependencyJointHandle);
        }
        else
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikGetJointDependency(int jointHandle,int* dependencyJointHandle,double* offset,double* mult)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
            _setLastError("Invalid joint handle: %i",jointHandle);
    }
    return(retVal);
}

bool ikEraseObject(int objectHandle)
{
    debugInfo inf(__FUNCTION__,objectHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CSceneObject* obj=CEnvironment::currentEnvironment->objectContainer->getObject(objectHandle);
        if (obj!=nullptr)
            retVal=CEnvironment::currentEnvironment->objectContainer->eraseObject(obj);
        else
            _setLastError("Invalid object handle: %i",objectHandle);
    }
    return(retVal);
}

bool ikGetJointInterval(int jointHandle,bool* cyclic,double* intervalMinAndRange)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
            _setLastError("Joint does not exist: %i",jointHandle);
    }
    return(retVal);
}

bool ikGetJointScrewPitch(int jointHandle,double* pitch)
{
    debugInfo inf(__FUNCTION__,jointHandle);
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
            _setLastError("Joint does not exist: %i",jointHandle);
    }
    return(retVal);
}

bool ikGetIkGroupJointLimitHits(int ikGroupHandle,std::vector<int>* jointHandles,std::vector<double>* underOrOvershots)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            it->getJointLimitHits(jointHandles,underOrOvershots);
            retVal=true;
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikGetIkGroupCalculation(int ikGroupHandle,int* method,double* damping,int* maxIterations)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle);
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
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikSetIkGroupCalculation(int ikGroupHandle,int method,double damping,int maxIterations)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,method);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            if (it->getCalculationMethod()!=method)
                it->setCalculationMethod(method);
            if ( fabs(it->getDlsFactor()-damping)>0.0001 )
                it->setDlsFactor(damping);
            if (it->getMaxIterations()!=maxIterations)
                it->setMaxIterations(maxIterations);
            retVal=true;
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikSetIkGroupFlags(int ikGroupHandle,int flags)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle,flags);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            int options=it->getOptions();
            if ((flags&1)!=0)
                options=options&~1; // enabled
            else
                options=options|1; // disabled
            if ((flags&2)!=0)
                options=options&~2;
            else
                options=options|2;
            if ((flags&4)!=0)
                options=options|4;
            else
                options=options&~4;
            if ((flags&8)!=0)
                options=options|8;
            else
                options=options&~8;
            if ((flags&16)!=0)
                options=options|16;
            else
                options=options&~16;
            if ((flags&32)!=0)
                options=options|32;
            else
                options=options&~32;
            if ((flags&64)!=0)
                options=options|64;
            else
                options=options&~64;
            it->setOptions(options);
            retVal=true;
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

bool ikGetIkGroupFlags(int ikGroupHandle,int* flags)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle);
    bool retVal=false;
    if (hasLaunched())
    {
        CikGroup* it=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (it!=nullptr)
        {
            retVal=true;
            flags[0]=0;
            int options=it->getOptions();
            if ((options&1)==0)
                flags[0]=flags[0]|1;
            if ((options&2)==0)
                flags[0]=flags[0]|2;
            if ((options&4)!=0)
                flags[0]=flags[0]|4;
            if ((options&8)!=0)
                flags[0]=flags[0]|8;
            if ((options&16)!=0)
                flags[0]=flags[0]|16;
            if ((options&32)!=0)
                flags[0]=flags[0]|32;
            if ((options&64)!=0)
                flags[0]=flags[0]|64;
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
    }
    return(retVal);
}

int ikFindConfig(int ikGroupHandle,size_t jointCnt,const int* jointHandles,double thresholdDist,int maxTimeInMs,double* retConfig,const double* metric/*=nullptr*/,bool(*validationCallback)(double*)/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle);
    std::vector<double> retConf(jointCnt);
    if (!hasLaunched())
        return(-1);
    CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
    if (ikGroup==nullptr)
    {
        _setLastError("Invalid IK group handle: %i",ikGroupHandle);
        return(-1);
    }
    if ( ((ikGroup->getOptions()&1)!=0)||(ikGroup->getIkElementCount()==0) )
    {
        _setLastError("Invalid IK group");
        return(-1);
    }
    if (jointCnt<=0)
    {
        _setLastError("Invalid number of selected joints");
        return(-1);
    }
    std::vector<CJoint*> selectedJoints;
    for (size_t i=0;i<jointCnt;i++)
    {
        CJoint* aJoint=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandles[i]);
        if (aJoint==nullptr)
        {
            _setLastError("Found invalid joint handle");
            return(-1);
        }
        selectedJoints.push_back(aJoint);
    }

    const double _defaultMetric[4]={1.0,1.0,1.0,0.1};
    const double* theMetric=_defaultMetric;
    if (metric!=nullptr)
        theMetric=metric;

    std::vector<CDummy*> tips;
    std::vector<CDummy*> targets;
    std::vector<CSceneObject*> bases;
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
        {
            _setLastError("Found invalid IK element");
            return(-1);
        }
        tips.push_back(tip);
        targets.push_back(target);
        bases.push_back(base);
    }

    std::vector<CJoint*> allJoints;
    std::vector<double> allJointInitValues;
    std::vector<double> allJointMinVals;
    std::vector<double> allJointRangeVals;
    for (size_t i=0;i<CEnvironment::currentEnvironment->objectContainer->jointList.size();i++)
    {
        CJoint* aJoint=CEnvironment::currentEnvironment->objectContainer->getJoint(CEnvironment::currentEnvironment->objectContainer->jointList[i]);
        allJoints.push_back(aJoint);
        allJointInitValues.push_back(aJoint->getPosition());
        double l=aJoint->getPositionIntervalMin();
        double r=aJoint->getPositionIntervalRange();
        if (aJoint->getPositionIsCyclic())
        {
            l=-piValue;
            r=piValT2;
        }
        if (aJoint->getJointMode()!=ik_jointmode_ik)
        {
            l=aJoint->getPosition();
            r=0.0;
        }
        allJointMinVals.push_back(l);
        allJointRangeVals.push_back(r);
    }

    int startTime=getTimeDiffInMs(-1);
    int retVal=0;
    while (getTimeDiffInMs(startTime)<maxTimeInMs)
    {
        // 1. Pick a random state:
        for (size_t i=0;i<allJoints.size();i++)
        {
            if (allJointRangeVals[i]!=0.0)
                allJoints[i]->setPosition(allJointMinVals[i]+(rand()/double(RAND_MAX))*allJointRangeVals[i]);
        }

        // 2. Check distances between tip and target pairs (there might be several pairs!):
        double cumulatedDist=0.0;
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
            double angle=tipTr.Q.getAngleBetweenQuaternions(targetTr.Q)*theMetric[3];
            cumulatedDist+=sqrt(dx(0)*dx(0)+dx(1)*dx(1)+dx(2)*dx(2)+angle*angle);
        }

        // 3. If distance<=threshold, try to perform IK:
        if (cumulatedDist<=thresholdDist)
        {
            if (ik_result_success==ikGroup->computeGroupIk(true,nullptr)) // allow here also the Jacobian callback, at a later stage
            { // 3.1 We found a configuration that works!
                /*
                // 3.2 Check joint limits:
                bool limitsOk=true;
                for (size_t i=0;i<jointCnt;i++)
                {
                    double pp=allJoints[i]->getPosition();
                    if (allJoints[i]->getPositionIsCyclic())
                    {
                        if (allJointRangeVals[i]<piValT2)
                        {
                            while (pp>allJointMinVals[i])
                                pp-=piValT2;
                            while (pp<allJointMinVals[i])
                                pp+=piValT2;
                            if (pp>allJointMinVals[i]+allJointRangeVals[i])
                                limitsOk=false;
                        }
                    }
                    else
                    {
                        if ( (pp<allJointMinVals[i])||(pp>allJointMinVals[i]+allJointRangeVals[i]) )
                            limitsOk=false;
                    }
                }
                */
                // 3.3 Finally check if the callback accepts that configuration:
                for (size_t i=0;i<jointCnt;i++)
                    retConf[i]=selectedJoints[i]->getPosition();
                if ( (validationCallback==nullptr)||validationCallback(&retConf[0]) )
                {
                    for (size_t i=0;i<jointCnt;i++)
                        retConfig[i]=retConf[i];
                    retVal=1;
                    break;
                }
            }
        }
    }

    // Restore joint positions:
    for (size_t i=0;i<allJoints.size();i++)
        allJoints[i]->setPosition(allJointInitValues[i]);
    return(retVal);
}

int ikGetConfigForTipPose(int ikGroupHandle,size_t jointCnt,const int* jointHandles,double thresholdDist,int maxIterations,double* retConfig,const double* metric/*=nullptr*/,bool(*validationCallback)(double*)/*=nullptr*/,const int* jointOptions/*=nullptr*/,const double* lowLimits/*=nullptr*/,const double* ranges/*=nullptr*/)
{
    debugInfo inf(__FUNCTION__,ikGroupHandle);
    int retVal=-1;
    std::vector<double> conf(jointCnt);
    if (hasLaunched())
    {
        CikGroup* ikGroup=CEnvironment::currentEnvironment->ikGroupContainer->getIkGroup(ikGroupHandle);
        if (ikGroup!=nullptr)
        {
            const double _defaultMetric[4]={1.0,1.0,1.0,0.1};
            const double* theMetric=_defaultMetric;
            if (metric!=nullptr)
                theMetric=metric;
            std::vector<CJoint*> joints;
            std::vector<double> minVals;
            std::vector<double> rangeVals;
            int err=0;
            for (size_t i=0;i<jointCnt;i++)
            {
                CJoint* aJoint=CEnvironment::currentEnvironment->objectContainer->getJoint(jointHandles[i]);
                if (aJoint==nullptr)
                    err=1;
                else
                {
                    joints.push_back(aJoint);

                    double l0=aJoint->getPositionIntervalMin();
                    double r0=aJoint->getPositionIntervalRange();
                    if (aJoint->getPositionIsCyclic())
                    {
                        l0=-piValue;
                        r0=piValT2;
                    }
                    double l=l0;
                    double r=r0;
                    if ( (lowLimits!=nullptr)&&(ranges!=nullptr) )
                    { // maybe we want different limits than the joint's limits?
                        double ll=lowLimits[i];
                        double rr=ranges[i];
                        if (rr!=0.0)
                        {
                            if (rr>0.0)
                            { // we want custom limits. Make sure they are valid
                                if (aJoint->getPositionIsCyclic())
                                {
                                    r=rr;
                                    if (r>piValT2)
                                        r=piValT2;
                                    l=ll;
                                    while (l<-piValue)
                                        l+=piValT2;
                                    while (l>piValue)
                                        l-=piValT2;
                                }
                                else
                                {
                                    l=ll;
                                    if (l<l0)
                                        l=l0;
                                    double u=ll+rr;
                                    if (u>l0+r0)
                                        u=l0+r0;
                                    r=u-l;
                                }
                            }
                            else
                            { // we want custom limits centered around current position
                                rr=-rr;
                                if (aJoint->getPositionIsCyclic())
                                {
                                    if (r>=piValT2)
                                    {
                                        l=-piValue;
                                        r=piValT2;
                                    }
                                    else
                                    {
                                        l=aJoint->getPosition()-rr*0.5;
                                        if (l<-piValue)
                                            l+=piValT2;
                                        r=rr;
                                    }
                                }
                                else
                                {
                                    l=aJoint->getPosition()-rr*0.5;
                                    if (l<l0)
                                        l=l0;
                                    double u=aJoint->getPosition()+rr*0.5;
                                    if (u>l0+r0)
                                        u=l0+r0;
                                    r=u-l;
                                }
                            }
                        }
                    }
                    if (aJoint->getJointMode()!=ik_jointmode_passive)
                    {
                        l=aJoint->getPosition();
                        r=0.0;
                    }
                    minVals.push_back(l);
                    rangeVals.push_back(r);
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
                std::vector<double> initSceneJointValues;
                std::vector<int> initSceneJointModes;
                for (size_t i=0;i<CEnvironment::currentEnvironment->objectContainer->jointList.size();i++)
                {
                    CJoint* aj=CEnvironment::currentEnvironment->objectContainer->getJoint(CEnvironment::currentEnvironment->objectContainer->jointList[i]);
                    sceneJoints.push_back(aj);
                    initSceneJointValues.push_back(aj->getPosition());
                    initSceneJointModes.push_back(aj->getJointMode());
                }

                ikGroup->setAllInvolvedJointsToPassiveMode_old();

                bool ikGroupWasActive=(ikGroup->getOptions()&1)==0;
                if (!ikGroupWasActive)
                    ikGroup->setOptions(ikGroup->getOptions()&~1);

                // It can happen that some IK elements get deactivated when the user provided wrong handles, so save the activation state:
                std::vector<bool> enabledElements;
                for (size_t i=0;i<ikGroup->getIkElementCount();i++)
                    enabledElements.push_back(ikGroup->getIkElementFromIndex(i)->getIsActive());

                // Set the correct mode for the joints involved:
                for (size_t i=0;i<jointCnt;i++)
                {
                    if ( (jointOptions==nullptr)||((jointOptions[i]&1)==0) )
                    {
                        if (rangeVals[i]==0.0)
                            joints[i]->setJointMode(ik_jointmode_passive);
                        else
                            joints[i]->setJointMode(ik_jointmode_ik);
                    }
                    else
                        joints[i]->setJointMode(ik_jointmode_ik);
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
                        joints[i]->setPosition(minVals[i]+(rand()/double(RAND_MAX))*rangeVals[i]);

                    // 2. Check distances between tip and target pairs (there might be several pairs!):
                    double cumulatedDist=0.0;
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
                        double angle=tipTr.Q.getAngleBetweenQuaternions(targetTr.Q)*theMetric[3];
                        cumulatedDist+=sqrt(dx(0)*dx(0)+dx(1)*dx(1)+dx(2)*dx(2)+angle*angle);
                    }

                    // 3. If distance<=threshold, try to perform IK:
                    if (cumulatedDist<=thresholdDist)
                    {
                        if (ik_result_success==ikGroup->computeGroupIk(true,nullptr))
                        { // 3.1 We found a configuration that works!
                            // 3.2 Check joint limits:
                            bool limitsOk=true;
                            for (size_t i=0;i<jointCnt;i++)
                            {
                                double pp=joints[i]->getPosition();
                                if (joints[i]->getPositionIsCyclic())
                                {
                                    if (rangeVals[i]<piValT2)
                                    {
                                        while (pp>minVals[i])
                                            pp-=piValT2;
                                        while (pp<minVals[i])
                                            pp+=piValT2;
                                        if (pp>minVals[i]+rangeVals[i])
                                            limitsOk=false;
                                    }
                                }
                                else
                                {
                                    if ( (pp<minVals[i])||(pp>minVals[i]+rangeVals[i]) )
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
                    ikGroup->setOptions(ikGroup->getOptions()|1);

                // Restore the IK element activation state:
                for (size_t i=0;i<ikGroup->getIkElementCount();i++)
                    ikGroup->getIkElementFromIndex(i)->setIsActive(enabledElements[i]);

                // Restore joint positions/modes:
                for (size_t i=0;i<sceneJoints.size();i++)
                {
                    if (fabs(sceneJoints[i]->getPosition()-initSceneJointValues[i])>0.0001)
                        sceneJoints[i]->setPosition(initSceneJointValues[i]);
                    if (sceneJoints[i]->getJointMode()!=initSceneJointModes[i])
                        sceneJoints[i]->setJointMode(initSceneJointModes[i]);
                }
            }
            else
            {
                if (err==1)
                    _setLastError("Found invalid joint handle(s)");
                if (err==2)
                    _setLastError("Found ill-defined IK element(s)");
                if (err==3)
                    _setLastError("Ill-defined IK group");
            }
        }
        else
            _setLastError("Invalid IK group handle: %i",ikGroupHandle);
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
