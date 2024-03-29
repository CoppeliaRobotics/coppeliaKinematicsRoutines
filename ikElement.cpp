#include "ikElement.h"
#include "environment.h"
#include <simMath/mathFuncs.h>
#include <unordered_set>

CikElement::CikElement(int theTooltip)
{
    _tipHandle=theTooltip;
    _baseHandle=-1;
    _altBaseHandleForConstraints=-1;
    _isActive=true;
    _constraints=ik_constraint_position;
    _precisions[0]=0.0005;
    _precisions[1]=0.1*degToRad;
    _weights[0]=1.0;
    _weights[1]=1.0;
    _weights[2]=1.0;
    _ikElementHandle=-1;
    _backCompatibility=1.0;
    if ((CEnvironment::currentEnvironment->getFlags()&2)!=0)
        _backCompatibility=0.01;
}

CikElement::~CikElement()
{
}

CikElement* CikElement::copyYourself() const
{
    CikElement* duplicate=new CikElement(-1);

    duplicate->_ikElementHandle=_ikElementHandle;
    duplicate->_tipHandle=_tipHandle;
    duplicate->_baseHandle=_baseHandle;
    duplicate->_altBaseHandleForConstraints=_altBaseHandleForConstraints;
    duplicate->_constraints=_constraints;
    duplicate->_isActive=_isActive;
    duplicate->_weights[0]=_weights[0];
    duplicate->_weights[1]=_weights[1];
    duplicate->_weights[2]=_weights[2];
    duplicate->_precisions[0]=_precisions[0];
    duplicate->_precisions[1]=_precisions[1];
    duplicate->jointHandles.assign(jointHandles.begin(),jointHandles.end());
    duplicate->jointDofIndex.assign(jointDofIndex.begin(),jointDofIndex.end());
    duplicate->equationTypes.assign(equationTypes.begin(),equationTypes.end());
    duplicate->jacobian.set(jacobian);
    duplicate->errorVector.set(errorVector);
    duplicate->_backCompatibility=_backCompatibility;

    return(duplicate);
}

bool CikElement::announceSceneObjectWillBeErased(int objectHandle)
{
    bool retVal=( (_baseHandle==objectHandle)||(_altBaseHandleForConstraints==objectHandle)||(_tipHandle==objectHandle) );
    return(retVal);
}

void CikElement::performSceneObjectLoadingMapping(const std::vector<int>* map)
{
    _tipHandle=_getLoadingMapping(map,_tipHandle);
    _baseHandle=_getLoadingMapping(map,_baseHandle);
    _altBaseHandleForConstraints=_getLoadingMapping(map,_altBaseHandleForConstraints);
}

void CikElement::serialize(CSerialization& ar)
{
    if (ar.isWriting())
    {
        ar.writeInt(_ikElementHandle);
        ar.writeInt(_tipHandle);
        ar.writeInt(_baseHandle);
        ar.writeInt(_altBaseHandleForConstraints);
        ar.writeFloat(float(_precisions[1]));
        ar.writeFloat(float(_precisions[0]));
        ar.writeInt(_constraints);
        ar.writeFloat(float(_weights[0]));
        ar.writeFloat(float(_weights[1]));
        unsigned char nothing=0;
        nothing=nothing+1*_isActive;
        ar.writeByte(nothing);
    }
    else
    {
        _ikElementHandle=ar.readInt();
        _tipHandle=ar.readInt();
        _baseHandle=ar.readInt();
        _altBaseHandleForConstraints=ar.readInt();
        _precisions[1]=double(ar.readFloat());
        _precisions[0]=double(ar.readFloat());
        _constraints=ar.readInt();
        _weights[0]=double(ar.readFloat());
        _weights[1]=double(ar.readFloat());
        _isActive=(ar.readByte()&1);
    }
}

int CikElement::getIkElementHandle() const
{
    return(_ikElementHandle);
}

void CikElement::setIkElementHandle(int handle)
{
    _ikElementHandle=handle;
}

int CikElement::getTipHandle() const
{
    return(_tipHandle);
}

int CikElement::getTargetHandle() const
{
    int retVal=-1;
    CDummy* tip=CEnvironment::currentEnvironment->objectContainer->getDummy(_tipHandle);
    if (tip!=nullptr)
    {
        retVal=tip->getTargetDummyHandle();
        if (retVal==-1)
            retVal=tip->getLinkedDummyHandle_old(); // for backward compatibility
    }
    return(retVal);
}

int CikElement::getBaseHandle() const
{
    return(_baseHandle);
}

void CikElement::setBaseHandle(int newBaseHandle)
{
    _baseHandle=newBaseHandle;
}

int CikElement::getAltBaseHandleForConstraints() const
{
    return(_altBaseHandleForConstraints);
}

void CikElement::setAltBaseHandleForConstraints(int newAltBaseHandle)
{
    _altBaseHandleForConstraints=newAltBaseHandle;
}

void CikElement::setRelatedJointsToPassiveMode_old()
{
    CSceneObject* it=CEnvironment::currentEnvironment->objectContainer->getDummy(_tipHandle);
    if (it!=nullptr)
    {
        CSceneObject* baseObj=CEnvironment::currentEnvironment->objectContainer->getObject(_baseHandle);
        it=it->getParentObject();
        while ( (it!=nullptr)&&(it!=baseObj) )
        {
            if (it->getObjectType()==ik_objecttype_joint)
                (static_cast<CJoint*>(it))->setJointMode(ik_jointmode_passive);
            it=it->getParentObject();
        }
    }
}

bool CikElement::getIsActive() const
{
    return(_isActive);
}

void CikElement::setIsActive(bool isActive)
{
    _isActive=isActive;
}

bool CikElement::getIsValid() const
{
    if (!_isActive)
        return(false);
    CDummy* tooltip=CEnvironment::currentEnvironment->objectContainer->getDummy(_tipHandle);
    CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(_baseHandle);
    if (tooltip==nullptr)
        return(false);
    // We check that tooltip is parented with base and has at least one joint in-between:
    bool hasJoint=false;
    bool hasBase=false;
    CSceneObject* iterat=tooltip;
    while ( (iterat!=base)&&(iterat!=nullptr) )
    {
        iterat=iterat->getParentObject();
        if (iterat==base)
            hasBase=true;
        if ( (iterat!=base)&&(iterat!=nullptr)&&(iterat->getObjectType()==ik_objecttype_joint) )
        {
            CJoint* joint=(CJoint*)iterat;
            if ( (joint->getJointMode()==ik_jointmode_ik)&&(joint->getDependencyJointHandle()==-1) )
                hasJoint=true;
        }
    }
    return(hasJoint&&hasBase);
}

int CikElement::getConstraints() const
{
    return(_constraints);
}

void CikElement::setConstraints(int constraints)
{
    _constraints=constraints;
}

void CikElement::getWeights(double w[3]) const
{
    w[0]=_weights[0];
    w[1]=_weights[1];
    w[2]=_weights[2];
}

void CikElement::setWeights(const double w[3])
{
    _weights[0]=w[0];
    _weights[1]=w[1];
    _weights[2]=w[2];
}

void CikElement::getPrecisions(double p[2]) const
{
    p[0]=_precisions[0];
    p[1]=_precisions[1];
}

void CikElement::setPrecisions(const double p[2])
{
    _precisions[0]=p[0];
    _precisions[1]=p[1];
}

void CikElement::prepareRawJacobian(double interpolationFactor)
{
    int constrBase=_baseHandle;
    if (_altBaseHandleForConstraints!=-1)
        constrBase=_altBaseHandleForConstraints;
    getJacobian(jacobian,errorVector,_tipHandle,_baseHandle,_constraints,constrBase,interpolationFactor,&equationTypes,&jointHandles,&jointDofIndex,_backCompatibility);
}

bool CikElement::getJacobian(CMatrix& jacob,CMatrix& errVect,int ttip,int tbase,int constraints,int constrBase,double interpolationFactor,std::vector<int>* equTypes,std::vector<int>* jHandles,std::vector<int>* jDofIndex,double backCompatibility/*=1.0*/)
{ // equTypes, jHandles and jDofIndex can be nullptr
    if (jHandles!=nullptr)
        jHandles->clear();
    if (jDofIndex!=nullptr)
        jDofIndex->clear();
    if (equTypes!=nullptr)
        equTypes->clear();
    errVect.resize(0,1,0.0);
    CDummy* tip=CEnvironment::currentEnvironment->objectContainer->getDummy(ttip);
    C7Vector tr(tip->getCumulativeTransformation());

    CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(tbase);
    if ( (tip==nullptr)||((base!=nullptr)&&(!tip->isObjectAffiliatedWith(base))) )
        return(false);
    CDummy* target=CEnvironment::currentEnvironment->objectContainer->getDummy(tip->getTargetDummyHandle());
    if (target==nullptr)
        target=CEnvironment::currentEnvironment->objectContainer->getDummy(tip->getLinkedDummyHandle_old()); // for backward compatibility
    CSceneObject* constraintBase=CEnvironment::currentEnvironment->objectContainer->getObject(constrBase);
    std::vector<double> mem;
    jacob=_getNakedJacobian(tip,target,base,constraintBase,constraints,interpolationFactor,jHandles,jDofIndex,backCompatibility);
    bool retVal=false;
    if ( (jacob.rows!=0)&&(jacob.cols!=0) )
    {
        retVal=true;
        if (target!=nullptr)
        {
            errVect.resize(jacob.rows,1,0.0);
            C7Vector constrBasePoseInv(C7Vector::identityTransformation);
            if (constraintBase!=nullptr)
                constrBasePoseInv=constraintBase->getCumulativeTransformationPart1().getInverse();
            C7Vector tipTrRel(constrBasePoseInv*tip->getCumulativeTransformationPart1());
            C7Vector targetTrRel(constrBasePoseInv*target->getCumulativeTransformationPart1());
            targetTrRel.buildInterpolation(tipTrRel,targetTrRel,interpolationFactor);

            if ( ((constraints&ik_constraint_orientation)==ik_constraint_alpha_beta)||((constraints&ik_constraint_orientation)==ik_constraint_gamma) )
            { // we have partial orient. constr.
                if ((constraints&ik_constraint_orientation)==ik_constraint_alpha_beta)
                { // We need to reorient the target around its z-axis to "ressemble" most the tip orientation
                    C3Vector tipXaxisProj(targetTrRel.Q.getInverse()*tipTrRel.Q.getMatrix().axis[0]);
                    double angle=tipXaxisProj.getAngle(C3Vector::unitXVector);
                    if (fabs(angle)>0.001)
                    {
                        if (tipXaxisProj(1)<0.0)
                            angle=-angle;
                        C4Vector q(angle,C3Vector::unitZVector);
                        targetTrRel.Q*=q;
                    }
                }
                else
                { // gamma constraint only. We need to reorient the target with a rotation that brings both z-axes together
                    C3Vector tipZaxis(tipTrRel.Q.getMatrix().axis[2]);
                    C3Vector targetZaxis(targetTrRel.Q.getMatrix().axis[2]);
                    double angle=targetZaxis.getAngle(tipZaxis);
                    if (angle>0.001)
                    {
                        C4Vector q(angle,(targetZaxis^tipZaxis).getNormalized());
                        targetTrRel.Q=q*targetTrRel.Q;
                    }
                }
            }

            C7Vector interpolTargetRel;
            double dq=0.01; // not that relevant apparently
            interpolTargetRel.buildInterpolation(tipTrRel,targetTrRel,dq);

            C3Vector euler( (tipTrRel.Q.getInverse()*interpolTargetRel.Q).getEulerAngles() );
            C3Vector dx_x(interpolTargetRel.X-tipTrRel.X);

            size_t rowIndex=0;
            if ((constraints&ik_constraint_x)!=0)
            {
                errVect(rowIndex++,0)=dx_x(0)/dq;
                if (equTypes!=nullptr)
                    equTypes->push_back(0);
            }
            if ((constraints&ik_constraint_y)!=0)
            {
                errVect(rowIndex++,0)=dx_x(1)/dq;
                if (equTypes!=nullptr)
                    equTypes->push_back(1);
            }
            if ((constraints&ik_constraint_z)!=0)
            {
                errVect(rowIndex++,0)=dx_x(2)/dq;
                if (equTypes!=nullptr)
                    equTypes->push_back(2);
            }
            if ((constraints&ik_constraint_orientation)!=0)
            {
                if ((constraints&ik_constraint_alpha_beta)!=0)
                {
                    errVect(rowIndex++,0)=backCompatibility*euler(0)/dq;
                    errVect(rowIndex++,0)=backCompatibility*euler(1)/dq;
                    if (equTypes!=nullptr)
                    {
                        equTypes->push_back(3);
                        equTypes->push_back(4);
                    }
                }
                if ((constraints&ik_constraint_gamma)!=0)
                {
                    errVect(rowIndex++,0)=backCompatibility*euler(2)/dq;
                    if (equTypes!=nullptr)
                        equTypes->push_back(5);
                }
            }
        }
    }

    return(retVal);
}

void CikElement::getTipTargetDistance(double& linDist,double& angDist) const
{ // returns the tip-target lin./ang. distances, taking into account the constraint settings
    linDist=0.0;
    angDist=0.0;
    CDummy* targetObject=CEnvironment::currentEnvironment->objectContainer->getDummy(getTargetHandle());
    if (targetObject!=nullptr)
    {
        C7Vector targetTr(targetObject->getCumulativeTransformationPart1());
        CDummy* tooltipObject=CEnvironment::currentEnvironment->objectContainer->getDummy(_tipHandle);
        C7Vector tooltipTr(tooltipObject->getCumulativeTransformationPart1());
        int constrBase=_baseHandle;
        if (_altBaseHandleForConstraints!=-1)
            constrBase=_altBaseHandleForConstraints;
        C7Vector baseTrInv(C7Vector::identityTransformation);
        CSceneObject* constrBaseObj=CEnvironment::currentEnvironment->objectContainer->getObject(constrBase);
        if (constrBaseObj!=nullptr)
            baseTrInv=constrBaseObj->getCumulativeTransformationPart1().getInverse();
        tooltipTr=baseTrInv*tooltipTr;
        targetTr=baseTrInv*targetTr;

        // Linear:
        double constr[3]={0.0,0.0,0.0};
        if ( (_constraints&ik_constraint_x)!=0 )
            constr[0]=1.0;
        if ( (_constraints&ik_constraint_y)!=0 )
            constr[1]=1.0;
        if ( (_constraints&ik_constraint_z)!=0 )
            constr[2]=1.0;
        C3Vector displ(tooltipTr.X-targetTr.X);
        linDist=sqrt(displ(0)*displ(0)*constr[0]+displ(1)*displ(1)*constr[1]+displ(2)*displ(2)*constr[2]);

        // Angular:
        if ( (_constraints&ik_constraint_orientation)==ik_constraint_orientation)
            (targetTr.getInverse()*tooltipTr).Q.getAngleAndAxis(angDist);
        else if ( (_constraints&ik_constraint_alpha_beta)!=0 )
            angDist=targetTr.getMatrix().M.axis[2].getAngle(tooltipTr.getMatrix().M.axis[2]);
        else if ( (_constraints&ik_constraint_gamma)!=0 ) // gamma constraint can exist also without alpha/beta constraint, e.g. in 2D
        { // gamma constraint only. We need to reorient the target with a rotation that brings both z-axes together
            C3Vector tipZaxis(tooltipTr.Q.getMatrix().axis[2]);
            C3Vector targetZaxis(targetTr.Q.getMatrix().axis[2]);
            double angle=targetZaxis.getAngle(tipZaxis);
            if (angle>0.001)
            {
                C4Vector q(angle,(targetZaxis^tipZaxis).getNormalized());
                targetTr.Q=q*targetTr.Q;
            }
            (targetTr.getInverse()*tooltipTr).Q.getAngleAndAxis(angDist);
        }
        else
            angDist=0.0;
    }
}

CMatrix CikElement::_getNakedJacobian(const CSceneObject* tip,const CSceneObject* target,const CSceneObject* base,const CSceneObject* constrBase,int constraints,double interpolationFactor,std::vector<int>* jHandles,std::vector<int>* jDofIndex,double backCompatibility/*=1.0*/)
{ // jHandles and jDofIndex can be nullptr
    size_t rows=0;
    if ((constraints&ik_constraint_x)!=0)
        rows++;
    if ((constraints&ik_constraint_y)!=0)
        rows++;
    if ((constraints&ik_constraint_z)!=0)
        rows++;
    if ((constraints&ik_constraint_alpha_beta)!=0)
        rows+=2;
    if ((constraints&ik_constraint_gamma)!=0)
        rows++;

    std::vector<CJoint*> joints;
    std::unordered_set<CJoint*> allJ;
    CSceneObject* object=tip->getParentObject();
    while (object!=base)
    {
        if (object->getObjectType()==ik_objecttype_joint)
        {
            CJoint* joint=(CJoint*)object;
            if (joint->getJointMode()==ik_jointmode_ik)
            {
                int d=joint->getDependencyJointHandle();
                bool involveDependentJoints=false; // for now, dependent joints are invisible
                if ( (d!=-1)&&(!involveDependentJoints) )
                    joint=nullptr;
                while (d!=-1)
                { // follow joints from slave to master, if applicable
                    joint=CEnvironment::currentEnvironment->objectContainer->getJoint(d);
                    d=-1;
                    if (joint!=nullptr)
                    { // should always pass
                        if (joint->getJointMode()!=ik_jointmode_ik)
                            joint=nullptr;
                        else
                            d=joint->getDependencyJointHandle();
                    }
                }
                if (joint!=nullptr)
                {
                    if (allJ.find(joint)==allJ.end())
                    { // make sure we do not add more than one occurence of the same joint
                        joints.insert(joints.begin(),joint);
                        allJ.insert(joint);
                    }
                }
            }
        }
        object=object->getParentObject();
    }

    CMatrix jacobian(rows,0);
    C7Vector constrBaseTrInv_init(C7Vector::identityTransformation);
    if (constrBase!=nullptr)
        constrBaseTrInv_init=constrBase->getCumulativeTransformationPart1().getInverse();
    C3Vector tipRelConstrBase((constrBaseTrInv_init*tip->getCumulativeTransformationPart1()).X);

    C4Vector baseQ_init(C4Vector::identityRotation);
    if (base!=nullptr)
        baseQ_init=base->getCumulativeTransformationPart1().Q;
    C4Vector tipOrientInv_init(tip->getCumulativeTransformationPart1().Q.getInverse()*baseQ_init);

    int dofIndex=0;
    size_t jointIndex=0;
    while (jointIndex<joints.size())
    {
        CJoint* joint=joints[jointIndex]; // can also include joints not in the IK chain (e.g. because of joint dependencies)
        if (jHandles!=nullptr)
            jHandles->push_back(joint->getObjectHandle());
        if (jDofIndex!=nullptr)
            jDofIndex->push_back(dofIndex);
        int colIndex=int(jacobian.cols);
        jacobian.resize(rows,jacobian.cols+1,0.0);
        double dq=0.01; // starts breaking at dj>0.05 or at dj<0.01

        C3Vector tipRelConstrBase_changed;
        C4Vector tipQ_changed;
        if (joint->getJointType()==ik_jointtype_spherical)
        {
            C7Vector jbabs(joint->getCumulativeTransformation());
            C7Vector jb(constrBaseTrInv_init*jbabs); // here we can use constrBaseTrInv_init, since a sph. joint does not have any dep. joints
            C7Vector x(jbabs.getInverse()*tip->getCumulativeTransformationPart1());
            C7Vector dj;
            dj.setIdentity();
            if (dofIndex==0)
                dj.Q.setAngleAndAxis(dq,C3Vector(1.0,0.0,0.0));
            if (dofIndex==1)
                dj.Q.setAngleAndAxis(dq,C3Vector(0.0,1.0,0.0));
            if (dofIndex==2)
                dj.Q.setAngleAndAxis(dq,C3Vector(0.0,0.0,1.0));
            dofIndex++;
            tipRelConstrBase_changed=(jb*dj*x).X;
            tipQ_changed=baseQ_init.getInverse()*jbabs.Q*dj.Q*x.Q; // here we can use baseQ_init, since a sph. joint does not have any dep. joints
        }
        else
        {
            double tmp=joint->getPosition();
            joint->setPosition(tmp+dq);

            C7Vector constrBaseTrInv(C7Vector::identityTransformation);
            if (constrBase!=nullptr)
                constrBaseTrInv=constrBase->getCumulativeTransformationPart1().getInverse(); // important to recompute the constraint base freshly, it could have moved too (b/c of joint dep. for instance)
            tipRelConstrBase_changed=(constrBaseTrInv*tip->getCumulativeTransformationPart1()).X;

            C4Vector baseQ(C4Vector::identityRotation);
            if (base!=nullptr)
                baseQ=base->getCumulativeTransformationPart1().Q; // important to recompute the base freshly, it could have moved too (b/c of joint dep. for instance)
            tipQ_changed=baseQ.getInverse()*tip->getCumulativeTransformationPart1().Q;

            joint->setPosition(tmp);
        }
        C3Vector euler((tipOrientInv_init*tipQ_changed).getEulerAngles());
        C3Vector dx_x(tipRelConstrBase_changed-tipRelConstrBase);

        size_t rowIndex=0;
        if ((constraints&ik_constraint_x)!=0)
            jacobian(rowIndex++,colIndex)=dx_x(0)/dq;
        if ((constraints&ik_constraint_y)!=0)
            jacobian(rowIndex++,colIndex)=dx_x(1)/dq;
        if ((constraints&ik_constraint_z)!=0)
            jacobian(rowIndex++,colIndex)=dx_x(2)/dq;
        if ((constraints&ik_constraint_orientation)!=0)
        {
            if ((constraints&ik_constraint_alpha_beta)!=0)
            {
                jacobian(rowIndex++,colIndex)=backCompatibility*euler(0)/dq;
                jacobian(rowIndex++,colIndex)=backCompatibility*euler(1)/dq;
            }
            if ((constraints&ik_constraint_gamma)!=0)
                jacobian(rowIndex++,colIndex)=backCompatibility*euler(2)/dq;
        }

        if (dofIndex==3)
            dofIndex=0;
        if (dofIndex==0)
            jointIndex++;
    }
    return(jacobian);
}
