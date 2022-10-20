#include "ikRoutines.h"
#include "environment.h"

void CIkRoutines::multiply(const C4X4FullMatrix& d0,const C4X4FullMatrix& dp,size_t index,std::vector<C4X4FullMatrix*>& allMatrices)
{
// Input transformation matrices:
// Right part:
// R=m0+m1*deltaQ1+m2*deltaQ2+m3*deltaQ3+...+m6*deltaQ6
// Left part:
// L=d0+dp*deltaQindex
//Output transformation matrices:
// R=L*R
// index should be between 1 and n (indication which deltaQ dp has to be multiplied with
// If index is different from that, d0+dp*deltQindex=d0=normal transformation matrix
// If index==1, it concerns the first joint in the chain (from the tooltip), etc.
    C4X4FullMatrix& m0=*allMatrices[0];
    C4X4FullMatrix m0Saved(m0);
    m0=d0*m0Saved;
    for (size_t i=1;i<allMatrices.size();i++)
        (*allMatrices[i])=d0*(*allMatrices[i]);
    if ((index>0)&&(index<allMatrices.size()))
    {
        C4X4FullMatrix w(dp*m0Saved);
        (*allMatrices[index])+=w;
    }
}

void CIkRoutines::buildDeltaZRotation(C4X4FullMatrix& d0,C4X4FullMatrix& dp,simReal screwCoeff)
{
    d0.setIdentity();
    dp.clear();
    dp(0,1)=-simOne;
    dp(1,0)=simOne;
    dp(2,3)=screwCoeff;
}

void CIkRoutines::buildDeltaZTranslation(C4X4FullMatrix& d0,C4X4FullMatrix& dp)
{
    d0.setIdentity();
    dp.clear();
    dp(2,3)=simOne;
}

CMatrix CIkRoutines::getJacobian(CikElement* ikElement,C4X4Matrix& tooltipTransf,std::vector<int>* jointHandles_tipToBase,std::vector<size_t>* jointStages_tipToBase)
{   // jointHandles_tipToBase is nullptr by default. If not nullptr, it will contain the handles of the joints
    // corresponding to the rows of the jacobian.
    // A zero return matrix means that is ikElement is either inactive, either invalid
    // tooltipTransf is the cumulative transformation matrix of the tooltip,
    // computed relative to the base!
    // We check if the ikElement's base is in the chain and that tooltip is valid!
    CDummy* tooltip=CEnvironment::currentEnvironment->objectContainer->getDummy(ikElement->getTipHandle());
    if (tooltip==nullptr)
    { // Should normally never happen!
        ikElement->setIsActive(false);
        return(CMatrix());
    }
    CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(ikElement->getBaseHandle());
    if ( (base!=nullptr)&&(!tooltip->isObjectAffiliatedWith(base)) )
    { // This case can happen (when the base's parenting was changed for instance)
        ikElement->setBaseHandle(-1);
        ikElement->setIsActive(false);
        return(CMatrix());
    }

    // We check the number of degrees of freedom and prepare the jointHandles_tipToBase vector:
    CSceneObject* iterat=tooltip;
    size_t doF=0;
    while (iterat!=base)
    {
        iterat=iterat->getParentObject();
        if ( (iterat!=nullptr)&&(iterat!=base) )
        {
            if (iterat->getObjectType()==ik_objecttype_joint)
            {
                if ( ((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_ik)||((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_reserved_previously_ikdependent)||((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_dependent) )
                {
                    size_t d=(static_cast<CJoint*>(iterat))->getDoFs();
                    for (int i=int(d-1);i>=0;i--)
                    {
                        if (jointHandles_tipToBase!=nullptr)
                        {
                            jointHandles_tipToBase->push_back(iterat->getObjectHandle());
                            jointStages_tipToBase->push_back(size_t(i));
                        }
                    }
                    doF+=d;
                }
            }
        }
    }
    std::vector<C4X4FullMatrix*> jMatrices;
    for (size_t i=0;i<doF+1;i++)
    {
        C4X4FullMatrix* matr=new C4X4FullMatrix();
        if (i==0)
            (*matr).setIdentity();
        else
            (*matr).clear();
        jMatrices.push_back(matr);
    }

    // Now we go from tip to base:
    iterat=tooltip;
    C4X4FullMatrix buff;
    buff.setIdentity();
    size_t positionCounter=0;
    C4X4FullMatrix d0;
    C4X4FullMatrix dp;
    C4X4FullMatrix paramPart;
    CJoint* lastJoint=nullptr;
    int indexCnt=-1;
    int indexCntLast=-1;
    while (iterat!=base)
    {
        CSceneObject* nextIterat=iterat->getParentObject();
        C7Vector local;
        if (iterat->getObjectType()==ik_objecttype_joint)
        {
            if ( ((static_cast<CJoint*>(iterat))->getJointMode()!=ik_jointmode_ik)&&((static_cast<CJoint*>(iterat))->getJointMode()!=ik_jointmode_reserved_previously_ikdependent)&&((static_cast<CJoint*>(iterat))->getJointMode()!=ik_jointmode_dependent) )
                local=iterat->getLocalTransformation(true);
            else
            {
                CJoint* it=static_cast<CJoint*>(iterat);
                if (it->getJointType()==ik_jointtype_spherical)
                {
                    if (indexCnt==-1)
                        indexCnt=int(it->getDoFs())-1;
                    it->getLocalTransformationExPart1(local,size_t(indexCnt--));
                    if (indexCnt!=-1)
                        nextIterat=iterat; // We keep the same object! (but indexCnt has decreased)
                }
                else
                    local=iterat->getLocalTransformationPart1(true);
            }
        }
        else
            local=iterat->getLocalTransformation(true); 

        buff=C4X4FullMatrix(local.getMatrix())*buff;
        iterat=nextIterat;
        bool activeJoint=false;
        if (iterat!=nullptr)
        {
            if (iterat->getObjectType()==ik_objecttype_joint)
                activeJoint=( ((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_ik)||((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_reserved_previously_ikdependent)||((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_dependent) );
        }
        if ( (iterat==base)||activeJoint )
        {   // If base is nullptr then the second part is not evaluated (iterat->getObjectType())
            if (positionCounter==0)
            {   // Here we have the first part (from tooltip to first joint)
                d0=buff;
                dp.clear();
                multiply(d0,dp,0,jMatrices);
            }
            else
            {   // Here we have a joint:
                if (lastJoint->getJointType()==ik_jointtype_revolute)
                {
                    buildDeltaZRotation(d0,dp,lastJoint->getScrewPitch());
                    paramPart.buildZRotation(lastJoint->getPosition(true));
                }
                else if (lastJoint->getJointType()==ik_jointtype_prismatic)
                {
                    buildDeltaZTranslation(d0,dp);
                    paramPart.buildTranslation(0.0,0.0,lastJoint->getPosition(true));
                }
                else 
                { // Spherical joint part!
                    buildDeltaZRotation(d0,dp,0.0);
                    if (indexCntLast==-1)
                        indexCntLast=int(lastJoint->getDoFs())-1;
                    paramPart.buildZRotation(lastJoint->getTempParameterEx(size_t(indexCntLast--)));
                }
                multiply(d0,dp,positionCounter,jMatrices);
                d0=buff*paramPart;
                dp.clear();
                multiply(d0,dp,0,jMatrices);
            }
            buff.setIdentity();
            lastJoint=static_cast<CJoint*>(iterat);
            positionCounter++;
        }
    }

//    printf("jMatrices0: %f\n",(*jMatrices[1])(0,0));
    int alternativeBaseForConstraints=ikElement->getAltBaseHandleForConstraints();
    if (alternativeBaseForConstraints!=-1)
    {
        CDummy* alb=CEnvironment::currentEnvironment->objectContainer->getDummy(alternativeBaseForConstraints);
        if (alb!=nullptr)
        { // We want everything relative to the alternativeBaseForConstraints dummy orientation!
            C7Vector alternativeBase(alb->getCumulativeTransformationPart1(true));
            C7Vector currentBase;
            currentBase.setIdentity();
            if (base!=nullptr)
                currentBase=base->getCumulativeTransformation(true); // could be a joint, we want also the joint intrinsic transformation part!
            C4X4FullMatrix correction((alternativeBase.getInverse()*currentBase).getMatrix());
            dp.clear();
            multiply(correction,dp,0,jMatrices);
        }
    }

    CMatrix J(6,doF);
    // The x-, y- and z-component:
    for (size_t i=0;i<doF;i++)
    {
        J(0,i)=(*jMatrices[1+i])(0,3);
        J(1,i)=(*jMatrices[1+i])(1,3);
        J(2,i)=(*jMatrices[1+i])(2,3);
    }
//    printf("jMatrices1: %f\n",(*jMatrices[1])(0,0));
    // We divide all delta components (to avoid distorsions)...
    for (size_t i=0;i<doF;i++)
        (*jMatrices[1+i])/=IK_DIVISION_FACTOR;
    // ...and add the cumulative transform to the delta-components:
//    printf("jMatrices2: %f\n",(*jMatrices[1])(0,0));
    for (size_t i=0;i<doF;i++)
        (*jMatrices[1+i])+=(*jMatrices[0]);
//    printf("jMatrices3: %f\n",(*jMatrices[1])(0,0));
    // We also copy the cumulative transform to 'tooltipTransf':
    tooltipTransf=(*jMatrices[0]);
    // Now we extract the delta Euler components:
    C4Vector q((*jMatrices[0]).getMatrix().M.getQuaternion());
    C4Vector qInv(q.getInverse());
    // Alpha-, Beta- and Gamma-components:
    for (size_t i=0;i<doF;i++)
    {
        C4Vector qq;
        qq.buildInterpolation(q,(*jMatrices[1+i]).getMatrix().M.getQuaternion(),1.0);///IK_DIVISION_FACTOR);
        C3X3Matrix diff(qInv*qq);
        C3Vector euler(diff.getEulerAngles());
        euler=euler*1.0;
        J(3,i)=euler(0); // here we would have to multiply the euler angle with IK_DIVISION_FACTOR to get the "correct" Jacobian
        J(4,i)=euler(1); // here we would have to multiply the euler angle with IK_DIVISION_FACTOR to get the "correct" Jacobian
        J(5,i)=euler(2); // here we would have to multiply the euler angle with IK_DIVISION_FACTOR to get the "correct" Jacobian
    }

//    printf("J(5,0): %f\n",J(5,0));

    // We free the memory allocated for each joint variable:
    for (size_t i=0;i<jMatrices.size();i++)
        delete jMatrices[i];
    return(J);
}



