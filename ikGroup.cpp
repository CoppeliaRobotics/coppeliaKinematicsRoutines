#include "ikGroup.h"
#include "environment.h"
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <unordered_set>

CikGroup::CikGroup()
{
    _objectHandle=2030003;
    _maxIterations=3;
    _explicitHandling_old=true;
    _dlsFactor=0.1;
    _calculationMethod=ik_method_pseudo_inverse;

    _options=ik_group_enabled|ik_group_ignoremaxsteps;
}

CikGroup::~CikGroup()
{
    while (_ikElements.size()!=0)
        removeIkElement(_ikElements[0]->getIkElementHandle());
}

void CikGroup::performObjectLoadingMapping(std::vector<int>* map)
{
    for (size_t i=0;i<_ikElements.size();i++)
        _ikElements[i]->performSceneObjectLoadingMapping(map);
}

bool CikGroup::getExplicitHandling_old() const
{
    return(_explicitHandling_old);
}

void CikGroup::setAllInvolvedJointsToPassiveMode_old()
{
    for (size_t i=0;i<_ikElements.size();i++)
        _ikElements[i]->setRelatedJointsToPassiveMode_old();
}

void CikGroup::setObjectName(std::string newName)
{
    _objectName=newName;
}

void CikGroup::setOptions(int options)
{
    _options=options;
}

int CikGroup::getOptions() const
{
    return(_options);
}

void CikGroup::setMaxIterations(int maxIter)
{
    _maxIterations=maxIter;
}
int CikGroup::getMaxIterations() const
{
    return(_maxIterations);
}

void CikGroup::setDlsFactor(double theFactor)
{
    _dlsFactor=theFactor;
}
double CikGroup::getDlsFactor() const
{
    return(_dlsFactor);
}
void CikGroup::setCalculationMethod(int theMethod)
{
    if ( (theMethod==ik_method_pseudo_inverse)||(theMethod==ik_method_damped_least_squares)||
        (theMethod==ik_method_jacobian_transpose)||(theMethod==ik_method_undamped_pseudo_inverse) )
    {
        _calculationMethod=theMethod;
    }
}
int CikGroup::getCalculationMethod() const
{
    return(_calculationMethod);
}

void CikGroup::removeIkElement(int elementHandle)
{
    for (size_t i=0;i<_ikElements.size();i++)
    {
        if (_ikElements[i]->getIkElementHandle()==elementHandle)
        {
            delete _ikElements[i];
            _ikElements.erase(_ikElements.begin()+i);
            break;
        }
    }
}

size_t CikGroup::getIkElementCount() const
{
    return(_ikElements.size());
}

CikElement* CikGroup::getIkElementFromIndex(size_t index) const
{
    CikElement* retVal=nullptr;
    if (index<_ikElements.size())
        retVal=_ikElements[index];
    return(retVal);
}

CikGroup* CikGroup::copyYourself() const
{
    CikGroup* duplicate=new CikGroup();

    for (size_t i=0;i<_ikElements.size();i++)
        duplicate->_ikElements.push_back(_ikElements[i]->copyYourself());

    duplicate->_objectHandle=_objectHandle;
    duplicate->_objectName=_objectName;
    duplicate->_maxIterations=_maxIterations;
    duplicate->_dlsFactor=_dlsFactor;
    duplicate->_calculationMethod=_calculationMethod;
    duplicate->_options=_options;
    duplicate->_explicitHandling_old=_explicitHandling_old;
    duplicate->_jacobian.set(_jacobian);
    duplicate->_jacobianPseudoinv.set(_jacobianPseudoinv);
    duplicate->_E.set(_E);
    duplicate->_dQ.set(_dQ);

    return(duplicate);
}

int CikGroup::addIkElement(CikElement* ikElement)
{
    int retVal=0;
    while (getIkElement(retVal)!=nullptr)
        retVal++;
    ikElement->setIkElementHandle(retVal);
    _ikElements.push_back(ikElement);
    return(retVal);
}

CikElement* CikGroup::getIkElement(int elementHandle) const
{
    for (size_t i=0;i<_ikElements.size();i++)
    {
        if (_ikElements[i]->getIkElementHandle()==elementHandle)
            return(_ikElements[i]);
    }
    return(nullptr);
}

CikElement* CikGroup::getIkElementWithTooltipHandle(int tooltipHandle) const
{
    if (tooltipHandle==-1)
        return(nullptr);
    for (size_t i=0;i<_ikElements.size();i++)
    {
        if (_ikElements[i]->getTipHandle()==tooltipHandle)
            return(_ikElements[i]);
    }
    return(nullptr);
}

int CikGroup::getObjectHandle() const
{
    return(_objectHandle);
}

void CikGroup::setObjectHandle(int handle)
{
    _objectHandle=handle;
}

std::string CikGroup::getObjectName() const
{
    return(_objectName);
}

bool CikGroup::announceSceneObjectWillBeErased(int objectHandle)
{ // Return value true means that this object should be destroyed
    size_t i=0;
    while (i<_ikElements.size())
    {
        if (_ikElements[i]->announceSceneObjectWillBeErased(objectHandle))
        {
            removeIkElement(_ikElements[i]->getIkElementHandle());
            i=0; // ordering may have changed
        }
        else
            i++;
    }
    return(_ikElements.size()==0);
}

bool CikGroup::announceIkGroupWillBeErased(int ikGroupHandle)
{ // Return value true means that this avoidance object should be destroyed
    return(false);
}

void CikGroup::clearJointLimitHits()
{
    _jointLimitHits.clear();
}

bool CikGroup::getJointLimitHits(std::vector<int>* jointHandles,std::vector<double>* underOrOvershots) const
{
    for (auto it=_jointLimitHits.begin();it!=_jointLimitHits.end();++it)
    {
        if (jointHandles!=nullptr)
            jointHandles->push_back(it->first);
        if (underOrOvershots!=nullptr)
            underOrOvershots->push_back(it->second);
    }
    return(!_jointLimitHits.empty());
}

void CikGroup::getJointHandles(std::vector<int>& handles)
{ // returns a joint for each Jacobian column (spherical joints will however have a single entry, even if 3 columns in the Jacobian)
    handles.clear();
    // prepare a vector with all valid and active elements:
    std::vector<CikElement*> validElements;
    for (size_t elNb=0;elNb<_ikElements.size();elNb++)
    {
        CikElement* element=_ikElements[elNb];
        CDummy* tooltip=CEnvironment::currentEnvironment->objectContainer->getDummy(element->getTipHandle());
        CSceneObject* base=CEnvironment::currentEnvironment->objectContainer->getObject(element->getBaseHandle());
        if ( element->getIsActive()&&(tooltip!=nullptr) )
        {
            bool valid=false;
            bool jointPresent=false;
            bool baseOk=false;
            CSceneObject* iterat=tooltip;
            while ( (iterat!=base)&&(iterat!=nullptr) )
            {
                iterat=iterat->getParentObject();
                if (iterat==base)
                {
                    baseOk=true;
                    if (jointPresent)
                        valid=true;
                }
                if ( (iterat!=base)&&(iterat!=nullptr)&&(iterat->getObjectType()==ik_objecttype_joint) )
                {
                    if ((static_cast<CJoint*>(iterat))->getJointMode()==ik_jointmode_ik)
                        jointPresent=true;
                }
            }
            if (valid)
                validElements.push_back(element);
        }
    }
    if (validElements.size()!=0)
    {
        prepareRawJacobians(validElements,1.0);
        selectJoints(&validElements,nullptr,nullptr);
        for (size_t i=0;i<_jointHandles.size();i++)
        {
            if (_jointDofIndex[i]==0)
                handles.push_back(_jointHandles[i]);
        }
    }
}

bool CikGroup::computeGroupIk(CMatrix& jacobian,CMatrix& errorVect)
{
    bool retVal=false;

    std::vector<CikElement*> validElements;
    if (getValidElements(validElements))
    {
        retVal=true;
        prepareRawJacobians(validElements,1.0);
        selectJoints(&validElements,nullptr,nullptr);
        computeDq(&validElements,true,-1,nullptr);
        errorVect=_E;
        jacobian=_jacobian;
    }
    return(retVal);
}

bool CikGroup::getValidElements(std::vector<CikElement*>& el)
{
    for (size_t i=0;i<_ikElements.size();i++)
    {
        CikElement* element=_ikElements[i];
        if (element->getIsValid())
            el.push_back(element);
    }
    return(el.size()>0);
}

void CikGroup::prepareRawJacobians(std::vector<CikElement*>& el,double interpolFact)
{
    for (size_t i=0;i<el.size();i++)
    {
        CikElement* element=el[i];
        element->prepareRawJacobian(interpolFact);
    }
}

bool CikGroup::selectJoints(std::vector<CikElement*>* validElements,std::vector<CJoint*>* allJoints,std::vector<int>* allJointDofIndices)
{ // allJoints and allJointDofIndices can contain additional joints to consider. In output, those will contain the totality
    // going through the Jacobian cols:
    if (allJoints!=nullptr)
    {
        _joints.assign(allJoints->begin(),allJoints->end());
        _jointDofIndex.assign(allJointDofIndices->begin(),allJointDofIndices->end()); // spherical joints have 3 entries
    }
    else
    {
        _joints.clear();
        _jointDofIndex.clear();
    }
    _jointHandles.clear();
    std::unordered_set<int> allJ;
    for (size_t i=0;i<_joints.size();i++)
    {
        int h=_joints[i]->getObjectHandle();
        _jointHandles.push_back(h);
        allJ.insert(3*h+_jointDofIndex[i]);
    }
    bool retVal=false;
    for (size_t elNb=0;elNb<validElements->size();elNb++)
    {
        CikElement* element=validElements->at(elNb);
        for (size_t i=0;i<element->jointHandles.size();i++)
        {
            int current=element->jointHandles[i];
            int currentDofIndex=element->jointDofIndex[i];
            retVal=true;
            if (allJ.find(3*current+currentDofIndex)==allJ.end())
            { // make sure we do not add more than one occurence of the same joint
                _joints.push_back(CEnvironment::currentEnvironment->objectContainer->getJoint(current));
                _jointHandles.push_back(current);
                _jointDofIndex.push_back(currentDofIndex);
                allJ.insert(3*current+currentDofIndex);
            }
        }
    }
    if (allJoints!=nullptr)
    {
        allJoints->assign(_joints.begin(),_joints.end());
        allJointDofIndices->assign(_jointDofIndex.begin(),_jointDofIndex.end());
    }
    return(retVal);
}

int CikGroup::computeDq(std::vector<CikElement*>* validElements,bool nakedJacobianOnly,int iteration,int(*cb)(const int*,double*,const int*,const int*,const int*,const int*,double*,double*,double*,int,int))
{   // Return value: one of ik_calc_...-values
    std::vector<int> _elementHandles; // going through the Jacobian rows
    std::vector<CikElement*> _elements; // going through the Jacobian rows
    std::vector<int> _equationType; // going through the Jacobian rows. 0-2: x,y,z, 3-5: alpha,beta,gamma, 6=jointLimits
    _jacobian.resize(0,_joints.size(),0.0);
    _E.resize(0,1,0.0);

    for (size_t elNb=0;elNb<validElements->size();elNb++)
    { // position and orientation constraints for all IK elements in that group:
        CikElement* element=validElements->at(elNb);
        for (size_t i=0;i<element->errorVector.rows;i++)
        { // We go through the rows:
            size_t rows=_jacobian.rows+1;
            size_t currentRow=rows-1;
            _jacobian.resize(rows,_joints.size(),0.0);
            _E.resize(rows,1,0.0);
            _elementHandles.push_back(element->getIkElementHandle());
            _elements.push_back(element);
            _equationType.push_back(element->equationTypes[i]);
            _E(currentRow,0)=element->errorVector(i,0);
            // Now we set the delta-parts:
            for (size_t j=0;j<element->jacobian.cols;j++)
            { // We go through the columns:
                // We search for the right entry
                int jointHandle=element->jointHandles[j];
                int dofIndex=element->jointDofIndex[j];
                size_t index=0;
                while ( (_joints[index]->getObjectHandle()!=jointHandle)||(_jointDofIndex[index]!=dofIndex) )
                    index++;
                _jacobian(currentRow,index)=element->jacobian(i,j);
            }
        }
    }

    if ((_options&ik_group_avoidlimits)!=0)
    { // handle joint limits by counter-acting:
        for (size_t jointCounter=0;jointCounter<_joints.size();jointCounter++)
        {
            CJoint* it=_joints[jointCounter];
            double minVal=it->getPositionIntervalMin();
            double range=it->getPositionIntervalRange();
            double limitMargin=it->getLimitMargin();
            double value=it->getPosition();
            double distFromMin=value-limitMargin-minVal;
            double distFromMax=value+limitMargin-minVal-range;
            double eq=0.0;
            double activate=-10.0;
            if (it->getJointType()==ik_jointtype_revolute)
            {
                if (!it->getPositionIsCyclic())
                {
                    if (distFromMin<0.0)
                    {
                        activate=1.0; // We correct in the positive direction
                        eq=-distFromMin;
                    }
                    if (distFromMax>0.0)
                    {
                        activate=-1.0; // We correct in the negative direction
                        eq=distFromMax;
                    }
                }
            }
            if (it->getJointType()==ik_jointtype_prismatic)
            {
                if ( (distFromMin<0.0)&&(fabs(distFromMin)<fabs(distFromMax)) )
                {
                    activate=1.0; // We correct in the positive direction
                    eq=-distFromMin;
                }
                if ( (distFromMax>0.0)&&(fabs(distFromMax)<fabs(distFromMin)) )
                {
                    activate=-1.0; // We correct in the negative direction
                    eq=distFromMax;
                }
            }
            if (activate>-5.0)
            { // We have to activate a joint limitation equation
                // If we are over the treshhold of more than 5%:
                // (important in case target and tooltip are within tolerance)

                int rows=int(_jacobian.rows)+1;
                int currentRow=rows-1;
                _jacobian.resize(rows,_joints.size(),0.0);
                _E.resize(rows,1,0.0);
                _elementHandles.push_back(-1);
                _equationType.push_back(6);
                _jacobian(currentRow,jointCounter)=activate;
                _E(currentRow,0)=eq;
            }
        }
    }

    if (nakedJacobianOnly)
        return(0);

    if (_elementHandles.size()==0)
        return(ik_calc_cannotinvert);

    // Now we just have to solve:
    _dQ.resize(_jacobian.cols,1,0.0);
    bool computeHere=true;
    if (cb)
    {
        int js[2]={int(_jacobian.rows),int(_jacobian.cols)};
        _jacobianPseudoinv.resize(_jacobian.cols,_jacobian.rows,0.0);
        int res=cb(js,_jacobian.data.data(),_equationType.data(),_elementHandles.data(),_jointHandles.data(),_jointDofIndex.data(),_E.data.data(),_dQ.data.data(),_jacobianPseudoinv.data.data(),_objectHandle,iteration);
        computeHere=((res&1)==0);
        if (computeHere)
        {
            if ((res&2)!=0)
            { // the callback provided a Jacobian pseudoinverse, but no dQ. We compute dQ here:
                _dQ=_jacobianPseudoinv*_E;
                computeHere=false;
            }
        }
    }
    if (computeHere)
    {
        // position/orientation weights, and element weights:
        for (size_t i=0;i<_elements.size();i++)
        {
            if (_equationType[i]<=5)
            {
                double w[3];
                _elements[i]->getWeights(w);
                if (_equationType[i]<=2)
                    _E(i,0)*=w[0]*w[2];
                else
                    _E(i,0)*=w[1]*w[2];
            }
        }

        bool useJointWeights=false;
        for (size_t j=0;j<_joints.size();j++)
        {
            if (_joints[j]->getIkWeight()!=1.0)
            {
                useJointWeights=true;
                break;
            }
        }

        int calcMethod=_calculationMethod;
        double dampingFact=_dlsFactor;

        if (calcMethod==ik_method_undamped_pseudo_inverse)
        {
            calcMethod=ik_method_damped_least_squares;
            dampingFact=0.0;
        }
        if (calcMethod==ik_method_pseudo_inverse)
        {
            calcMethod=ik_method_damped_least_squares;
            dampingFact=0.000001;
        }
        if (calcMethod==ik_method_damped_least_squares)
        { // pseudo inverse with damping and joint weights: inv(W)*transp(J)*inv(J*inv(W)*transp(J)+damp*damp*I)
            CMatrix Idamp(_jacobian.rows,_jacobian.rows);
            Idamp.clear();
            for (size_t i=0;i<Idamp.rows;i++)
                Idamp(i,i)=dampingFact*dampingFact;
            CMatrix JT(_jacobian);
            JT.transpose();
            if (useJointWeights)
            {
                CMatrix Winv(_jacobian.cols,_jacobian.cols);
                Winv.clear();
                for (size_t i=0;i<_joints.size();i++)
                    Winv(i,i)=_joints[i]->getIkWeight();
                _jacobianPseudoinv=Winv*JT*inv(_jacobian*Winv*JT+Idamp);
            }
            else
                _jacobianPseudoinv=JT*inv(_jacobian*JT+Idamp);
            _dQ=_jacobianPseudoinv*_E;
        }
        if (calcMethod==ik_method_jacobian_transpose)
        {
            _jacobianPseudoinv=_jacobian;
            _jacobianPseudoinv.transpose();
            _dQ=_jacobianPseudoinv*_E;
        }
    }

    for (size_t i=0;i<_dQ.rows;i++)
    {
        CJoint* it=_joints[i];
        if (it->getJointType()!=ik_jointtype_prismatic)
            _dQ(i,0)=atan2(sin(_dQ(i,0)),cos(_dQ(i,0)));
    }
    return(0);
}

const CMatrix& CikGroup::getJacobian() const
{
    return(_jacobian);
}

const CMatrix& CikGroup::getInvJacobian() const
{
    return(_jacobianPseudoinv);
}

const CMatrix& CikGroup::getDq() const
{
    return(_dQ);
}


void CikGroup::applyDq(const CMatrix* dq)
{ // Set the computed values. dq can be nullptr
    const CMatrix* ddq=dq;
    if (ddq==nullptr)
        ddq=&_dQ;
    for (size_t i=0;i<ddq->rows;i++)
    {
        CJoint* it=_joints[i];
        if (it->getJointType()==ik_jointtype_spherical)
        {
            C4Vector tr;
            tr.setEulerAngles(ddq[0](i,0),ddq[0](i+1,0),ddq[0](i+2,0));
            it->setSphericalTransformation(it->getSphericalTransformation()*tr);
            i+=2;
        }
        else
            it->setPosition(it->getPosition()+ddq[0](i,0));
    }
}

int CikGroup::checkDq(const CMatrix* dq,double* maxStepFact)
{ // dq can be nullptr
    const CMatrix* ddq=dq;
    if (ddq==nullptr)
        ddq=&_dQ;
    int retVal=0;
    // We check if some variations are too big:
    if (maxStepFact!=nullptr)
        maxStepFact[0]=0.0;
    for (size_t i=0;i<_joints.size();i++)
    {
        CJoint* it=_joints[i];
        if (fabs(ddq[0](i,0))>it->getMaxStepSize())
            retVal=retVal|ik_calc_stepstoobig;
        if (maxStepFact!=nullptr)
        {
            if (it->getMaxStepSize()>0.0)
            {
                double v=fabs(ddq[0](i,0))/it->getMaxStepSize();
                if (v>maxStepFact[0])
                    maxStepFact[0]=v;
            }
        }
    }

    // Check which joints hit a joint limit:
    for (size_t i=0;i<_joints.size();i++)
    {
        CJoint* it=_joints[i];
        if ( (it->getJointType()!=ik_jointtype_spherical)&&(!it->getPositionIsCyclic()) )
        {
            double nv=it->getPosition()+ddq[0](i,0);
            if ( (ddq[0](i,0)>0.0)&&(nv>it->getPositionIntervalMin()+it->getPositionIntervalRange()) )
            {
                _jointLimitHits[it->getObjectHandle()]=nv-(it->getPositionIntervalMin()+it->getPositionIntervalRange());
                retVal=retVal|ik_calc_limithit;
            }
            if ( (ddq[0](i,0)<0.0)&&(nv<it->getPositionIntervalMin()) )
            {
                _jointLimitHits[it->getObjectHandle()]=nv-it->getPositionIntervalMin();
                retVal=retVal|ik_calc_limithit;
            }
        }
    }
    return(retVal);
}

CMatrix CikGroup::pinv(const CMatrix& J,const CMatrix& E,CMatrix* Jinv)
{
    Eigen::MatrixXd jacob(J.rows,J.cols);
    for (size_t i=0;i<J.rows;i++)
    {
        for (size_t j=0;j<J.cols;j++)
            jacob(i,j)=J(i,j);
    }
    auto jacob_od=jacob.completeOrthogonalDecomposition();
    Eigen::MatrixXd e(E.rows,E.cols);
    for (size_t i=0;i<E.rows;i++)
    {
        for (size_t j=0;j<E.cols;j++)
            e(i,j)=E(i,j);
    }
    if (Jinv!=nullptr)
    {
        Eigen::MatrixXd jacobInv=jacob_od.pseudoInverse();
        Jinv->resize(jacobInv.rows(),jacobInv.cols(),0.0);
        for (int i=0;i<jacobInv.rows();i++)
        {
            for (int j=0;j<jacobInv.cols();j++)
                Jinv[0](i,j)=jacobInv(i,j);
        }
    }
    Eigen::MatrixXd dq=jacob_od.solve(e);
    CMatrix dQ(dq.rows(),dq.cols());
    for (int i=0;i<dq.rows();i++)
    {
        for (int j=0;j<dq.cols();j++)
            dQ(i,j)=dq(i,j);
    }
    return(dQ);
}

CMatrix CikGroup::inv(const CMatrix& M)
{
    Eigen::MatrixXd m(M.rows,M.cols);
    for (size_t i=0;i<M.rows;i++)
    {
        for (size_t j=0;j<M.cols;j++)
            m(i,j)=M(i,j);
    }

//    auto minv=m.inverse();
    auto m_od=m.completeOrthogonalDecomposition();
    Eigen::MatrixXd minv=m_od.pseudoInverse();

    CMatrix Minv(minv.rows(),minv.cols());
    for (int i=0;i<minv.rows();i++)
    {
        for (int j=0;j<minv.cols();j++)
            Minv(i,j)=minv(i,j);
    }
    return(Minv);
}

bool CikGroup::computeOnlyJacobian_old()
{
    std::vector<CikElement*> validElements;
    if (!getValidElements(validElements))
        return(false);

    std::vector<int> memorizedConf_handles;
    std::vector<double> memorizedConf_vals;
    CEnvironment::currentEnvironment->objectContainer->memorizeJointConfig(memorizedConf_handles,memorizedConf_vals);

    prepareRawJacobians(validElements,1.0);
    selectJoints(&validElements,nullptr,nullptr);
    int retVal=computeDq(&validElements,true,-1,nullptr);
    CEnvironment::currentEnvironment->objectContainer->restoreJointConfig(memorizedConf_handles,memorizedConf_vals);
    return(retVal!=ik_calc_cannotinvert);
}

const double*  CikGroup::getLastJacobianData_old(size_t matrixSize[2])
{ // back compat. function. Deprecated
    if (_jacobian.data.size()==0)
        return(nullptr);
    matrixSize[0]=_jacobian.cols;
    matrixSize[1]=_jacobian.rows;
    _lastJacobian_flipped=_jacobian;
    for (size_t i=0;i<_jacobian.rows;i++)
    {
        size_t jj=0;
        for (size_t j=_jacobian.cols;j>0;j--)
        {
            size_t cnt=size_t(_jointDofIndex[j-1]+1);
            j=j-cnt+1;
            for (size_t k=0;k<cnt;k++)
                _lastJacobian_flipped(i,jj++)=_jacobian(i,j-1+k);
        }
    }
    return(_lastJacobian_flipped.data.data());
}


double  CikGroup::getLastManipulabilityValue_old(bool& ok) const
{
    double retVal=0.0;
    if (_jacobian.data.size()==0)
        ok=false;
    else
    {
        ok=true;
        CMatrix JT(_jacobian);
        JT.transpose();
        CMatrix JJT(_jacobian*JT);
        retVal=sqrt(getDeterminant_old(JJT,nullptr,nullptr));
    }
    return(retVal);
}

double CikGroup::getDeterminant_old(const CMatrix& m,const std::vector<size_t>* activeRows,const std::vector<size_t>* activeColumns) const
{ // activeRows and activeColumns are nullptr by default (--> all rows and columns are active)
    // Routine is recursive! (i.e. Laplace expansion, which is not efficient for large matrices!)
    if (activeRows==nullptr)
    { // First call goes here:
        std::vector<size_t> actR;
        std::vector<size_t> actC;
        for (size_t i=0;i<m.cols;i++)
        {
            actR.push_back(i);
            actC.push_back(i);
        }
        return(getDeterminant_old(m,&actR,&actC));
    }

    // If we arrived here, we have to compute the determinant of the sub-matrix obtained
    // by removing all rows and columns not listed in activeRows, respectively activeColumns
    if (activeRows->size()==2)
    { // We compute this directly, we have a two-by-two matrix:
        double retVal=0.0;
        retVal+=m(activeRows->at(0),activeColumns->at(0))*m(activeRows->at(1),activeColumns->at(1));
        retVal-=m(activeRows->at(0),activeColumns->at(1))*m(activeRows->at(1),activeColumns->at(0));
        return(retVal);
    }

    if (activeRows->size()==3)
    { // We compute this directly, we have a three-by-three matrix:
        double retVal=0.0;
        retVal+=m(activeRows->at(0),activeColumns->at(0)) * ( (m(activeRows->at(1),activeColumns->at(1))*m(activeRows->at(2),activeColumns->at(2))) - (m(activeRows->at(1),activeColumns->at(2))*m(activeRows->at(2),activeColumns->at(1))) );
        retVal-=m(activeRows->at(0),activeColumns->at(1)) * ( (m(activeRows->at(1),activeColumns->at(0))*m(activeRows->at(2),activeColumns->at(2))) - (m(activeRows->at(1),activeColumns->at(2))*m(activeRows->at(2),activeColumns->at(0))) );
        retVal+=m(activeRows->at(0),activeColumns->at(2)) * ( (m(activeRows->at(1),activeColumns->at(0))*m(activeRows->at(2),activeColumns->at(1))) - (m(activeRows->at(1),activeColumns->at(1))*m(activeRows->at(2),activeColumns->at(0))) );
        return(retVal);
    }

    // The general routine
    std::vector<size_t> actR;
    std::vector<size_t> actC;
    double retVal=0.0;

    for (size_t colInd=1;colInd<activeColumns->size();colInd++)
        actC.push_back(activeColumns->at(colInd));
    for (size_t rowInd=0;rowInd<activeRows->size();rowInd++)
    {
        actR.clear();
        size_t i=activeRows->at(rowInd);
        for (size_t rowInd2=0;rowInd2<activeRows->size();rowInd2++)
        {
            size_t j=activeRows->at(rowInd2);
            if (j!=i)
                actR.push_back(j);
        }
        retVal+=m(i,activeColumns->at(0))*getDeterminant_old(m,&actR,&actC)*pow(-1.0,rowInd+2); // was rowInd+1 until 3.1.3 rev2.
    }
    return(retVal);
}

void CikGroup::serialize(CSerialization &ar)
{
    if (ar.isWriting())
    {
        ar.writeInt(_objectHandle);
        ar.writeString(_objectName.c_str());
        ar.writeInt(_maxIterations);
        ar.writeInt(_options);
        ar.writeFloat(float(1.0)); // prev. jointLimitWeight
        ar.writeFloat(float(5.0*degToRad));
        ar.writeFloat(float(0.01));
        ar.writeFloat(float(_dlsFactor));
        ar.writeInt(_calculationMethod);
        ar.writeInt(-1);

        unsigned char nothing=0;
        // SIM_SET_CLEAR_BIT(nothing,0,active);
        //SIM_SET_CLEAR_BIT(nothing,1,restoreIfPositionNotReached);
        //SIM_SET_CLEAR_BIT(nothing,2,restoreIfOrientationNotReached);
        //SIM_SET_CLEAR_BIT(nothing,3,doOnFail);
        //SIM_SET_CLEAR_BIT(nothing,4,doOnPerformed);
        // SIM_SET_CLEAR_BIT(nothing,5,!ignoreMaxStepSizes);
        SIM_SET_CLEAR_BIT(nothing,6,_explicitHandling_old);
        //SIM_SET_CLEAR_BIT(nothing,7,_failOnJointLimits);
        ar.writeByte(nothing);

        nothing=0;
        //SIM_SET_CLEAR_BIT(nothing,0,_correctJointLimits);
        //SIM_SET_CLEAR_BIT(nothing,1,_forbidOvershoot);
        ar.writeByte(nothing);

        ar.writeInt(int(_ikElements.size()));
        for (size_t i=0;i<_ikElements.size();i++)
            _ikElements[i]->serialize(ar);
    }
    else
    {
        while (_ikElements.size()!=0)
            removeIkElement(_ikElements[0]->getIkElementHandle());

        _objectHandle=ar.readInt();

        _objectName=ar.readString();

        _maxIterations=ar.readInt();

        _options=ar.readInt();

        ar.readFloat(); // prev. jointLimitWeight

        ar.readFloat(); // prev. _jointTreshholdAngular
        ar.readFloat(); // prev. _jointTreshholdLinear

        _dlsFactor=double(ar.readFloat());

        _calculationMethod=ar.readInt();

        ar.readInt(); // previously doOnFailOrSuccessOf

        unsigned char nothing=ar.readByte();
        // active=SIM_IS_BIT_SET(nothing,0);
        //restoreIfPositionNotReached=SIM_IS_BIT_SET(nothing,1);
        //restoreIfOrientationNotReached=SIM_IS_BIT_SET(nothing,2);
        //doOnFail=SIM_IS_BIT_SET(nothing,3);
        //doOnPerformed=SIM_IS_BIT_SET(nothing,4);
        // ignoreMaxStepSizes=!SIM_IS_BIT_SET(nothing,5);
        _explicitHandling_old=SIM_IS_BIT_SET(nothing,6);
        //_failOnJointLimits=SIM_IS_BIT_SET(nothing,7);

        nothing=ar.readByte();
        //_correctJointLimits=SIM_IS_BIT_SET(nothing,0);
        //_forbidOvershoot=SIM_IS_BIT_SET(nothing,1);

        int el=ar.readInt();
        for (int i=0;i<el;i++)
        {
            CikElement* it=new CikElement(-1);
            it->serialize(ar);
            _ikElements.push_back(it);
        }
    }
}
