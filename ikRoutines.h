#pragma once

#include "ik.h"
#include "MyMath.h"
#include "joint.h"
#include "dummy.h"
#include "ikElement.h"
#include "ikGroup.h"

class CIkRoutines  
{
public:
    static void multiply(const C4X4FullMatrix& d0,const C4X4FullMatrix& dp,size_t index,std::vector<C4X4FullMatrix*>& allMatrices);
    static void buildDeltaZRotation(C4X4FullMatrix& d0,C4X4FullMatrix& dp,simReal screwCoeff);
    static void buildDeltaZTranslation(C4X4FullMatrix& d0,C4X4FullMatrix& dp);
    static CMatrix getJacobian(CikElement* ikElement,C4X4Matrix& tooltipTransf,std::vector<int>* jointHandles_tipToBase=nullptr,std::vector<size_t>* jointStages_tipToBase=nullptr);
    static void performGroupIK(CikGroup* ikGroup);
};
