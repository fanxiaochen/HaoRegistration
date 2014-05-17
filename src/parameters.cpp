#include "parameters.h"

Parameters::Parameters()
{
    affi_rot_.setZero();
    affi_trans_.setZero();
    correspondence_.setZero();
    weights_.setZero();
}

Parameters::~Parameters()
{

}
