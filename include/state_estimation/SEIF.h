#ifndef SEIF_H
#define SEIF_H

#include "EIF.h"

class Self_EIF : public EIF
{
protected:
    int self_state_size;
    int self_measurement_size;
    EIF_data self;

public:
    Self_EIF();
    ~Self_EIF();
    virtual void computePredPairs();
    virtual void computeCorrPairs();
    virtual void setData();
};



#endif