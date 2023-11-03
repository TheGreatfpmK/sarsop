/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */

#ifndef _POMDPINTERFACE_H_
#define _POMDPINTERFACE_H_

#include <vector>
#include <string>
#include <map>
#include <unordered_map>

using namespace std;

class PomdpInterface
{
private:
    /* data */
public:
    PomdpInterface(/* args */){};
    virtual ~PomdpInterface(){};
    virtual double GetDiscount()=0;
    virtual int GetSizeOfS()=0;
    virtual int GetSizeOfA()=0;
    virtual int GetSizeOfObs()=0;
    // virtual std::vector<double> GetInitBelief()=0;
    virtual double TransFunc(int sI, int aI, int s_newI)=0;
    virtual double ObsFunc(int oI, int s_newI, int aI)=0;
    virtual double Reward(int sI, int aI)=0;
    virtual std::vector<string> GetAllStates() = 0;
    virtual std::vector<string> GetAllActions() = 0;
    virtual std::vector<string> GetAllObservations() = 0;
    // for sparse representation
    // virtual map<int,double>* GetTransProbDist(int sI, int aI){(void)(sI);(void)(aI); return nullptr;};
    virtual unordered_map<int,double>* GetTransProbDist(int sI, int aI){(void)(sI);(void)(aI); return nullptr;};

    virtual map<int,double>* GetObsFuncProbDist(int s_newI, int aI){(void)(s_newI);(void)(aI); return nullptr;};
    virtual map<int, double>* GetInitBeliefSparse() {return nullptr;}; 

};
// PomdpInterface::PomdpInterface(/* args */){};
// PomdpInterface::~PomdpInterface(){};

#endif /* !_POMDPINTERFACE_H_ */