/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */


#ifndef _DECPOMDPINTERFACE_H_
#define _DECPOMDPINTERFACE_H_

#include <vector>
#include <string>
#include "BeliefSparse.h"
#include <map>

using namespace std;

class DecPomdpInterface
{
private:
    /* data */
public:
    DecPomdpInterface(){};
    virtual ~DecPomdpInterface(){};
    virtual int GetNbAgents()=0;
    virtual double GetDiscount()=0;
    virtual int GetSizeOfS()=0;
    virtual int GetSizeOfJointA()=0;
    virtual int GetSizeOfJointObs()=0;
    virtual int GetSizeOfA(int agentI)=0;
    virtual int GetSizeOfObs(int agentI)=0;
    virtual vector<vector<string>> GetAllActionsVecs()=0;
    virtual vector<vector<string>> GetAllObservationsVecs() = 0;
    virtual vector<string> GetActionVec(int agentI)=0;
    virtual vector<string> GetObservationVec(int agentI) = 0;
    virtual string GetActionName(int agentI, int aI) =0;
    virtual string GetObservationName(int agentI, int oI) = 0;
    virtual vector<string> GetAllStates() = 0;
    virtual vector<int> JointToIndividualActionsIndices(int JI) = 0; 
    virtual vector<int> JointToIndividualObsIndices(int JI) = 0; 
    virtual int IndividualToJointActionIndex(vector<int>& Indicies) = 0;
    virtual int IndividualToJointObsIndex(vector<int>& Indicies) = 0;
    // virtual vector<double> GetInitBelief()=0;
    virtual double TransFunc(int sI, int JaI, int s_newI)=0;
    virtual double ObsFunc(int JoI, int s_newI, int JaI)=0;
    virtual double Reward(int sI, int JaI)=0;
    // for sparse representation
    virtual map<int,double>* GetTransProbDist(int sI, int JaI){(void)(sI);(void)(JaI); return nullptr;};
    virtual map<int,double>* GetObsFuncProbDist(int s_newI, int JaI){(void)(s_newI);(void)(JaI); return nullptr;};
    virtual BeliefSparse* GetInitialBeliefSparse(){return nullptr;};
};


#endif /* !_DECPOMDPINTERFACE_H_ */