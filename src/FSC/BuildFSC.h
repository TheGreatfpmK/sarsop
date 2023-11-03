/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */

#ifndef _BUILDFSC_H_
#define _BUILDFSC_H_

#include "PomdpNode.h"
#include "PomdpInterface.h"
#include "Utils.h"
#include <map>
#include <set>


using namespace std;




class FSC
{
private:
    // std::map<node, std::map<size_t, std::map<node, double>>>  eta;
    // vector<vector<vector<double>>> eta; // swap indicies to nodeI, ohi and node_newI
    // vector<vector<vector<int>>> eta; // swap indicies to nodeI, ohi and node_newI
    vector<int> eta; // swap indicies to nodeI, ohi and node_newI
    double error_gap;
    int ObsSize;
    std::vector<Pomdpnode> Nodes;
    PomdpInterface* pomdp;
    int size;
    int type;
    // std::vector<AlphaVector> AlphaVecs;
protected:
    vector<AlphaVector> BuildInitValueFromFSC();
    double IterValueFunc(vector<AlphaVector>& V , double gamma);
    double IterValueFuncMomdp(vector<AlphaVector>& V , double gamma);

    void ProcessEta(vector<vector<vector<int>>>& eta_vec);
public:
    FSC(vector<AlphaVector>& alpha_vecs, PomdpInterface* pomdp, int type, double error_gap);
    int CheckAlphaExist(AlphaVector& alpha);
    int CheckBeliefExist(BeliefSparse &belief);
    // Need to add it afterwards
    // node StartNode; 
    void ProcessNode(vector<AlphaVector>& alpha_vecs, int n_index, std::vector<Pomdpnode>& UnProcessedSet, PomdpInterface* pomdp, vector<vector<vector<int>>>& eta_vec);
    void ProcessNodeNew(vector<AlphaVector>& alpha_vecs, int n_index, std::vector<Pomdpnode>& UnProcessedSet, PomdpInterface* pomdp, vector<vector<vector<int>>>& eta_vec);
    double GetTransitionProb(int n_newI, int oI, int nI);

    // vector<AlphaVector> GetAlphaVectors();
    void PrintGraph(PomdpInterface* decpomdp);
    vector<Pomdpnode>& GetNodes();
    ~FSC(){};
    void InitNodeProcess(Pomdpnode n0,vector<vector<vector<int>>>& eta_vec);
    void ExportFSC(string filename);
    void ExportFSCNew(string filename);
    int GetNodesSize(){return this->size;};
    int NextNode(int nI, int oI);
    double PolicyEvaluation();


    // Not used
    // int CheckBeliefExist(Belief& belief); // New added, to test if a belief is already exist 
    double MaxNormDiff(BeliefSparse& b1, BeliefSparse& b2);
};





#endif