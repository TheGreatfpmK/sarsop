/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */

#ifndef _FSC_H_
#define _FSC_H_

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "FSCNode.h"
#include <ctime>
#include "DecPomdpInterface.h"
#include "Utils.h"

using namespace std;

class FSCBase
{
private:
    // vector<double> eta; // swap indicies to nodeI, ohi and node_newI
    vector< vector< map<int,double> > > eta;

    std::vector<int> Nodes; // Each node only store an action index
    string AgentName;
    int SizeNodes;
    int SizeObs;
    int type;
    void GenerateFSCwithParameters(int ActionsSize, int ObsSize, vector<double>& matrix_nodes_actions_pb_dist, vector<double>& matrix_nodes_transition_pb_dist ); 
    double IterValueFunc(PomdpInterface* pomdp, vector<AlphaVector>& V , double gamma);
    void ProcessEta(vector<vector<vector<int>>>& eta_vec);
    vector<AlphaVector> BuildInitValueFromFSC(PomdpInterface* pomdp);
    int label;
public:
    FSCBase(){};
    FSCBase(const string filename, int ObsSize, int type);
    FSCBase(int MaxNodeSize, int ActionsSize, int ObsSize, int type); // used for random a FSC with uniform dist
    FSCBase(int SizeNodes, int ActionsSize, int ObsSize, int type, vector<double>& matrix_nodes_actions_pb_dist, vector<double>& matrix_nodes_transition_pb_dist ); // random FSC with given paramter vecs

    ~FSCBase();
    string GetAgentDescription();
    int GetNodesSize();
    int GetActionIndexForNodeI(int nodeI);
    // virtual int GetObsSize();
    double ProbTrans(int nI, int oI, int n_newI);
    // virtual void LoadFSC(const string filename) = 0;
    // virtual void ExportFSC() = 0;
    int SampleToNewNodeI(int nI, int oI, double d_rand);
    void PrintGraph(DecPomdpInterface* DecPomdp, int agentI);
    void ExportFSC(string filename);

    // used for compare if two fscs are equal or not
    void compute_label(double sum_eta);
    bool operator < (const FSCBase& fsc) const ;

    // used for policy evaluation
    int NextNode(int nI, int oI);
    double PolicyEvaluation(PomdpInterface* pomdp, double error_gap);

    // for sparse representation
    map<int,double>* GetNodeTransProbDist(int nI, int oI);

};



#endif