


#ifndef _FSC_H_
#define _FSC_H_

#include "MOMDP.h"
#include "POMDP.h"
#include "BackupAlphaPlaneMOMDP.h"

#include "Node.h"
#include "FSCUtils.h"

#include <fstream>

#include <list>
#include <map>
#include <set>
#include <vector>

using namespace std;


class FSC {
    private:
        vector<int> transition_function_vector; // indexed by node_index, observation_index, new_node_index

        int type;   // type decides the behaviour of initial state, type>0 adds explicit initial state
        int size;   // number of memory nodes
        POMDP* pomdp;
        int ObsSize;
        double error_gap;

        vector<Node> Nodes;

    public:
        FSC(list<SharedPointer<AlphaPlane>> alpha_vecs, POMDP* pomdp, int type, double error_gap);
        void process_node(list<SharedPointer<AlphaPlane>> alpha_vecs, int node_index, vector<Node> &unprocessed_nodes, POMDP* pomdp, vector<vector<vector<int>>> &transition_function);
        int check_alpha_exists(int alpha_index);
        void produce_transition_vector(vector<vector<vector<int>>> &transition_function);
        void export_fsc(string filename, SharedPointer<MOMDP> pomdp);

        ~FSC(void);
};



#endif
