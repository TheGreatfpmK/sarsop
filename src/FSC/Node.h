

#ifndef _NODE_H_
#define _NODE_H_


#include "MOMDP.h"
#include "POMDP.h"
#include "BackupAlphaPlaneMOMDP.h"

#include "FSCUtils.h"

#include <list>
#include <map>
#include <set>
#include <vector>


using namespace std;


class Node {

    AlphaPlane best_alpha;
    int alpha_index;
    belief_vector belief;
    int action_index;
    double value;

    int belief_counter = 1;
    double weight = 1.0;

    public:
        Node(AlphaPlane& alpha, int alpha_index, belief_vector& belief, int act);
        Node(){};
        Node(const Node& node);
        AlphaPlane get_alpha_vector() {return this->best_alpha;};
        int get_alpha_index() {return this->alpha_index;};
        belief_vector get_belief() {return this->belief;};
        int get_action() {return this->action_index;};
        double get_value() {return this->value;};


        double get_weight() {return this->weight;};
        void set_weight(double w) {this->weight = w;};

        void merge_belief_with_weights(belief_vector& belief, double weight_new);

        ~Node(){};
};



#endif _NODE_H_