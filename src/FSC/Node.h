

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
        Node(AlphaPlane& alpha, int alpha_index, belief_vector& belief, int act) {
            this->best_alpha = alpha;
            this->alpha_index = alpha_index;
            this->belief = belief;
            this->action_index = act;
            this->value = alpha_belief_value(this->best_alpha, this->belief);
        };
        Node(){};
        AlphaPlane get_alpha_vector() {return this->best_alpha;};
        int get_alpha_index() {return this->alpha_index;};
        belief_vector get_belief() {return this->belief;};
        int get_action() {return this->action_index;};
        double get_value() {return this->value;};


        double get_weight() {return this->weight;};
        void set_weight(double w) {this->weight = w;};

        void merge_belief_with_weights(belief_vector& belief, double weight_new) {
            double updated_weight = weight_new + this->weight;
            int index = 0;
            for (auto belief_state = belief.data.begin(); belief_state != belief.data.end(); ++belief_state) {
                double new_belief_state = (this->belief.data[index].value*(this->weight/updated_weight) + belief.data[index].value*(weight_new/updated_weight));

                if (new_belief_state > 0) {
                    this->belief.data[index].value = new_belief_state;
                }

                index++;
            }
            this->weight = updated_weight;
        }

        //~Node(){};
};



#endif _NODE_H_