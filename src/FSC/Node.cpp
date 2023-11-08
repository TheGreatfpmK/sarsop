


#include "Node.h"


using namespace std;

Node::Node(int alpha_index, vector<double> belief, int act) {
    this->alpha_index = alpha_index;
    //vector<double> my_belief(belief.data.size(), 0);
    // for (auto b_iter = belief.data.begin(); b_iter < belief.data.end(); ++b_iter) {
    //     my_belief[b_iter->index] = b_iter->value;
    // }
    this->belief = belief;
    this->action_index = act;
};

Node::Node(const Node& node){
    this->alpha_index = node.alpha_index;
    this->belief = node.belief;
    this->action_index = node.action_index;
}


Node::~Node(void){
    ;
}


belief_vector Node::get_belief() {
    belief_vector bel_vector(this->belief.size());
    int c_index = 0;
    for (auto b_iter = this->belief.begin(); b_iter < this->belief.end(); ++b_iter) {
        bel_vector.push_back(c_index, *b_iter);
        c_index++;
    }

    return bel_vector;
}



void Node::merge_belief_with_weights(vector<double> belief, double weight_new) {
    double updated_weight = weight_new + this->weight;
    int index = 0;
    for (auto belief_state = belief.begin(); belief_state != belief.end(); ++belief_state) {
        double new_belief_state = (this->belief[index]*(this->weight/updated_weight) + belief[index]*(weight_new/updated_weight));

        if (new_belief_state > 0) {
            this->belief[index] = new_belief_state;
        }

        index++;
    }
    this->weight = updated_weight;
}

