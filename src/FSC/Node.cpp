


#include "Node.h"


using namespace std;


Node::Node(AlphaPlane& alpha, int alpha_index, belief_vector& belief, int act) {
    this->best_alpha = alpha;
    this->alpha_index = alpha_index;
    this->belief = belief;
    this->action_index = act;
    this->value = alpha_belief_value(this->best_alpha, this->belief);
};

Node::Node(const Node& node){
    this->best_alpha = node.best_alpha;
    this->alpha_index = node.alpha_index;
    this->belief = node.belief;
    this->action_index = node.action_index;
}



void Node::merge_belief_with_weights(belief_vector& belief, double weight_new) {
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

