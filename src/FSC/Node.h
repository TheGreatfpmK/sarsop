

#ifndef _NODE_H_
#define _NODE_H_


#include <list>
#include <map>
#include <set>
#include <vector>


using namespace std;


class Node {

    int alpha_index = -1;
    vector<double> belief;
    int action_index;

    int belief_counter = 1;
    double weight = 1.0;

    public:
        Node(int alpha_index, vector<double> belief, int act);
        Node(){};
        Node(const Node& node);
        int get_alpha_index() {return this->alpha_index;};
        belief_vector get_belief();
        int get_action() {return this->action_index;};


        double get_weight() {return this->weight;};
        void set_weight(double w) {this->weight = w;};

        void merge_belief_with_weights(vector<double> belief, double weight_new);

        ~Node(void);
};



#endif _NODE_H_