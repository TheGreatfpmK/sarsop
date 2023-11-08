


#include "FSC.h"



using namespace std;


FSC::FSC(list<SharedPointer<AlphaPlane>> alpha_vecs, POMDP* pomdp, int type, double error_gap) {
    this->pomdp = pomdp;
    this->type = type;
    this->error_gap = error_gap;
    this->ObsSize = this->pomdp->getNumObservations();

    int alpha_vecs_size = alpha_vecs.size();
    vector<vector<vector<int>>> transition_function(alpha_vecs_size + 1, vector<vector<int>>(this->ObsSize, vector<int>(alpha_vecs_size + 1, 0)));

    vector<Node> unprocessed_nodes;
    auto initial_belief = pomdp->getInitialState();
    AlphaPlane init_alpha = argmax_alpha(alpha_vecs, initial_belief);
    int init_alpha_index = argmax_alpha_index(alpha_vecs, initial_belief);
    int init_alpha_action = init_alpha.action;

    Node init_node(init_alpha, init_alpha_index, initial_belief, init_alpha_action);

    int node_index = 0;

    if (type == 0) {
        this->Nodes.push_back(init_node);
    }
    else {
        Node type_init_node;
        this->Nodes.push_back(type_init_node);
        this->Nodes.push_back(init_node);
        for (int obs_index = 0; obs_index < this->ObsSize; ++obs_index) {
            transition_function[0][obs_index][0] = 1;
        }
        node_index = 1;
    }

    unprocessed_nodes.push_back(init_node);

    while (!unprocessed_nodes.empty())
    {
        unprocessed_nodes.erase(unprocessed_nodes.begin());
        process_node(alpha_vecs, node_index, unprocessed_nodes, pomdp, transition_function);
        node_index++;
    }

    this->size = this->Nodes.size();
    produce_transition_vector(transition_function);
    cout << "Created FSC from set of alpha vectors. Number of memory nodes: " << this->size << endl;
}


void FSC::process_node(list<SharedPointer<AlphaPlane>> alpha_vecs, int node_index, vector<Node> &unprocessed_nodes, POMDP* pomdp, vector<vector<vector<int>>> &transition_function) {
    Node current_node = this->Nodes[node_index];
    int current_action_index = current_node.get_action();
    belief_vector current_belief = current_node.get_belief();
    double current_weight = current_node.get_weight();

    for (int observation_index = 0; observation_index < this->ObsSize; ++observation_index) {
        double observation_probability = belief_action_observation_probability(current_belief, current_action_index, observation_index, pomdp);
        double new_weight = current_weight * observation_probability;

        if (observation_probability == 0) {
            transition_function[node_index][observation_index][node_index] = 1; //self-loop
            continue;
        }

        belief_vector new_belief;
        pomdp->getNextBelief(new_belief, current_belief, current_action_index, observation_index);

        if (!belief_exists(new_belief)) {
            transition_function[node_index][observation_index][node_index] = 1; //self-loop
            continue;
        }

        AlphaPlane new_alpha = argmax_alpha(alpha_vecs, new_belief);
        int new_alpha_index = argmax_alpha_index(alpha_vecs, new_belief);
        int new_alpha_action = new_alpha.action;
        Node new_node(new_alpha, new_alpha_index, new_belief, new_alpha_action);
        new_node.set_weight(new_weight);

        int alpha_vector_exists = check_alpha_exists(new_alpha_index);

        if (alpha_vector_exists != -1) {
            transition_function[node_index][observation_index][alpha_vector_exists] = 1;
            this->Nodes[alpha_vector_exists].merge_belief_with_weights(new_belief, new_weight);
        }
        else {
            this->Nodes.push_back(new_node);
            int new_node_index = this->Nodes.size() - 1;
            transition_function[node_index][observation_index][new_node_index] = 1;
            unprocessed_nodes.push_back(new_node);
        }
    }
}


int FSC::check_alpha_exists(int alpha_index) {
    int index = 0;
    for (auto nodes_it = this->Nodes.begin(); nodes_it < this->Nodes.end(); ++nodes_it) {
        if (nodes_it->get_alpha_index() == alpha_index) {
            return index;
        }
        index++;
    }
    return -1;
}


void FSC::produce_transition_vector(vector<vector<vector<int>>> &transition_function) {
    this->transition_function_vector.resize(this->size * this->ObsSize * this->size);
    for (int node_index = 0; node_index < this->size; ++node_index) {
        for (int obs_index = 0; obs_index < this->ObsSize; ++obs_index) {
            for (int new_node_index = 0; new_node_index < this->size; ++new_node_index) {
                int index = node_index * this->ObsSize * this->size + obs_index * this->size + new_node_index;
                this->transition_function_vector[index] = transition_function[node_index][obs_index][new_node_index];
            }
        }
    }
}


void FSC::export_fsc(string filename, SharedPointer<MOMDP> pomdp) {
    ofstream fp(filename.c_str());
    fp << "# Transitions are tuples (current node, observation, next node, probability)\n\n";
    fp << "action labels: ";
    for (auto my_iter = pomdp->actions->begin(); my_iter != pomdp->actions->end(); ++my_iter) {
        fp << my_iter.index() << " ";
    }
    fp << endl;
    fp << "observation labels: ";
    for (auto my_iter = pomdp->observations->begin(); my_iter != pomdp->observations->end(); ++my_iter) {
        fp << my_iter.index() << " ";
    }
    fp << endl;
    fp << "nodes: ";

    int node_start = 0;
    if (this->type > 0) {
        node_start = 1;
    }

    for (int node = node_start; node < this->size; ++node) {
        fp << this->Nodes[node].get_action() << " ";
    }
    fp << endl;

    for (int node = 0; node < this->size; ++node) {
        for (int observation = 0; observation < this->ObsSize; ++observation) {
            for (int new_node = 0; new_node < this->size; ++new_node) {
                int index = node * this->ObsSize * this->size + observation * this->size + new_node;
                double probability = this->transition_function_vector[index];
                if (probability > 0) {
                    fp << node << " " << observation << " " << new_node << " " << probability << endl;
                }
            }
        }
    }

    fp.close();
}


FSC::~FSC(void) {
    ;
}

