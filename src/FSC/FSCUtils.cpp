

#include "FSCUtils.h"



AlphaPlane argmax_alpha(list<SharedPointer<AlphaPlane>> alphas, belief_vector& belief) {
    AlphaPlane best_alpha;
    double best_value;
    int current_index = 0;
    for (auto alpha_v = alphas.begin(); alpha_v != alphas.end(); ++alpha_v) {
        if (current_index == 0){
            double current_value = alpha_belief_value(*alpha_v->get(), belief);
            best_alpha = *alpha_v->get();
            best_value = current_value;
            current_index++;
            continue;
        }

        double current_value = alpha_belief_value(*alpha_v->get(), belief);
        if (current_value > best_value) {
            best_value = current_value;
            best_alpha = *alpha_v->get();
        }
        current_index++;
		
	}

    return best_alpha;
}

int argmax_alpha_index(list<SharedPointer<AlphaPlane>> alphas, belief_vector& belief) {
    int best_alpha;
    double best_value;
    int current_index = 0;
    for (auto alpha_v = alphas.begin(); alpha_v != alphas.end(); ++alpha_v) {
        if (current_index == 0){
            double current_value = alpha_belief_value(*alpha_v->get(), belief);
            best_alpha = current_index;
            best_value = current_value;
            current_index++;
            continue;
        }

        double current_value = alpha_belief_value(*alpha_v->get(), belief);
        if (current_value > best_value) {
            best_value = current_value;
            best_alpha = current_index;
        }
        current_index++;
		
	}

    return best_alpha;
}

double alpha_belief_value(AlphaPlane alpha, belief_vector& belief) {
    double current_value = 0;
    int state_index = 0;
    for (auto belief_state = belief.data.begin(); belief_state != belief.data.end(); ++belief_state) {
        current_value += belief_state->value * alpha.alpha->data[state_index];
        state_index++;
    }

    return current_value;
}

double belief_action_observation_probability(belief_vector& belief, int action_index, int observation_index, POMDP* pomdp) {

    obs_prob_vector observation_probability_vector;
    pomdp->getObsProbVector(observation_probability_vector, belief, action_index);

    return observation_probability_vector.data[observation_index].value;
}



bool belief_exists(belief_vector& belief) {
    double value = 0;
    for (auto my_iter = belief.data.begin(); my_iter < belief.data.end(); ++my_iter) {
        value += my_iter->value;
    }

    return (value > 0);
}