

#ifndef _FSC_UTILS_H_
#define _FSC_UTILS_H_

#include "MOMDP.h"
#include "POMDP.h"
#include "BackupAlphaPlaneMOMDP.h"

#include <list>
#include <map>
#include <set>
#include <vector>


AlphaPlane argmax_alpha(list<SharedPointer<AlphaPlane>> alphas, belief_vector& belief);

int argmax_alpha_index(list<SharedPointer<AlphaPlane>> alphas, belief_vector& belief);

double alpha_belief_value(AlphaPlane alpha, belief_vector& belief);

double belief_action_observation_probability(belief_vector& belief, int action_index, int observation_index, POMDP* pomdp);

bool belief_exists(belief_vector& belief);


#endif