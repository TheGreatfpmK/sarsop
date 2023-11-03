/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */

#ifndef _POMDPNODE_H_
#define _POMDPNODE_H_ 1

/* the include directives */
#include <iostream>
#include "AlphaVector.h"
#include "BeliefSparse.h"
#include "FSCNode.h"


#define WHEREAMI std::cerr << __FILE__ << ":" << __func__ << ":" << __LINE__ << std::endl;
#define ARGH(x) { WHEREAMI; std::cerr << "MSG: " << x << std::endl; exit(1); }

using namespace std;

class Pomdpnode : public FSCNode {
  AlphaVector best_alpha;
  BeliefSparse belief_sparse;
  // Belief belief;
  size_t action_index;
  string description;
  int belief_counter = 1; // used for average merging
  double weight = 1.0; // used for weight merging
  double V;

 public:
  Pomdpnode(AlphaVector& alpha, BeliefSparse& belief, size_t aI){
    this->best_alpha = alpha;
    this->belief_sparse = belief;
    this->action_index = aI;
    this->V = computeV();
  };

  Pomdpnode(){
    this->description = "InitNode";
  };
  ~Pomdpnode(){};
  void SetDescript(string s) {this->description = s;};
  string GetDescript() {return this->description;};
  size_t GetAction() {return this->action_index;};
  AlphaVector& GetAlphaVector() {return this->best_alpha;};
  BeliefSparse& GetJointBeliefSparse() {return this->belief_sparse;};
  double computeV(){
    double res = 0;
    map<int, double>::iterator it;
    for (it = this->belief_sparse.GetBeliefSparse()->begin(); it!= this->belief_sparse.GetBeliefSparse()->end(); it++)
    {
      res += best_alpha.GetValues()[it->first]*it->second;
    }
    return res;
  }
  // Better idea is to compare the alpha vector, need to test later
  bool operator<(const Pomdpnode & n) const {
    if(this->V < n.V){
      return true;
    }else
    {
      return false;
    }
    
  };

  void MergeBelief(BeliefSparse& b){
    int Size = this->belief_sparse.GetSize();
    map<int, double> _m_belief_sparse;
    for (int i = 0; i < Size; i++)
    {
      double pb_i = (this->belief_sparse[i]*belief_counter + b[i])/(belief_counter + 1);
      // merged_pb_states[i] = pb_i;
      if (pb_i > 0)
      {
        _m_belief_sparse[i] = pb_i;
      }
      
    }
    this->belief_sparse.GetValues(_m_belief_sparse, Size);
    this->belief_counter += 1;
  };

  void MergeBeliefWithWeights(BeliefSparse& b, double w_new){
    int Size = this->belief_sparse.GetSize();
    double updated_weight = w_new + this->weight;
    map<int, double> _m_belief_sparse;

    for (int i = 0; i < Size; i++)
    {
      double pb_i = (this->belief_sparse[i]*(this->weight/updated_weight) + b[i]*(w_new/updated_weight));
      if (pb_i > 0)
      {
        _m_belief_sparse[i] = pb_i;
      }
    }
    this->belief_sparse.GetValues(_m_belief_sparse, Size);
    this->weight = updated_weight;
  };

  double GetWeight(){
    return this->weight;
  }

  void SetWeight(double w){
    this->weight = w;
  }

  //void print();
};

#endif
