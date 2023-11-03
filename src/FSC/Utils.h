/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */

/* Only include this header file once. */
#ifndef _UTILS_H_
#define _UTILS_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include "BeliefSparse.h"
#include "DecPomdpInterface.h"
#include "PomdpInterface.h"
#include "AlphaVector.h"
#include "FSCBase.h"
#include <math.h>
#include <float.h>

using namespace std;

#define MIN_B_ACC 1e-3




// double ComputeAvgL1Distance(Belief& b,vector<Belief>& bs, int NrB);
// void ExpandBeliefSet(vector<Belief>& bs, vector<Belief>& temp_bs, int NrB );
// double dot(Belief& b, AlphaVector& alpha);
// double ComputeDistanceAlphas(AlphaVector& a, AlphaVector& b);
// bool CheckConvergence(vector<AlphaVector>& a_vecs, vector<AlphaVector>& b_vecs, double err);
// void PrintBeliefSet(vector<Belief>& bs);

// InfJesp functions
bool CheckAlphaExist(vector<AlphaVector>& a_vecs, AlphaVector& alpha);
void PrintVector(vector<double>& V);
void PrintAlphaVectors(vector<AlphaVector>& a_vecs);
void PrintAllAlphaAOVecs(vector< vector<vector<AlphaVector> > >& a_ao_vecs);
double compute_p_oba(int o, BeliefSparse& b, int a, PomdpInterface* pomdp);
double compute_p_oba(int o, BeliefSparse& b, int a, DecPomdpInterface* decpomdp);
// Belief Update
BeliefSparse Update(PomdpInterface* Pb, BeliefSparse& b, int aI, int oI, double p_oba);
BeliefSparse Update(DecPomdpInterface* Pb, BeliefSparse& b, int aI, int oI, double p_oba);
// Belief Update
BeliefSparse Update(PomdpInterface* Pb, BeliefSparse& b, int aI, int oI);
BeliefSparse Update(DecPomdpInterface* Pb, BeliefSparse& b, int aI, int oI);
AlphaVector argmax_alpha_vector(vector<AlphaVector>& alpha_vecs, BeliefSparse* b);
int argmax_alpha(vector<AlphaVector>& alpha_vecs, BeliefSparse& b);
// Import Value Function
vector<AlphaVector> ImportValueFunction(const string & filename);
double EvaluationWithAlphaVecs(PomdpInterface* Pb, vector<AlphaVector>& alpha_vecs);


template<typename T>
T RandT(T _min, T _max)
{
	T temp;
	if (_min > _max)
	{
		temp = _max;
		_max = _min;
		_min = temp;
	}
	return rand() / (double)RAND_MAX *(_max - _min) + _min;
};


#endif /* !_UTILS_H_ */