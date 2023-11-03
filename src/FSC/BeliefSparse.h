/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */


#ifndef _BELIEFSPARSE_H_
#define _BELIEFSPARSE_H_

#include <iostream>
#include <vector>
#include <map>

using namespace std;

class BeliefSparse
{
private:
    std::map<int, double> pb_states;
    int state_size = -1;
public:
    BeliefSparse();
    BeliefSparse(std::vector<double>& vec_pb_states);
    BeliefSparse(const std::map<int, double>& b, int state_size);
    void PrintBeliefSparse();
    ~BeliefSparse(){};
    void GetValues(std::map<int, double>& b, int state_size);
    std::map<int, double>* GetBeliefSparse();
    BeliefSparse& operator=(const BeliefSparse& o) = default;
    double operator[](int i);
    bool operator==(BeliefSparse& o);
    int GetSize();
    void InsertValue(int key, double value);
    void SetSize(int sizeofS);

};



#endif /* !_BELIEFSPARSE_H_ */
