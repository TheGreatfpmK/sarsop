/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */

#ifndef _ALPHAVECTOR_H_
#define _ALPHAVECTOR_H_ 1

#include <iostream>
#include <vector>

using namespace std;

class AlphaVector
{
private:
    std::vector<double> values;
    unsigned int action_Index;
    /* data */
public:
    AlphaVector(){};
    AlphaVector(int NbStates);
    void SetAction(int action);
    void SetValues(vector<double>& values);
    AlphaVector(vector<double>& V, const unsigned int aI);
    vector<double>& GetValues();
    unsigned int GetSize() const;
    unsigned int GetActionIndex() const;
    double operator[](unsigned int i);
    bool operator==(AlphaVector & alpha);
    bool operator!=(AlphaVector & alpha);
    void ChangeValue(unsigned int sI, double v);
    void Print();
    ~AlphaVector();
};



#endif /* !_ALPHAVECTOR_H_ */
