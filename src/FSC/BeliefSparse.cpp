/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */

#include "BeliefSparse.h"

BeliefSparse::BeliefSparse(){

};

BeliefSparse::BeliefSparse(std::vector<double> &vec_pb_states)
{
    for (size_t i = 0; i < vec_pb_states.size(); i++)
    {
        double pb_i = vec_pb_states[i];
        if (pb_i > 0)
        {
            this->pb_states[i] = pb_i;
        }
    }

    this->state_size = vec_pb_states.size();
};

BeliefSparse::BeliefSparse(const std::map<int, double> &b, int state_size)
{
    this->pb_states = b;
    this->state_size = state_size;
}

void BeliefSparse::GetValues(std::map<int, double> &b, int state_size)
{
    this->pb_states = b;
    this->state_size = state_size;
};

std::map<int, double> *BeliefSparse::GetBeliefSparse()
{
    return &this->pb_states;
};

int BeliefSparse::GetSize()
{
    return this->state_size;
}

// BeliefSparse &BeliefSparse::operator=(const BeliefSparse &o)
// {
//     this->pb_states = o.pb_states;
//     this->state_size = o.state_size;
//     return *this;
// };

double BeliefSparse::operator[](int i)
{
    // if key absent
    if ((this->pb_states).find(i) == this->pb_states.end())
    {
        // returns proba 0
        return 0.;
    }
    // key present
    else
    {
        // returns associated value
        return this->pb_states[i];
    }
};

bool BeliefSparse::operator==(BeliefSparse &o)
{
    bool res = true;
    if (o.GetSize() != this->GetSize())
    {
        throw("Two BeliefSparse point sizes don't match!");
    }

    map<int, double>::iterator it;
    for (it = this->pb_states.begin(); it != this->pb_states.end(); it++)
    {
        if (it->second != o[it->first])
        {
            res = false;
            return res;
        }
    }
    return res;
};

void BeliefSparse::PrintBeliefSparse()
{
    std::cout << "<";

    map<int, double>::iterator it;
    for (it = this->pb_states.begin(); it != this->pb_states.end(); it++)
    {
        std::cout << it->first << ":" << it->second << " ";
    }

    std::cout << ">" << std::endl;
};

void BeliefSparse::InsertValue(int key, double value)
{
    this->pb_states[key] = value;
};

void BeliefSparse::SetSize(int sizeofS)
{
    this->state_size = sizeofS;
};
