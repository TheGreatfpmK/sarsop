/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */

#include "AlphaVector.h"

AlphaVector::AlphaVector(vector<double> &V, const unsigned int aI)
{
    this->values = V;
    this->action_Index = aI;
}

AlphaVector::~AlphaVector()
{
}

AlphaVector::AlphaVector(int NbStates)
{
    if (NbStates < 1)
    {
        cerr << "Nb of states less than 1, error!" << endl;
        throw("ERROR");
    }

    std::vector<double> V(NbStates, 0);
    this->values = V;
};

void AlphaVector::SetAction(int action)
{
    if (action > -1)
    {
        this->action_Index = action;
    }
    else
    {
        cerr << "action index less than 0" << endl;
    }
}

void AlphaVector::SetValues(vector<double> &values)
{
    this->values = values;
}

unsigned int AlphaVector::GetSize() const
{
    return this->values.size();
};

double AlphaVector::operator[](unsigned int i)
{
    return this->values[i];
};

bool AlphaVector::operator==(AlphaVector &alpha)
{
    if (this->action_Index == alpha.action_Index)
    {
        for (unsigned int i = 0; i < this->GetSize(); i++)
        {
            if (this->values[i] != alpha[i])
            {
                return false;
            }
        }
    }
    else
    {
        return false;
    }
    return true;
};

bool AlphaVector::operator!=(AlphaVector &alpha)
{
    return !(*this == alpha);
}

unsigned int AlphaVector::GetActionIndex() const
{
    return this->action_Index;
};

void AlphaVector::ChangeValue(unsigned int sI, double v)
{
    this->values[sI] = v;
};

void AlphaVector::Print()
{
    cout << "The action index in this alpha vector: " << this->action_Index << endl;
    std::cout << "<";
    for (unsigned int i = 0; i < this->GetSize(); i++)
    {
        std::cout << this->values[i] << " ";
    }
    std::cout << ">" << std::endl;
};

vector<double> &AlphaVector::GetValues()
{
    return this->values;
};
