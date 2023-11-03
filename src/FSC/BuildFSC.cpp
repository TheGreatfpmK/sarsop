/* This file has been written and/or modified by the following people:
 *
 * You Yang;
 * Vincent Thomas;
 * Francis Colas;
 * Olivier Buffet.
 * 
 * This program is an implementation of algorithm inf-JESP. */

#include "BuildFSC.h"
using namespace std;

// Belief update
BeliefSparse BeliefUpdate(BeliefSparse &b, int aI, int oI, double p_oba, PomdpInterface *pomdp)
{
    return Update(pomdp, b, aI, oI, p_oba);
};

// Check an alpha-vector exist or not in the building FSC
int FSC::CheckAlphaExist(AlphaVector &alpha)
{
    for (size_t i = 0; i < this->Nodes.size(); i++)
    {
        // check two alpha vector equal or not
        AlphaVector current_alpha = this->Nodes[i].GetAlphaVector();
        if (current_alpha.GetValues() == alpha.GetValues() && current_alpha.GetActionIndex() == alpha.GetActionIndex())
        {
            return i;
        }
    }

    return -1;
};

// Check wether given belief exists or not in the building FSC
int FSC::CheckBeliefExist(BeliefSparse &belief)
{
    for (size_t i = 0; i < this->Nodes.size(); i++)
    {
        // check two beliefs equal or not
        BeliefSparse current_belief = this->Nodes[i].GetJointBeliefSparse();
        if (belief == current_belief)
        {
            return i;
        }
    }

    return -1;
};

void FSC::ProcessNode(vector<AlphaVector> &alpha_vecs, int n_index, std::vector<Pomdpnode> &UnProcessedSet, PomdpInterface *pomdp, vector<vector<vector<int>>> &eta_vec)
{

    Pomdpnode n = this->Nodes[n_index];
    int aI = n.GetAction();
    BeliefSparse b = n.GetJointBeliefSparse();
    double w = n.GetWeight(); // Get the current weight
    for (int OI = 0; OI < ObsSize; OI++)
    {
        double pr_oba = compute_p_oba(OI, b, aI, pomdp);

        // compute a new weight
        double w_new = w * pr_oba;

        if (pr_oba == 0)
        {
            // it means we found a belief that this observation is impossible
            // ! Linking back to itself !
            eta_vec[n_index][OI][n_index] = 1; // Point back to the current node
            continue;
        }

        BeliefSparse b_new = BeliefUpdate(b, aI, OI, pr_oba, pomdp);

        // int FLAG_Belief_Exist = CheckBeliefExist(b_new);

        int alpha_newI = argmax_alpha(alpha_vecs, b_new);
        int new_aI = alpha_vecs[alpha_newI].GetActionIndex();
        Pomdpnode n_new(alpha_vecs[alpha_newI], b_new, new_aI);
        // add a descript of action name
        n_new.SetDescript(pomdp->GetAllActions()[new_aI]);

        n_new.SetWeight(w_new);

        // Need to check if an alpha-vector is already exist in FSC
        int Node_index = CheckAlphaExist(alpha_vecs[alpha_newI]);
        // If this alpha-vector is not exist, create new node and corresponding link

        if (Node_index == -1)
        {
            this->Nodes.push_back(n_new);
            int n_new_index = this->Nodes.size() - 1;
            eta_vec[n_index][OI][n_new_index] = 1;
            UnProcessedSet.push_back(n_new);
        }
        else
        {
            // if this alpha vector already exist, link to the node with existed alpha vector
            eta_vec[n_index][OI][Node_index] = 1;
            // // check belief exist or not
            // if (FLAG_Belief_Exist >= 0)
            // {
            //     cout << "Belief exist when Alpha vec exist! FLAG:"<< FLAG_Belief_Exist <<"," <<FLAG_Exist << endl;
            // }else
            // {
            //     cout << "Belief not exist but Alpha vec exist! FLAG:"<< FLAG_Exist << endl;
            //     cout << "Merge belief!"<<endl;
            //     // this->Nodes[FLAG_Exist].MergeBelief(b_new);
            // }
            // this->Nodes[Node_index].MergeBelief(b_new);
            this->Nodes[Node_index].MergeBeliefWithWeights(b_new, w_new);
        }
    }
};


void FSC::ProcessNodeNew(vector<AlphaVector> &alpha_vecs, int n_index, std::vector<Pomdpnode> &UnProcessedSet, PomdpInterface *pomdp, vector<vector<vector<int>>> &eta_vec)
{

    Pomdpnode n = this->Nodes[n_index];
    int aI = n.GetAction();
    BeliefSparse b = n.GetJointBeliefSparse();
    double w = n.GetWeight(); // Get the current weight
    for (int OI = 0; OI < ObsSize; OI++)
    {
        double pr_oba = compute_p_oba(OI, b, aI, pomdp);

        // compute a new weight
        double w_new = w * pr_oba;

        if (pr_oba == 0)
        {
            // it means we found a belief that this observation is impossible
            // ! Linking back to itself !
            eta_vec[n_index][OI][n_index] = 1; // Point back to the current node
            continue;
        }

        BeliefSparse b_new = BeliefUpdate(b, aI, OI, pr_oba, pomdp);

        // int FLAG_Belief_Exist = CheckBeliefExist(b_new);

        int alpha_newI = argmax_alpha(alpha_vecs, b_new);
        int new_aI = alpha_vecs[alpha_newI].GetActionIndex();
        Pomdpnode n_new(alpha_vecs[alpha_newI], b_new, new_aI);
        // add a descript of action name
        n_new.SetDescript(pomdp->GetAllActions()[new_aI]);

        n_new.SetWeight(w_new);

        // Need to check if an alpha-vector is already exist in FSC
        //int Node_index = CheckAlphaExist(alpha_vecs[alpha_newI]);
        // If this alpha-vector is not exist, create new node and corresponding link

        // need to check if the new belief corresponds to a node already in the FSC
        int Node_index = CheckBeliefExist(b_new);

        if (Node_index == -1)
        {
            this->Nodes.push_back(n_new);
            int n_new_index = this->Nodes.size() - 1;
            eta_vec[n_index][OI][n_new_index] = 1;
            UnProcessedSet.push_back(n_new);
        }
        else
        {
            // if this alpha vector already exist, link to the node with existed alpha vector
            eta_vec[n_index][OI][Node_index] = 1;
            // // check belief exist or not
            // if (FLAG_Belief_Exist >= 0)
            // {
            //     cout << "Belief exist when Alpha vec exist! FLAG:"<< FLAG_Belief_Exist <<"," <<FLAG_Exist << endl;
            // }else
            // {
            //     cout << "Belief not exist but Alpha vec exist! FLAG:"<< FLAG_Exist << endl;
            //     cout << "Merge belief!"<<endl;
            //     // this->Nodes[FLAG_Exist].MergeBelief(b_new);
            // }
            // this->Nodes[Node_index].MergeBelief(b_new);
            this->Nodes[Node_index].MergeBeliefWithWeights(b_new, w_new);
        }
    }
};

FSC::FSC(vector<AlphaVector> &alpha_vecs, PomdpInterface *pomdp, int type, double error_gap)
{
    this->pomdp = pomdp;
    this->type = type;
    this->ObsSize = pomdp->GetSizeOfObs();
    this->error_gap = error_gap;
    // Initialize eta
    int alpha_vecs_size = alpha_vecs.size();
    vector<vector<vector<int>>> eta_init(alpha_vecs_size + 1,
                                         vector<vector<int>>(ObsSize,
                                                             vector<int>(alpha_vecs_size + 1, 0)));

    // this->eta = eta_init;
    vector<Pomdpnode> UnProcessedSet; // Initlize a set to store the unprocessed nodes, Call it openlist
    BeliefSparse b0(*pomdp->GetInitBeliefSparse(), pomdp->GetSizeOfS());

    int alpha0I = argmax_alpha(alpha_vecs, b0);
    int aI = alpha_vecs[alpha0I].GetActionIndex();

    Pomdpnode n0(alpha_vecs[alpha0I], b0, aI);
    // add a descript of action name
    n0.SetDescript(pomdp->GetAllActions()[aI]);

    int n_index = 0; // start with n0

    // commented for test momdp formalization
    if (type == 0)
    {
        this->Nodes.push_back(n0);
    }
    else if (type == 1)
    {
        InitNodeProcess(n0, eta_init);
        n_index = 1; // start with n0
    }
    else
    {
        cerr << "Wrong argument for FSC type!" << endl;
        throw("");
    }
    UnProcessedSet.push_back(n0);

    while (!UnProcessedSet.empty())
    {
        UnProcessedSet.erase(UnProcessedSet.begin());
        ProcessNode(alpha_vecs, n_index, UnProcessedSet, pomdp, eta_init);
        n_index++;
    }
    this->size = this->Nodes.size();
    ProcessEta(eta_init);
    // Print FSC size
    cout << "Built FSC with node size:" << this->size << endl;
}

void FSC::ProcessEta(vector<vector<vector<int>>> &eta_vec)
{
    this->eta.resize(size * ObsSize * size);
    for (int nI = 0; nI < this->size; nI++)
    {
        for (int oI = 0; oI < this->ObsSize; oI++)
        {
            for (int n_newI = 0; n_newI < this->size; n_newI++)
            {
                int Index = nI * ObsSize * size + oI * size + n_newI;
                this->eta[Index] = eta_vec[nI][oI][n_newI];
            }
        }
    }
}

void FSC::PrintGraph(PomdpInterface *pomdp)
{
    cout << endl;
    cout << " -------- " << endl;
    cout << "digraph RobotFSC {" << endl;
    // define nodes in Graph
    for (int i = 0; i < this->size; i++)
    {
        if (i == 0 && this->type == 1)
        {
            cout << "n" << i << " [label = \" Init Node \"]" << endl;
        }
        else
        {

            cout << "n" << i << "[label = \" aH:  " << this->Nodes[i].GetDescript() << "\"]" << endl;
        }
    }
    cout << endl;

    for (int nI = 0; nI < this->size; nI++)
    {
        // node n_new = this->Nodes[i];
        for (int OI = 0; OI < ObsSize; OI++)
        {

            for (int n_newI = 0; n_newI < this->size; n_newI++)
            {
                // node n = this->Nodes[j];
                int Index = nI * ObsSize * size + OI * size + n_newI;
                double pr_trans = this->eta[Index];
                // Dont print self loops
                // if ( pr_trans > 0 )
                if (pr_trans > 0 && nI != n_newI)
                {
                    cout << "n" << nI << " -> "
                         << "n" << n_newI << "[label = \"oh: " << pomdp->GetAllObservations()[OI] << ", pb: " << pr_trans << " \"]" << endl;
                }
            }
        }
    }
    cout << "}" << endl;
}

vector<Pomdpnode> &FSC::GetNodes()
{
    return this->Nodes;
};

// be attention! n0 is the start node at b0, not init node which is node at t=-1
void FSC::InitNodeProcess(Pomdpnode n0, vector<vector<vector<int>>> &eta_vec)
{
    Pomdpnode InitNode;
    if (this->Nodes.size() != 0)
    {
        cout << "Current Nodes size:" << this->Nodes.size() << endl;
        cout << "Error! Init Node must be added at beginning with Nodes size = 0!" << endl;
        throw("Error!");
    }
    this->Nodes.push_back(InitNode); // Index 0
    this->Nodes.push_back(n0);       // Index 1
    for (int OI = 0; OI < ObsSize; OI++)
    {
        eta_vec[0][OI][1] = 1;
    }
}

int FSC::NextNode(int nI, int oI)
{
    // need change n_nextI start from 0
    for (int n_nextI = 0; n_nextI < this->size; n_nextI++)
    {
        int Index = nI * ObsSize * size + oI * size + n_nextI;

        if (this->eta[Index])
        {
            return n_nextI;
        }
    }
    return -1;
};

double FSC::PolicyEvaluation()
{
    bool conv = false;
    int iter = 0;
    double gamma = this->pomdp->GetDiscount();
    vector<AlphaVector> V_a = BuildInitValueFromFSC();
    cout << "State Size: " << this->pomdp->GetSizeOfS() << endl;
    clock_t startTime, endTime;

    while (!conv)
    {
        double max_norm = 0;
        startTime = clock();
        if (type == 1)
        {
            max_norm = IterValueFunc(V_a, gamma);
        }
        else
        {
            max_norm = IterValueFuncMomdp(V_a, gamma);
        }

        endTime = clock();
        cout << "Iter: " << iter << ", error: " << max_norm << ", T: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
        if (max_norm < error_gap)
        {
            conv = true;
            cout << "Converged at iter " << iter << endl;
        }
        iter++;
    }
    return EvaluationWithAlphaVecs(this->pomdp, V_a);
};

double FSC::IterValueFunc(vector<AlphaVector> &V, double gamma)
{
    double max_norm = 0;
    PomdpInterface *Pb = this->pomdp;
    int SizeNode = this->size;
    int SizeS = Pb->GetSizeOfS();
    // int SizeO = Pb->GetSizeOfObs();
    int nI_start = 1;

    for (int nI = nI_start; nI < SizeNode; nI++)
    {
        // be carefull here, NI is start from 1, but AlphaVector index is from 0, so NI -1
        int V_nI = nI - 1;
        int aI = Nodes[nI].GetAction();
        for (int sI = 0; sI < SizeS; sI++)
        {
            double alpha_nI_sI_temp = Pb->Reward(sI, aI);
            double sum_temp = 0;

            // sparse representation
            // map<int, double>* prob_dist_trans = Pb->GetTransProbDist(sI,aI);
            // map<int, double>::iterator it;
            unordered_map<int, double> *prob_dist_trans = Pb->GetTransProbDist(sI, aI);
            unordered_map<int, double>::iterator it;

            for (it = prob_dist_trans->begin(); it != prob_dist_trans->end(); it++)
            {
                int s_newI = it->first;
                double pr_s_newI = it->second;

                map<int, double> *prob_dist_obs = Pb->GetObsFuncProbDist(s_newI, aI);
                map<int, double>::iterator it_obs;

                for (it_obs = prob_dist_obs->begin(); it_obs != prob_dist_obs->end(); it_obs++)
                {
                    int n_newI = this->NextNode(nI, it_obs->first);

                    // be carefull here, NI is start from 1, but AlphaVector index is from 0, so NI -1
                    int V_n_newI = n_newI - 1;

                    sum_temp += pr_s_newI * it_obs->second * V[V_n_newI][s_newI];
                }
            }

            alpha_nI_sI_temp += gamma * sum_temp;

            double norm1 = fabs(V[V_nI][sI] - alpha_nI_sI_temp);
            // sum_norm += norm1;
            max_norm = max_norm > norm1 ? max_norm : norm1;
            V[V_nI].ChangeValue(sI, alpha_nI_sI_temp);
        }
    }
    return max_norm;
};

double FSC::IterValueFuncMomdp(vector<AlphaVector> &V, double gamma)
{
    double max_norm = 0;
    PomdpInterface *Pb = this->pomdp;
    int SizeNode = this->size;
    int SizeS = Pb->GetSizeOfS();
    int nI_start = 0;
    for (int nI = nI_start; nI < SizeNode; nI++)
    {
        int V_nI = nI;
        int aI = Nodes[nI].GetAction();

        // map<int, double>* sI_b0_sparse = this->pomdp->GetInitBeliefSparse();
        // map<int, double>::iterator it_sI;
        // for (it_sI = sI_b0_sparse->begin(); it_sI != sI_b0_sparse->end(); it_sI++)
        // {
        //     int sI = it_sI->first;
        //     double alpha_nI_sI_temp = Pb->Reward(sI, aI);
        //     double sum_temp = 0;

        for (int sI = 0; sI < SizeS; sI++)
        {
            double alpha_nI_sI_temp = Pb->Reward(sI, aI);
            double sum_temp = 0;

            // sparse representation
            // map<int, double>* prob_dist_trans = Pb->GetTransProbDist(sI,aI);
            // map<int, double>::iterator it;

            unordered_map<int, double> *prob_dist_trans = Pb->GetTransProbDist(sI, aI);
            unordered_map<int, double>::iterator it;

            for (it = prob_dist_trans->begin(); it != prob_dist_trans->end(); it++)
            {
                int s_newI = it->first;
                double pr_s_newI = it->second;

                map<int, double> *prob_dist_obs = Pb->GetObsFuncProbDist(s_newI, aI);
                map<int, double>::iterator it_obs;

                for (it_obs = prob_dist_obs->begin(); it_obs != prob_dist_obs->end(); it_obs++)
                {
                    int n_newI = this->NextNode(nI, it_obs->first);
                    int V_n_newI = n_newI;
                    sum_temp += pr_s_newI * it_obs->second * V[V_n_newI][s_newI];
                }
            }

            alpha_nI_sI_temp += gamma * sum_temp;

            double norm1 = fabs(V[V_nI][sI] - alpha_nI_sI_temp);
            max_norm = max_norm > norm1 ? max_norm : norm1;
            V[V_nI].ChangeValue(sI, alpha_nI_sI_temp);
        }
    }
    return max_norm;
};

vector<AlphaVector> FSC::BuildInitValueFromFSC()
{
    vector<AlphaVector> V;
    int nI_start = 0;
    if (type == 1)
    {
        nI_start = 1;
    }
    for (int nI = nI_start; nI < this->size; nI++)
    {
        V.push_back(this->Nodes[nI].GetAlphaVector());
        // V[nI].Print();
    }

    return V;
}

void FSC::ExportFSC(string filename)
{
    ofstream fp(filename.c_str());
    fp << "agent: "
       << "AgentI" << endl;
    fp << "nodes: ";

    int nI_start = 0;
    if (type == 1)
    {
        nI_start = 1;
        fp << "99999 ";
    }

    for (size_t nI = nI_start; nI < this->Nodes.size(); nI++)
    {
        fp << this->Nodes[nI].GetAction() << " ";
    }
    fp << endl;

    // T: obs_I : start-node_I : end-node_I %f

    for (size_t nI = 0; nI < this->Nodes.size(); nI++)
    {
        for (int oI = 0; oI < this->ObsSize; oI++)
        {
            for (size_t n_newI = 0; n_newI < this->Nodes.size(); n_newI++)
            {
                // check prob is not 0
                int Index = nI * ObsSize * size + oI * size + n_newI;
                double temp_pr = this->eta[Index];
                if (temp_pr > 0)
                {
                    fp << "T: " << oI << " : " << nI << " : " << n_newI << " " << temp_pr << endl;
                }
            }
        }
    }
}

void FSC::ExportFSCNew(string filename)
{
    ofstream fp(filename.c_str());
    fp << "# Transitions are tuples (current node, observation, next node, probability)\n\n";
    fp << "action labels: ";
    for (size_t actIndex = 0; actIndex < pomdp->GetAllActions().size(); actIndex++)
    {
        fp << pomdp->GetAllActions()[actIndex] << " ";
    }
    fp << std::endl;
    fp << "observation labels: ";
    for (size_t obsIndex = 0; obsIndex < pomdp->GetAllObservations().size(); obsIndex++)
    {
        fp << pomdp->GetAllObservations()[obsIndex] << " ";
    }
    fp << std::endl;
    fp << "nodes: ";

    int nI_start = 0;
    if (type == 1)
    {
        nI_start = 1;
    }

    for (size_t nI = nI_start; nI < this->Nodes.size(); nI++)
    {
        fp << this->Nodes[nI].GetAction() << " ";
    }
    fp << endl;

    // T: obs_I : start-node_I : end-node_I %f

    for (size_t nI = 0; nI < this->Nodes.size(); nI++)
    {
        for (int oI = 0; oI < this->ObsSize; oI++)
        {
            for (size_t n_newI = 0; n_newI < this->Nodes.size(); n_newI++)
            {
                // check prob is not 0
                int Index = nI * ObsSize * size + oI * size + n_newI;
                double temp_pr = this->eta[Index];
                if (temp_pr > 0)
                {
                    fp << nI << " " << oI << " " << n_newI << " " << temp_pr << endl;
                }
            }
        }
    }
}

double FSC::GetTransitionProb(int n_newI, int oI, int nI)
{
    int Index = nI * ObsSize * size + oI * size + n_newI;
    return this->eta[Index];
};

double FSC::MaxNormDiff(BeliefSparse &b1, BeliefSparse &b2)
{
    double max_norm = 0;
    for (int si = 0; si < b1.GetSize(); si++)
    {
        double norm1 = fabs(b1[si] - b2[si]);
        max_norm = max_norm > norm1 ? max_norm : norm1;
    }
    return max_norm;
};
