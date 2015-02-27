#include "LKH.h"

/*
 * The RecordBetterTour function is called by FindTour each time
 * the LinKernighan function has returned a better tour.
 *
 * The function records the tour in the BetterTour array and in the
 * BestSuc field of each node. Furthermore, for each node the previous 
 * value of BestSuc is saved in the NextBestSuc field.
 *
 * Recording a better tour in the BetterTour array when the problem is 
 * asymmetric requires special treatment since the number of nodes has
 * been doubled.  
 */

void RecordBetterTour()
{
    Node *N;
    int i = 0;

    N = FirstNode;
    do {
        if (ProblemType != ATSP)
            BetterTour[++i] = N->Id;
        else if (N->Id <= Dimension / 2) {
            i++;
            if (N->Suc->Id != N->Id + Dimension / 2)
                BetterTour[i] = N->Id;
            else
                BetterTour[Dimension / 2 - i + 1] = N->Id;
        }
        N->NextBestSuc = N->BestSuc;
        N->BestSuc = N->Suc;
    }
    while ((N = N->Suc) != FirstNode);
    BetterTour[0] = BetterTour[ProblemType != ATSP ? Dimension : Dimension / 2];
}
