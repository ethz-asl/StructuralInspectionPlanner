#include "LKH.h"

/*
 * The RecordBestTour function records the current best tour in the BestTour 
 * array. 
 *
 * The function is called by LKHmain each time a run has resulted in a
 * shorter tour. Thus, when the predetermined number of runs have been
 * completed, BestTour contains an array representation of the best tour
 * found.    
 */

void RecordBestTour()
{
    int i, Dim = ProblemType != ATSP ? Dimension : Dimension / 2;

    for (i = 0; i <= Dim; i++)
        BestTour[i] = BetterTour[i];
}
