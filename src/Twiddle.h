#ifndef TWIDDLE_H
#define TWIDDLE_H

#include "PID.h"
#include<iostream>
#include<vector>

using namespace std;


class Twiddle{

    public:


        vector<double> dp;
        bool isInitialized;
	    bool flag_2;
	    bool flag_1;
	    int step;
	    int step_total;
	    int i;
	    double bestError;
	    double error_total;
	    PID pid;

        Twiddle(PID pid, double maxsteps);
        virtual ~Twiddle();
        void twiddle(double cte);
};

#endif