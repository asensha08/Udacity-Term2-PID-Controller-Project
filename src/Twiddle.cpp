#include "Twiddle.h"
#include <cmath>
#include <iostream>

using namespace std;

Twiddle::Twiddle(PID pid, double maxsteps){
    isInitialized=false;
    flag_1=true;
    flag_2=true;
    step=1;
    i=0;
    step_total=maxsteps;
    bestError=0;
    error_total=0;
    pid=pid;
    //dp={pid.Kp/4, pid.Ki/4, 0.1};
    dp={0.1*pid.Kp,0.1*pid.Ki,0.1*pid.Kd};
    cout<<"Init";
}

Twiddle::~Twiddle() {
}

void Twiddle::twiddle(double cte){
    
    error_total+=cte*cte;

    if(step>=step_total){
        error_total+=cte*cte;
        double err=error_total;
        cout<<"Steps";

        if(!isInitialized){
            cout<<"If entered";
            isInitialized=true;
            bestError=err;
            cout<<"P="<<pid.Kp<<'\t'<<"I="<<pid.Ki<<'\t'<<"D="<<pid.Kd<<endl;
        }
        else{
            if(flag_1){
                cout<<"flag_1";
                switch(i){
                    case 0:
                        pid.Kp += dp[i];
					    break;
                    case 1:
                        pid.Ki += dp[i];
					    break;
                    case 2:
                        pid.Kd += dp[i];
                }
                flag_1=false;
                flag_2=true;
            } else if(err<bestError){
                cout<<"bestfound";
                bestError=err;
                dp[i]*=1.1;
                i=(i+1) % 3;
                flag_1=true;
                cout<<"P_update="<<pid.Kp<<'\t'<<"I_update=="<<pid.Ki<<'\t'<<"D_update=="<<pid.Kd<<endl;
            } else if(flag_2){
                switch(i){
                    case 0:
                        pid.Kp -=2* dp[i];
					    break;
                    case 1:
                        pid.Ki -= 2*dp[i];
					    break;
                    case 2:
                        pid.Kd -= 2*dp[i];
                }
                flag_2=false;
            } else{
                switch(i){
                    case 0:
                        pid.Kp += dp[i];
					    break;
                    case 1:
                        pid.Ki += dp[i];
					    break;
                    case 2:
                        pid.Kd += dp[i];
                }
                dp[i]*=0.9;
                flag_1=true;
                i=(i+1) % 3;
            }
            
        }
        std::cout << " Final_Kp: " << pid.Kp << " Final_Ki: " << pid.Ki << " Final_Kd: " << pid.Kd << std::endl;
		step = 0;
		err = 0;    

    }
    step+=1;   
}
    