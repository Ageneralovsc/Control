#include <stdio.h>
#include <stdlib.h>
#include "Control.cc"

int main(int argc, char **argv) {
    
    CONTROL control;
    
    control.goawayflag = false;
    control.isStartingManeuver = false;

    double DKpsi[3] = {0.15, 0, -M_PI/20};
    double heur[5] = {0, 0, 0, 0, 0};
    double uv[2] = {0, 0};
    
    
    if(control.Run(DKpsi, heur, uv))
        printf("Control is good; u = %.5f; v = %.5f \n", uv[0], uv[1]);
    else
        printf("Control stop \n");
    

    return 0;
}
