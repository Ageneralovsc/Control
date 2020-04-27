#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <vector>
#include <array>

class CONTROL {
public:
    CONTROL();
    ~CONTROL();
    
    bool goawayflag;
    bool isStartingManeuver;
    bool isAtDomainNow;
    
    double lambda;
    double Rmin;
    double H;
    double thrhld;
    double ulast;
    double vlast;
    
    double CONTROL_V_SPL;
    double HEURISTIC_GOAWAY_BACK_VEL;
    double HEURISTIC_GOAWAY_FORW_VEL;
    double HEURISTIC_BACK_VEL;
    double HEURISTIC_FORW_VEL;
    
    bool Run(double* DKpsi, double* heur, double* uv);
    bool isAtDomain(double D, double psi);
    
private:
    std::vector<std::array<double, 2>> P; //  Lyapunov function matrix
    double Vfun(double z1, double z2, double theta, double Rmin, double lambda, std::vector<std::array<double, 2>> &Pmatrix); // Lyapunov function
    void Spline(double *DKPsi, double *uv); // Spline following feedback linearization algorithm
    void Heuristic(double* DKpsi, double *heur, double *uv); // Starting maneuver (heuristic) algorithm
};


#endif
