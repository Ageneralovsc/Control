#include "Control.h"
#include <math.h>
#include <stdio.h>
#include <vector>
#include <array>

CONTROL::CONTROL()
{
//
// Init before start
//
    lambda = 1;
    Rmin = 2.5; 
    H = 0.7; // [m] base between front and back
    thrhld = 30*M_PI/180; // [rad] kinematic constraint for steering angle
    Rmin = H/tan(thrhld); // [m]
    CONTROL_V_SPL = 0.6; // [0..1] velocity value for spline following 
    HEURISTIC_GOAWAY_BACK_VEL = -0.3; //[-1..0] velovity values for starting maneuver (heuristic goaway)
    HEURISTIC_GOAWAY_FORW_VEL = 0.3;  //[0..1] velovity values for starting maneuver (heuristic goaway)
    HEURISTIC_BACK_VEL = -0.35;        //[-1..0] velovity values for starting maneuver (heuristic)
    HEURISTIC_FORW_VEL = 0.4;         //[0..1] velovity values for starting maneuver (heuristic)
    
    isAtDomainNow = false;
    isStartingManeuver = false;
    ulast = 0;
    vlast = 0;
}

CONTROL::~CONTROL()
{

}

void CONTROL::Spline(double *DKpsi, double *uv) 
{
//
// Spline following - feedback linearization control algorithm
// output: uv[0] - curvature [m^-1], uv[1] - velocity [0..1]
//
    // Spline following control parameters
    double D = DKpsi[0];
    double K = DKpsi[1];
    double psi = DKpsi[2];
    
    double alpha = 0;
    double u = 0;
    double v = 0;
    

    double sigma = 2*lambda*sin(psi)+lambda*lambda*D;

    u = K*cos(psi)/(K*D+1) + (-sigma)/cos(psi);
    v = CONTROL_V_SPL;

    
    alpha = atan2(u*H,1);
    
    if (alpha>thrhld)
        alpha=thrhld;
    else if (alpha<-thrhld)
        alpha=-thrhld;    
    
    u = tan(alpha)/H;

    
    uv[0] = u;
    uv[1] = v;   
}

void CONTROL::Heuristic(double* DKpsi, double *heur, double *uv) 
{
//
// Starting maneuver (Heuristic) algorithms to go to target start of trajectory;
// output: uv[0] - curvature [m^-1], uv[1] - velocity [-1..1]
//
    // heuristic parameters
    double dalpha = heur[0];
    double dang = heur[1];
    double comp = heur[2];
    double comp2 = heur[3];
    double params = heur[4];
    
    double alpha = 0;
    double u = 0;
    double v = 0;
    double k_dang = 5;
    double d_comp = 0.1;

    if (goawayflag) {
        // GoAway algorithm
        if (comp2 > 0) { //dot(tau_vec,orient) >= 0
            v = HEURISTIC_GOAWAY_BACK_VEL;
            u = k_dang*tan(dang);
        }
        else { //if (dot(tau_vec,orient) <= 0) 
            v = HEURISTIC_GOAWAY_FORW_VEL;
            u = -k_dang*tan(dang); 
        }
    } 
    else { //if (goawayflag == false)
        // MovingToStart 
        if (comp >= (0.8 + d_comp))
            v = HEURISTIC_FORW_VEL;
        else if (comp <= (0.8 - d_comp))
            v = HEURISTIC_BACK_VEL;
        else {
            if (vlast!=0)
                v = vlast;
            else
                v = HEURISTIC_BACK_VEL;  
        }
        
        if ( (v > 0) && (DKpsi[2] != M_PI) )
            u = -k_dang*tan(dalpha);
        else
            u = -params/Rmin;
    }  
    
    alpha = atan2(u*H,1);
    
    if (alpha>thrhld)
        alpha=thrhld;
    else if (alpha<-thrhld)
        alpha=-thrhld;    
    
    u = tan(alpha)/H;

    
    uv[0] = u;
    uv[1] = v;   
}


bool CONTROL::Run(double* DKpsi, double* heur, double* uv)
{
//
// Processing control switching between spline following and starting maneuver (heuristic) algorithms
// output: uv[0] - curvature [m^-1],  uv[1] - velocity [-1..1]
//    
    bool ret = true;

    isAtDomainNow = isAtDomain(DKpsi[0], DKpsi[2]);

    if  ( (isStartingManeuver) && (!isAtDomainNow ) ) {
        // Starting maneuver (heuristic)
        Heuristic(DKpsi, heur, uv);
    }
    else if (isAtDomainNow){
        // Continue with spline following
        Spline(DKpsi, uv);
    }
    else{
        printf("[CONTROL::Run]: Emergency Stop. Outside AttrDomain \n");
        uv[0] = 0;
        uv[1] = 0;  
        ret = false;
    }

    ulast = uv[0];
    vlast = uv[1];

    return ret;
};


bool CONTROL::isAtDomain(double D, double psi)
{
// 
// Is current lateral distace (D, [m]) and angular deviation (psi, [rad]) inside the Attraction Domain? 
//

    double z1 = D;
    double z2 = tan(psi);
    bool attr = false;

    // defining sets of Domain estimations for different lambda
    std::vector<std::array<double, 2>> P08{ { 36.015276255922110, 27.294581971649464 }, { 27.294581971649464, 67.657754995904200 } };
    double lambda08 = 0.8; 
    double theta08 = -44.228974888947740;

    std::vector<std::array<double, 2>> P1{ { 39.815052334731924, 30.021201049505690 }, { 30.021201049505690, 60.930005254749450 } }; 
    double lambda1 = 1; 
    double theta1 = -26.727990955940450;

    std::vector<std::array<double, 2>> P15{ { 53.590138212703700, 38.794377319214800 }, {38.794377319214800, 53.207560182062420 } }; 
    double lambda15 = 1.5; 
    double theta15 = -10.986753437494730;

    std::vector<std::array<double, 2>> P2{ {59.332830061664560, 34.913367956132944 }, {34.913367956132944, 36.199614068700980 } }; 
    double lambda2 = 2; 
    double theta2 = -4.231907625174246;


    if (Vfun(z1, z2, theta08, Rmin, lambda08, P08) <= 1){
        lambda = lambda08; 
        attr = true;
    }
    else if (Vfun(z1, z2, theta1, Rmin, lambda1, P1) <= 1){
        lambda = lambda1; 
        attr = true;
    }
    else if (Vfun(z1, z2, theta15, Rmin, lambda15, P15) <= 1){
        lambda = lambda15;
        attr = true;
    }
    else if (Vfun(z1, z2, theta2, Rmin, lambda2, P2) <= 1){
        lambda = lambda2; 
        attr = true;
    }
    else{
        attr = false;
    }

    return (attr) && (abs(psi)<M_PI/2);
}



double CONTROL::Vfun(double z1, double z2, double theta, double Rmin, double lambda, std::vector<std::array<double, 2>> &Pmatrix)
{
//
// Return the Lyapunov function value
//    
    P = Pmatrix;
    double umax = 1.0/Rmin;
    double sigma = pow(lambda,2)*z1 + 2*lambda*z2;
    double integral = 0.0;
    double kappa = umax*pow(1+z2*z2, 1.5);
    
    if (sigma >= kappa) { 
        integral = kappa*(sigma-0.5*kappa);
    }
    else if (sigma <= -kappa) {
        integral = kappa*(-sigma-0.5*kappa);
    }
    else{ //if abs(sigma) < umax*(1+z2*z2)^1.5
        integral = pow(sigma,2)/2;
    }

    double val = z1*P[0][0]*z1 + z1*P[0][1]*z2 + z2*P[1][0]*z1 + z2*P[1][1]*z2 + theta*integral;
    return val;
}



