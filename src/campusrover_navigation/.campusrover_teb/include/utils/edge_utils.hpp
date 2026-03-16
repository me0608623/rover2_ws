#ifndef EDGE_UTILS_HPP
#define EDGE_UTILS_HPP

#include <cmath>

namespace EdgeUtils{
    inline double penaltyBoundToInterval(const double& var,const double& a, const double& b, const double& epsilon){
        if (var < a){
            return (a - var);
        }
        if (var <= b){
            return 0.;
        }
        else{
            return (var - b);
        }
    }

    inline double penaltyBoundToInterval(const double& var,const double& a,const double& epsilon){
        if(var < -a){
            return (-a - var);
        }
        
        if(var <= a){
            return 0.;
        }else{
            return (var - a);
        }
    }

    inline double penaltyBoundFromBelow(const double& var, const double& a,const double& epsilon){
        if (var >= a){
            return 0.;

        }else{
            return (a - var);
        }
    }

    inline double normalize_theta(double theta){
        if (theta <= -M_PI || theta > M_PI){
            double multiplier = std::floor(theta / (2*M_PI));
            theta = theta - multiplier*2*M_PI;

            if (theta >= M_PI){
                theta -= 2*M_PI;

            }else if(theta < -M_PI){
                theta += 2*M_PI;
            }
        }

        return theta;
    }

    inline double fast_sigmoid(double x){
        return x / (1 + fabs(x));
    }
};

#endif