#pragma once

namespace TASK03
{
    extern double x0, y0; // initial position, set before calling solve()
    extern double h,w; // frame height and width, set in detect_main()
    struct Measurement
    {
        double t,x,y;
    };
    struct Params
    {
        double v[4]; // x0,y0,vx0,vy0,k_log,g
        // make x0, y0 ... as aliases for v[0], v[1] ...
        double& vx0   = v[0];
        double& vy0   = v[1];
        double& k_log = v[2];
        double& g     = v[3];
        Params() : v{0,0,0,0} {} // initialize all to zero
    };
}