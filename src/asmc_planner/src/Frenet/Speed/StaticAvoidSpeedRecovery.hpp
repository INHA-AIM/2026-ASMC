#pragma once

#include <algorithm>
#include <cmath>

// A checked nominal path can recover speed, but repeated planning of one scan
// and time spent stationary cannot grant an increase. The raised cap applies
// to the NEXT candidate, which must pass its full checks at that speed.
struct StaticAvoidSpeedRecovery {
    double cap=0;
    double last_frame=0;
    double proof_stamp=0;
    double proof_progress=0;
    unsigned frames=0;

    void ResetProof() {frames=0;proof_stamp=0;proof_progress=0;}
    void Enter(double limit) {cap=limit;ResetProof();}
    bool ObserveNominal(double stamp,double progress,double maximum,
        double confirm_time,double confirm_distance,double step) {
        if(!std::isfinite(stamp)||stamp<=0||!std::isfinite(progress)) {
            ResetProof();return false;
        }
        if(!std::isfinite(maximum)||maximum<=0||!std::isfinite(confirm_time)||confirm_time<=0||
           !std::isfinite(confirm_distance)||confirm_distance<=0||!std::isfinite(step)||step<=0) {
            ResetProof();return false;
        }
        cap=std::min(cap,maximum);
        if(stamp<last_frame) {last_frame=stamp;ResetProof();return false;}
        if(stamp==last_frame)return false;
        if((last_frame>0&&stamp-last_frame>.25)||
           (frames&&progress<proof_progress-.1))ResetProof();
        last_frame=stamp;
        if(!frames){proof_stamp=stamp;proof_progress=progress;}
        ++frames;
        if(frames<5||stamp-proof_stamp+1e-9<confirm_time||
           progress-proof_progress+1e-9<confirm_distance)return false;
        const double previous=cap;
        cap=std::min(maximum,cap+step);ResetProof();
        return cap>previous+1e-9;
    }
};
