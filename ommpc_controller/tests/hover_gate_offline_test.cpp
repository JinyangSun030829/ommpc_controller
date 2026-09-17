#include "stable_hover.h"
#include <iostream>
#include <limits>

void require(bool x, const char* message) { if (!x) throw std::runtime_error(message); }
int main() {
    using Eigen::Vector3d;
    const Vector3d zero = Vector3d::Zero();
    minisnap::StableHover gate;
    for (int i=0;i<=100;++i) gate.update(zero,zero,10+i*.01,20+i*.01);
    require(gate.stable(21),"stable full window");
    // A small difference between scheduling time and sensor time must not
    // periodically evict the full one-second measurement anchor.
    gate.reset();
    for(int i=0;i<=300;++i) {
        double stamp=10+i*.01,wall=20+i*.01+(i%2?.003:0);
        gate.update(zero,zero,stamp,wall);
        if(i>=110) require(gate.stable(wall),"jitter must not reopen stable gate");
    }
    gate.reset();
    for (int i=0;i<=100;++i) gate.update(zero,zero,10+i*.01,20+i*.01);
    require(!gate.stable(21.3),"stale wall sample");
    gate.update(zero,Vector3d(.11,0,0),11.01,21.01);
    require(!gate.stable(21.01),"speed reset");
    gate.reset();
    for(int i=0;i<150;++i) gate.update(zero,zero,10,20+i*.01);
    require(!gate.stable(21.49),"repeated sample cannot create stability");
    gate.reset();
    for(int i=0;i<=100;++i) gate.update(Vector3d(i*.0004,0,0),zero,10+i*.01,20+i*.01);
    require(gate.stable(21),"small position span allowed");
    gate.update(Vector3d(.20,0,0),zero,11.01,21.01);
    require(!gate.stable(21.01),"position span reset");
    gate.update(zero,zero,11.02,21.02,false);
    require(!gate.stable(21.02),"ineligible flight mode reset");
    for(int i=0;i<=110;++i) gate.update(zero,zero,12+i*.01,22+i*.01);
    gate.update(zero,zero,13.5,23.5);
    require(!gate.stable(23.5),"sample gap resets window");
    gate.update(zero,zero,12.5,23.51);
    require(!gate.stable(23.51),"clock reversal resets window");
    minisnap::ActivationOptions o;
    auto reject = [&](Vector3d p,Vector3d v,Vector3d sv,Vector3d sa,Vector3d sj,double age,double ready,bool stable,bool permitted) {
        return minisnap::activationRejection(o,p,v,zero,sv,sa,sj,age,ready,stable,permitted);
    };
    require(reject(zero,zero,zero,zero,zero,.01,.1,true,true).empty(),"valid activation");
    require(!reject(zero,zero,zero,zero,zero,.01,3,true,true).empty(),"READY expiry");
    require(!reject(Vector3d(.11,0,0),zero,zero,zero,zero,.01,.1,true,true).empty(),"start position mismatch");
    require(!reject(zero,Vector3d(.11,0,0),zero,zero,zero,.01,.1,true,true).empty(),"moving activation");
    require(!reject(zero,zero,zero,zero,zero,.3,.1,true,true).empty(),"stale state");
    require(!reject(zero,zero,zero,zero,zero,.01,.1,false,true).empty(),"unstable activation");
    require(!reject(zero,zero,zero,zero,zero,.01,.1,true,false).empty(),"flight permission");
    for(int d=0;d<3;++d) {
        Vector3d v=zero,a=zero,j=zero;
        (d==0?v:(d==1?a:j))(0)=.01;
        require(!reject(zero,zero,v,a,j,.01,.1,true,true).empty(),"nonzero hover boundary");
    }
    Vector3d bad=zero;bad(0)=std::numeric_limits<double>::quiet_NaN();
    require(!reject(bad,zero,zero,zero,zero,.01,.1,true,true).empty(),"nonfinite state");
    std::cout<<"ALL HOVER / ACTIVATION OFFLINE TESTS PASS\n";
}
