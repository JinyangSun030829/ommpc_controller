#include "traj_tracking_controller/box_flight_safety.hpp"
#include <chrono>
#include <iostream>
#include <random>

void require(bool value,const char *message)
{
    if (!value) throw std::runtime_error(message);
}
int main()
{
    using namespace flight_safety;
    using Eigen::Vector3d;
    BoxOptions box; box.enabled=true;
    box.minimum=Vector3d(-10,-10,-10); box.maximum=Vector3d(10,10,10);
    box.margin.setZero(); box.validate();
    require(box.contains(Vector3d(10,10,10)),"zero margin uses original faces");
    BoxOptions inset=box; inset.margin.setConstant(.5); inset.validate();
    require(inset.contains(Vector3d(9.5,9.5,10)) &&
            !inset.contains(Vector3d(9.6,0,0)),"0.5 margin shrinks XY faces, not Z");
    require(inset.clampInside(Vector3d(11,11,10)).isApprox(Vector3d(9.5,9.5,10)),
            "endpoint clamp must not contract Z");
    BoxOptions landing_box=inset;
    landing_box.minimum.z()=-.1; landing_box.maximum.z()=.2; landing_box.validate();
    BoxGuard landing_guard; landing_guard.configure(landing_box);
    require(landing_guard.segmentAllowed(Vector3d(0,0,.1),Vector3d(0,0,-.1)),
            "XY margin does not reject relative LAND target on original Z lower face");
    require(!landing_guard.segmentAllowed(Vector3d(0,0,.1),Vector3d(0,0,-.11)),
            "original Z bounds still enforced");
    bool invalid=false;
    try { auto bad=box; bad.margin.setConstant(10); bad.validate(); }
    catch (const std::invalid_argument &) { invalid=true; }
    require(invalid,"empty inset box rejected");
    BoxGuard guard; guard.configure(box);
    Eigen::MatrixXd overshoot(3,3); overshoot.setZero();
    overshoot(0,0)=-48; overshoot(0,1)=48; // x(0)=x(1)=0, x(.5)=12.
    require(!guard.polynomialAllowed(overshoot,1),"interior overshoot rejected despite safe endpoints");
    Eigen::MatrixXd linear=Eigen::MatrixXd::Zero(3,2);
    linear(0,0)=2; require(guard.polynomialAllowed(linear,1),"safe whole polynomial accepted");
    require(!guard.segmentAllowed(Vector3d(0,0,0),Vector3d(0,0,-11)),
            "LAND/TAKEOFF vertical target out of bounds rejected");

    BoxStop stop; stop.configure(box);
    Reference stationary; stationary.p=Vector3d(0,0,2);
    auto check_stationary=[&]() {
        require(stop.tryStart(stationary,3,4),"stationary stop starts");
        for(double t : {0.0,.01,.049,.05,.2}) {
            const auto r=stop.at(t);
            require((r.p-stationary.p).norm()<1e-12 && r.v.norm()==0 && r.a.norm()==0 &&
                    stop.jerk(t).norm()==0,"stationary stop has initialized zero derivatives");
        }
    };
    check_stationary();
    Reference fast; fast.v=Vector3d(5,0,0);
    require(stop.tryStart(fast,3,4),"fast stop inside box found");
    const auto end=stop.at(stop.duration());
    require(end.p.x()<10 && end.v.norm()==0 && end.a.norm()==0 &&
            stop.jerk(stop.duration()).norm()==0,"safe zero-v/a/j stop endpoint");
    SmoothStop old; old.start(fast,3,4);
    require(old.at(old.duration()).p.x()>10,"fixture demonstrates previous overshoot");
    std::cout<<"5 m/s: old distance="<<old.at(old.duration()).p.x()
             <<" m; box stop distance="<<end.p.x()<<" m, T="<<stop.duration()<<" s\n";
    check_stationary(); // Reuse after a moving stop must not retain its derivatives.

    Reference impossible; impossible.p=Vector3d(9.99,0,0); impossible.v=Vector3d(5,0,0);
    require(!stop.tryStart(impossible,3,4),"outward high speed near wall has no accepted curve");
    Reference over_acceleration; over_acceleration.a=Vector3d(4,0,0);
    require(!stop.tryStart(over_acceleration,3,4),"no silent acceleration-limit expansion");
    std::mt19937 random(42);
    std::uniform_real_distribution<double> sample(-2,2);
    const auto begin=std::chrono::steady_clock::now();
    int successes=0;
    for (int trial=0;trial<60;++trial)
    {
        Reference initial;
        initial.p=Vector3d(sample(random),sample(random),sample(random));
        initial.v=Vector3d(sample(random),sample(random),sample(random));
        initial.a=Vector3d(.3*sample(random),.3*sample(random),.3*sample(random));
        if (!stop.tryStart(initial,3,4)) continue;
        ++successes;
        const auto first=stop.at(0);
        require((first.p-initial.p).norm()<1e-10 && (first.v-initial.v).norm()<1e-10 &&
                (first.a-initial.a).norm()<1e-10,"entry p/v/a continuity");
        for (int i=0;i<=1500;++i)
        {
            const double time=stop.duration()*i/1500;
            const auto value=stop.at(time);
            require(box.contains(value.p) && value.a.norm()<=3+1e-8 &&
                    stop.jerk(time).norm()<=4+1e-8,"whole curve box and dynamic limits");
            if (i>0 && i<1500)
            {
                const double h=1e-5;
                require(((stop.at(time+h).p-stop.at(time-h).p)/(2*h)-value.v).norm()<1e-5,
                        "position derivative matches velocity");
            }
        }
    }
    require(successes>=50,"normal interior states should be feasible");
    auto preview=[&](double,Reference &r) { r=impossible; return true; };
    auto result=guard.assess(Reference(),Vector3d::Zero(),Vector3d::Zero(),3,4,preview);
    require(result.decision==BoxDecision::BRAKE,"unsafe future triggers early brake");
    require(result.reusable(Reference(),Vector3d::Zero(),Vector3d::Zero(),box,3,4),
            "preventive brake retains an executable current-state plan");
    require(result.stop.at(0).v.norm()==0,"retained stationary plan is executable");
    require(!result.reusable(Reference(),Vector3d(.01,0,0),Vector3d::Zero(),box,3,4),
            "changed measured state invalidates retained plan");
    result=guard.assess(impossible,impossible.p,impossible.v,3,4,preview);
    require(result.decision==BoxDecision::FAILSAFE,"uncertifiable current state is latched fault");
    BoxOptions small=box;
    small.minimum=Vector3d(-1.01,-1.01,-.5);
    small.maximum=Vector3d(1.01,1.01,3);
    BoxGuard circle_guard; circle_guard.configure(small);
    BoxStop circle_stop; circle_stop.configure(small);
    const double omega=1.4;
    int curved_stops=0, generic_failures=0;
    const auto circle_begin=std::chrono::steady_clock::now();
    for(int phase=0;phase<24;++phase)
    {
        const double theta=2*std::acos(-1.)*phase/24;
        Preview circle=[=](double t,Reference &r) {
            const double angle=theta+omega*t;
            r.p=Vector3d(std::cos(angle),std::sin(angle),2);
            r.v=omega*Vector3d(-std::sin(angle),std::cos(angle),0);
            r.a=-omega*omega*Vector3d(std::cos(angle),std::sin(angle),0);
            return true;
        };
        Reference entry; circle(0,entry);
        if(!circle_guard.stoppable(entry,3,4)) ++generic_failures;
        require(circle_stop.tryStart(entry,3,4,circle),"small-box circle has certified curved exit");
        if(circle_stop.followsPath()) ++curved_stops;
        const auto first=circle_stop.at(0);
        require((first.p-entry.p).norm()<1e-9 && (first.v-entry.v).norm()<1e-9 &&
                (first.a-entry.a).norm()<1e-9,"curved stop preserves entry p/v/a");
        for(int i=0;i<=1000;++i)
        {
            const double t=circle_stop.duration()*i/1000;
            const auto r=circle_stop.at(t);
            require(small.contains(r.p) && r.a.norm()<=3+1e-8 &&
                    circle_stop.jerk(t).norm()<=4+1e-8,"curved exit whole-reference bounds");
            if(i>0 && i<1000)
            {
                const double h=1e-6;
                require(((circle_stop.at(t+h).p-circle_stop.at(t-h).p)/(2*h)-r.v).norm()<1e-5,
                        "curved reference velocity is actual position derivative");
            }
        }
        const auto decision=circle_guard.assess(entry,entry.p,entry.v,3,4,circle);
        require(decision.decision==BoxDecision::CLEAR,"safe tight circle must not falsely brake");
        const auto offset_decision=circle_guard.assess(entry,
            entry.p+Vector3d(.003,.002,0),entry.v,3,4,circle);
        require(offset_decision.decision==BoxDecision::CLEAR,
                "small constant position tracking error retains feasible curved exit");
        if(phase==0)
        {
            const double total=circle_stop.duration();
            for(int k=0;k<16;++k)
            {
                const double elapsed=total*k/16;
                Preview remaining=[&](double t,Reference &r) { r=circle_stop.at(elapsed+t); return true; };
                const auto value=circle_stop.at(elapsed);
                const auto assessment=circle_guard.assess(value,value.p,value.v,3,4,remaining,
                                                          &circle_stop,elapsed);
                if(assessment.decision==BoxDecision::FAILSAFE)
                    std::cout<<"brake elapsed="<<elapsed<<", reason="<<assessment.reason<<std::endl;
                // An already active BRAKE ignores preventive BRAKE requests.
                require(assessment.decision==BoxDecision::CLEAR,
                        "nominal execution of certified brake must not trigger a fault");
                const auto measured=circle_guard.assess(value,
                    value.p+Vector3d(.001,.001,0),value.v,3,4,remaining,&circle_stop,elapsed);
                require(measured.decision!=BoxDecision::FAILSAFE,
                        "small measured deviation still has a separately certified stopping plan");
                require(!circle_stop.certifiesRemaining(
                    Reference(),elapsed,small,3,4),"committed curve cannot certify a different state");
            }
        }
    }
    std::cout<<"curved="<<curved_stops<<", generic misses="<<generic_failures<<std::endl;
    require(curved_stops==24 && generic_failures>0,"circle fixture exercises new curved-stop capability");
    Preview outward=[](double t,Reference &r) {
        r.p=Vector3d(t*2,0,2); r.v=Vector3d(2,0,0); r.a.setZero(); return true;
    };
    Reference drift; outward(0,drift);
    // This start cannot stop inside the small box: it must not be admitted as safe.
    require(circle_guard.assess(drift,drift.p,drift.v,3,4,outward).decision!=BoxDecision::CLEAR,
            "outward exit is not excused by trajectory-aware prediction");
    std::cout<<"circle phases="<<curved_stops<<", previous generic misses="<<generic_failures
             <<", circle suite ms="<<std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now()-circle_begin).count()<<"\n";
    BoxOptions off=box; off.enabled=false;
    guard.configure(off); stop.configure(off);
    require(stop.tryStart(fast,3,4) && std::abs(stop.duration()-old.duration())<1e-12 &&
            (stop.at(stop.duration()).p-old.at(old.duration()).p).norm()<1e-12,
            "disabled protection preserves legacy stop");
    const auto millis=std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now()-begin).count();
    std::cout<<"ALL BOX SAFETY TESTS PASS; random curves="<<successes<<", sweep="<<millis<<" ms\n";
}
