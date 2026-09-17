#include "traj_tracking_controller/flight_safety.hpp"
#include <iostream>
#include <random>
void require(bool x,const char* m) { if(!x)throw std::runtime_error(m); }
int main() {
    using Eigen::Vector3d;
    const double pi=std::acos(-1.0),tilt=65*pi/180;
    for(int yi=0;yi<32;++yi) for(int fi=0;fi<128;++fi) {
        double az=2*pi*fi/128,yaw=2*pi*yi/32;
        Vector3d f(std::tan(tilt)*std::cos(az),std::tan(tilt)*std::sin(az),1);
        auto q=flight_safety::attitude(f,yaw); Vector3d b=q*Vector3d::UnitZ();
        require((b-f.normalized()).norm()<1e-12,"force/attitude collinearity");
        require(std::acos(std::max(-1.0,std::min(1.0,b.z())))<=tilt+1e-12,"final tilt cap");
    }
    std::mt19937 gen(7); std::uniform_real_distribution<double> rnd(-3,3);
    for(int k=0;k<100;++k) {
        flight_safety::Reference r; r.p=Vector3d(1,2,3);
        r.v=Vector3d(rnd(gen),rnd(gen),rnd(gen)); r.a=Vector3d(rnd(gen),rnd(gen),rnd(gen));
        flight_safety::SmoothStop stop; stop.start(r,3,4);
        const auto first=stop.at(0),last=stop.at(stop.duration());
        require((first.p-r.p).norm()<1e-12 && (first.v-r.v).norm()<1e-12 && (first.a-r.a).norm()<1e-12,"continuous brake entry p/v/a");
        require(last.v.norm()==0 && last.a.norm()==0 && stop.jerk(stop.duration()).norm()==0,"zero derivatives at brake exit");
        for(int i=0;i<=1000;++i) {
            double t=stop.duration()*i/1000; auto s=stop.at(t);
            require(s.a.norm()<=stop.accelerationLimit()+1e-10,"brake acceleration global cap");
            require(stop.jerk(t).norm()<=4+1e-10,"brake jerk global cap");
            if(i>0 && i<1000) {
                double h=1e-5; auto l=stop.at(t-h),u=stop.at(t+h);
                require(((u.p-l.p)/(2*h)-s.v).norm()<1e-6,"brake p derivative");
                require(((u.v-l.v)/(2*h)-s.a).norm()<1e-6,"brake v derivative");
            }
        }
    }
    for(double d:{0.0,.001,.01,.1,1.0,3.0}) {
        flight_safety::SmoothDescent descent; descent.start(Vector3d(1,2,3),3-d,.25,.5,1);
        double previous=3;
        for(int i=0;i<=2000;++i) {
            double t=descent.duration()*i/2000; auto r=descent.at(t);
            require(r.p.z()<=previous+1e-10 && r.p.z()>=3-d-1e-10,"monotonic descent no overshoot"); previous=r.p.z();
            require(r.v.norm()<=.25+1e-10 && r.a.norm()<=.5+1e-10,"descent V/A cap");
            if(i>0 && i<2000) {
                double h=1e-6; auto l=descent.at(t-h),u=descent.at(t+h);
                require(((u.a-l.a)/(2*h)).norm()<=1+1e-5,"descent jerk cap");
                require(((u.p-l.p)/(2*h)-r.v).norm()<1e-6,"descent p derivative");
            }
        }
        auto last=descent.at(descent.duration()); require(std::abs(last.p.z()-(3-d))<1e-10 && last.v.norm()==0 && last.a.norm()==0,"descent endpoint");
    }
    flight_safety::DisturbanceObserver obs; Vector3d v(20,-15,4),tau(1,1,1),applied(2,-1,.5),wind(.7,.3,-.2);
    obs.reset(v,10); Vector3d estimate;
    for(int i=1;i<=2000;++i) {
        v+=(applied+wind)*.01;
        estimate=obs.update(v,10+i*.01,applied,tau,12,.1);
        require(estimate.norm()<2,"bounded observer during high-speed/saturated input");
    }
    require((estimate-wind).norm()<1e-7,"observer estimates disturbance, not saturation or absolute speed");
    require((obs.update(v+Vector3d(100,0,0),30,applied,tau,12,.1)-estimate).norm()==0,"duplicate stamp freezes observer");
    require(obs.update(v,31,applied,tau,12,.1).norm()==0,"observer gap resets");
    std::cout<<"ALL FLIGHT SAFETY OFFLINE TESTS PASS\n";
}
