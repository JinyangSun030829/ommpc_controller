#include "trajectory_planner.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}
void rejects(const std::function<void()>& operation, const std::string& message) {
    bool failed = false;
    try { operation(); } catch (const std::exception&) { failed = true; }
    require(failed, message);
}
double falling(int n, int d) { double x=1; for(int i=0;i<d;++i)x*=n-i; return x; }

// Independent dense KKT solve, used ONLY for small regression cases. This does
// not reuse the new Hessian assembly, selector indexing, or normalized basis.
Eigen::MatrixXd reference(int order, const Eigen::MatrixXd& path,
    const Eigen::MatrixXd& v, const Eigen::MatrixXd& a, const Eigen::VectorXd& t) {
    const int s=t.size(), n=2*order, count=n*s, rows=2*order+(s-1)*(order+1);
    Eigen::MatrixXd q=Eigen::MatrixXd::Zero(count,count), constraints=Eigen::MatrixXd::Zero(rows,count);
    Eigen::MatrixXd rhs=Eigen::MatrixXd::Zero(rows,3);
    for(int k=0;k<s;++k)
        for(int i=order;i<n;++i)for(int j=order;j<n;++j)
            q(k*n+n-1-i,k*n+n-1-j)=falling(i,order)*falling(j,order)
                *std::pow(t(k),i+j-2*order+1)/double(i+j-2*order+1);
    auto add=[&](int row,int segment,int derivative,double time,double sign) {
        for(int i=derivative;i<n;++i)constraints(row,segment*n+n-1-i)
            +=sign*falling(i,derivative)*std::pow(time,i-derivative);
    };
    int r=0;
    for(int d=0;d<order;++d) {
        add(r,0,d,0,1);if(d==0)rhs.row(r)=path.row(0);if(d==1)rhs.row(r)=v.row(0);if(d==2)rhs.row(r)=a.row(0);++r;
        add(r,s-1,d,t(s-1),1);if(d==0)rhs.row(r)=path.row(s);if(d==1)rhs.row(r)=v.row(1);if(d==2)rhs.row(r)=a.row(1);++r;
    }
    for(int knot=1;knot<s;++knot) {
        add(r,knot-1,0,t(knot-1),1);rhs.row(r)=path.row(knot);++r;
        for(int d=0;d<order;++d){add(r,knot-1,d,t(knot-1),1);add(r,knot,d,0,-1);++r;}
    }
    require(r==rows,"reference constraints count");
    Eigen::MatrixXd kkt=Eigen::MatrixXd::Zero(count+rows,count+rows);
    kkt.topLeftCorner(count,count)=q;
    kkt.topRightCorner(count,rows)=constraints.transpose();
    kkt.bottomLeftCorner(rows,count)=constraints;
    Eigen::MatrixXd load=Eigen::MatrixXd::Zero(count+rows,3);load.bottomRows(rows)=rhs;
    const Eigen::MatrixXd x=kkt.fullPivLu().solve(load);
    require((kkt*x-load).norm()<1e-5,"reference KKT residual");
    Eigen::MatrixXd coeff(s,3*n);
    for(int i=0;i<s;++i)for(int axis=0;axis<3;++axis)
        coeff.block(i,axis*n,1,n)=x.block(i*n,axis,n,1).transpose();
    return coeff;
}

void checkPlan(const std::string& name, const minisnap::PlanResult& p,
               const minisnap::PlannerOptions& o, const Eigen::Vector3d& initial=Eigen::Vector3d::Zero()) {
    const auto& t=p.trajectory;
    require(t.times.size()+1==p.path.rows(),name+": size");
    const Eigen::Vector3d limits(o.max_velocity,o.max_acceleration,o.max_jerk);
    require((p.peak_bounds.array()<=limits.array()+1e-8).all(),name+": limits");
    double error=0;
    for(int s=0;s<t.times.size();++s) {
        error=std::max(error,(t.evaluate(s,0)-p.path.row(s).transpose()).norm());
        error=std::max(error,(t.evaluate(s,t.times(s))-p.path.row(s+1).transpose()).norm());
        if(s+1<t.times.size())for(int d=0;d<o.order;++d)
            error=std::max(error,(t.evaluate(s,t.times(s),d)-t.evaluate(s+1,0,d)).norm());
        // Independent dense samples verify conservative bounds never UNDERESTIMATE.
        for(int k=0;k<=200;++k)for(int d=1;d<=3;++d)
            require(t.evaluate(s,t.times(s)*k/200.0,d).norm()<=p.peak_bounds(d-1)+1e-8,name+": bound underestimate");
    }
    require(error<1e-5,name+": continuity/waypoints");
    require((t.evaluate(0,0,1)-initial).norm()<1e-6,name+": actual initial velocity");
    require(t.evaluate(t.times.size()-1,t.times(t.times.size()-1),1).norm()<1e-6,name+": terminal velocity");
    auto wire=t.serialize();
    require(wire.size()==2+t.times.size()+t.coefficients.size(),name+": wire size");
    require(wire[0]==t.times.size()&&wire[1]==2*o.order,name+": wire header");
    for(int s=0;s<t.times.size();++s)for(int k=0;k<=8;++k) {
        const double x=t.times(s)*k/8.0;
        Eigen::Vector3d physical=Eigen::Vector3d::Zero();
        for(int axis=0;axis<3;++axis)for(int j=0;j<2*o.order;++j)
            physical(axis)=physical(axis)*x+t.coefficients(s,axis*2*o.order+j);
        require((physical-t.evaluate(s,x)).norm()<1e-5,name+": physical wire compatibility");
    }
    std::cout<<name<<": segments="<<t.times.size()<<", ms="<<p.planning_ms
        <<", iterations="<<p.iterations<<", duration="<<t.times.sum()
        <<", V/A/J bounds="<<p.peak_bounds.transpose()<<", error="<<error
        <<", nnz="<<t.system_nonzeros<<"\n";
    require(p.feasible_duration + 1e-8 >= t.times.sum(),name+": compression increased duration");
    require(p.compression_trials <= o.compression_max_trials,name+": compression trial budget");
}
}  // namespace

int main() {
    try {
        std::cout<<std::fixed<<std::setprecision(6);
        TrajectoryGeneratorTool generator;
        std::mt19937 random(90210);
        std::uniform_real_distribution<double> distribution(-1,1);
        for(int order : {3,4}) {
            for(int trial=0;trial<5;++trial) {
                const int s=2+trial%3;
                Eigen::MatrixXd path(s+1,3),v=Eigen::MatrixXd::Zero(2,3),a=v;
                for(int i=0;i<path.rows();++i)for(int j=0;j<3;++j)path(i,j)=distribution(random);
                v(0,0)=0.25;a(0,1)=0.1;v(1,2)=-0.15;a(1,0)=0.08;
                Eigen::VectorXd t(s);for(int i=0;i<s;++i)t(i)=1.2+0.1*i;
                const auto expected=reference(order,path,v,a,t);
                const auto actual=generator.Solve(order,path,v,a,t);
                require((actual.coefficients-expected).norm()/std::max(1.0,expected.norm())<2e-6,"dense KKT equivalence");
            }
            std::cout<<"order "<<order<<": independent dense KKT regression PASS\n";
            minisnap::PlannerOptions options;options.order=order;
            minisnap::TrajectoryPlanner planner(options);
            Eigen::MatrixXd line(5,3);line<<0,0,2,.15,0,2,.3,0,2,.45,0,2,.6,0,2;
            checkPlan("short multi-segment order "+std::to_string(order),planner.plan(line,Eigen::Vector3d::Zero()),options);
            Eigen::MatrixXd single(2,3);single<<0,0,2,.1,0,2;
            checkPlan("single short order "+std::to_string(order),planner.plan(single,Eigen::Vector3d::Zero()),options);
            Eigen::MatrixXd moving(5,3);moving<<0,0,2,.5,0,2,1,0,2,1.5,0,2,2,0,2;
            const Eigen::Vector3d forward(.8,0,0),reverse(-.4,0,0),lateral(0,.4,0);
            checkPlan("moving start order "+std::to_string(order),planner.plan(moving,forward),options,forward);
            checkPlan("reverse start order "+std::to_string(order),planner.plan(moving,reverse),options,reverse);
            checkPlan("lateral start order "+std::to_string(order),planner.plan(moving,lateral),options,lateral);
            Eigen::MatrixXd limit_start(4,3);limit_start<<0,0,2,2,0,2,4,0,2,6,0,2;
            const Eigen::Vector3d at_limit(3,0,0);
            checkPlan("start at speed limit",planner.plan(limit_start,at_limit),options,at_limit);
            const Eigen::Vector3d initial_acceleration(.3,0,0);
            auto accelerating=planner.plan(moving,forward,initial_acceleration);
            checkPlan("nonzero initial acceleration",accelerating,options,forward);
            require((accelerating.trajectory.evaluate(0,0,2)-initial_acceleration).norm()<1e-6,"initial acceleration preserved");
            Eigen::MatrixXd turn(5,3);turn<<0,0,2,.5,0,2,.5,.2,2,.1,.2,2,.1,.4,2;
            checkPlan("sharp turns order "+std::to_string(order),planner.plan(turn,Eigen::Vector3d::Zero()),options);
            Eigen::MatrixXd duplicates(5,3);duplicates<<0,0,2,0,0,2,.2,0,2,.2,0,2,.4,0,2;
            auto dedup=planner.plan(duplicates,Eigen::Vector3d::Zero());
            require(dedup.removed_duplicates==2,"duplicate sanitization");checkPlan("duplicates",dedup,options);
            Eigen::MatrixXd hover=Eigen::MatrixXd::Zero(4,3);hover.col(2).setConstant(2);
            checkPlan("hover",planner.plan(hover,Eigen::Vector3d::Zero()),options);
            Eigen::MatrixXd tiny(4,3);tiny<<0,0,2,.0002,0,2,.0004,0,2,.0006,0,2;
            checkPlan("tiny",planner.plan(tiny,Eigen::Vector3d::Zero()),options);
            Eigen::MatrixXd circle(246,3);circle.row(0)<<0,0,2;
            const double pi=std::acos(-1.0);
            for(int i=1;i<245;++i)circle.row(i)<<1.5*std::sin(i*pi/4),1.5*std::cos(i*pi/4),2;
            circle.row(245)<<0,0,2;
            auto long_plan=planner.plan(circle,Eigen::Vector3d::Zero());
            checkPlan("legacy 245 segments order "+std::to_string(order),long_plan,options);
            require(long_plan.planning_ms<5000,"245-segment performance budget exceeded");
            require(long_plan.trajectory.system_nonzeros<245*40,"system should have linear sparsity");
            Eigen::MatrixXd dense_circle(246,3);dense_circle.row(0)<<0,1.5,2;
            for(int i=1;i<=245;++i)dense_circle.row(i)<<1.5*std::sin(2*pi*i/245),1.5*std::cos(2*pi*i/245),2;
            checkPlan("dense one-circle 245 segments",planner.plan(dense_circle,Eigen::Vector3d::Zero()),options);
            for(int trial=0;trial<50;++trial) {
                const int s=2+trial%8;
                Eigen::MatrixXd random_path(s+1,3);random_path.row(0)<<0,0,2;
                for(int i=1;i<=s;++i)random_path.row(i)=random_path.row(i-1)+Eigen::RowVector3d(
                    .2+.15*distribution(random),.15*distribution(random),.05*distribution(random));
                const Eigen::Vector3d initial(.2*distribution(random),.1*distribution(random),0);
                const auto fuzz=planner.plan(random_path,initial);
                require((fuzz.peak_bounds.array()<=Eigen::Vector3d(3,2,4).array()+1e-8).all(),"randomized limits");
            }
            std::cout<<"50 randomized motion cases PASS, order "<<order<<"\n";
            rejects([&]{planner.plan(single,Eigen::Vector3d(2,0,0));},"stopping-distance guard");
            rejects([&]{planner.plan(hover,Eigen::Vector3d(.1,0,0));},"moving stationary path");
            rejects([&]{planner.plan(line,Eigen::Vector3d(4,0,0));},"initial speed limit");
            rejects([&]{planner.plan(line,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),[]{return true;});},"cancellation");
            auto bad=line;bad(1,1)=std::numeric_limits<double>::quiet_NaN();
            rejects([&]{planner.plan(bad,Eigen::Vector3d::Zero());},"NaN guard");
        }
        // Same geometry, different interpolation densities. Compare both the
        // previous 0.1s floor and the new numerical floor + time compression.
        double shortest=std::numeric_limits<double>::infinity(),longest=0;
        for(int count : {30,60,120,245}) {
            Eigen::MatrixXd circle(count+1,3);
            for(int i=0;i<=count;++i) circle.row(i)<<1.5*std::sin(2*std::acos(-1.)*i/count),1.5*std::cos(2*std::acos(-1.)*i/count),2;
            minisnap::PlannerOptions old;old.min_segment_time=.1;old.compress_time=false;
            old.use_s_curve_seed=false;old.compare_legacy_seed=false;old.stationary_uniform_retime=false;
            auto baseline=minisnap::TrajectoryPlanner(old).plan(circle,Eigen::Vector3d::Zero());
            minisnap::PlannerOptions options;
            auto optimized=minisnap::TrajectoryPlanner(options).plan(circle,Eigen::Vector3d::Zero());
            checkPlan("density "+std::to_string(count),optimized,options);
            require(optimized.trajectory.times.sum() <= baseline.trajectory.times.sum()+1e-6,"density timing regression");
            require(optimized.local_compressions <= optimized.compression_trials,"accepted compression accounting");
            std::cout<<"density comparison "<<count<<": old="<<baseline.trajectory.times.sum()<<", new="<<optimized.trajectory.times.sum()
                <<", feasible="<<optimized.feasible_duration<<", trials="<<optimized.compression_trials
                <<", accepted="<<optimized.local_compressions<<"\n";
            shortest=std::min(shortest,optimized.trajectory.times.sum());longest=std::max(longest,optimized.trajectory.times.sum());
        }
        std::cout<<"density max/min duration ratio="<<longest/shortest<<"\n";
        require(longest/shortest<1.20,"density dependence exceeds 20 percent");
        // Disabled/zero-budget compression must preserve a checked feasible plan.
        Eigen::MatrixXd budget_path(3,3);budget_path<<0,0,2,1,0,2,2,0,2;
        minisnap::PlannerOptions no_budget;no_budget.compression_max_trials=0;no_budget.compare_legacy_seed=false;
        auto untouched=minisnap::TrajectoryPlanner(no_budget).plan(budget_path,Eigen::Vector3d::Zero());
        require(untouched.compression_trials==0&&std::abs(untouched.feasible_duration-untouched.trajectory.times.sum())<1e-8,"zero-budget fallback");
        // Mixed times, translated coordinates and nonzero acceleration.
        Eigen::MatrixXd path(5,3),v=Eigen::MatrixXd::Zero(2,3),a=v;
        path<<0,0,2,.001,0,2,.01,.02,2,.5,.4,2,2,1,2;
        Eigen::VectorXd times(4);times<<.1,.3,1.0,7.5;
        a(0,0)=.1;
        for(int order : {3,4}) {
            const auto near=generator.Solve(order,path,v,a,times);
            Eigen::MatrixXd translated=path.rowwise()+Eigen::RowVector3d(1e5,-1e5,1e5);
            const auto far=generator.Solve(order,translated,v,a,times);
            for(int s=0;s<4;++s)for(int k=0;k<=10;++k)for(int d=1;d<=3;++d)
                require((near.evaluate(s,times(s)*k/10,d)-far.evaluate(s,times(s)*k/10,d)).norm()<1e-6,"translation invariance");
        }
        rejects([&]{generator.Solve(5,path,v,a,times);},"unsupported order");
        times(0)=0;rejects([&]{generator.Solve(4,path,v,a,times);},"zero time");
        minisnap::PlannerOptions bad;bad.max_jerk=0;
        rejects([&]{minisnap::TrajectoryPlanner planner(bad);},"zero jerk configuration");
        std::cout<<"ALL OFFLINE TESTS PASS\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"TEST FAILED: "<<error.what()<<"\n";return 1;
    }
}
