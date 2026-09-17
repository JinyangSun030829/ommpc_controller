#!/usr/bin/env python3
"""Offline checks of the actual OMMPC TXT loader/reference methods (no ROS/PX4)."""
import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    source = args.source.read_text()
    points = source.index("        case POINTS:", source.index("switch (exec_traj_state_)"))
    assert source.count("if (!readDataFromFile())") == 1
    assert source.index("if (!readDataFromFile())") > points
    assert "readDataFromFile()" not in source[source.index("    void init("):]
    assert "param_.use_ref_txt && txt_start_pending_ && !trajectory_ready_" in source
    assert "line_cnt_++" not in source
    assert "return txtReferenceAt(stamp, r)" in source
    takeoff = source.index("        case TAKEOFF:", points)
    assert "&reference_accelerations_" in source[points:takeoff]
    assert "&reference_jerks_" in source[points:takeoff]

    begin = source.index("    bool readDataFromFile()")
    end = source.index("    template <typename", begin)
    methods = source[begin:end]
    prefix = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
template<typename... Args> void fakeLog(const char *, Args...) {}
#define ROS_ERROR(...) fakeLog(__VA_ARGS__)
#define ROS_INFO(...) fakeLog(__VA_ARGS__)
constexpr int nstep = 4;
namespace ros {
struct Duration { double value; explicit Duration(double v=0):value(v){} double toSec() const { return value; } };
struct Time { double value; explicit Time(double v=0):value(v){} };
Duration operator-(const Time &a, const Time &b) { return Duration(a.value-b.value); }
Time operator+(const Time &a, const Duration &b) { return Time(a.value+b.value); }
namespace package { std::string getPath(const std::string &) { return ""; } }
}
namespace Eigen {
struct Vector3d {
    double values[3];
    Vector3d(double x=0,double y=0,double z=0):values{x,y,z}{}
    static Vector3d Zero() { return Vector3d(); }
    void setZero() { for(double &v:values) v=0; }
};
Vector3d operator-(const Vector3d &a,const Vector3d &b) {
    return Vector3d(a.values[0]-b.values[0],a.values[1]-b.values[1],a.values[2]-b.values[2]);
}
Vector3d operator/(const Vector3d &a,double b) {
    return Vector3d(a.values[0]/b,a.values[1]/b,a.values[2]/b);
}
}
namespace flight_safety {
struct Reference { Eigen::Vector3d p,v,a; };
}
class Harness {
public:
    struct Params {
        std::string ref_filename;
        double ref_time_step=0.02;
        double step_T=0.01;
    } param_;
    int line_cnt_=0, number_of_steps_=0;
    ros::Time txt_start_stamp_;
    std::vector<std::vector<double>> test_trajectory_;
    std::vector<Eigen::Vector3d> quad_positions_=std::vector<Eigen::Vector3d>(nstep+1);
    std::vector<Eigen::Vector3d> quad_velocities_=std::vector<Eigen::Vector3d>(nstep+1);
    std::vector<Eigen::Vector3d> reference_accelerations_=std::vector<Eigen::Vector3d>(nstep+1);
    std::vector<Eigen::Vector3d> reference_jerks_=std::vector<Eigen::Vector3d>(nstep+1);
    std::vector<double> yaws_=std::vector<double>(nstep+1);
'''
    suffix = r'''
};
int main(int argc, char **argv) {
    assert(argc==2);
    Harness h;
    h.param_.ref_filename=argv[1];
    auto write = [&](const std::string &data) {
        std::ofstream file(h.param_.ref_filename);
        file << data;
    };
    write("# header\n\n0 0 2 1 2 3 0 99\n1 0 2 1 2 3 0\n"
          "2 0 2 1 2 3 0\n3 0 2 1 2 3 0\n"
          "4 0 2 1 2 3 0\n5 0 2 1 2 3 0 # end\n");
    assert(h.readDataFromFile());
    assert(h.number_of_steps_==6);
    h.txt_start_stamp_=ros::Time(0);
    h.get_txt_des(ros::Time(0));
    const int first[] = {0,0,1,1,2};
    for(int i=0;i<=nstep;++i)
        assert(h.quad_positions_[i].values[0]==first[i]);
    assert(h.quad_velocities_[0].values[0]==1);
    h.get_txt_des(ros::Time(0.2));
    for(int i=0;i<=nstep;++i) {
        assert(h.quad_positions_[i].values[0]==5);
        assert(h.quad_velocities_[i].values[0]==0);
    }
    // The reference follows elapsed time, not the number of calls.
    h.get_txt_des(ros::Time(0.041));
    assert(h.line_cnt_==2);
    assert(h.quad_positions_[0].values[0]==2);
    h.get_txt_des(ros::Time(0.041));
    assert(h.line_cnt_==2);
    h.get_txt_des(ros::Time(0.081));
    assert(h.quad_positions_[0].values[0]==4);
    assert(h.quad_velocities_[0].values[0]==1);
    assert(h.quad_positions_[2].values[0]==5);
    assert(h.quad_velocities_[2].values[0]==0);
    for(double t : {0.0,0.041,0.081,0.1,0.2}) {
        h.get_txt_des(ros::Time(t));
        for(int i=0;i<=nstep;++i) {
            flight_safety::Reference reference; Eigen::Vector3d jerk;
            assert(h.txtReferenceAt(ros::Time(t+i*h.param_.step_T),reference,&jerk));
            for(int axis=0;axis<3;++axis) {
                assert(h.quad_positions_[i].values[axis]==reference.p.values[axis]);
                assert(h.quad_velocities_[i].values[axis]==reference.v.values[axis]);
                assert(h.reference_accelerations_[i].values[axis]==reference.a.values[axis]);
                assert(h.reference_jerks_[i].values[axis]==jerk.values[axis]);
            }
        }
    }
    // Differenced derivatives are also identical in the control reference.
    write("0 0 2 0 0 0 0\n0.0002 0 2 0.02 0 0 0\n0.001 0 2 0.08 0 0 0\n");
    assert(h.readDataFromFile());
    h.get_txt_des(ros::Time(0));
    assert(std::fabs(h.reference_accelerations_[0].values[0]-1)<1e-9);
    assert(std::fabs(h.reference_jerks_[0].values[0]-100)<1e-9);
    flight_safety::Reference terminal; Eigen::Vector3d terminal_jerk;
    assert(h.txtReferenceAt(ros::Time(.04),terminal,&terminal_jerk));
    assert(terminal.v.values[0]==0 && terminal.a.values[0]==0 && terminal_jerk.values[0]==0);
    // Reload replaces the previous trajectory and resets progress.
    write("8 0 2 0 0 0 0\n9 0 2 0 0 0 0\n");
    assert(h.readDataFromFile());
    assert(h.number_of_steps_==2 && h.test_trajectory_.size()==2);
    assert(h.line_cnt_==0);
    for(const auto &bad : std::vector<std::string>{
        "", "# only comment\n", "1 2 3\n", "garbage\n",
        "0 0 2 0 0 0 0 trailing\n", "0 0 2 0 0 0 nan\n",
        "0 0 2 0 0 0 inf\n", "0 0 2 0 0 0 0 1e309\n"}) {
        write(bad);
        assert(!h.readDataFromFile());
    }
    // A single-point file is valid; the reference is a stationary endpoint.
    write("6 0 2 1 2 3 0\n");
    assert(h.readDataFromFile());
    h.get_txt_des(ros::Time(0));
    for(int i=0;i<=nstep;++i) {
        assert(h.quad_positions_[i].values[0]==6);
        assert(h.quad_velocities_[i].values[0]==0);
    }
    h.param_.ref_filename += ".missing";
    assert(!h.readDataFromFile());
    std::cout << "PASS: TXT loader, reload, malformed input, elapsed-time "
                 "sampling and endpoint clamping; static lazy-load checks.\n";
}
'''
    with tempfile.TemporaryDirectory(prefix="ommpc_txt_test_") as temporary:
        directory = Path(temporary)
        cpp = directory / "test.cpp"
        cpp.write_text(prefix + methods + suffix)
        executable = directory / "test"
        subprocess.run(["g++", "-std=c++14", "-Wall", "-Wextra", "-Werror",
                        str(cpp), "-o", str(executable)], check=True)
        subprocess.run([str(executable), str(directory / "trajectory.txt")], check=True)


if __name__ == "__main__":
    main()
