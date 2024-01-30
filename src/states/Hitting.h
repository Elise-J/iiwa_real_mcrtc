#pragma once

#include <mc_control/fsm/State.h>

#include <mc_tasks/TransformTask.h>
#include <mc_tasks/SplineTrajectoryTask.h>
// #include <dynamical_system.h>

#include <cpp_dynamical_system/dynamical_system.h>
#include <cpp_passive_inertial_control/passive_control.h>
#include <fstream>
#include <string>


struct Hitting : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    bool isHit_ = 0;

    std::shared_ptr<mc_tasks::TransformTask> efTask1_;
    std::shared_ptr<mc_tasks::TransformTask> efTaskBox_;
    

    std::string robot_;

    Eigen::Vector3f eePos;
    Eigen::Matrix3f eeOri;
    Eigen::Vector3f boxPos;
    Eigen::Vector3f hitDirection_ = {0.0, 1.0, 0.0};
    Eigen::Vector3f iiwaReturnPosition_ = {0.3, 0.0, 0.5};
    Eigen::Vector3f refVelocity_ = {0.0, 0.0, 0.0};

    Eigen::Matrix3f taskInertia_;
    Eigen::VectorXd jointPosition;
    Eigen::VectorXd jointVelocity;
    Eigen::VectorXd jointTorque;

    Eigen::Vector3f angularVel_;
    Eigen::Vector3f velEEFrame_;
    

    std::unique_ptr<hitting_DS> generateHitting_;
    std::unique_ptr<PassiveControl> controller_;

};
