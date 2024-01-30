#include "Hitting.h"

#include "../IiwaFsmController.h"

void Hitting::configure(const mc_rtc::Configuration & config)
{
  // config("iiwa7", robot_);
}

void Hitting::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<IiwaFsmController &>(ctl_);

  // Get robot joint
  jointPosition = rbd::dofToVector(ctl_.robot("iiwa7").mb(), ctl_.robot("iiwa7").mbc().q);
  // Get robot joint vel
  jointVelocity = rbd::dofToVector(ctl_.robot("iiwa7").mb(), ctl_.robot("iiwa7").mbc().alpha);
  // Get robot joint torque
  jointTorque = rbd::dofToVector(ctl_.robot("iiwa7").mb(), ctl_.robot("iiwa7").mbc().jointTorque);
  // Get EE position
  const int eeIndex = ctl_.realRobot("iiwa7").mb().bodyIndexByName("iiwa_link_ee");
  eePos = ctl_.realRobot("iiwa7").mbc().bodyPosW[ eeIndex ].translation().cast <float> ();
  // Get Box position
  const int boxIndex = ctl_.realRobot("Box").mb().bodyIndexByName("small_box_body");
  boxPos = ctl_.realRobot("Box").mbc().bodyPosW[ boxIndex ].translation().cast <float> ();
  mc_rtc::log::info("Box : {}", boxPos);
  // boxPos << 0.6, 0.4, 0.215 ;

  // Init hitting controller
  generateHitting_ = std::make_unique<hitting_DS>(eePos, boxPos);
  generateHitting_->set_current_position(eePos);
  generateHitting_->set_DS_attractor(boxPos);
  generateHitting_->set_des_direction(hitDirection_);

  // Init passive controller to get inertia 
  std::ifstream f("/home/ros/ros_ws/src/iiwa_fsm_controller/urdf/iiwa7.urdf"); //taking file as inputstream
  std::string str;
  if(f) {
    std::ostringstream ss;
    ss << f.rdbuf(); // reading data
    str = ss.str();
  }
  controller_ = std::make_unique<PassiveControl>(str, "iiwa_link_ee");
  controller_->updateRobot(jointPosition, jointVelocity, jointTorque);

  // Create vel task
  efTask1_ = std::make_shared<mc_tasks::TransformTask>("iiwa_link_ee", ctl.robots(), 0, 10, 1e5);
  sva::MotionVecd stiffness = sva::MotionVecd::Zero();
  Eigen::Vector6d d = (Eigen::Vector6d() << 3, 3, 3, 3, 3, 3).finished(); //set damping entre 5 et 10
  sva::MotionVecd damping = sva::MotionVecd(d);
  efTask1_->setGains(stiffness, damping);

  // Utiliser transformTask prend des input en CD ?! SUR refVel
  // 1. rotation 2. translation
  sva::MotionVecd target = sva::MotionVecd::Zero();
  
  ctl.solver().addTask(efTask1_);

  efTask1_->refVelB(target);
}

bool Hitting::run(mc_control::fsm::Controller & ctl_) // Called every iteration until it returns true
{
  auto & ctl = static_cast<IiwaFsmController &>(ctl_);


  // Update hitting controller
  const int eeIndex = ctl_.realRobot("iiwa7").mb().bodyIndexByName("iiwa_link_ee");
  eePos = ctl_.realRobot("iiwa7").mbc().bodyPosW[ eeIndex ].translation().cast <float> ();
  generateHitting_->set_current_position(eePos);
  mc_rtc::log::info("eePos Hitting : {}", eePos);
  mc_rtc::log::info("Box : {}", boxPos);

  // Get inertia from toolkit TODO GET RID OF TOOLKIT
  // Get robot joint
  jointPosition = rbd::dofToVector(ctl_.realRobot("iiwa7").mb(), ctl_.realRobot("iiwa7").mbc().q);
  // Get robot joint vel
  jointVelocity = rbd::dofToVector(ctl_.realRobot("iiwa7").mb(), ctl_.realRobot("iiwa7").mbc().alpha);
  // Get robot joint torque
  jointTorque = rbd::dofToVector(ctl_.realRobot("iiwa7").mb(), ctl_.realRobot("iiwa7").mbc().jointTorque);
  controller_->updateRobot(jointPosition, jointVelocity, jointTorque);
  taskInertia_ = controller_->getTaskInertiaPos().cast <float> ();

  // Get ref velocity
  if (!isHit_) {
    refVelocity_ = generateHitting_->flux_DS(0.5, taskInertia_);
  } else {
    refVelocity_ = generateHitting_->linear_DS(iiwaReturnPosition_);
  }

  mc_rtc::log::info("Dot product: {}", generateHitting_->get_des_direction().dot(
                        generateHitting_->get_DS_attractor() - generateHitting_->get_current_position()));
  mc_rtc::log::info("Is hit: {}", isHit_);

  // Detect if robot hit the box
  if (!isHit_
      && generateHitting_->get_des_direction().dot(generateHitting_->get_DS_attractor()
                                                    - generateHitting_->get_current_position())
          < 0) {
    isHit_ = 1;
  }

  // Update commands
  eeOri = ctl_.realRobot("iiwa7").mbc().bodyPosW[ eeIndex ].rotation().cast <float> ();
  angularVel_ = (Eigen::Vector3f() << 0.0, 0.0,  0.0).finished();
  velEEFrame_ = eeOri * refVelocity_;

  efTask1_->refVelB(sva::MotionVecd(angularVel_.cast <double> (), velEEFrame_.cast <double> ()));

  // speed à la place d' eval mais pas trop de sens
  if(!isHit_){ //true
    // std::cout << "Controller " << efTask1_->eval().norm() << std::endl;
    return false;
  }
  else{
    output("OK");
    return true;
  }

}

void Hitting::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<IiwaFsmController &>(ctl_);
  ctl_.solver().removeTask(efTask1_);

}

EXPORT_SINGLE_STATE("Hitting", Hitting)
