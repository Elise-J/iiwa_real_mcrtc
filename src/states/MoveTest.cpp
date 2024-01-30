#include "MoveTest.h"

#include "../IiwaFsmController.h"

void MoveTest::configure(const mc_rtc::Configuration & config)
{
  //config("robot", robot_);
}

void MoveTest::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<IiwaFsmController &>(ctl_);
  
  efTask1_ = std::make_shared<mc_tasks::TransformTask>("iiwa_link_7", ctl.robots(), ctl_.realRobot("iiwa14").robotIndex(), 5, 10);


  Eigen::Vector6d s = (Eigen::Vector6d() << 3.0, 3.0, 3.0, 3.0, 3.0, 3.0).finished(); 
  sva::MotionVecd stiffness = sva::MotionVecd(s);

  Eigen::Vector6d weight = (Eigen::Vector6d() << 0.0, 0.0, 0.0, 1.0, 1.0, 1.0).finished(); 

  Eigen::Vector6d d = (Eigen::Vector6d() << 3, 3, 3, 3, 3, 3).finished(); //set damping entre 5 et 10
  sva::MotionVecd damping = sva::MotionVecd(d);
  // efTask1_->setGains(stiffness, damping);
  // efTask1_->stiffness(1);
  efTask1_->dimWeight(weight);

  ctl.solver().addTask(efTask1_);
  //auto target1 = sva::PTransformd{Eigen::Vector3d(0.16, 0.004, 1.24)};
  // Eigen::Quaterniond eeOriDes(0.7071068, 0.7071068, 0.0,  0.0);
  // Eigen::Quaterniond eeOriDes(0.0, 0.0, 0.0,  0.0);
  // target1.rotation() = eeOriDes.toRotationMatrix();
  //efTask1_->target(target1);
}

bool MoveTest::run(mc_control::fsm::Controller & ctl_) // Called every iteration until it returns true
{
  auto & ctl = static_cast<IiwaFsmController &>(ctl_);

  const int eeIndex = ctl_.realRobot("iiwa14").mb().bodyIndexByName("iiwa_link_7");
  Eigen::Vector3f eePos = ctl_.realRobot("iiwa14").mbc().bodyPosW[ eeIndex ].translation().cast <float> ();
  mc_rtc::log::info("eePos : {}", eePos);


  const int eeIndexRobot = ctl_.robot("iiwa14").mb().bodyIndexByName("iiwa_link_7");
  Eigen::Vector3f eePosRobot = ctl_.robot("iiwa14").mbc().bodyPosW[ eeIndex ].translation().cast <float> ();
  mc_rtc::log::info("eePosRobot : {}", eePosRobot);

  auto jointPosition_ = rbd::dofToVector(ctl_.realRobot("iiwa14").mb(), ctl_.realRobot("iiwa14").mbc().q).cast<float>();
  mc_rtc::log::info("Joint position {} ", jointPosition_);

  auto jointPositionRob = rbd::dofToVector(ctl_.robot("iiwa14").mb(), ctl_.robot("iiwa14").mbc().q).cast<float>();
  mc_rtc::log::info("Joint jointPositionRob {} ", jointPositionRob);


  // const int boxIndex = ctl_.realRobot("Box").mb().bodyIndexByName("small_box_body");
  // Eigen::Vector3f boxPos = ctl_.realRobot("Box").mbc().bodyPosW[ boxIndex ].translation().cast <float> ();
  // // mc_rtc::log::info("Box : {}", boxPos);

  // const int baseIndex = ctl_.realRobot("iiwa14").mb().bodyIndexByName("iiwa_link_0");
  // Eigen::Vector3f basePose = ctl_.realRobot("iiwa14").mbc().bodyPosW[ baseIndex ].translation().cast <float> ();
  // // mc_rtc::log::info("basePose : {}", basePose);

  // const std::string sensorName_ = "ft_sensor";
  // auto sensorIdx = ctl.realRobot("iiwa14").data()->forceSensorsIndex.at(sensorName_);
  // auto ftSensor = ctl.realRobot("iiwa14").data()->forceSensors[sensorIdx].wrench();
  // mc_rtc::log::info("[MoveTest] - sensor {}", ftSensor);

  mc_rtc::log::info("Eval {}" , efTask1_->eval().norm());
  if(efTask1_->eval().norm() > 0.001) {
    return false;
  }
  else{
    mc_rtc::log::success("Controller {}" , efTask1_->eval().norm());
    output("OK");
    return true;
  }
}

void MoveTest::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<IiwaFsmController &>(ctl_);
  ctl_.solver().removeTask(efTask1_);

}

EXPORT_SINGLE_STATE("MoveTest", MoveTest)
