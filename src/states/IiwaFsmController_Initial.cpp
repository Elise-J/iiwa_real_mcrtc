#include "IiwaFsmController_Initial.h"

#include "../IiwaFsmController.h"

void IiwaFsmController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void IiwaFsmController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<IiwaFsmController &>(ctl_);
  ctl.robot("iiwa14").mbc().q = ctl.realRobot("iiwa14").mbc().q;
  ctl.robot("iiwa14").mbc().alpha = ctl.realRobot("iiwa14").mbc().alpha;
  ctl.getPostureTask("iiwa14")->posture(ctl.realRobot("iiwa14").mbc().q);


  ctl.robot("iiwa7").mbc().q = ctl.realRobot("iiwa7").mbc().q;
  ctl.robot("iiwa7").mbc().alpha = ctl.realRobot("iiwa7").mbc().alpha;
  ctl.getPostureTask("iiwa7")->posture(ctl.realRobot("iiwa7").mbc().q);
}

bool IiwaFsmController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<IiwaFsmController &>(ctl_);

  auto jointPosition_ = rbd::dofToVector(ctl_.realRobot("iiwa14").mb(), ctl_.realRobot("iiwa14").mbc().q).cast<float>();
  // mc_rtc::log::info("Joint position {} ", jointPosition_);

  const int eeIndex = ctl_.realRobot("iiwa14").mb().bodyIndexByName("iiwa_link_ee");
  Eigen::Vector3f eePos = ctl_.realRobot("iiwa14").mbc().bodyPosW[ eeIndex ].translation().cast <float> ();
  // mc_rtc::log::info("eePos : {}", eePos);

  // ctl.robot("iiwa14").mbc().q = ctl.realRobot("iiwa14").mbc().q;
  // ctl.robot("iiwa14").mbc().alpha = ctl.realRobot("iiwa14").mbc().alpha;

  output("OK");
  return true;
}

void IiwaFsmController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<IiwaFsmController &>(ctl_);
}

EXPORT_SINGLE_STATE("IiwaFsmController_Initial", IiwaFsmController_Initial)
