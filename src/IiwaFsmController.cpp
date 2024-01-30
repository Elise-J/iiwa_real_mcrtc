#include "IiwaFsmController.h"

IiwaFsmController::IiwaFsmController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  mc_rtc::log::success("IiwaFsmController init done ");
}

bool IiwaFsmController::run()
{
  robot("iiwa14").mbc().q = realRobot("iiwa14").mbc().q;
  robot("iiwa14").mbc().alpha = realRobot("iiwa14").mbc().alpha;
  
  //return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ObservedRobots);
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::None);
}

void IiwaFsmController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


