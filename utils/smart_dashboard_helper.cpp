#include "smart_dashboard_helper.h"
#include <WPILib.h>

SmartDashboardHelper::SmartDashboardHelper(const muan::CSVLog* log)
    : log_(log) {}

SmartDashboardHelper::~SmartDashboardHelper() {}

void SmartDashboardHelper::Update() {
  std::string prefix = log_->GetName() + "_";
  for (auto entry : log_->GetEntries()) {
    SmartDashboard::PutString(prefix + entry.first, entry.second);
  }
}
