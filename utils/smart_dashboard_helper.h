#ifndef UTILS_SMART_DASHBOARD_HELPER_H_
#define UTILS_SMART_DASHBOARD_HELPER_H_

#include "muan/logging/csv_log.h"

/*
 * SmartDashboardHelper
 * Puts the contents of a csv log to the smart dashboard
 */
class SmartDashboardHelper {
 public:
  SmartDashboardHelper(const muan::CSVLog* log);
  ~SmartDashboardHelper();

  void Update();

 private:
  const muan::CSVLog* log_;
};

#endif /* UTILS_SMART_DASHBOARD_HELPER_H_ */
