#ifndef INCLUDE_ROBOT_UTIL_H_
#define INCLUDE_ROBOT_UTIL_H_

#include <string>
#include <glog/logging.h>

#include "robot_conf.h"
#include "slash/include/env.h"


void daemonize();
void close_std();
void create_pid_file();
class FileLocker {
 public:
  explicit FileLocker(const std::string& file);
  ~FileLocker();
  slash::Status Lock();

 private:
  slash::FileLock* file_lock_;
  const std::string file_;
};


#endif
