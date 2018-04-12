
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "robot_util.h"
#include "robot_conf.h"
#include "robot_const.h"
#include "slash/include/env.h"
extern RobotConf *g_robot_conf; 

void daemonize() {
  if (fork() != 0) exit(0); /* parent exits */
  setsid(); /* create a new session */
}

void close_std() {
  int fd;
  if ((fd = open("/dev/null", O_RDWR, 0)) != -1) {
    dup2(fd, STDIN_FILENO);
    dup2(fd, STDOUT_FILENO);
    dup2(fd, STDERR_FILENO);
    if (fd > STDERR_FILENO) close(fd);
  }
}

void create_pid_file() {
  /* Try to write the pid file in a best-effort way. */
  std::string path(g_robot_conf->pid_file());

  size_t pos = path.find_last_of('/');
  if (pos != std::string::npos) {
    slash::CreateDir(path.substr(0, pos));
  } else {
    path = g_robot_conf->log_path() + "/" + kRobotPidFile;
  }

  FILE *fp = fopen(path.c_str(), "w");
  if (fp) {
    fprintf(fp,"%d\n",(int)getpid());
    fclose(fp);
  }
}

FileLocker::FileLocker(const std::string& file)
  : file_(file) {
}

slash::Status FileLocker::Lock() {
  slash::Status s = slash::LockFile(file_, &file_lock_);
  return s;
}

FileLocker::~FileLocker() {
  if (file_lock_) {
    slash::UnlockFile(file_lock_);
  }
}
