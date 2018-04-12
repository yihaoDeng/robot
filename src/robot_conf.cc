#include "robot_conf.h"
#include "slash/include/base_conf.h"

RobotConf::RobotConf(const std::string &path):
       local_port_(9999), 
       conf_adaptor_(path),
       daemonize_(false) {
}
int RobotConf::Load() {
    int res = conf_adaptor_.LoadConf();  
    if (res != 0) {
        return res;
    }
    bool ret = false;
    ret = conf_adaptor_.GetConfInt("server_port", &local_port_);
    ret = conf_adaptor_.GetConfStr("log_path", &log_path_);
    ret = conf_adaptor_.GetConfBool("daemonize", &daemonize_);
    
    std::string lock_path = log_path_; 
    pid_file_ =  lock_path + "pid";
    lock_file_ = lock_path + "lock";
    if (log_path_.back() != '/') {
        log_path_.append("/");
    }
    return ret == true ? 0: 1;
}
RobotConf::~RobotConf() {

}
