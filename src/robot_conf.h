#ifndef _ROBOT_CONF_H_
#define _ROBOT_CONF_H_

#include "slash/include/slash_string.h"
#include "slash/include/slash_mutex.h"
#include "slash/include/base_conf.h"

class RobotConf {
    public:   
        RobotConf(const std::string &path); 
        ~RobotConf();
        int Load();

        bool Daemonize() {
            return daemonize_ == true;
        }

        int local_port() {
            return local_port_;
        }
        std::string log_path() {
            return log_path_; 
        }
        std::string pid_file() {
            return pid_file_;
        }
        std::string lock_file() {
            return lock_file_; 
        }
    private:
        int local_port_; 
        slash::BaseConf conf_adaptor_;
        bool daemonize_; 
        std::string log_path_;
        std::string pid_file_;
        std::string lock_file_;
        RobotConf(const RobotConf&); 
};
#endif
