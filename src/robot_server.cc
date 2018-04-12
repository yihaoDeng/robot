#include <stdio.h>
#include <signal.h>
#include <atomic>
#include <glog/logging.h>

#include "slash/include/env.h"
#include "robot_conf.h"
#include "robot_client_conn.h"
#include "robot_util.h"

RobotConf *g_robot_conf; 

static void GlogInit() {
    if (!slash::FileExists(g_robot_conf->log_path())) {
        slash::CreatePath(g_robot_conf->log_path());
    }
    FLAGS_alsologtostderr = true;
    FLAGS_log_dir = g_robot_conf->log_path();
    FLAGS_minloglevel = 0;
    FLAGS_max_log_size = 1800;
    ::google::InitGoogleLogging("robot");
}

void Usage() {
    printf("Usage:\n"
            "  ./robot_server -c confi\n"); 
}
void RobotConfInit(int argc, char* argv[]) {
    if (argc < 1) {
        Usage();
        exit(-1);

    }
    bool path_opt = false;
    char c;
    char path[1024];
    while (-1 != (c = getopt(argc, argv, "c:h"))) {
        switch (c) {
            case 'c':
                snprintf(path, sizeof(path), "%s", optarg);
                path_opt = true;
                break;
            case 'h':
                Usage();
                exit(-1);
            default:
                Usage();
                exit(-1);
        }
    }
    if (path_opt == false) {
        fprintf(stderr, "Please specify the conf file path\n");
        Usage();
        exit(-1);
    }
    g_robot_conf = new RobotConf(path);
    if (g_robot_conf->Load() != 0) {
        fprintf(stderr, "robot load conf file error\n");
        exit(-1);
    }
}


static std::atomic<bool> running(false);
static void IntSigHandle(const int sig) {
    printf("Catch Signal %d, cleanup...\n", sig);
     
    running.store(false);
    printf("server Exit");
}

static void SignalSetup() {
    signal(SIGHUP, SIG_IGN);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, &IntSigHandle);
    signal(SIGQUIT, &IntSigHandle);
    signal(SIGTERM, &IntSigHandle);
}
int main(int argc, char* argv[]) {
    RobotConfInit(argc, argv);
    GlogInit();

    LOG(INFO) << "test" << std::endl; 
    SignalSetup();
    if (g_robot_conf->Daemonize()) {
        daemonize();
    } 


    FileLocker db_lock(g_robot_conf->lock_file());
    slash::Status s = db_lock.Lock();
    if (!s.ok()) {
        printf("Start ZPMetaServer failed, because LOCK file error\n");
        return 0;

    }
    if (g_robot_conf->Daemonize()) {
        create_pid_file();
        close_std();
    }

    pink::ConnFactory *conn_factory = new RobotConnFactory();

    ServerThread* my_thread = NewHolyThread(g_robot_conf->local_port(), conn_factory);
    if (0 != my_thread->StartThread()) {
        delete my_thread; 
        delete conn_factory;
        delete g_robot_conf; 
        exit(-1);
    }

    running.store(true);
    while (running.load()) {
        sleep(1);
    }

    if (g_robot_conf->Daemonize()) {
       unlink(g_robot_conf->pid_file().c_str());
    }
    my_thread->StopThread();

    delete my_thread;
    delete conn_factory;
    delete g_robot_conf;
    ::google::ShutdownGoogleLogging();
    return 0;
}

