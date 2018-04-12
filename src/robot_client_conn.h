#ifndef _ROBOT_CLIENT_CONN_H_
#define _ROBOT_CLIENT_CONN_H_

#include <unistd.h>
#include "pink/include/pink_define.h"
#include "pink/include/server_thread.h"
#include "pink/include/pink_conn.h"
#include "pink/include/pink_thread.h"


const int kBufMaxLen = 1024;

using namespace pink;

class RobotConn: public PinkConn {
    public:
        RobotConn(const int fd, const std::string &ip_port, ServerThread *thread,
                void *worker_specific_data); 
        virtual ~RobotConn();

        ReadStatus GetRequest() override;
        WriteStatus SendReply() override;     

        virtual int DealMessage(); 
    private:
        ConnStatus connStatus_;

        char *rBuf_; 
        uint32_t rBuf_len_;

        char *wBuf_; 
        uint32_t wBuf_len_;
        uint32_t wBuf_pos_; 
};  
class RobotConnFactory : public ConnFactory {
    public:
        RobotConnFactory() {}
        virtual PinkConn *NewPinkConn(int connfd, const std::string &ip_port,
                ServerThread *thread,
                void* worker_specific_data) const {
            return new RobotConn(connfd, ip_port, thread, worker_specific_data);
        }
    private:       
        
};

#endif
