#include "robot_client_conn.h"
using namespace pink;
RobotConn::RobotConn(const int fd, const std::string& ip_port, ServerThread *thread, void *worker_specific_data):
    PinkConn(fd, ip_port, thread),
    connStatus_(kHeader), 
    rBuf_len_(0),     
    wBuf_len_(0),
    wBuf_pos_(0) {
        rBuf_ = reinterpret_cast<char *>(malloc(sizeof(char) * kBufMaxLen)); 
        wBuf_ = reinterpret_cast<char *>(malloc(sizeof(char) * kBufMaxLen));
} 
RobotConn::~RobotConn() {
    free(rBuf_);
    free(wBuf_);
}

ReadStatus RobotConn::GetRequest() {
    while (true) {
        switch (connStatus_) {
            case pink::kHeader: {   
                  ssize_t nread = read(fd(), rBuf_ + rBuf_len_, kBufMaxLen - rBuf_len_);
                  if (nread == -1) {
                      if (errno == EAGAIN) {
                          return kReadHalf;
                      } else {
                          return kReadError;
                      }
                  } else if (nread == 0) {
                      return kReadHalf; 
                  } else {
                      rBuf_len_ += nread;
                      connStatus_ = kComplete; 
                      continue; 
                  }
            }
            case pink::kComplete: {
                   if (DealMessage() != 0) {
                       return kDealError;                    
                   }                     
                   connStatus_ = kHeader;
                   rBuf_len_ = 0;
                   return kReadAll;
            }
            default:
                   break;
        }
    }
    return kReadHalf;
}
WriteStatus RobotConn::SendReply() {
    ssize_t nwritten = 0;
    while (wBuf_len_ > 0) {
        nwritten = write(fd(), wBuf_ + wBuf_pos_, wBuf_len_ - wBuf_pos_); 
        if (nwritten <= 0) {
            break;
        }
        wBuf_pos_ += nwritten;
        if (wBuf_pos_ == wBuf_len_) {
            wBuf_len_ = 0;
            wBuf_pos_ = 0;
        }
    }
    if (nwritten == -1) {
        if (errno == EAGAIN) {
            return kWriteHalf;
        } else {
            return kWriteError;
        }
    }
    if (wBuf_len_ == 0) {
        return kWriteAll;
    } else {
        return kWriteHalf;
    }
}

int RobotConn::DealMessage() {
    std::cout << "read data" << std::endl;       
    set_is_reply(true);
    return 0;
}



