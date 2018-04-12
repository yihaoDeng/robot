#include <iostream>
#include <stdio.h>
#include <cstdio>
#include <math.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>
#include <cstring>
#include "string_header.h"
#define alpha 0.000001//定义实数之间的差值
#define PI 3.1415926
//2018.3.29
using namespace std;

#define PORT 8089
int transform_1(string sf, char ca[])
{
    for(int i=0;i<sf.length();i++)
    {
        ca[i]=sf[i];
    }
}

string transform_2(double *a)
{
    string out;
    for (int i = 0; i < 7; i++)
    {

        out += std::to_string(a[i]);
        out += ',';
    }
    out +="\r";
    //cout<<"out"<<out<<endl;
    return out;
    //tran(out,ca);
}

double normal(double *v,int sign)//向量归一化
{
    double result;

    result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if(1==sign)
    {
        if(fabs(result-0)>alpha)
        {
            v[0] = v[0]/result;
            v[1] = v[1]/result;
            v[2] = v[2]/result;
        }
    }
    return result;
}

double compute(double *q,double *vectorBefore,double *vectorAfter)//求两个向量变换的四元数
{
    double rotationAxis[3];//旋转轴
    double rotationAngle;//向量夹角
    double dot_sum;
    rotationAxis[0] = vectorBefore[1] * vectorAfter[2] - vectorBefore[2] * vectorAfter[1];
    rotationAxis[1] = vectorBefore[2] * vectorAfter[0] - vectorBefore[0] * vectorAfter[2];
    rotationAxis[2] = vectorBefore[0] * vectorAfter[1] - vectorBefore[1] * vectorAfter[0];
    int result1 = normal(rotationAxis,1);
    dot_sum = vectorBefore[0] * vectorAfter[0] + vectorBefore[1] * vectorAfter[1] + vectorBefore[2] * vectorAfter[2];
    double angle = dot_sum/normal(vectorAfter,0)/normal(vectorBefore,0);//求向量夹角
    if (fabs(angle-1)<alpha)
        rotationAngle = acos(1.0);
    else
        rotationAngle = acos(angle);

    q[0] = cos(rotationAngle/2);
    q[1] = rotationAxis[0]*sin(rotationAngle/2);
    q[2] = rotationAxis[1]*sin(rotationAngle/2);
    q[3] = rotationAxis[2]*sin(rotationAngle/2);
    return 1;

}
int main()
{
    double pos[6] = {500.42,148.01,400.79,-2,-2,-8};

    struct sockaddr_in s_in;//server address structure(服务器地址结构)
    struct sockaddr_in c_in;//client address structure(客户端地址结构)
    int l_fd,c_fd;
    socklen_t len;
    char buf[40];//content buff area(内容缓冲区)
    memset((void *)&s_in,0,sizeof(s_in));

    s_in.sin_family = AF_INET;//IPV4 communication domain(通信领域)0.78,-0.02,0.62,-0.09
    s_in.sin_addr.s_addr = INADDR_ANY;//accept any address(接受任何地址)
    s_in.sin_port = htons(PORT);//change port to netchar (改变端口 netchar)

    l_fd = socket(AF_INET,SOCK_STREAM,0);//socket(int domain, int type, int protocol)
    bind(l_fd,(struct sockaddr *)&s_in,sizeof(s_in));
    listen(l_fd, 1);
    cout<<"begin"<<endl;

    c_fd = accept(l_fd,(struct sockaddr *)&c_in,&len);
    read(c_fd, buf, 1);//read the message send by client    o

    double pose_post[7] = {30,-300,600,-0,0,1,0};
    int grab_num=0;
    do{
        double pos_out[7];
        double pos_bef[7];
        double nor[3];

        //read(position_read_fd, pos, sizeof(pos));
        std::cout << "new pos" << std::endl;
        for (int i = 0; i < 7; i++) {
                std::cout << pos[i] << " ";
        }
        std::cout <<std::endl;
        if(pos[5] > 0){
            for(int i=0; i<3; i++){
                pos[i+3] = -pos[i+3];
            }
        }

        double vectorAfter[3];
        for(int i=0; i<3; i++){
            pos_out[i] = pos[i];
            nor[i] = pos[i+3];
            vectorAfter[i] = pos[i+3];
            std::cout<<pos_out[i]<<",";
        }
        normal(nor,1);
        for(int i=0; i<3; i++){
            pos_bef[i] = pos[i] - 100*nor[i];
        }

        double vectorBefore[3]={0,0,1};//输入向量
        double q[4]={0,0,0,0};//四元数初始化b
        compute(q,vectorBefore,vectorAfter);//计算四元数

        for(int i=0; i<4; i++){
            pos_out[i+3] = q[i];
            pos_bef[i+3] = q[i];
            std::cout<<pos_out[i+3]<<",";
        }
        std::cout<<std::endl;

        std::string csf_;
        csf_ = transform_2(pos_bef);
        char out_bef[csf_.length()];
        transform_1(csf_, out_bef);
        int str_len_post = sizeof(out_bef)/sizeof(out_bef[0]);
        read(c_fd, buf, 1);//read the message send by client    i
        write(c_fd,out_bef,str_len_post);
        read(c_fd, buf, 1);//read the message send by client    a

        std::string csf;
        csf = transform_2(pos_out);
        char out[csf.length()];
        transform_1(csf, out);
        str_len_post = sizeof(out)/sizeof(out[0]);
        cout<<"str_len is : "<<str_len_post<<endl;
        cout<<"out is : "<<out<<endl;
        read(c_fd, buf, 1);//read the message send by client    i
        write(c_fd,out,str_len_post);             //zhua
        std::cout<<"grab the objct "<<std::endl;
        read(c_fd, buf, 1);//read the message send by client    a
        std::cout<<" buf : "<< buf[0] <<std::endl;

        std::string csf1;
        csf1 = transform_2(pos_bef);
        char lo1[csf1.length()];
        transform_1(csf1, lo1);
        str_len_post = sizeof(lo1)/sizeof(lo1[0]);
        read(c_fd, buf, 1);//read the message send by client    i
        write(c_fd,lo1,str_len_post);
        read(c_fd, buf, 1);//read the message send by client    a

        double pos_mul1[7] ={374.76,-0.01,668.00,-0,0,1,0};
        std::string csf2;
        csf2 = transform_2(pos_mul1);
        char lo2[csf2.length()];
        transform_1(csf2, lo2);
        str_len_post = sizeof(lo2)/sizeof(lo2[0]);
        read(c_fd, buf, 1);//read the message send by client    i
        write(c_fd,lo2,str_len_post);
        read(c_fd, buf, 1);//read the message send by client    a

        int post=0;
        do{
            if(post > 2){
                break;
            }

            if(0 == post){
                int which_row = grab_num/3;
                std::cout<<"row: "<<which_row<<std::endl;
                int which_col = grab_num%3;
                std::cout<<"col: "<<which_col<<std::endl;
                pose_post[0] += which_col*110;
                pose_post[1] += which_row*50;
            }else if(1 == post){
                pose_post[2] -= 200;
            }else if(2 == post){
                pose_post[2] += 200;
            }

            read(c_fd, buf, 1);//read the message send by client    i
            std::cout<<" buf : "<< buf <<std::endl;
            std::string csf;
            csf = transform_2(pose_post);
            char out_post[csf.length()];
            transform_1(csf, out_post);
            str_len_post = sizeof(out_post)/sizeof(out_post[0]);
            write(c_fd,out_post,str_len_post);

            if(0 == post){
                std::cout<<"放--"<<std::endl;
            }

            read(c_fd, buf, 1);         //a
            std::cout<<" buf : "<< buf <<std::endl;

            for(int i=0; i<7; i++){
                std::cout<<pose_post[i]<<",";
            }
            std::cout<<std::endl;
            std::cout<<"out is : "<<out_post<<std::endl;
            post++;
        }while('a' == buf[0]);
        std::cout<<" buf : "<< buf <<std::endl;

        read(c_fd, buf, 1);//read the message send by client    i
        std::string csf3 = transform_2(pos_mul1);
        char out_post[csf3.length()];
        transform_1(csf3, out_post);
        str_len_post = sizeof(out_post)/sizeof(out_post[0]);
        write(c_fd,out_post,str_len_post);
        read(c_fd, buf, 1); //                                  a;
        std::cout<<" buf : "<< buf[0] <<std::endl;

        grab_num++;
    }while('a' == buf[0]);
    return 0;
}
