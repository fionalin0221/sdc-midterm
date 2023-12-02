#include<ros/ros.h>

class A{
public:
    int x_;
    int y_;
    A(){

    }
    A(int x, int y, int z){
        
    }
    A(int x, int y){ //建構子，用來初始化class的數值
        x_ = x;
        y_ = y;
    }

    void print(){ //成員函式
        ROS_INFO("x:%d, y:%d", x_, y_);
    }

};

class B{
public:
    int x;
    int y;
    B(){
        ROS_INFO("Initialized");
    }
    ~B(){ //解構子：程式跑完會歸還所有記憶體，當此class的變數被刪除時，就會跑解構子
        ROS_INFO("Delete %d", x);
    }
};
void transform(A a, B* b){
    
};

void test(){
    B b;
    b.x = 2;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    // radar_pub = nh.advertise<sensor_msgs::PointCloud2>("/radar_pc", 1);
 
    A a; 
    B b;
    b.x = 0;
    // A a(2,3);
    // transform(a, &b);
 
    // a.print();
    test();

    return 0;
}