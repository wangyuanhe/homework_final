#include "worker.h"


int main(int argc, char **argv) {
    FILE* fp;
    fp = fopen("data/data.txt","r");
    if(fp == NULL){
        printf("File Open Failed!\n");
        return 0;
    }
    string serial_port_name;
    serial_port_name.resize(10);
    
    fscanf(fp,"%s",&serial_port_name[0]);
    fclose(fp);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Homework>(serial_port_name));
    rclcpp::shutdown();
    return 0;
}