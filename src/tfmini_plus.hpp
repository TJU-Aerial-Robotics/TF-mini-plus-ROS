#ifndef TFMINI_PLUS_HPP
#define TFMINI_PLUS_HPP
#include <string>
#include <vector>
#include <memory>
#include "ros/ros.h"
#include <thread>
#include <mutex>
#include "filter.hpp"

#include <sensor_msgs/FluidPressure.h>

using namespace serial;

class TF_mini_plus {
    public:
        TF_mini_plus(ros::NodeHandle & _nh) : nh(_nh) {
            nh.param<std::string>("port", serialport, "/dev/ttyUSB0");
            nh.param<int>("baudrate",  baudrate, 115200);
            nh.param<int>("pollrate",  pollrate, 50);
            int cutoff_freq;
            nh.param<int>("cutoff_freq", cutoff_freq, 10);
            std::string topic_name;
            nh.param<std::string>("topic_name", topic_name, "lidar_data");

            filter_ptr.reset(new Filter(pollrate, cutoff_freq));

            device_obj.reset(new serial::Serial);
            device_obj->setPort(serialport);
            device_obj->setBaudrate(baudrate);
            Timeout timeout = Timeout::simpleTimeout(1000 / pollrate);
            device_obj->setTimeout(timeout);
            
            device_obj->open();
            device_obj->flush();
            
            if (device_obj->isOpen()) {
                ROS_INFO_STREAM(device_obj->getPort() + " open success!");
            }

            lidar_pub = nh.advertise<sensor_msgs::FluidPressure>(topic_name, 10);

            listen_thread_handle();
            sleep(1);
            set_freq(pollrate);
        }

        ~TF_mini_plus() {
            listen_thread_mutex.unlock();
            device_obj->close();
        }

        void listen_thread_handle() {
            if (listen_thread_mutex.try_lock()) {
                std::thread listen_th(&TF_mini_plus::listen_thread_loop, this);
                if (listen_th.joinable())
                    listen_th.detach();
            } else {
                ROS_WARN("listen thread is already working");
            }
        }

        void listen_thread_loop() {
            int state = 0;
            int buf_size = 0;
            ros::WallRate loop_rate(pollrate);
            while (ros::ok()) {
                ros::spinOnce();
                loop_rate.sleep();
                int count = device_obj->available();
                while ((count > 0) && ros::ok()) {
                    uint8_t tmp;
                    if (state == 0 && count > 0) {
                        device_obj->read(&tmp, 1);
                        count--;
                        if (tmp == 0x59u) {
                            state = 1;
                        }
                        if (tmp == 0x5Au) {
                            state = 3;
                        }
                    }

                    if (state == 1 && count > 0) {
                        device_obj->read(&tmp, 1);
                        count--;
                        if (tmp == 0x59u) {
                            state = 2;
                            buf_size = 7;
                        } else {
                            state = 0;
                        }
                    }

                    if (state == 2) {
                        if (count >= buf_size) {
                            uint8_t read_data[9];
                            read_data[0] = 0x59u;
                            read_data[1] = 0x59u;
                            device_obj->read(&read_data[2], buf_size);
                            count -= buf_size;
                            uint8_t checksum = 0u;
                            for (int i = 0; i < 8; i++) {
                                checksum = (checksum + read_data[i]) & 0xFFu;
                            }
                            if (checksum == read_data[8]) {
                                int16_t distance = (int16_t) ((read_data[3] << 8) | read_data[2]);
                                int16_t strength = (int16_t) ((read_data[5] << 8) | read_data[4]);
                                if (distance < 0) {
                                    ROS_WARN("[tfminiplus driver]: detected distance to far");
                                    distance = 1200;
                                }

                                double dis_msg = (double)distance / 100.0f;
                                double str_msg = (double)strength;
                                double dis_after_filter = dis_msg;
                                if (filter_ptr->input(dis_msg, str_msg, dis_after_filter)) {
                                    sensor_msgs::FluidPressure tmp_msg;
                                    tmp_msg.header.stamp = ros::Time::now();
                                    tmp_msg.fluid_pressure = dis_after_filter;
                                    tmp_msg.variance = str_msg;
                                    lidar_pub.publish(tmp_msg);
                                // } else {
                                    // ROS_WARN("[tfminiplus driver]: bad measurement");
                                }
                                // std::cout << "distance: " << dis_msg << " strength: " << str_msg << std::endl;
                            } else {
                                ROS_WARN("[tfminiplus driver]: unpackage wrong");
                            }
                            state = 0;
                        } else {
                            // partial_msg = true;
                            break;
                        }
                    }

                    if (state == 3 && count > 0) {
                        device_obj->read(&tmp, 1);
                        count--;
                        if (tmp > 0x08u) {
                            state = 0;
                            ROS_WARN("[tfminiplus driver]: unpackage wrong, wrong set message");
                            break; 
                        } else {
                            buf_size = int(tmp - 0x02u);
                            state = 4;
                        }
                    }

                    if (state == 4) {
                        if (count >= buf_size) {
                            std::vector<uint8_t> read_data;
                            read_data.resize(buf_size + 2);
                            read_data[0] = 0x5Au;
                            read_data[1] = uint8_t(buf_size + 2);
                            device_obj->read(&read_data[2], buf_size);
                            count -= buf_size;
                            uint8_t checksum = 0u;
                            for (int i = 0; i < read_data.size()-1; i++) {
                                checksum = (checksum + read_data[i]) & 0xFFu;
                            }
                            if (checksum == read_data[buf_size+1]) {
                                if (read_data[2] == 0x03u) {
                                    uint16_t freq = (uint16_t) ((read_data[4] << 8)| read_data[3]);
                                    ROS_WARN(" Freq at %d", freq);
                                }
                            } else {
                                ROS_WARN("[tfminiplus driver]: unpackage wrong");
                            }
                            state = 0;
                        } else {
                            break;
                        }
                    }
                }
            }
            listen_thread_mutex.unlock();
        }

        void set_freq(const uint16_t& _n) {
            uint8_t pack[6];
            pack[0] = 0x5Au;
            pack[1] = 0x06u;
            pack[2] = 0x03u;
            pack[3] = (uint8_t)(_n);
            pack[4] = (uint8_t)(_n >> 8);
            uint8_t checksum = 0u;
            for (int i = 0; i < 5; i++) {
                checksum = (checksum + pack[i]) & 0xFFu;
            }
            pack[5] = checksum;
            device_obj->write(pack, sizeof(pack));
            ROS_INFO("set deveice freq: %f", (double)_n);
        }

    private:
        std::shared_ptr<serial::Serial> device_obj;
        ros::NodeHandle nh;
        std::string serialport;
        int baudrate;
        int pollrate;

        ros::Publisher lidar_pub;
        std::mutex listen_thread_mutex;

        std::shared_ptr<Filter> filter_ptr;

};
#endif