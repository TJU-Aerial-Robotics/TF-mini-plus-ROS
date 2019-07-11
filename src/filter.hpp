#ifndef FILTER_HPP
#define FILTER_HPP

#include "LowPassFilter2p.hpp"
#include "ros/ros.h"

class Filter: private LowPassFilter2p{
    public:
        Filter(float sample_freq , float cutoff_freq):LowPassFilter2p(sample_freq, cutoff_freq) {
            last_timestamp = ros::WallTime::now();
            has_init = false;
            distanceThreshold = 5.0f;
            strengthThreshold = 30.0f;
        }
        ~Filter() {

        }

        void set_distanceThreshold(const float& _n) {
            distanceThreshold = _n;
        }

        void set_strengthThreshold(const float& _n) {
            strengthThreshold = _n;
        }

        float get_distanceThreshold() {
            return distanceThreshold;
        }

        float get_strengthThreshold() {
            return strengthThreshold;
        }

        bool input(const double& _data, const double& _strength, double& _res) {
            if (_data > distanceThreshold) {
                // ROS_WARN("bad distance: %f", _data);
                return false;
            }

            if (_strength < strengthThreshold) {
                // ROS_WARN("bad strength: %f", _strength);
                return false;
            }

            if (!has_init || (ros::WallTime::now() - last_timestamp).toSec() > 0.5f) {
                _res = this->reset((float)_data);
                ROS_WARN("[tfminiplus driver]: init filter");
                has_init = true;
            } else {
                _res = this->apply((float)_data);
            }

            last_timestamp = ros::WallTime::now();

            return true;
        }


    private:

        float distanceThreshold;
        float strengthThreshold;

        ros::WallTime last_timestamp;
        bool has_init;

};


#endif