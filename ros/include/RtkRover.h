/***************************************************************************************
 *
 * @Copyright 2023, Inertial Sense Inc. <devteam@inertialsense.com>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************************/

#ifndef INERTIAL_SENSE_IMX_RTKROVER_H
#define INERTIAL_SENSE_IMX_RTKROVER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "mavros_msgs/RTCM.h"

#include "ParamHelper.h"

class RtkRoverCorrectionProvider {
protected:
    ParamHelper ph_;
    InertialSense* is_;
    ros::NodeHandle* nh_;

public:
    std::string type_;
    std::string protocol_; // format
    RtkRoverCorrectionProvider(YAML::Node& node, InertialSense* is_, ros::NodeHandle* nh_, const std::string& t) : ph_(node), is_(is_), nh_(nh_), type_(t) { };
    virtual void configure(YAML::Node& node) = 0;
};

class RtkRoverCorrectionProvider_Ntrip : public RtkRoverCorrectionProvider {
public:
    std::string ip_ = "127.0.0.1";
    int port_ = 7777;
    std::string mount_point_;
    std::string username_;
    std::string password_;

    // todo: move these to private fields
    bool connecting_ = false;
    bool connected_ = false;
    double traffic_time = 0.;

    int connection_attempt_limit_ = 1;
    int connection_attempt_backoff_ = 2;
    int traffic_total_byte_count_ = 0;
    int data_transmission_interruption_count_ = 0;
    int data_transmission_interruption_limit_ = 5;
    bool connectivity_watchdog_enabled_ = true;
    float connectivity_watchdog_timer_frequency_ = 1;
    ros::Timer connectivity_watchdog_timer_;

    RtkRoverCorrectionProvider_Ntrip(YAML::Node& node, InertialSense* is_, ros::NodeHandle* nh_) : RtkRoverCorrectionProvider(node, is_, nh_, "ntrip") { configure(node); }
    void configure(YAML::Node& node);
    std::string get_connection_string();
    void connect_rtk_client();
    void start_connectivity_watchdog_timer();
    void stop_connectivity_watchdog_timer();
    void connectivity_watchdog_timer_callback(const ros::TimerEvent &timer_event);

};

class RtkRoverCorrectionProvider_Serial : public RtkRoverCorrectionProvider {
public:
    std::string port_ = "/dev/ttyACM0";
    int baud_rate_ = 115200;
    bool connecting_ = false;
    int connection_attempt_limit_ = 1;
    int connection_attempt_backoff_ = 2;
    RtkRoverCorrectionProvider_Serial(YAML::Node& node, InertialSense* is_, ros::NodeHandle* nh_) : RtkRoverCorrectionProvider(node, is_, nh_, "serial") { configure(node); }
    virtual void configure(YAML::Node& node);
    std::string get_connection_string();
    void connect_rtk_client();
};

class RtkRoverCorrectionProvider_ROS : public RtkRoverCorrectionProvider {
public:
    std::string topic_ = "/rtcm3_corrections";
    std::string topic_datatype_ = "";
    RtkRoverCorrectionProvider_ROS(YAML::Node& node, InertialSense* is_, ros::NodeHandle* nh_) : RtkRoverCorrectionProvider(node, is_, nh_, "ros_topic"){ configure(node); }
    virtual void configure(YAML::Node& node);
    ros::Subscriber sub_;
    void callback_std_msgs_String(const std_msgs::String::ConstPtr& msg);
    void callback_std_msgs_UInt8MultiArray(const std_msgs::UInt8MultiArray::ConstPtr& msg);
    void callback_mavros_msgs_RTCM(const mavros_msgs::RTCM::ConstPtr& msg);
};

class RtkRoverCorrectionProvider_EVB : public RtkRoverCorrectionProvider {
public:
    std::string port_ = "xbee";
    RtkRoverCorrectionProvider_EVB(YAML::Node &node, InertialSense* is_, ros::NodeHandle* nh_) : RtkRoverCorrectionProvider(node, is_, nh_, "evb") { configure(node); }
    virtual void configure(YAML::Node &node);
};


class RtkRoverProvider {
protected:
    ParamHelper ph_;
    InertialSense* is_;
    ros::NodeHandle* nh_;

public:
    bool enable = true;                 // Enables/Disables the entire provider - enabled until explicitly disabled
    bool compassing_enable = false;     // Enable RTK compassing (dual GNSS moving baseline RTK) at GPS2
    bool positioning_enable = false;    // Enable RTK precision positioning at GPS1

    RtkRoverCorrectionProvider* correction_input;
    RtkRoverProvider(YAML::Node node, InertialSense* is_, ros::NodeHandle* nh_) : ph_((YAML::Node&)node), is_(is_), nh_(nh_) { configure(node); }
    void configure(YAML::Node& n);
};

class RtkRoverCorrectionProviderFactory {
public:
    static RtkRoverCorrectionProvider* buildCorrectionProvider(YAML::Node &node, InertialSense *is_, ros::NodeHandle *nh_) {
        if (node.IsDefined() && !node.IsNull()) {
            std::string type = node["type"].as<std::string>();
            std::transform(type.begin(), type.end(), type.begin(), ::tolower);
            if (type == "ntrip") return new RtkRoverCorrectionProvider_Ntrip(node, is_, nh_);
            else if (type == "serial") return new RtkRoverCorrectionProvider_Serial(node, is_, nh_);
            else if (type == "ros_topic") return new RtkRoverCorrectionProvider_ROS(node, is_, nh_);
            else if (type == "evb") return new RtkRoverCorrectionProvider_EVB(node, is_, nh_);
        } else {
            ROS_ERROR("Unable to configure RosRoverCorrectionProvider. The YAML node was null or undefined.");
        }
        return nullptr;
    }
};

#endif //INERTIAL_SENSE_IMX_RTKROVER_H
