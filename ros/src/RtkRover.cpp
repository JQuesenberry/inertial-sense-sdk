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

#include "RtkRover.h"

void RtkRoverProvider::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam(node, "compassing_enable", compassing_enable, false);
        ph_.nodeParam(node, "positioning_enable", positioning_enable, false);

        if (node["correction_input"].IsDefined() && !node["correction_input"].IsNull()) {
            std::string correction_src = node["correction_input"]["select"].as<std::string>();
            YAML::Node inputNode = node["correction_input"][correction_src];
            if (inputNode.IsDefined() && !inputNode.IsNull()) {
                correction_input = RtkRoverCorrectionProviderFactory::buildCorrectionProvider(inputNode, is_, nh_);
                if (correction_input == nullptr) {
                    ROS_ERROR_STREAM("Unable to configure RosRoverCorrectionProviders. Please validate the configuration:\n\n" << node << "\n\n");
                }
            } else {
                ROS_ERROR_STREAM("The specified Correction Provider [" << correction_src << "] can't be located in the config. Please validate the configuration:\n\n" << node << "\n\n");
            }
        } else {
            ROS_ERROR_STREAM("No \"correction_input\" configuration has been provided.  Please validate the configuration:\n\n" << node << "\n\n");
        }
    } else {
        ROS_ERROR("Unable to configure RosRoverProvider. The YAML node was null or undefined.");
    }
}

/*
 *==================  RtkRoverCorrectionProvider_Ntrip ==================*
 */
void RtkRoverCorrectionProvider_Ntrip::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("type", type_, "NTRIP");
        ph_.nodeParam("format", protocol_, "RTCM3");
        ph_.nodeParam("ip_address", ip_);
        ph_.nodeParam("ip_port", port_);
        ph_.nodeParam("mount_point", mount_point_);
        ph_.nodeParam("username", username_);
        ph_.nodeParam("password", password_);
        ph_.setCurrentNode(node["connection_attempts"]);
        ph_.nodeParam("limit", connection_attempt_limit_, 1);
        ph_.nodeParam("backoff", connection_attempt_backoff_, 2);
        ph_.setCurrentNode(node["watchdog"]);
        ph_.nodeParam("enable", connectivity_watchdog_enabled_, true);
        ph_.nodeParam("interval", connectivity_watchdog_timer_frequency_, 1);
    } else {
        ROS_ERROR("Unable to configure RtkRoverCorrectionProvider_Ntrip. The YAML node was null or undefined.");
    }
}

std::string RtkRoverCorrectionProvider_Ntrip::get_connection_string() {
    std::string RTK_connection = "TCP:" + protocol_ + ":" + ip_ + ":" + std::to_string(port_);
    if (!mount_point_.empty() || !username_.empty())
        RTK_connection.append(":" + mount_point_);
    if (!username_.empty()) {
        RTK_connection.append(":" + username_);
        if (!password_.empty())
            RTK_connection.append(":" + password_);
    }

    return RTK_connection;
}

void RtkRoverCorrectionProvider_Ntrip::connect_rtk_client()
{
    if (is_ == nullptr) {
        ROS_FATAL("RTK Client connection requested, but configureIS() hasn't been called in the provider.");
        ros::shutdown();
        connecting_ = false;
        return;
    }
    connecting_ = true;

    // [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
    std::string RTK_connection = get_connection_string();

    int RTK_connection_attempt_count = 0;
    while (RTK_connection_attempt_count < connection_attempt_limit_)
    {
        ++RTK_connection_attempt_count;

        bool connected = is_->OpenConnectionToServer(RTK_connection);

        if (connected)
        {
            ROS_INFO_STREAM("Successfully connected to " << RTK_connection << " RTK server");
            break;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to connect to base server at " << RTK_connection);

            if (RTK_connection_attempt_count >= connection_attempt_limit_)
            {
                ROS_ERROR_STREAM("Giving up after " << RTK_connection_attempt_count << " failed attempts");
            }
            else
            {
                int sleep_duration = RTK_connection_attempt_count * connection_attempt_backoff_;
                ROS_WARN_STREAM("Retrying connection in " << sleep_duration << " seconds");
                ros::Duration(sleep_duration).sleep();
            }
        }
    }

    connecting_ = false;
}

void RtkRoverCorrectionProvider_Ntrip::connectivity_watchdog_timer_callback(const ros::TimerEvent &timer_event)
{
    if (connecting_ && (is_ != nullptr))
        return;

    int latest_byte_count = is_->GetClientServerByteCount();
    if (traffic_total_byte_count_ == latest_byte_count)
    {
        ++data_transmission_interruption_count_;

        if (data_transmission_interruption_count_ >= data_transmission_interruption_limit_)
        {
            ROS_WARN("RTK transmission interruption, reconnecting...");
            connect_rtk_client();
        }
    }
    else
    {
        traffic_total_byte_count_ = latest_byte_count;
        data_transmission_interruption_count_ = 0;
    }
}

void RtkRoverCorrectionProvider_Ntrip::start_connectivity_watchdog_timer()
{
    if (!connectivity_watchdog_enabled_) {
        return;
    }

    if (!connectivity_watchdog_timer_.isValid()) {
        connectivity_watchdog_timer_ = nh_->createTimer(ros::Duration(connectivity_watchdog_timer_frequency_), &RtkRoverCorrectionProvider_Ntrip::connectivity_watchdog_timer_callback, this);
    }

    connectivity_watchdog_timer_.start();
}

void RtkRoverCorrectionProvider_Ntrip::stop_connectivity_watchdog_timer()
{
    connectivity_watchdog_timer_.stop();
    traffic_total_byte_count_ = 0;
    data_transmission_interruption_count_ = 0;
}

/*
 *==================  RtkRoverCorrectionProvider_Serial ==================*
 */
void RtkRoverCorrectionProvider_Serial::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("format", protocol_, "RTCM3");
        ph_.nodeParam("port", port_);
        ph_.nodeParam("baud_rate", baud_rate_);
    } else {
        ROS_ERROR("Unable to configure RtkRoverCorrectionProvider_Serial. The YAML node was null or undefined.");
    }
}

std::string RtkRoverCorrectionProvider_Serial::get_connection_string() {
    std::string RTK_connection = "SERIAL:" + protocol_ + ":" + port_ + ":" + std::to_string(baud_rate_);

    return RTK_connection;
}

void RtkRoverCorrectionProvider_Serial::connect_rtk_client()
{
    if (is_ == nullptr) {
        ROS_FATAL("RTK Client connection requested, but configureIS() hasn't been called in the provider.");
        ros::shutdown();
        connecting_ = false;
        return;
    }
    connecting_ = true;

    // [type]:[protocol]:[ip/url]:[port]:[mountpoint]:[username]:[password]
    std::string RTK_connection = get_connection_string();

    int RTK_connection_attempt_count = 0;
    while (RTK_connection_attempt_count < connection_attempt_limit_)
    {
        ++RTK_connection_attempt_count;

        bool connected = is_->OpenConnectionToServer(RTK_connection);

        if (connected)
        {
            ROS_INFO_STREAM("Successfully connected to " << RTK_connection << " RTK server");
            break;
        }
        else
        {
            ROS_ERROR_STREAM("Failed to connect to base server at " << RTK_connection);

            if (RTK_connection_attempt_count >= connection_attempt_limit_)
            {
                ROS_ERROR_STREAM("Giving up after " << RTK_connection_attempt_count << " failed attempts");
            }
            else
            {
                int sleep_duration = RTK_connection_attempt_count * connection_attempt_backoff_;
                ROS_WARN_STREAM("Retrying connection in " << sleep_duration << " seconds");
                ros::Duration(sleep_duration).sleep();
            }
        }
    }

    connecting_ = false;
}

/*
 *==================  RtkRoverCorrectionProvider_ROS ==================*
 */
void RtkRoverCorrectionProvider_ROS::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("format", protocol_, "RTCM3");
        ph_.nodeParam("topic", topic_);
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
            const ros::master::TopicInfo& info = *it;
            if (topic_ == info.name) {
                topic_datatype_ = info.datatype;
                break;
            }
        }
        if (topic_datatype_ == "std_msgs/String") {
            sub_ = nh_->subscribe(topic_, 1, &RtkRoverCorrectionProvider_ROS::callback_std_msgs_String, this);
            ROS_ERROR("RtkRoverCorrectionProvider_ROS - Subscribed to ROS Topic (%s) with DataType (%s).", topic_.c_str(), topic_datatype_.c_str());
        } else if (topic_datatype_ == "std_msgs/UInt8MultiArray") {
            sub_ = nh_->subscribe(topic_, 1, &RtkRoverCorrectionProvider_ROS::callback_std_msgs_String, this);
            ROS_ERROR("RtkRoverCorrectionProvider_ROS - Subscribed to ROS Topic (%s) with DataType (%s).", topic_.c_str(), topic_datatype_.c_str());
        } else if (topic_datatype_ == "mavros_msgs/RTCM") {
            sub_ = nh_->subscribe(topic_, 1, &RtkRoverCorrectionProvider_ROS::callback_mavros_msgs_RTCM, this);
            ROS_ERROR("RtkRoverCorrectionProvider_ROS - Subscribed to ROS Topic (%s) with DataType (%s).", topic_.c_str(), topic_datatype_.c_str());
        } else if (topic_datatype_ == "") {
            ROS_ERROR("RtkRoverCorrectionProvider_ROS - Specified ROS Topic (%s) with DataType (%s) Not Found.", topic_.c_str(), topic_datatype_.c_str());
        } else {
            ROS_ERROR("RtkRoverCorrectionProvider_ROS - Specified ROS Topic (%s) with DataType (%s) has No Callback Implemented.", topic_.c_str(), topic_datatype_.c_str());
        }
    } else {
        ROS_ERROR("Unable to configure RtkRoverCorrectionProvider_ROS. The YAML node was null or undefined.");
    }
}

void RtkRoverCorrectionProvider_ROS::callback_std_msgs_String(const std_msgs::String::ConstPtr& msg) {
    serialPortWrite(is_->GetSerialPort(), reinterpret_cast<const unsigned char*>(&msg->data[0]), msg->data.size());
}

void RtkRoverCorrectionProvider_ROS::callback_std_msgs_UInt8MultiArray(const std_msgs::UInt8MultiArray::ConstPtr& msg) {
    serialPortWrite(is_->GetSerialPort(), &msg->data[0], msg->data.size());
}

void RtkRoverCorrectionProvider_ROS::callback_mavros_msgs_RTCM(const mavros_msgs::RTCM::ConstPtr& msg) {
    serialPortWrite(is_->GetSerialPort(), &msg->data[0], msg->data.size());
}

/*
 *==================  RtkRoverCorrectionProvider_EVB ==================*
 */
void RtkRoverCorrectionProvider_EVB::configure(YAML::Node& node) {
    if (node.IsDefined() && !node.IsNull()) {
        ph_.setCurrentNode(node);
        ph_.nodeParam("format", protocol_, "RTCM3");
        ph_.nodeParam("port", port_);
    } else {
        ROS_ERROR("Unable to configure RtkRoverCorrectionProvider_EVB. The YAML node was null or undefined.");
    }
}

