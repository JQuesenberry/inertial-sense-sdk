#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <mutex>

#include "timepps.h"

#include <std_msgs_stamped/BoolStamped.h>
#include <std_msgs_stamped/Float64Stamped.h>
#include <std_msgs_stamped/TimeStamped.h>

class PPSHandlerInfo
{
  public:
    PPSHandlerInfo() :
        ref_time(0)
    {
        assert_msg.header.frame_id = "local";
        clear_msg.header.frame_id = "local";
        pulse_msg.header.frame_id = "local";
        local_to_device_time_offset_msg.header.frame_id = "local";
    }
    void operator=(const PPSHandlerInfo& rhs)
    {
        assert_msg = rhs.assert_msg;
        clear_msg = rhs.clear_msg;
        pulse_msg = rhs.pulse_msg;
        local_to_device_time_offset_msg = rhs.local_to_device_time_offset_msg;
        ref_time = rhs.ref_time;
        assert_hz = rhs.assert_hz;
    }

    std_msgs_stamped::TimeStamped assert_msg;
    std_msgs_stamped::TimeStamped clear_msg;
    std_msgs_stamped::BoolStamped pulse_msg;
    std_msgs_stamped::Float64Stamped local_to_device_time_offset_msg;

    ros::Time ref_time;

    double assert_hz;
};

class PPSHandler
{
  public:
    PPSHandler(std::string device) :
        device(device)
    {
        int ret = find_source();
        if (ret < 0)
            return;
        status = 1;
        ROS_INFO("ok, found %s, now start fetching data...", device.c_str());

        mtx.lock();
        info.assert_msg.header.stamp = ros::Time(0);
        info.clear_msg.header.stamp = ros::Time(0);
        info.pulse_msg.header.stamp = ros::Time(0);
        info.local_to_device_time_offset_msg.header.stamp = ros::Time(0);
        mtx.unlock();

        ros::NodeHandle n;
        assert_pub = n.advertise<std_msgs_stamped::TimeStamped>("pps/assert", 1, true);
        clear_pub = n.advertise<std_msgs_stamped::TimeStamped>("pps/clear", 1, true);
        pulse_pub = n.advertise<std_msgs_stamped::BoolStamped>("pps/pulse", 1, true);
        local_to_device_time_offset_pub = n.advertise<std_msgs_stamped::Float64Stamped>("pps/local_to_device_time_offset", 1, true);
    }
    ~PPSHandler()
    {
        close();
    }
    void close()
    {
        if (status > 0)
        {
            time_pps_destroy(handle);
            status = 0;
        }
    }
    void timer_callback(const ros::TimerEvent& e)
    {
        if (status > 0)
        {
            int ret = fetch_source();
            if (ret < 0 && errno != ETIMEDOUT)
                close();
        }
    }
    PPSHandlerInfo get_info()
    {
        PPSHandlerInfo tmp;
        mtx.lock();
        tmp = info;
        mtx.unlock();
        return tmp;
    }
    ros::Time get_assert_time()
    {
        ros::Time tmp;
        mtx.lock();
        tmp = info.assert_msg.header.stamp;
        mtx.unlock();
        return tmp;
    }
    ros::Time get_ref_time()
    {
        ros::Time tmp;
        mtx.lock();
        tmp = info.ref_time;
        tmp = ros::Time(tmp.sec + 1, 0);
        mtx.unlock();
        return tmp;
    }
    void set_ref_time(const ros::Time& t)
    {
        mtx.lock();
        info.ref_time = t;
        mtx.unlock();
    }
    double get_local_to_device_time_offset()
    {
        double tmp;
        mtx.lock();
        tmp = info.local_to_device_time_offset_msg.data;
        mtx.unlock();
        return tmp;
    }
    int find_source()
    {
        pps_params_t params;
        int ret;

        ROS_INFO("trying PPS source \"%s\"", device.c_str());

        /* Try to find the source by using the supplied "device" name */
        ret = open(device.c_str(), O_RDWR);
        if (ret < 0) {
            ROS_FATAL("unable to open device \"%s\" (%m)", device.c_str());
            return ret;
        }

        /* Open the PPS source (and check the file descriptor) */
        ret = time_pps_create(ret, &handle);
        if (ret < 0) {
            ROS_FATAL("cannot create a PPS source from device "
                    "\"%s\" (%m)", device.c_str());
            return -1;
        }
        ROS_INFO("found PPS source \"%s\"", device.c_str());

        /* Find out what features are supported */
        ret = time_pps_getcap(handle, &avail_mode);
        if (ret < 0) {
            ROS_FATAL("cannot get capabilities (%m)");
            return -1;
        }
        if ((avail_mode & PPS_CAPTUREASSERT) == 0) {
            ROS_FATAL("cannot CAPTUREASSERT");
            return -1;
        }

        /* Capture assert timestamps */
        ret = time_pps_getparams(handle, &params);
        if (ret < 0) {
            ROS_FATAL("cannot get parameters (%m)");
            return -1;
        }
        params.mode |= PPS_CAPTUREASSERT;
        /* Override any previous offset if possible */
        if ((avail_mode & PPS_OFFSETASSERT) != 0) {
            params.mode |= PPS_OFFSETASSERT;
            params.assert_offset = offset_assert;
        }
        ret = time_pps_setparams(handle, &params);
        if (ret < 0) {
            ROS_FATAL("cannot set parameters (%m)");
            return -1;
        }

        return 0;
    }
    int fetch_source()
    {
        struct timespec timeout;
        pps_info_t infobuf;
        int ret;

        /* create a zero-valued timeout */
        timeout.tv_sec = 0;
        timeout.tv_nsec = 0;

      retry:
        if (avail_mode & PPS_CANWAIT) /* waits for the next event */
            ret = time_pps_fetch(handle, PPS_TSFMT_TSPEC, &infobuf,
                       &timeout);
        else {
            ret = time_pps_fetch(handle, PPS_TSFMT_TSPEC, &infobuf,
                       &timeout);
        }
        if (ret < 0) {
            if (ret == -EINTR) {
                ROS_ERROR("time_pps_fetch() got a signal!");
                goto retry;
            }

            ROS_FATAL("time_pps_fetch() error %d (%m)", ret);
            return -1;
        }

        #if 0
        ROS_DEBUG("source %s - "
                 "assert %ld.%09ld, sequence: %ld - "
                 "clear  %ld.%09ld, sequence: %ld",
                 device.c_str(),
                 infobuf.assert_timestamp.tv_sec,
                 infobuf.assert_timestamp.tv_nsec,
                 infobuf.assert_sequence,
                 infobuf.clear_timestamp.tv_sec,
                 infobuf.clear_timestamp.tv_nsec, infobuf.clear_sequence);
        #endif

        mtx.lock();
        if (!info.ref_time.isZero())
        {
            ros::Time now = ros::Time::now();

            ros::Time assert_stamp(infobuf.assert_timestamp.tv_sec, infobuf.assert_timestamp.tv_nsec);
            ros::Time clear_stamp(infobuf.clear_timestamp.tv_sec, infobuf.clear_timestamp.tv_nsec);

            // header timestamp comes from kernel and should be earlier than current ROS time
            if (assert_stamp > clear_stamp && assert_stamp != info.assert_msg.header.stamp)
            {
                // header timestamp comes from kernel and should be earlier than current ROS time
                info.assert_msg.header.stamp = assert_stamp;
                // info.assert_msg data should be the ref_time seconds only and add 1 because the assert comes before the data with the time
                info.assert_msg.data = ros::Time(info.ref_time.sec + 1, 0);
                assert_pub.publish(info.assert_msg);

                info.pulse_msg.header.stamp = info.assert_msg.header.stamp - ros::Duration(1e-6);
                pulse_pub.publish(info.pulse_msg);
                info.pulse_msg.header.stamp = info.assert_msg.header.stamp;
                info.pulse_msg.data = 1;
                pulse_pub.publish(info.pulse_msg);

                info.local_to_device_time_offset_msg.header.stamp = info.assert_msg.header.stamp;
                info.local_to_device_time_offset_msg.data = info.assert_msg.data.toSec() - assert_stamp.toSec();
                local_to_device_time_offset_pub.publish(info.local_to_device_time_offset_msg);

                while(assert_deltas.size() >= 10)
                    assert_deltas.pop_front();
                assert_deltas.push_back(assert_deltas.empty() ? 0 : assert_stamp.toSec() - assert_prev);
                double tmp = 0;
                for(const auto& it : assert_deltas)
                    tmp += it;
                info.assert_hz = 1.0 / (tmp / double(assert_deltas.size()));
                assert_prev = assert_stamp.toSec();

                #if 0
                ROS_INFO("now: %f, assert: %f, ref_time: %f, ref_time_2: %f, offset: %f",
                    now.toSec(),
                    assert_stamp.toSec(),
                    info.ref_time.toSec(),
                    ros::Time(info.ref_time.sec + 1, 0).toSec(),
                    info.local_to_device_time_offset_msg.data
                );
                #endif
            }
            if (assert_stamp < clear_stamp && clear_stamp != info.clear_msg.header.stamp)
            {
                // header timestamp comes from kernel and should be earlier than current ROS time
                info.clear_msg.header.stamp = clear_stamp;
                // info.clear_msg data should be the GPS assert time + delta between assert and clear?
                info.clear_msg.data = info.assert_msg.data + (info.clear_msg.header.stamp - info.assert_msg.header.stamp);
                clear_pub.publish(info.clear_msg);

                info.pulse_msg.header.stamp = info.clear_msg.header.stamp - ros::Duration(1e-6);
                pulse_pub.publish(info.pulse_msg);
                info.pulse_msg.header.stamp = info.clear_msg.header.stamp;
                info.pulse_msg.data = 0;
                pulse_pub.publish(info.pulse_msg);
            }
        }
        mtx.unlock();

        return 0;
    }

    std::string device;
    pps_handle_t handle;
    int avail_mode;
    bool status = 0;

    constexpr static struct timespec offset_assert = {0, 0};

    PPSHandlerInfo info;

    ros::Publisher assert_pub;
    ros::Publisher clear_pub;
    ros::Publisher pulse_pub;
    ros::Publisher local_to_device_time_offset_pub;

    std::list<double> assert_deltas;
    double assert_prev;

    std::mutex mtx;
};
