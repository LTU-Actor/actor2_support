#include <ros/ros.h>

#include <string>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <piksi_rtk_msgs/BaselineHeading.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

// Macro for loading params and printing messages if they are not set
#define LOAD_PARAM_INTO_STRING(name)                                    \
    std::string name;                                                   \
    if (!nh_.getParam(#name, name)) {                                   \
        ROS_ERROR_STREAM("odom_pub: '" << #name << "' not defined");    \
    }



/*
 * This node takes various information from the `ethz_piksi_ros` driver
 *    to generate Odometry.
 *
 * Piksi publishes RTK and SPP location information on separate topics. This node
 *    will always prefer to use RTK fix. If the RTK fix is lost for a time longer
 *    than `spp_fallback_timeout_`, this node will switch to using SPP until a
 *    RTK fix is received.
 *
 * Input:
 *   - RTK Pose (`geometry_msgs::PoseWithCovarianceStamped`)
 *   - SPP Pose (`geometry_msgs::PoseWithCovarianceStamped`)
 *     - Used for fall back in case RTK fix is lost
 *   - Baseline Heading (`piksi_rtk_msgs::BaselineHeading`)
 *
 * Output:
 *   - Publish `nav_msgs::Odometry` to "/odom"
 *   - Brodcast `geometry_msgs::TransformStamped` from "enu" to "gps_receiver"
 *     - "enu": East-North-Up reference frame with origin defined in 'params/enu_origin.yaml'
 *     - "gps_receiver": Physical location of the GPS
 *
 * Params:
 *   - `pose_rtk_topic`: Name of topic publishing RTK Pose messages
 *   - `pose_spp_topic`: Name of topic publishing SPP Pose messages
 *   - `heading_topic`: Name of topic publishing Heading Messages
 *   - `spp_fallback_timeout`: Amount of time (in seconds) to wait before switching to SPP 
 *          if RTK fix is not received
 *
 *  TODO: Velocity is not currently included in the Odom message. Use interpret velocity
 *        information from `/piksi/vel_ned`
 *
 */
class OdomPub {

public:
    OdomPub() : nh_{"~"}
    {
        // Get Topics

        // Pose (RTK)
        LOAD_PARAM_INTO_STRING(pose_rtk_topic);
        // Pose (SPP)
        LOAD_PARAM_INTO_STRING(pose_spp_topic);
        // Heading
        LOAD_PARAM_INTO_STRING(heading_topic);

        nh_.param("spp_fallback_timeout", spp_fallback_timeout_, 3.0);
        ROS_INFO_STREAM("Using spp_fallback_timeout of " << spp_fallback_timeout_ << " seconds");

        tf_server_  = tf2_ros::TransformBroadcaster();

        pose_fix_sub_       = nh_.subscribe(pose_rtk_topic, 10, &OdomPub::gotRTKPose, this);
        pose_spp_sub_       = nh_.subscribe(pose_spp_topic, 10, &OdomPub::gotSPPPose, this);
        heading_sub_        = nh_.subscribe(heading_topic, 10, &OdomPub::updateHeading, this);

        odom_pub_   = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

        // From piksi
        child_frame_id_ = "gps_receiver";
        builder_.odom_msg.child_frame_id = child_frame_id_;
    }

private:
    double spp_fallback_timeout_;

    ros::NodeHandle nh_;
    tf2_ros::TransformBroadcaster tf_server_;
    ros::Publisher  odom_pub_;

    std::string child_frame_id_;

    // Subscribers
    ros::Subscriber heading_sub_;
    ros::Subscriber pose_fix_sub_;
    ros::Subscriber pose_spp_sub_;

    // Keep track of when we received the last RTK Fix message
    ros::Time last_pose_fix_;


    // This is used to keep track of the odom message as
    //   it is being built by several other messages coming in from
    //   the piksi driver.
    // In general, once all three components are received, the message will
    //   be published.
    // If messages are dropped and we begin receiving messages with a new
    //   time stamp, the previous odom_msg will be published before beginning
    //   to build the new one.
    struct {
        nav_msgs::Odometry odom_msg;

        bool got_pose;    // Translation
        bool got_twist;   // Velocity
        bool got_heading; // Rotation
    } builder_;

public:

    void gotRTKPose(const geometry_msgs::PoseWithCovarianceStamped& msg)
    {
        //reset the timer
        last_pose_fix_ = msg.header.stamp;

        updatePose(msg);
    }

    void gotSPPPose(const geometry_msgs::PoseWithCovarianceStamped& msg)
    {
        ros::Duration elapsed = msg.header.stamp - last_pose_fix_;
        if (elapsed.toSec() > spp_fallback_timeout_) {
            ROS_ERROR_STREAM("Lost Fix, fallback to SPP");
            updatePose(msg);
        }
    }

    void updatePose(const geometry_msgs::PoseWithCovarianceStamped& msg)
    {
        // Update metadata
        builder_.odom_msg.header.frame_id   = msg.header.frame_id;

        // Update odom pose
        builder_.odom_msg.pose.pose.position.x = msg.pose.pose.position.x;
        builder_.odom_msg.pose.pose.position.y = msg.pose.pose.position.y;
        builder_.odom_msg.pose.pose.position.z = msg.pose.pose.position.z;

        // Covariance
        for (int i = 0; i < builder_.odom_msg.pose.covariance.size(); i++)
        {
            builder_.odom_msg.pose.covariance[i] = msg.pose.covariance[i];
        }

        builder_.got_pose = true;
        publishOdomAndTransform(msg.header.stamp);
    }


    void updateHeading(const piksi_rtk_msgs::BaselineHeading& heading_msg)
    {
        tf2::Quaternion q;
        double deg = heading_msg.heading / 1000.0 - 90;
        double rad = deg / 180 * 3.14159265;
        q.setRPY(0, 0, -rad);

        // Update odom heading
        builder_.odom_msg.pose.pose.orientation.x = q.x();
        builder_.odom_msg.pose.pose.orientation.y = q.y();
        builder_.odom_msg.pose.pose.orientation.z = q.z();
        builder_.odom_msg.pose.pose.orientation.w = q.w();

        builder_.got_heading = true;
        publishOdomAndTransform(heading_msg.header.stamp);
    }

    void publishOdomAndTransform(const ros::Time& stamp)
    {
        // Only publish if we have received both or we have a new stamp (missed a msg)
        // TODO: Check for `got_twist` when velocity estimates are added
        if ( (builder_.got_pose && builder_.got_heading)
                || stamp > builder_.odom_msg.header.stamp )
        {
            geometry_msgs::TransformStamped tf_msg;

            // Update to new stamp
            builder_.odom_msg.header.stamp = stamp;
            tf_msg.header.stamp = stamp;

            // tf metadata
            tf_msg.child_frame_id  = builder_.odom_msg.child_frame_id;
            tf_msg.header.frame_id = builder_.odom_msg.header.frame_id;

            // Update tf_msg
            tf_msg.transform.translation.x  = builder_.odom_msg.pose.pose.position.x;
            tf_msg.transform.translation.y  = builder_.odom_msg.pose.pose.position.y;
            tf_msg.transform.translation.z  = builder_.odom_msg.pose.pose.position.z;
            tf_msg.transform.rotation.x     = builder_.odom_msg.pose.pose.orientation.x;
            tf_msg.transform.rotation.y     = builder_.odom_msg.pose.pose.orientation.y;
            tf_msg.transform.rotation.z     = builder_.odom_msg.pose.pose.orientation.z;
            tf_msg.transform.rotation.w     = builder_.odom_msg.pose.pose.orientation.w;

            // Publish odom message
            odom_pub_.publish(builder_.odom_msg);

            // Broadcast transform
            tf_server_.sendTransform(tf_msg);

            // Wait for pose, heading, and twist
            builder_.got_pose      = false;
            builder_.got_twist     = false;
            builder_.got_heading   = false;
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "piksi_odom_pub");

    OdomPub odom;

    ros::spin();

    return 0;
}
