#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/impl/utils.h>
#include <tf2/transform_datatypes.h>

/// Class that contains the ros utility and functions that implement the follow the gap method
class follow_the_gap
{
public:
    follow_the_gap() :
            node_handle_(ros::NodeHandle()),
            lidar_sub_(node_handle_.subscribe("scan", 100, &follow_the_gap::scan_callback, this)),
            odom_sub_(node_handle_.subscribe("odom", 100, &follow_the_gap::odom_callback, this)),
            drive_pub_(node_handle_.advertise<ackermann_msgs::AckermannDriveStamped>("nav", 100))
    {
        node_handle_.getParam("/bubble_radius", bubble_radius_);
        node_handle_.getParam("/velocity", velocity_);
        node_handle_.getParam("/error_based_velocities", error_based_velocities_);
    }

    /// PreProcess Lidar Scan to replace nans by zeros and find the closest point in the lidar scan
    /// @param scan_msg - Lidar Scan Msg Callback
    /// @param filtered_ranges - Filtered Ranges from the scan_msg
    /// @param closest_index - Index of the Closest Point
    /// @return The closest point in the lidar scan
    size_t preprocess_lidar_scan(const sensor_msgs::LaserScan::ConstPtr &scan_msg,
                               std::vector<double>* filtered_ranges, size_t* closest_index)
    {
        size_t index = 0;
        double closest_point = std::numeric_limits<double>::max();

        for(const auto& scan : scan_msg->ranges)
        {
            if(!std::isnan(scan))
            {
                filtered_ranges->push_back(scan);
                if(closest_point > scan_msg->ranges[index])
                {
                    closest_point = scan_msg->ranges[index];
                    *closest_index = index;
                }
            }
            else
            {
                filtered_ranges->push_back(0.0);
            }
            index++;
        }
    }

    /// Zero out all the elements which lie within the bubble radius adjacent to the closest point
    /// @param scan_msg - Scan msg type returned by the scan callback
    /// @param filtered_ranges
    /// @param closest_point_index
    void zero_bubble(const sensor_msgs::LaserScan::ConstPtr &scan_msg,
                     std::vector<double>* filtered_ranges,
                     const size_t* closest_point_index)
    {
        size_t current_index = *closest_point_index;
        while(filtered_ranges->at(current_index++) < bubble_radius_)
        {
            filtered_ranges->at(current_index) = 0;
        }

        current_index = *closest_point_index;
        while(filtered_ranges->at(current_index--) < bubble_radius_)
        {
            filtered_ranges->at(current_index) = 0;
        }
    }

    /// Finds the maximum gap of non-zero elements in the filtered_ranges vector
    /// @param filtered_ranges
    /// @return pair of start and end points of the maximum gap
    std::pair<size_t, size_t> find_max_gap(const std::vector<double>* filtered_ranges)
    {
        size_t index = 0;
        size_t current_start = 0;
        size_t current_end = 0;
        size_t max_gap = 0;
        size_t max_gap_start = 0;
        size_t max_gap_end = 0;

        auto update_max_gap = [&](size_t current_start, size_t current_end)
        {
            if(const auto candidate_max_gap = current_start - current_end; candidate_max_gap > max_gap)
            {
                max_gap = candidate_max_gap;
                max_gap_start = current_start;
                max_gap_end = current_end;
            }
        };

        for(const auto& range : *filtered_ranges)
        {
            if(range != 0)
            {
                current_end++;
            }
            else
            {
                update_max_gap(current_start, current_end);
                current_start = index;
                current_end = index;
            }
            index++;
        }
        update_max_gap(current_start, current_end);

        return {max_gap_start, max_gap_end};
    }

    /// Get the Best Point in a Range of Max-Gap (according to some metric (In this case farthest point))
    /// @param filtered_ranges vector of max gap ranges
    /// @param start_index start index of max_gap_ranges in the filtered_ranges
    /// @param end_index end index of max_gap_ranges in the filtered_ranges
    /// @return return undex of the Max Element in the Vector
    static size_t get_best_point(std::vector<double>* max_gap_ranges, int start_index, int end_index)
    {
        const auto max_value_iterator = std::max_element(max_gap_ranges->begin(), max_gap_ranges->end());
        return std::distance(max_gap_ranges->begin(), max_value_iterator) + start_index;
    }

    /// Calculates the required steering angle based on the angles of the best point and the current heading of vehicle
    /// @param scan_msg - Scan Msg received by the callback
    /// @param best_point_index - The best point for heading as per FGM algorithm
    /// @return required steering angle
    size_t get_steering_angle(const sensor_msgs::LaserScan::ConstPtr &scan_msg, size_t best_point_index)
    {
        const auto best_point_steering_angle = scan_msg->angle_min + scan_msg->angle_increment*best_point_index;
        const auto interpolated_steering_angle = (best_point_steering_angle + current_yaw_)/2;
        return interpolated_steering_angle;
    }

    /// Odom Scan Callback to update the current yaw of the vehicle
    /// @param odom_msg
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        const auto odom_quaternion = odom_msg->pose.pose.orientation;
        current_yaw_ = tf2::impl::getYaw({odom_quaternion.x, odom_quaternion.y, odom_quaternion.z, odom_quaternion.w});
    }

    /// Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
    /// @param scan_msg
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
    {
        std::vector<double> filtered_ranges;
        size_t closest_index = 0;

        // Pre-Process and find the closest point to LiDAR
        preprocess_lidar_scan(scan_msg, &filtered_ranges, &closest_index);

        // Eliminate all points inside 'bubble' (set them to zero)
        zero_bubble(scan_msg, &filtered_ranges, &closest_index);

        // Find max length gap
        const auto [start_index, end_index] = find_max_gap(&filtered_ranges);

        // Find the best point in the gap
        const size_t best_point_index = get_best_point(&filtered_ranges, start_index, end_index);

        const auto steering_angle = get_steering_angle(scan_msg, best_point_index);

        // Publish Drive message
        ackermann_msgs::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = ros::Time::now();
        drive_msg.header.frame_id = "laser";
        drive_msg.drive.steering_angle = steering_angle;
        drive_msg.drive.speed = velocity_;
        drive_pub_.publish(drive_msg);
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher drive_pub_;

    double current_yaw_;

    int bubble_radius_;
    double velocity_;

    std::map<std::string, double> error_based_velocities_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "follow_the_gap_node");
    follow_the_gap gap_follower;
    ros::spin();
    return 0;
}