
/**

\author Mark Dunn. rewritten from original from Willow Garage

@b bag_to_pcd is a simple node that reads in a BAG file and saves all the PointCloud messages to disk in PCD (Point
Cloud Data) format.

 **/

#include <sstream>
#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/common/io.h>
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include "geometry_msgs/Vector3.h"

typedef sensor_msgs::PointCloud2 PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
float filterSize=100;
tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf)
{
    tf::Transform tf(sTf.getBasis(), sTf.getOrigin()); //construct a transform using elements of sTf
    return tf;
}
bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}
/* ---[ */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag_to_pcd");
    if (argc < 4)
    {
        std::cerr << "Syntax is: " << argv[0] << " <file_in.bag> <topic> <fileout.ply>" << std::endl;
        std::cerr << "Example: " << argv[0] << " data.bag /laser_tilt_cloud /mnt/data/date_scan.ply" << std::endl;
        return (-1);
    }

    // TF
    tf::TransformListener tf_listener;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAccum (new pcl::PointCloud<pcl::PointXYZI>());

    tf::TransformBroadcaster tf_broadcaster;
    tfListener = new tf2_ros::TransformListener(tfBuffer);

    rosbag::Bag bag;
    rosbag::View view;
    rosbag::View::iterator view_it;
std::cout <<" opening file";
    try
    {
        bag.open(argv[1], rosbag::bagmode::Read);
    }
    catch (rosbag::BagException)
    {
        std::cerr << "Error opening file " << argv[1] << std::endl;
        return (-1);
    }

    view.addQuery(bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
    view.addQuery(bag, rosbag::TypeQuery("tf2_msgs/TFMessage"));
    view_it = view.begin();

    std::string output_file = std::string(argv[3]);
//    std::string output_file_subsample = output_file.replace(".pcd","_subsample.pcd");
//std::string output_file_subsample=output_file;
//replace(output_file_subsample, ".pcd", "_subsample.ply");
if (argc>4)
{
filterSize=atof(argv[4]);
}

//    boost::filesystem::path outpath(output_dir);
/*    if (!boost::filesystem::exists(outpath))
    {
        if (!boost::filesystem::create_directories(outpath))
        {
            std::cerr << "Error creating directory " << output_dir << std::endl;
            return (-1);
        }
        std::cerr << "Creating directory " << output_dir << std::endl;
    }
*/
    // Add the PointCloud2 handler
    std::cerr << "Saving recorded sensor_msgs::PointCloud2 messages on topic " << argv[2] << " to " << output_file << std::endl;

    PointCloud cloud_t;
    ros::Duration r(0.001);
    // Loop over the whole bag file
    int cnt = 0;
    geometry_msgs::TransformStamped tf_baselink_static;
    geometry_msgs::TransformStamped tf_static_moving;
    geometry_msgs::TransformStamped tf_static_moving_last;
    geometry_msgs::TransformStamped tf_moving_laser;
    geometry_msgs::TransformStamped transform;
    double delta = 0;
    double thisTime = 0;
    double lastTime = 0;
    while (view_it != view.end())
    {
        cnt++;
        tf2_msgs::TFMessage::ConstPtr tf = view_it->instantiate<tf2_msgs::TFMessage>();
        if (tf != NULL)
        {
            //ROS_INFO_STREAM("s "<<tf->transforms[0].child_frame_id <<"," <<tf->transforms[0].transform.rotation.x <<"," <<tf->transforms[0].transform.rotation.y <<"," <<tf->transforms[0].transform.rotation.z <<"," <<tf->transforms[0].transform.rotation.w);
            //        tf_broadcaster.sendTransform(tf->transforms);
            if (tf->transforms[0].child_frame_id == "rotator_moving")
            {
                thisTime = tf->transforms[0].header.stamp.toSec();
                if (lastTime == 0)
                    lastTime = thisTime;
                delta = thisTime - lastTime;
                lastTime = thisTime;
                tf_static_moving_last = tf_static_moving;
                tf_static_moving = tf->transforms[0];
            }
            else if (tf->transforms[0].child_frame_id == "/rotator_static")
                tf_baselink_static = tf->transforms[0];
            else if (tf->transforms[0].child_frame_id == "/laser")
                tf_moving_laser = tf->transforms[0];
            ++view_it;
            continue;
        }
        PointCloudPtr cloud = view_it->instantiate<PointCloud>();
        pcl::PointCloud<pcl::PointXYZI> pcloud;

        if (cloud)
        {
            //        ROS_INFO_STREAM("cloud " << cloud->height << " " << cloud->width << " " << cloud->point_step);
            sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
            sensor_msgs::PointCloud2Iterator<double> iter_time(*cloud, "timestamp");
            sensor_msgs::PointCloud2Iterator<float> iter_intens(*cloud, "intensity");
            sensor_msgs::PointCloud2Iterator<uint8_t> iter_ring(*cloud, "ring");
            tf2::Stamped<tf2::Transform> stamped_transform1;
            tf2::Stamped<tf2::Transform> stamped_transform2;
            tf2::Stamped<tf2::Transform> stamped_transform3;
            tf2::fromMsg(tf_baselink_static, stamped_transform1);
            tf2::fromMsg(tf_static_moving, stamped_transform2);
            tf2::fromMsg(tf_moving_laser, stamped_transform3);

            pcl::PointXYZI p;
            geometry_msgs::Vector3 p_in;
            geometry_msgs::Vector3 p_out;
            double starttime = 0;
            double pointTime = 0;
            while (iter_x != iter_x.end())
            {

                p_in.x = iter_x[0];
                p_in.y = iter_x[1];
                p_in.z = iter_x[2];
                geometry_msgs::Vector3 t1;
                t1.x = p_in.x;
                t1.y = p_in.y;
                t1.z = p_in.z;
                geometry_msgs::Vector3 t2;

                if (starttime == 0)
                    starttime = iter_time[0];
                pointTime = iter_time[0] - starttime;
                double ratio = 0;
                if (delta > 0)
                    ratio = pointTime / delta;
                double x1 = tf_static_moving_last.transform.rotation.x + ratio * (tf_static_moving.transform.rotation.x - tf_static_moving_last.transform.rotation.x);
                double y1 = tf_static_moving_last.transform.rotation.y + ratio * (tf_static_moving.transform.rotation.y - tf_static_moving_last.transform.rotation.y);
                double z1 = tf_static_moving_last.transform.rotation.z + ratio * (tf_static_moving.transform.rotation.z - tf_static_moving_last.transform.rotation.z);
                double w1 = tf_static_moving_last.transform.rotation.w + ratio * (tf_static_moving.transform.rotation.w - tf_static_moving_last.transform.rotation.w);

                stamped_transform2.setRotation(tf2::Quaternion(x1, y1, z1, w1));
                tf2::Transform stamped_transform4 = stamped_transform1 * stamped_transform2 * stamped_transform3;

                transform.transform.translation.x = stamped_transform4.getOrigin().x();
                transform.transform.translation.y = stamped_transform4.getOrigin().y();
                transform.transform.translation.z = stamped_transform4.getOrigin().z();
                transform.transform.rotation.x = stamped_transform4.getRotation().x();
                transform.transform.rotation.y = stamped_transform4.getRotation().y();
                transform.transform.rotation.z = stamped_transform4.getRotation().z();
                transform.transform.rotation.w = stamped_transform4.getRotation().w();
                tf2::doTransform(t1, t2, transform);
if (!isnan(t2.x))
{
                //rotate the point
                p.x = t2.x*1000.0; //p_out.x;
                p.y = t2.y*1000.0; //p_out.y;
                p.z = t2.z*1000.0; //p_out.z;
                p.intensity = iter_intens[0];
                pcloud.points.push_back(p);
}
                ++iter_x;
                ++iter_time;
                ++iter_intens;
                ++iter_ring;
            }
        }
        ROS_INFO_STREAM("cloud transform " << tf_static_moving.transform.translation.x << "," << tf_static_moving.transform.translation.y << "," << tf_static_moving.transform.translation.z << "," << tf_static_moving.transform.rotation.x << "," << tf_static_moving.transform.rotation.y << "," << tf_static_moving.transform.rotation.z << "," << tf_static_moving.transform.rotation.w);
        (*cloudAccum) += pcloud;

        ++view_it;
    }
//    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
 
  std::vector<int> indices;
ROS_INFO("point cnt saved: %i", cloudAccum->size());
cloudAccum->is_dense=false;
  pcl::removeNaNFromPointCloud(*cloudAccum, *cloudAccum, indices);
ROS_INFO("point cnt cleaned: %i", cloudAccum->size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>());

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (cloudAccum);
  sor.setLeafSize (filterSize,filterSize,filterSize);
  sor.filter (*cloud_filtered);

    ROS_INFO("%imm subsampled to:: %i", filterSize,cloud_filtered->size());

//    pcl::io::savePCDFileBinary(output_file, *cloudAccum);
	
  pcl::io::savePLYFileBinary(output_file, *cloud_filtered);

    ROS_INFO("point save complete");
ros::shutdown();
}
/* ]--- */
