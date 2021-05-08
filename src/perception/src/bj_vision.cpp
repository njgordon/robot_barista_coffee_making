#include "ros/ros.h"
#include "stdlib.h"
#include "string.h"
#include "std_msgs/String.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"

// For TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

// Classes
class item
{
  public:
    uint64_t u_centre;
    uint64_t v_centre;
    std::string name;
};

// Vars
ros::Publisher pub_info;

darknet_ros_msgs::BoundingBoxes msgBB;
sensor_msgs::Image msgD;

uint8_t BB_hasBeenCalled = 0;
uint8_t D_hasBeenCalled = 0;

// Prototypes
void Callback_BoundingBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
void Callback_Depth(const sensor_msgs::Image::ConstPtr& msg);
void BroadcastItems();

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Callback for subscriber
// Input: msg = ptr to object encoding the message recieved on the topic
void BroadcastItems()
{
  // Vars - for publisher
  std::stringstream info;
  std_msgs::String msg_pub_info;
  info.precision(2);
  info.setf( std::ios::fixed, std:: ios::floatfield ); // floatfield set to fixed

  // Must recieve all reaquired message first
  if(! (BB_hasBeenCalled && D_hasBeenCalled)) { return; }

  // Timeout if too long since recieved message
  double timeNow = ros::Time::now().toSec();
  double timeBB = msgBB.header.stamp.toSec();
  double timeD = msgD.header.stamp.toSec();
  info << "Lag: BB,D = " << timeNow - timeBB << "  " << timeNow - timeD;
  if(
    timeNow - timeBB > 5 ||
    timeNow - timeD > 5
  )
  {
    // Send time
    info << "  Timeout";
    msg_pub_info.data = info.str();
    pub_info.publish(msg_pub_info);
    info.str( std::string() ); info.clear(); // Clear var

    // Timeout occured
    return;
  }
  else
  {
    // Send time
    msg_pub_info.data = info.str();
    pub_info.publish(msg_pub_info);
    info.str( std::string() ); info.clear(); // Clear var
  }

  // Depth image dimenstions
  int width_px = msgD.width; // Image width in px
  int height_px = msgD.height;
  int numLen_idx = 2; // Length of encoded number in data array (encoding: "16UC1" => 2 byte length)
  int pxLen_idx = 1*numLen_idx; // Length of encoded px in data array
  int width_idx = width_px*pxLen_idx; // Image width in bytes (for addressing)

  // Camera intrinsic properties
  //  From topic "/head_camera/depth_registered/camera_info"
  // boost::shared_ptr<sensor_msgs::CameraInfo const> info;
  // info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/head_camera/depth_registered/camera_info");
  // ROS_INFO("AAAAAAAAAAAAA: %f", info->K[0]);
  double fx = 527.4717075034799;
  double cx = 312.0351467124519;
  double fy = 524.6676472258492;
  double cy = 228.0943284280418;

  // Extract data from msgBB
  int itemsSize = msgBB.bounding_boxes.size();
  for (uint8_t idx_object = 0; idx_object < itemsSize; idx_object++)
  {
    // Centre of bounding box in px coords (units:px)
    uint64_t u = ( msgBB.bounding_boxes[idx_object].xmax + msgBB.bounding_boxes[idx_object].xmin ) / 2;
    uint64_t v = ( msgBB.bounding_boxes[idx_object].ymax + msgBB.bounding_boxes[idx_object].ymin ) / 2;

    // Distance to centre of bounding box from depth camera (units:mm)
    // Take from average of 5 px, horizontally about centre of bounding box
    uint64_t  w = 0;
    for (int64_t deviate_px = -2; deviate_px <= 2; deviate_px++)
    {
      // Out of bounds check
      uint64_t u_deviated = (int64_t(u) - deviate_px < 0 || int64_t(u) + deviate_px > width_px) ? u : (u + deviate_px);

      int idx_point = v*width_idx + u_deviated*pxLen_idx;
      uint16_t w_tmp = msgD.data[idx_point] | (msgD.data[idx_point+1] << 8); // Re-encode uint8 little endian data into original uint16
      w += w_tmp;
    }
    w /= 5;

    double x = (static_cast<double>(u) - cx) * static_cast<double>(w) / fx;
    double y = (static_cast<double>(v) - cy) * static_cast<double>(w) / fy;
    double z = static_cast<double>(w);

    // ROS_INFO("%d : [%s]", idx_object, msgBB.bounding_boxes[idx_object].Class.c_str());
    // ROS_INFO("u,v,w = %lu,\t%lu,\t%u", u, v, w);
    // ROS_INFO("x,y,z = %.0f,\t%.0f,\t%.0f", x,y,z);

    // Orientation - set 0 for now
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    geometry_msgs::TransformStamped transformStamped;
  
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "head_camera_rgb_optical_frame";
    transformStamped.child_frame_id = "BJ_item";
    transformStamped.transform.translation.x = x/1000;
    transformStamped.transform.translation.y = y/1000;
    transformStamped.transform.translation.z = z/1000;
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    // Broadcast if cup
    std::string strCup ("cup");
    if(!strCup.compare(msgBB.bounding_boxes[idx_object].Class.c_str()))
    {
      static tf2_ros::TransformBroadcaster br;
      br.sendTransform(transformStamped);

      // Braodcast only one item
      return;
    }

  }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Callback for subscriber
// Input: msg = ptr to object encoding the message recieved on the topic
void Callback_BoundingBoxes(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  // ROS_INFO("head seq [%d]", msg->header.seq); // Test msg
  BB_hasBeenCalled = 1;
  msgBB = *msg; // Copy message into global var
  BroadcastItems();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void Callback_Depth(const sensor_msgs::Image::ConstPtr& msg)
{
  D_hasBeenCalled = 1;
  msgD = *msg; // Copy message into global var
  BroadcastItems();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision");

  ros::NodeHandle nodePub1;

  ros::NodeHandle nodeSub1;
  ros::NodeHandle nodeSub2;
  ros::NodeHandle nodeSub3;

  // Declared as global var
  pub_info = nodePub1.advertise<std_msgs::String>("bj_vis_info", 10);

  ros::Subscriber sub1 = nodeSub1.subscribe("/darknet_ros/bounding_boxes", 10, Callback_BoundingBoxes);
  ros::Subscriber sub2 = nodeSub2.subscribe("/head_camera/depth_registered/image", 10, Callback_Depth);
  // ros::Subscriber sub3 = nodeSub3.subscribe("/head_camera/depth_registered/camera_info", 10, Callback_Info);

  ros::spin();

  return 0;
}
