/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <string>
#include <sstream>
#include "../include/pointcloud_publisher_qt/qnode.hpp"
#include "../include/pointcloud_publisher_qt/getfile.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pointcloud_publisher_qt {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char **argv) : init_argc(argc), init_argv(argv) {}

QNode::~QNode() {
  if (ros::isStarted()) {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc, init_argv, "pointcloud_publisher_qt");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  n = std::make_shared<ros::NodeHandle>();
  // Add your ros communications here.
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "pointcloud_publisher_qt");
  if (!ros::master::check()) {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of
                 // scope.
  n = std::make_shared<ros::NodeHandle>();
  // Add your ros communications here.
  start();
  return true;
}

void QNode::loopPub() {
  for (std::vector<sensor_msgs::PointCloud2>::const_iterator it =
           output_set.begin();
       it != output_set.end(); it++) {
    cloud_publisher.publish(*it);
    ros::spinOnce();
    loop_rate->sleep();
  }
}

void QNode::setParam(std::string _topic_name, std::string _frame_id,
                     int _rate) {
  ros::param::param<std::string>("~topic_name", _topic_name,
                                 "/kinect2/sd/points");
  ros::param::param<std::string>("~frame_id", frame_id, "kinect2_link");
  ros::param::param("~rate", rate, 10);
}

void QNode::run() {
  frame = 0;
  loop = next = last = publish = false;
  path = ros::package::getPath("pointcloud_publisher_qt") + "/pcd";
  ros::param::param<std::string>("~topic_name", topic_name,
                                 "/kinect2/sd/points");
  ros::param::param<std::string>("~frame_id", frame_id, "kinect2_link");
  ros::param::param("~rate", rate, 10);
  file_names pcd_files = get_files(path, "pcd");
  for (std::vector<std::string>::const_iterator it =
           pcd_files.ext_files.begin();
       it != pcd_files.ext_files.end(); it++) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 output;
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(*it, *cloud);
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = ros::Time::now();

    output_set.push_back(output);
    cloud_set.push_back(cloud);
  }
  total = output_set.size();
  log(Info, "topic name: " + topic_name);
  log(Info, "frame id: " + frame_id);
  log(Info, "rate: " + std::to_string(rate));
  log(Info, "total frame: " + std::to_string(total));
  Q_EMIT
  nodeReady();
  while (ros::ok()) {
    ros::param::param<std::string>("~topic_name", topic_name,
                                   "/kinect2/sd/points");
    ros::param::param<std::string>("~frame_id", frame_id, "kinect2_link");
    ros::param::param("~rate", rate, 10);
    loop_rate = std::make_shared<ros::Rate>(rate);
    cloud_publisher = n->advertise<sensor_msgs::PointCloud2>(topic_name, 1);
    if (publish) {
      if (loop) {
        loopPub();
      } else {
        cloud_publisher.publish(output_set[frame]);
        ros::spinOnce();
        loop_rate->sleep();
      }
    }
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT
  rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log(const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(), 1);
  std::stringstream logging_model_msg;
  switch (level) {
    case (Debug): {
      ROS_DEBUG_STREAM(msg);
      logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Info): {
      ROS_INFO_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Warn): {
      ROS_WARN_STREAM(msg);
      logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Error): {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
    }
    case (Fatal): {
      ROS_FATAL_STREAM(msg);
      logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
      break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                        new_row);
  Q_EMIT loggingUpdated();  // used to readjust the scrollbar
}

}  // namespace pointcloud_publisher_qt
