/**
 * @file /include/pointcloud_publisher_qt/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef pointcloud_publisher_qt_QNODE_HPP_
#define pointcloud_publisher_qt_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pointcloud_publisher_qt {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
  Q_OBJECT
 public:
  QNode(int argc, char **argv);
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();
  void setParam(std::string _topic_name, std::string _frame_id, int _rate);
  void loopPub();
  int rate, frame, total;
  bool loop, next, last, publish;
  /*********************
  ** Logging
  **********************/
  enum LogLevel { Debug, Info, Warn, Error, Fatal };

  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void nodeReady();

 private:
  int init_argc;
  char **init_argv;
  std::shared_ptr<ros::NodeHandle> n;
  ros::Publisher cloud_publisher;
  QStringListModel logging_model;
  std::string topic_name, path, frame_id;
  std::shared_ptr<ros::Rate> loop_rate;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_set;
  std::vector<sensor_msgs::PointCloud2> output_set;
};

}  // namespace pointcloud_publisher_qt

#endif /* pointcloud_publisher_qt_QNODE_HPP_ */
