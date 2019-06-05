/**
 * @file /include/pointcloud_publisher_qt/main_window.hpp
 *
 * @brief Qt based gui for pointcloud_publisher_qt.
 *
 * @date November 2010
 **/
#ifndef pointcloud_publisher_qt_MAIN_WINDOW_H
#define pointcloud_publisher_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace pointcloud_publisher_qt {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(int argc, char **argv, QWidget *parent = 0);
  ~MainWindow();

  void ReadSettings();   // Load up qt program settings at startup
  void WriteSettings();  // Save qt program settings when closing

  void closeEvent(QCloseEvent *event);  // Overloaded function
  void showNoMasterMessage();

 public Q_SLOTS:
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/
  void on_actionAbout_triggered();
  void on_button_connect_clicked(bool check);
  void on_checkbox_use_environment_stateChanged(int state);
  void disableAllwidgets();
  void enableAllwidgets();
  /******************************************
  ** Manual connections
  *******************************************/
  void updateLoggingView();  // no idea why this can't connect automatically

 private Q_SLOTS:
  void on_pushButton_right_clicked(bool checked);

  void on_pushButton_left_clicked(bool checked);

  void on_pushButton_loop_clicked(bool checked);

  void on_pushButton_P_toggled(bool checked);

  void on_spinBox_r_editingFinished();

  void on_lineEdit_tn_editingFinished();

  void on_lineEdit_fi_editingFinished();

 private:
  Ui::MainWindowDesign ui;
  QNode qnode;
};

}  // namespace pointcloud_publisher_qt

#endif  // pointcloud_publisher_qt_MAIN_WINDOW_H
