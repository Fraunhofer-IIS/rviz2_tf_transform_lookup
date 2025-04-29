/****************************************************************************************************
 * MIT License
 * 
 * Copyright (c) 2025 Fraunhofer Institute for Integrated Circuits
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ****************************************************************************************************/

/*
 * Author(s) : Gokhul Raj Ravikumar (gokhulraj6200@gmail.com)
 * Maintainer(s) : Sebastian Zarnack (sebastian.zarnack@iis.fraunhofer.de)
 * Desc      : rviz2 plugin to lookup TF transforms
 * Created   : 2025 - April - 29
*/

#ifndef RVIZ2_TF_TRANSFORM_LOOKUP__TF_LOOKUP_HPP_
#define RVIZ2_TF_TRANSFORM_LOOKUP__TF_LOOKUP_HPP_

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <std_msgs/msg/string.hpp>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QComboBox>
#include <QClipboard>

namespace rviz2_tf_transform_lookup
{
class TfLookup
  : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit TfLookup(QWidget * parent = 0);
  QVBoxLayout* main_layout;
  ~TfLookup() override;
  void onInitialize() override;
signals:
    void updateTransformUI(const QString &translation, const QString &rotation, const QString &euler);
protected:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  QLabel *label_1, *label_2, *label_3, *label_4, *label_5, *label_6;
  QComboBox *button_1, *button_2;
  QPushButton *copy_translation, *copy_rotation, *copy_euler;
  QLineEdit *translation, *rotation, *euler;
  std::string frame_1, frame_2;
private Q_SLOTS:
  void tflookupTransform();
  void handleOption(QComboBox *button, const QString &frame);
  void populateFrames();
  void updateUI(const QString &translation, const QString &rotation, const QString &euler);
  void copyStatus(QWidget *parent, const QString &message);
};
}  // namespace rviz2_tf_transform_lookup

#endif  // RVIZ2_TF_TRANSFORM_LOOKUP__TF_LOOKUP_HPP_
