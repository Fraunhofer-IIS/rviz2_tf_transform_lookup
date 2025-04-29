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

#include <rviz2_tf_transform_lookup/tf_lookup.hpp>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QHBoxLayout>
#include <QMenu>
#include <QDebug>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>
#include <QTimer>
#include <QThread>
#include <QtConcurrent/QtConcurrent>
#include <QGuiApplication>

namespace rviz2_tf_transform_lookup
{
TfLookup::TfLookup(QWidget* parent) : Panel(parent)
{
  // Create a main vertical layout for the panel
  main_layout = new QVBoxLayout(this);
  main_layout->setContentsMargins(10, 10, 10, 10);

  // Create a horizontal layout for the "From" section
  auto layout_1 = new QHBoxLayout();
  layout_1->setContentsMargins(10, 0, 10, 0);
  label_1 = new QLabel("From:");
  button_1 = new QComboBox(this);
  layout_1->addWidget(label_1);
  layout_1->addWidget(button_1);

  // Create a horizontal layout for the "To" section
  auto layout_2 = new QHBoxLayout();
  layout_2->setContentsMargins(10, 0, 10, 0);
  label_2 = new QLabel("To:");
  button_2 = new QComboBox(this);
  layout_2->addWidget(label_2);
  layout_2->addWidget(button_2);

  auto layout_3 = new QHBoxLayout();
  layout_3->setContentsMargins(10, 0, 10, 0);
  label_3 = new QLabel("Translation:");
  translation = new QLineEdit("[0.0,0.0,0.0]");
  translation->setMaximumWidth(215);
  copy_translation = new QPushButton();
  copy_translation->setIcon(QIcon(":/resources/clipboard.svg"));
  copy_translation->setToolTip("Copy translation to clipboard");
  copy_translation->setFixedSize(20, 20);
  layout_3->addWidget(label_3);
  // layout_3->addSpacing(150);
  layout_3->addWidget(translation);
  layout_3->addWidget(copy_translation);

  auto layout_4 = new QHBoxLayout();
  layout_4->setContentsMargins(10, 0, 10, 0);
  label_4 = new QLabel("Rotation:");
  rotation = new QLineEdit("[0.0,0.0,0.0]");
  rotation->setMaximumWidth(215);
  copy_rotation = new QPushButton();
  copy_rotation->setIcon(QIcon(":/resources/clipboard.svg"));
  copy_rotation->setToolTip("Copy rotation to clipboard");
  copy_rotation->setFixedSize(20, 20);
  layout_4->addWidget(label_4);
  layout_4->addWidget(rotation);
  layout_4->addWidget(copy_rotation);

  auto layout_5 = new QHBoxLayout();
  layout_5->setContentsMargins(10, 0, 10, 0);
  label_5 = new QLabel("Euler:");
  euler = new QLineEdit("[0.0,0.0,0.0]");
  euler->setMaximumWidth(215);
  copy_euler = new QPushButton();
  copy_euler->setIcon(QIcon(":/resources/clipboard.svg"));
  copy_euler->setToolTip("Copy euler to clipboard");
  copy_euler->setFixedSize(20, 20);
  layout_5->addWidget(label_5);
  layout_5->addWidget(euler);
  layout_5->addWidget(copy_euler);

  // Add the horizontal layouts to the main layout
  main_layout->addLayout(layout_1);
  main_layout->addLayout(layout_2);
  main_layout->addLayout(layout_3);
  main_layout->addLayout(layout_4);
  main_layout->addLayout(layout_5);
  // Connect the button's click event to the corresponding callback function
  QObject::connect(button_1, &QComboBox::currentTextChanged, this, [this](const QString &frame) {
    handleOption(button_1, frame);
  });
  QObject::connect(button_2, &QComboBox::currentTextChanged, this, [this](const QString &frame) {
    handleOption(button_2, frame);
  });
  // Connect the signal to the slot for UI updates
  QObject::connect(this, &TfLookup::updateTransformUI, this, &TfLookup::updateUI);
  // Connect the copy buttons to the copyStatus function
  QObject::connect(copy_translation, &QPushButton::clicked, this, [this]() {
      QGuiApplication::clipboard()->setText(translation->text());
      copyStatus(copy_translation, "Copied!");
  });
  QObject::connect(copy_rotation, &QPushButton::clicked, this, [this]() {
      QGuiApplication::clipboard()->setText(rotation->text());
      copyStatus(copy_rotation, "Copied!");
  });
  QObject::connect(copy_euler, &QPushButton::clicked, this, [this]() {
      QGuiApplication::clipboard()->setText(euler->text());
      copyStatus(copy_euler, "Copied!");
  });
}

TfLookup::~TfLookup() = default;

void TfLookup::onInitialize()
{
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();
  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  // Initialize TF2 Buffer and Listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Introduce a timer delay before populating the frames
  QTimer::singleShot(1000, this, &TfLookup::populateFrames);
  // dynamic refresh of Transform, to set the update rate change the sleep rate..
  QtConcurrent::run([this]() {
    while (true) {
            tflookupTransform();
            QThread::sleep(0.2);
    }
  });
}

void TfLookup::copyStatus(QWidget *parent, const QString &message)
{
    auto *status = new QLabel(parent);
    status->setText(message);
    status->setStyleSheet("background-color: #444; color: white; padding: 4px 10px; border-radius: 5px;");
    status->setAlignment(Qt::AlignCenter);
    status->setWindowFlags(Qt::ToolTip);
    status->adjustSize();

    // Position it above the parent widget
    QPoint pos = parent->mapToGlobal(QPoint(0, -status->height() - 5));
    status->move(pos);
    status->show();

    QTimer::singleShot(1000, status, [status]() {
        status->close();
        status->deleteLater();
    });
}

void TfLookup::populateFrames()
{
  button_1->clear();  // Clear the existing items
  button_2->clear();
  button_1->addItem("select TF Frame_1");
  button_2->addItem("select TF Frame_2");
  // Declare a vector to store the frame names
  std::vector<std::string> frame_ids;
  try
  {
      tf_buffer_->_getFrameStrings(frame_ids);  // Retrieve frame names
  }
  catch (const std::exception &e)
  {
      qDebug() << "Error getting frames: " << e.what();
      return;
  }

  // Populate the QComboBox with frame names
  for (const auto &frame : frame_ids)
  {
      button_1->addItem(QString::fromStdString(frame));
      button_2->addItem(QString::fromStdString(frame));
  }
  // Sort the combo boxes alphabetically
  button_1->model()->sort(0);
  button_2->model()->sort(0);
}

void TfLookup::handleOption(QComboBox *button, const QString &frame)
{
    button->setCurrentText(frame);
    qDebug() << frame << " selected";
    tflookupTransform();
}

void TfLookup::updateUI(const QString &translation, const QString &rotation, const QString &euler)
{
    // Update the QLineEdit fields with the new transform data
    this->translation->setText(translation);
    this->rotation->setText(rotation);
    this->euler->setText(euler);
}
void TfLookup::tflookupTransform()
{
    // Get the frame names from button_1 and button_2
    auto frame_1 = button_1->currentText().toStdString();
    auto frame_2 = button_2->currentText().toStdString();
    try
    {
        std::string error_msg;
        if (!tf_buffer_->canTransform(frame_2, frame_1, rclcpp::Time(0), rclcpp::Duration::from_seconds(1), &error_msg))
        {
            if (node_ptr_)
            {
                auto logger = node_ptr_->get_raw_node()->get_logger();
                RCLCPP_DEBUG(logger, "Cannot transform from '%s' to '%s': %s",
                            frame_1.c_str(), frame_2.c_str(), error_msg.c_str());
            }
            emit updateTransformUI("N/A", "N/A", "N/A");
            return;
        }

        // Use the tf_buffer_ to look up the transform from frame_1 to frame_2
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped = tf_buffer_->lookupTransform(frame_2, frame_1, rclcpp::Time(0)); // Lookup transform from frame_1 to frame_2

        // Format translation and rotation as text and update the UI
        QString translation_str = "[" + QString::number(transformStamped.transform.translation.x, 'f', 4) + ", "
                                  + QString::number(transformStamped.transform.translation.y, 'f', 4) + ", "
                                  + QString::number(transformStamped.transform.translation.z, 'f', 4) + "]";

        QString rotation_str = "[" + QString::number(transformStamped.transform.rotation.x, 'f', 4) + ", "
                               + QString::number(transformStamped.transform.rotation.y, 'f', 4) + ", "
                               + QString::number(transformStamped.transform.rotation.z, 'f', 4) + ", "
                               + QString::number(transformStamped.transform.rotation.w, 'f', 4) + "]";

        // Convert quaternion to Euler angles
        tf2::Quaternion quat(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                             transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        QString euler_str = "[" + QString::number(roll, 'f', 4) + ", "
                            + QString::number(pitch, 'f', 4) + ", "
                            + QString::number(yaw, 'f', 4) + "]";

        // Emit the UI update signal
        emit updateTransformUI(translation_str, rotation_str, euler_str);
    }
    catch (const tf2::TransformException &e)
    {
        if (node_ptr_)
        {
            auto logger = node_ptr_->get_raw_node()->get_logger();
            RCLCPP_DEBUG(logger, "Transform lookup failed from '%s' to '%s': %s",
                        frame_1.c_str(), frame_2.c_str(), e.what());
        }
        emit updateTransformUI("N/A", "N/A", "N/A");
    }
}
}  // namespace rviz2_tf_transform_lookup

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2_tf_transform_lookup::TfLookup, rviz_common::Panel)