#pragma once

#include <QString>
#include <QStringList>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

class QLabel;
class QLineEdit;
class QTreeWidget;
class QTreeWidgetItem;

namespace ugv_model_viewer
{

// RViz panel listing every chassis model from /model_viewer/catalog, grouped by
// family. Selecting a row publishes it on /model_viewer/select.
class ModelPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ModelPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void applyCatalog(const QString & json);
  void applyCurrent(const QString & model);
  void filter(const QString & text);
  void onCurrentItemChanged(QTreeWidgetItem * item);
  void step(int offset);

private:
  void select(const QString & model);
  QTreeWidgetItem * findModel(const QString & model) const;
  void markCurrent();

  QLineEdit * search_;
  QTreeWidget * tree_;
  QLabel * position_;
  QLabel * details_;

  QStringList models_;  // catalog order, used by Prev / Next
  QString current_;
  bool updating_ = false;  // suppresses select() while the tree is changed programmatically

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr select_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr catalog_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_sub_;
};

}  // namespace ugv_model_viewer
