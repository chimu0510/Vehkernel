#include "ugv_model_viewer/model_panel.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollBar>
#include <QTreeWidget>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace ugv_model_viewer
{

namespace
{

enum Column { kName, kSize, kTrack, kWheelbase, kWheel, kColumnCount };

constexpr int kModelRole = Qt::UserRole;

QString mm(const QJsonValue & metres)
{
  return metres.isDouble() ? QString::number(qRound(metres.toDouble() * 1000.0)) : "-";
}

QString sizeText(const QJsonValue & lwh)
{
  const QJsonArray a = lwh.toArray();
  return a.size() == 3 ?
         QString("%1 × %2 × %3").arg(mm(a[0])).arg(mm(a[1])).arg(mm(a[2])) : "-";
}

}  // namespace

ModelPanel::ModelPanel(QWidget * parent)
: rviz_common::Panel(parent),
  search_(new QLineEdit),
  tree_(new QTreeWidget),
  position_(new QLabel),
  details_(new QLabel)
{
  search_->setPlaceholderText("Search models…");
  search_->setClearButtonEnabled(true);

  auto * prev = new QPushButton("◀ Prev");
  auto * next = new QPushButton("Next ▶");
  auto * nav = new QHBoxLayout;
  nav->addWidget(prev);
  nav->addWidget(next);
  nav->addStretch();
  nav->addWidget(position_);

  tree_->setColumnCount(kColumnCount);
  tree_->setHeaderLabels({"Model", "L × W × H", "Track", "Wheelbase", "Wheel Ø"});
  tree_->headerItem()->setToolTip(kSize, "Overall size (mm)");
  tree_->headerItem()->setToolTip(kTrack, "Wheel centre to wheel centre, across (mm)");
  tree_->headerItem()->setToolTip(kWheelbase, "Wheel centre to wheel centre, along (mm)");
  tree_->headerItem()->setToolTip(kWheel, "Wheel diameter (mm)");
  for (int c = kSize; c < kColumnCount; ++c) {
    tree_->headerItem()->setTextAlignment(c, Qt::AlignRight | Qt::AlignVCenter);
  }
  tree_->header()->setStretchLastSection(false);
  tree_->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
  tree_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  tree_->setAlternatingRowColors(true);
  tree_->setUniformRowHeights(true);
  tree_->setRootIsDecorated(true);

  details_->setWordWrap(true);
  details_->setTextInteractionFlags(Qt::TextSelectableByMouse);

  auto * layout = new QVBoxLayout(this);
  layout->addWidget(search_);
  layout->addLayout(nav);
  layout->addWidget(tree_, 1);
  layout->addWidget(details_);

  connect(search_, &QLineEdit::textChanged, this, &ModelPanel::filter);
  connect(prev, &QPushButton::clicked, this, [this] {step(-1);});
  connect(next, &QPushButton::clicked, this, [this] {step(1);});
  connect(
    tree_, &QTreeWidget::currentItemChanged, this,
    [this](QTreeWidgetItem * item) {onCurrentItemChanged(item);});
}

void ModelPanel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  const auto latched = rclcpp::QoS(1).transient_local();

  select_pub_ = node->create_publisher<std_msgs::msg::String>("/model_viewer/select", 10);
  // Callbacks may run off the GUI thread: hand the data over with queued calls.
  catalog_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/model_viewer/catalog", latched, [this](const std_msgs::msg::String & msg) {
      QMetaObject::invokeMethod(
        this, "applyCatalog", Qt::QueuedConnection,
        Q_ARG(QString, QString::fromStdString(msg.data)));
    });
  current_sub_ = node->create_subscription<std_msgs::msg::String>(
    "/model_viewer/current", latched, [this](const std_msgs::msg::String & msg) {
      QMetaObject::invokeMethod(
        this, "applyCurrent", Qt::QueuedConnection,
        Q_ARG(QString, QString::fromStdString(msg.data)));
    });
}

void ModelPanel::applyCatalog(const QString & json)
{
  const QJsonArray entries =
    QJsonDocument::fromJson(json.toUtf8()).object().value("models").toArray();

  updating_ = true;
  tree_->clear();
  models_.clear();
  QHash<QString, QTreeWidgetItem *> families;
  for (const QJsonValue & value : entries) {
    const QJsonObject entry = value.toObject();
    const QString family = entry.value("family").toString();
    QTreeWidgetItem *& group = families[family];
    if (group == nullptr) {
      group = new QTreeWidgetItem(tree_, {family});
      group->setFlags(Qt::ItemIsEnabled);
      group->setFirstColumnSpanned(true);
      QFont font = group->font(kName);
      font.setBold(true);
      group->setFont(kName, font);
    }
    const QString name = entry.value("name").toString();
    auto * item = new QTreeWidgetItem(
      group, {name, sizeText(entry.value("size")), mm(entry.value("track")),
        mm(entry.value("wheelbase")), mm(entry.value("wheel_diameter"))});
    item->setData(kName, kModelRole, name);
    for (int c = kSize; c < kColumnCount; ++c) {
      item->setTextAlignment(c, Qt::AlignRight | Qt::AlignVCenter);
    }
    models_.append(name);
  }
  for (QTreeWidgetItem * group : families) {
    group->setText(kName, QString("%1  (%2)").arg(group->text(kName)).arg(group->childCount()));
  }
  tree_->expandAll();
  updating_ = false;

  // Widen the panel (and its dock) so every column fits without scrolling.
  tree_->setMinimumWidth(
    tree_->header()->length() + tree_->verticalScrollBar()->sizeHint().width() +
    2 * tree_->frameWidth());

  filter(search_->text());
  markCurrent();
}

void ModelPanel::applyCurrent(const QString & model)
{
  current_ = model;
  markCurrent();
}

void ModelPanel::filter(const QString & text)
{
  for (int g = 0; g < tree_->topLevelItemCount(); ++g) {
    QTreeWidgetItem * group = tree_->topLevelItem(g);
    bool any = false;
    for (int i = 0; i < group->childCount(); ++i) {
      const bool match = group->child(i)->text(kName).contains(text, Qt::CaseInsensitive);
      group->child(i)->setHidden(!match);
      any = any || match;
    }
    group->setHidden(!any);
  }
}

void ModelPanel::onCurrentItemChanged(QTreeWidgetItem * item)
{
  if (updating_ || item == nullptr) {
    return;
  }
  const QString model = item->data(kName, kModelRole).toString();
  if (!model.isEmpty() && model != current_) {
    select(model);
  }
}

void ModelPanel::step(int offset)
{
  // Step through the models left visible by the search filter.
  QStringList visible;
  for (const QString & model : models_) {
    QTreeWidgetItem * item = findModel(model);
    if (item != nullptr && !item->isHidden() && !item->parent()->isHidden()) {
      visible.append(model);
    }
  }
  if (visible.isEmpty()) {
    return;
  }
  const int index = visible.indexOf(current_);
  const int n = visible.size();
  select(visible[index < 0 ? 0 : ((index + offset) % n + n) % n]);
}

void ModelPanel::select(const QString & model)
{
  if (select_pub_) {
    std_msgs::msg::String msg;
    msg.data = model.toStdString();
    select_pub_->publish(msg);
  }
}

QTreeWidgetItem * ModelPanel::findModel(const QString & model) const
{
  for (int g = 0; g < tree_->topLevelItemCount(); ++g) {
    QTreeWidgetItem * group = tree_->topLevelItem(g);
    for (int i = 0; i < group->childCount(); ++i) {
      if (group->child(i)->data(kName, kModelRole).toString() == model) {
        return group->child(i);
      }
    }
  }
  return nullptr;
}

void ModelPanel::markCurrent()
{
  QTreeWidgetItem * current = findModel(current_);
  for (const QString & model : models_) {
    QTreeWidgetItem * item = findModel(model);
    QFont font = item->font(kName);
    font.setBold(item == current);
    item->setFont(kName, font);
  }

  const int index = models_.indexOf(current_);
  position_->setText(index < 0 ? "" : QString("%1 / %2").arg(index + 1).arg(models_.size()));
  if (current == nullptr) {
    details_->clear();
    return;
  }
  details_->setText(
    QString("<b>%1</b> &nbsp;·&nbsp; %2<br>"
            "Size %3 mm &nbsp;·&nbsp; track %4 &nbsp;·&nbsp; wheelbase %5 &nbsp;·&nbsp; "
            "wheel Ø %6 mm")
    .arg(current_, current->parent()->text(kName).section("  (", 0, 0),
    current->text(kSize), current->text(kTrack), current->text(kWheelbase),
    current->text(kWheel)));

  updating_ = true;
  tree_->setCurrentItem(current);
  tree_->scrollToItem(current);
  updating_ = false;
}

}  // namespace ugv_model_viewer

PLUGINLIB_EXPORT_CLASS(ugv_model_viewer::ModelPanel, rviz_common::Panel)
