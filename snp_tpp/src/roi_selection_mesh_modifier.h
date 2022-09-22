#pragma once

#include <Eigen/Dense>
#include <noether_tpp/core/mesh_modifier.h>
#include <noether_filtering/submesh_extraction/extruded_polygon_mesh_extractor.h>
#include <noether_gui/widgets.h>
#include <QWidget>
#include <rclcpp/node.hpp>
#include <rclcpp/client.hpp>
#include <rviz_polygon_selection_tool/srv/get_selection.hpp>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <noether_tpp/core/mesh_modifier.h>
#include <noether_filtering/submesh_extraction/extruded_polygon_mesh_extractor.h>
#include <noether_gui/widgets.h>
#include <pcl/point_types.h>
#include <QWidget>

namespace Ui
{
class ROISelectionMeshModifier;
class ColorSelectionMeshModifier;
}

namespace snp_tpp
{
class ROISelectionMeshModifier : public noether::MeshModifier
{
public:
  ROISelectionMeshModifier(rclcpp::Node::SharedPtr node,
                           noether::ExtrudedPolygonSubMeshExtractor extractor,
                           std::vector<geometry_msgs::msg::PointStamped> boundary);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

private:
  const std::vector<geometry_msgs::msg::PointStamped> boundary_;
  const noether::ExtrudedPolygonSubMeshExtractor extractor_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

class ColorSelectionMeshModifier : public noether::MeshModifier
{
public:
  ColorSelectionMeshModifier(
      noether::ExtrudedPolygonSubMeshExtractor extractor,
      const pcl::PointXYZRGB background_color_);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

private:
  const noether::ExtrudedPolygonSubMeshExtractor extractor_;
  pcl::PointXYZRGB background_color_;
};

class ROISelectionMeshModifierWidget : public noether::MeshModifierWidget
{
  Q_OBJECT
public:
  ROISelectionMeshModifierWidget(QWidget* parent = nullptr);
  virtual ~ROISelectionMeshModifierWidget();

  noether::MeshModifier::ConstPtr create() const override;

private:
  void spin();

  Ui::ROISelectionMeshModifier* ui_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<rviz_polygon_selection_tool::srv::GetSelection>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

class ColorSelectionMeshModifierWidget : public noether::MeshModifierWidget
{
  Q_OBJECT
public:
  ColorSelectionMeshModifierWidget(QWidget* parent = nullptr);
  virtual ~ColorSelectionMeshModifierWidget();

  noether::MeshModifier::ConstPtr create() const override;

private:
  Ui::ColorSelectionMeshModifier* ui_;
};

}  // namespace snp_tpp
