#include "color_selection_mesh_modifier.h"
#include "ui_color_selection_mesh_modifier_widget.h"

#include <noether_gui/plugin_interface.h>

namespace snp_tpp
{

ColorSelectionMeshModifier::ColorSelectionMeshModifier(
    noether::ExtrudedPolygonSubMeshExtractor extractor,
    const pcl::PointXYZRGB background_color)
  : extractor_(extractor)
  , background_color_(background_color)
{
  return;
}

std::vector<pcl::PolygonMesh> ColorSelectionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // TODO: Find points sufficiently different from the selected color
  // TODO: Cluster those points
  // TODO: Throw a bounding box / polygon on each area
  // TODO: Select out the enclosed submesh, including enclosed areas of background color
  return { mesh };
}


ColorSelectionMeshModifierWidget::ColorSelectionMeshModifierWidget(QWidget* parent)
  : noether::MeshModifierWidget(parent)
  , ui_(new Ui::ColorSelectionMeshModifier())
{
  ui_->setupUi(this);
  return;
}

ColorSelectionMeshModifierWidget::~ColorSelectionMeshModifierWidget()
{
  return;
}

noether::MeshModifier::ConstPtr ColorSelectionMeshModifierWidget::create() const
{
  noether::ExtrudedPolygonSubMeshExtractor extractor;
  extractor.params.max_cluster_size = ui_->max_cluster_size->value();
  extractor.params.min_cluster_size = ui_->min_cluster_size->value();
  extractor.params.cluster_tolerance = ui_->cluster_tolerance->value();
  extractor.params.plane_distance_threshold = ui_->plane_distance_threshold->value();

  pcl::PointXYZRGB color;
  color.r = static_cast<std::uint8_t>(ui_->spin_box_red->value());
  color.g = static_cast<std::uint8_t>(ui_->spin_box_green->value());
  color.b = static_cast<std::uint8_t>(ui_->spin_box_blue->value());

  return std::make_unique<ColorSelectionMeshModifier>(extractor, color);
}

} // namespace snp_tpp
