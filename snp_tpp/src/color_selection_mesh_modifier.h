#pragma once

#include <Eigen/Dense>
#include <noether_tpp/core/mesh_modifier.h>
#include <noether_filtering/submesh_extraction/extruded_polygon_mesh_extractor.h>
#include <noether_gui/widgets.h>
#include <pcl/point_types.h>
#include <QWidget>

namespace Ui
{
class ColorSelectionMeshModifier;
}

namespace snp_tpp
{
class ColorSelectionMeshModifier : public noether::MeshModifier
{
public:
  ColorSelectionMeshModifier(
      noether::ExtrudedPolygonSubMeshExtractor& extractor,
      const pcl::PointXYZRGB& background_color_);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

private:
  const noether::ExtrudedPolygonSubMeshExtractor extractor_;
  pcl::PointXYZRGB background_color_;
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
} // namespace snp_tpp
