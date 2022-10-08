#pragma once

#include <vector>

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_gui/widgets.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <QWidget>

namespace Ui
{
class FitToPlaneMeshModifier;
}

namespace snp_tpp
{
class FitToPlaneMeshModifier : public noether::MeshModifier
{
public:
  FitToPlaneMeshModifier(
      const double& parameter_1,
      const double& parameter_2,
      const double& parameter_3,
      const double& parameter_4,
      const double& parameter_5);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

private:
  double parameter_1_;
  double parameter_2_;
  double parameter_3_;
  double parameter_4_;
  double parameter_5_;
};

class FitToPlaneMeshModifierWidget : public noether::MeshModifierWidget
{
  Q_OBJECT
public:
  FitToPlaneMeshModifierWidget(QWidget* parent = nullptr);
  virtual ~FitToPlaneMeshModifierWidget();

  noether::MeshModifier::ConstPtr create() const override;

private:
  Ui::FitToPlaneMeshModifier* ui_;
};
} // namespace snp_tpp
