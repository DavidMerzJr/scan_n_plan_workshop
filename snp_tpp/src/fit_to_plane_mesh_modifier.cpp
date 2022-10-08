#include "fit_to_plane_mesh_modifier.h"
#include "ui_fit_to_plane_mesh_modifier_widget.h"

#include <noether_gui/plugin_interface.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

namespace snp_tpp
{

FitToPlaneMeshModifier::FitToPlaneMeshModifier(
    const double& parameter_1,
    const double& parameter_2,
    const double& parameter_3,
    const double& parameter_4,
    const double& parameter_5)
  : parameter_1_(parameter_1)
  , parameter_2_(parameter_2)
  , parameter_3_(parameter_3)
  , parameter_4_(parameter_4)
  , parameter_5_(parameter_5)
{
  return;
}

std::vector<pcl::PolygonMesh> FitToPlaneMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Extract the point cloud
  pcl::PointCloud<pcl::PointXYZ> input_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, input_cloud);

  // If you modify the values of points, but do not add, remove, or reorder them in the point
  // cloud, you don't have to mess with the connections between points at all.  You can just put
  // them right back with the original connectivity in the original mesh.
  pcl::PointCloud<pcl::PointXYZ> output_cloud = input_cloud;

  //===================================================//
  //  DO LOTS OF EXCITING STUFF TO OUTPUT_CLOUD HERE   //
  // (using the parameters you passed in from the gui) //
  //===================================================//

  // And then put it back.
  pcl::PolygonMesh output_mesh (mesh);
  pcl::toPCLPointCloud2(output_cloud, output_mesh.cloud);

  return { output_mesh };
}

FitToPlaneMeshModifierWidget::FitToPlaneMeshModifierWidget(QWidget* parent)
  : noether::MeshModifierWidget(parent)
  , ui_(new Ui::FitToPlaneMeshModifier())
{
  ui_->setupUi(this);
  return;
}

FitToPlaneMeshModifierWidget::~FitToPlaneMeshModifierWidget()
{
  return;
}

noether::MeshModifier::ConstPtr FitToPlaneMeshModifierWidget::create() const
{
  return std::make_unique<FitToPlaneMeshModifier>(
        ui_->parameter_1->value(),
        ui_->parameter_2->value(),
        ui_->parameter_3->value(),
        ui_->parameter_4->value(),
        ui_->parameter_5->value());
}

}
