#include "color_selection_mesh_modifier.h"
#include "ui_color_selection_mesh_modifier_widget.h"

#include <algorithm>  // std::max()
#include <limits>     // std::numeric_limits::lowest(), std::numeric_limits::max()
#include <utility>    // std::make_pair(), std::pair
#include <vector>     // std::vector

#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include <noether_gui/plugin_interface.h>

namespace snp_tpp
{

ColorSelectionMeshModifier::ColorSelectionMeshModifier(
    noether::ExtrudedPolygonSubMeshExtractor extractor,
    const pcl::PointXYZRGB target_color,
    const int threshold)
  : extractor_(extractor)
  , target_color_(target_color)
  , threshold_(threshold)
{
  return;
}

std::vector<pcl::PolygonMesh> ColorSelectionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Extract the colorized point cloud
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);

  // Find points sufficiently different from the selected color
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  selected_cloud->reserve(cloud.size());
  for (const pcl::PointXYZRGB& p : cloud)
  {
    // Find the scaling factor needed to make each number into 255.  Then, pick the smallest
    // scaling factor.  (This will take the largest r/g/b value to 255.)
    double scaling_factor = std::min<double>(
        {
          1.0 + static_cast<double>(255 - p.r) / static_cast<double>(std::max<std::uint8_t>(p.r, 1)),
          1.0 + static_cast<double>(255 - p.g) / static_cast<double>(std::max<std::uint8_t>(p.g, 1)),
          1.0 + static_cast<double>(255 - p.b) / static_cast<double>(std::max<std::uint8_t>(p.b, 1))
        });

    // Apply the scalar to normalize the color
    pcl::PointXYZRGB p_normalized = p;
    p_normalized.r *= scaling_factor;
    p_normalized.g *= scaling_factor;
    p_normalized.b *= scaling_factor;

    // Calculate the distance in color space between the normalized color and target color
    int distance_squared =
        static_cast<int>(p_normalized.r - target_color_.r) * static_cast<int>(p_normalized.r - target_color_.r) +
        static_cast<int>(p_normalized.g - target_color_.g) * static_cast<int>(p_normalized.g - target_color_.g) +
        static_cast<int>(p_normalized.b - target_color_.b) * static_cast<int>(p_normalized.b - target_color_.b);

    // Keep the point if it's close enough
    if (distance_squared < threshold_)
    {
      selected_cloud->push_back(p);
    }
  }

  // Cluster the colored points
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZRGB>());
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clusterer;
  clusterer.setClusterTolerance(0.025); // 2.5cm, ~1 in
  clusterer.setMinClusterSize(10);      // Small
  clusterer.setMaxClusterSize(1000);    // Large
  clusterer.setSearchMethod(kd_tree);
  clusterer.setInputCloud(selected_cloud);
  clusterer.extract(cluster_indices);

  // Throw a bounding box / polygon on each area
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> bounding_boxes;
  for (const pcl::PointIndices& cluster : cluster_indices)
  {
    // Find the min and max coordinates of the points in this cluster
    Eigen::Vector3d min = Eigen::Vector3d(1.0, 1.0, 1.0) * std::numeric_limits<double>::max();
    Eigen::Vector3d max = Eigen::Vector3d(1.0, 1.0, 1.0) * std::numeric_limits<double>::lowest();
    for (const int& i : cluster.indices)
    {
      min.x() = std::min<double>(min.x(), static_cast<double>(selected_cloud->at(i).x));
      min.y() = std::min<double>(min.y(), static_cast<double>(selected_cloud->at(i).y));
      min.z() = std::min<double>(min.z(), static_cast<double>(selected_cloud->at(i).z));
      max.x() = std::max<double>(max.x(), static_cast<double>(selected_cloud->at(i).x));
      max.y() = std::max<double>(max.y(), static_cast<double>(selected_cloud->at(i).y));
      max.z() = std::max<double>(max.z(), static_cast<double>(selected_cloud->at(i).z));
    }

    // Save the min point and the dimensions
    bounding_boxes.push_back(std::pair<Eigen::Vector3d, Eigen::Vector3d>(min, max - min));
  }

  // Select out the enclosed submesh, including enclosed areas of background color
  std::vector<pcl::PolygonMesh> return_meshes;
  for (const std::pair<Eigen::Vector3d, Eigen::Vector3d>& bb : bounding_boxes)
  {
    Eigen::MatrixX3d boundary(4, 3);
    boundary.row(0) = bb.first;
    boundary.row(1) = bb.first + Eigen::Vector3d(bb.second.x(), 0.0, 0.0);
    boundary.row(2) = bb.first + Eigen::Vector3d(bb.second.x(), bb.second.y(), 0.0);
    boundary.row(3) = bb.first + Eigen::Vector3d(0.0, bb.second.y(), 0.0);

    return_meshes.push_back(extractor_.extract(mesh, boundary));
  }
  return return_meshes;
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

  int threshold = 25;

  return std::make_unique<ColorSelectionMeshModifier>(extractor, color, threshold);
}

} // namespace snp_tpp
