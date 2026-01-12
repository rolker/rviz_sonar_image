#ifndef RVIZ_SONAR_PROJECTED_IMAGE_SONAR_IMAGE_CURTAIN_H
#define RVIZ_SONAR_PROJECTED_IMAGE_SONAR_IMAGE_CURTAIN_H

#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <Ogre.h>

#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"
#include "rviz_rendering/objects/mesh_shape.hpp"
#include "rviz_sonar_image/color_map.h"

namespace rviz_sonar_image
{

class ProjectedSonarImageCurtain
{
public:
  ProjectedSonarImageCurtain( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, std::shared_ptr<ColorMap> color_map);
  ~ProjectedSonarImageCurtain();

  void addMessage(const marine_acoustic_msgs::msg::ProjectedSonarImage::ConstSharedPtr msg, uint32_t start_row, uint32_t end_row, int beam_number, const Ogre::Vector3& position, const Ogre::Quaternion& orientation );

  bool full() const;

  void updateAlpha(double alpha);

  std::pair<float,float> getDataValueRange() const { return std::make_pair(minimum_data_value_, maximum_data_value_); }

private:
  rviz_rendering::MeshShape* mesh_shape_;

  std::vector<std::vector<Ogre::Vector3> > vertices_;
  std::vector<float> texture_coordinates_;

  sensor_msgs::msg::Image::SharedPtr image_;
  rviz_default_plugins::displays::ROSImageTexture* texture_;
  std::shared_ptr<ColorMap> color_map_;

  int max_ping_count_=4096;
  int row_count_ = 0;

  float minimum_data_value_ = std::numeric_limits<float>::max();
  float maximum_data_value_ = std::numeric_limits<float>::lowest();
};

} // namespace rviz_sonar_image

#endif
