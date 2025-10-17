#ifndef RVIZ_SONAR_IMAGE_PROJECTED_SONAR_IMAGE_VISUAL_H
#define RVIZ_SONAR_IMAGE_PROJECTED_SONAR_IMAGE_VISUAL_H

#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"

#include <Ogre.h>

#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"
#include "rviz_rendering/objects/mesh_shape.hpp"

#include "rviz_sonar_image/color_map.h"

namespace rviz_sonar_image
{

class ProjectedSonarImageFan
{
public:
  ProjectedSonarImageFan( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, std::shared_ptr<ColorMap> color_map);
  ~ProjectedSonarImageFan();

  // if beam_number is negative, show all beams in the XY plane, otherwise
  // show selected beam in the XZ plane.
  void setMessage(const marine_acoustic_msgs::msg::ProjectedSonarImage::ConstSharedPtr msg, uint32_t start_row, uint32_t end_row);

  void setFramePosition( const Ogre::Vector3& position );
  void setFrameOrientation( const Ogre::Quaternion& orientation );

private:
  Ogre::SceneNode* frame_node_;

  // The SceneManager, kept here only so the destructor can ask it to
  // destroy the ``frame_node_``.
  Ogre::SceneManager* scene_manager_;

  rviz_rendering::MeshShape* mesh_shape_;

  rviz_default_plugins::displays::ROSImageTexture* texture_;
  std::shared_ptr<ColorMap> color_map_;
  float alpha_ = 0.8;
};

} // namespace rviz_sonar_image

#endif
