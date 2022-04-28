#include <rviz_sonar_image/sonar_image_visual.h>
#include <rviz_sonar_image/color_map.h>

#include <rviz/ogre_helpers/mesh_shape.h>
#include <rviz/image/ros_image_texture.h>


#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgrePolygon.h>
#include <OGRE/OgreManualObject.h>

namespace rviz_sonar_image
{

SonarImageVisual::SonarImageVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, std::shared_ptr<ColorMap> color_map )
  :scene_manager_(scene_manager), color_map_(color_map)
{
  frame_node_ = parent_node->createChildSceneNode();

  mesh_shape_ = new rviz::MeshShape(scene_manager, frame_node_);
  mesh_shape_->getMaterial()->getTechnique(0)->setLightingEnabled(false);
  Ogre::TextureUnitState* tu = mesh_shape_->getMaterial()->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureFiltering(Ogre::FT_MIN, Ogre::FO_ANISOTROPIC);
  tu->setTextureFiltering(Ogre::FT_MAG, Ogre::FO_POINT);
  tu->setTextureFiltering(Ogre::FT_MIP, Ogre::FO_POINT);
  
  texture_ = new rviz::ROSImageTexture();
  if (!texture_)
  {
    ROS_ERROR("Failed to create ROSImageTextures.");
  }

  mesh_shape_->setColor(1.0, 1.0, 1.0, 0.8);
}

SonarImageVisual::~SonarImageVisual()
{
  delete mesh_shape_;
  delete texture_;
  scene_manager_->destroySceneNode(frame_node_);
}

void SonarImageVisual::setMessage(const acoustic_msgs::RawSonarImage::ConstPtr& msg, uint32_t start_row, uint32_t end_row, int beam_number)
{
  mesh_shape_->clear();

  int max_steps = 128;
  int step_size = 1;
  start_row = std::min(start_row, msg->samples_per_beam);
  end_row = std::min(end_row, msg->samples_per_beam);
  if(!end_row>start_row)
    return;

  float start_range = ((start_row+msg->sample0)*msg->ping_info.sound_speed/msg->sample_rate)/2.0;

  auto sample_count = end_row-start_row;
  auto steps_per_beam = sample_count;

  if (steps_per_beam > max_steps)
  {
    step_size = sample_count/max_steps;
    steps_per_beam /= step_size;
  }

  float sample_length = (step_size*msg->ping_info.sound_speed/msg->sample_rate)/2.0;

  struct AnglesTexture
  {
    float y_angle;
    float z_angle;
    float texture_coordinate;
  };

  // beam edge at each end and all the beam centers
  std::vector<AnglesTexture> angles;
  if(beam_number < 0)
  {
    AnglesTexture at;
    at.y_angle = msg->rx_angles.front()-msg->ping_info.rx_beamwidths.front();
    at.z_angle = msg->tx_angles.front();
    at.texture_coordinate = 0.0;
    angles.push_back(at);
    for(int i = 0; i < msg->rx_angles.size(); i++)
    {
      AnglesTexture at;
      at.y_angle = msg->rx_angles[i];
      at.z_angle = msg->tx_angles[i];
      if(msg->rx_angles.size() > 1)
        at.texture_coordinate = i/float(msg->rx_angles.size()-1);
      else
        at.texture_coordinate = 0.5;
      angles.push_back(at);
    }
    at.y_angle = msg->rx_angles.back()+msg->ping_info.rx_beamwidths.back();
    at.z_angle = msg->tx_angles.back();
    at.texture_coordinate = 1.0;
    angles.push_back(at);
  }
  else
  {
    AnglesTexture at;
    at.y_angle = msg->rx_angles[beam_number];
    at.z_angle = msg->tx_angles[beam_number]-msg->ping_info.tx_beamwidths[beam_number];
    at.texture_coordinate = 0.0;
    angles.push_back(at);

    at.z_angle = msg->tx_angles[beam_number];
    at.texture_coordinate = 0.5;
    angles.push_back(at);

    at.z_angle = msg->tx_angles[beam_number]+msg->ping_info.tx_beamwidths[beam_number];
    at.texture_coordinate = 1.0;
    angles.push_back(at);
  }

  mesh_shape_->estimateVertexCount(angles.size()*steps_per_beam);
  mesh_shape_->beginTriangles();

  for(auto angle: angles)
  {
    float cosy = cos(angle.y_angle);
    float cosz = cos(angle.z_angle);
    float siny = sin(angle.y_angle);
    float sinz = sin(angle.z_angle);

    float startx = cosy*cosz*start_range;
    float starty = siny*start_range;
    float startz = sinz*start_range; 

    float dx = cosy*cosz*sample_length;
    float dy = siny*sample_length;
    float dz = sinz*sample_length;
    for(int i = 0; i < steps_per_beam; i++)
    {
      mesh_shape_->addVertex(Ogre::Vector3(startx+i*dx, starty+i*dy, startz+i*dz));
      mesh_shape_->getManualObject()->textureCoord(angle.texture_coordinate, i/float(steps_per_beam-1));
    }
  }

  for(int i = 0; i < angles.size()-1; i++)
  {
    int c1 = i*steps_per_beam;
    int c2 = c1 + steps_per_beam;
    for(int j = 0; j < steps_per_beam-1; j++)
    {
      mesh_shape_->addTriangle(c1+j, c1+j+1, c2+j+1);
      mesh_shape_->addTriangle(c2+j+1, c2+j, c1+j);
    }
  }

  mesh_shape_->endTriangles();

  sensor_msgs::Image::Ptr image(new sensor_msgs::Image);
  image->header.stamp = msg->header.stamp;
  image->width = msg->rx_angles.size();
  image->height = sample_count;
  // TODO, support other than floats
  image->encoding = "rgba8";
  image->step = image->width*4;

  const float* sonar_data = reinterpret_cast<const float*>(msg->image.data.data());
  for (uint32_t i = start_row*image->width; i < end_row*image->width; i++)
  {
    auto c = color_map_->lookup(sonar_data[i]);
    image->data.push_back(c.r*255);
    image->data.push_back(c.g*255);
    image->data.push_back(c.b*255);
    image->data.push_back(c.a*255);
  }

  texture_->addMessage(image);
  texture_->update();

  Ogre::Pass* pass = mesh_shape_->getMaterial()->getTechnique(0)->getPass(0);
  if (!pass)
  {
    ROS_ERROR("setMessage(): pass is NULL.");
    return;
  }

  if (pass->getNumTextureUnitStates() < 1)
  {
    ROS_ERROR("setMessage(): Number of texture unit states is less than 1.");
    return;
  }

  Ogre::TextureUnitState* unit_state = pass->getTextureUnitState(0);
  if (!unit_state)
  {
    ROS_ERROR("Failed to getTextureUnitState(%d).", 0);
    return;
  }

  unit_state->setTexture(texture_->getTexture());
}

void SonarImageVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void SonarImageVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

} // namespace rviz_sonar_image
