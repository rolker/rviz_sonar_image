#include <rviz_sonar_image/sonar_image_curtain.h>
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

SonarImageCurtain::SonarImageCurtain( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, std::shared_ptr<ColorMap> color_map )
  :color_map_(color_map)
{
  mesh_shape_ = new rviz::MeshShape(scene_manager, parent_node);
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

SonarImageCurtain::~SonarImageCurtain()
{
  delete mesh_shape_;
  delete texture_;
}

void SonarImageCurtain::addMessage(const acoustic_msgs::RawSonarImage::ConstPtr& msg, uint32_t start_row, uint32_t end_row, int beam_number, const Ogre::Vector3& position, const Ogre::Quaternion& orientation )
{
  Ogre::Matrix4 transform;
  transform.makeTransform(position, Ogre::Vector3(1, 1, 1), orientation);

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

  if(!image_)
  {
    image_ = sensor_msgs::Image::Ptr(new sensor_msgs::Image);
    image_->header.stamp = msg->header.stamp;
    image_->width = max_ping_count_;
    image_->height = sample_count;
    image_->encoding = "rgba8";
    image_->step = image_->width*4;
    image_->data.resize(image_->step*image_->height);
  }

  const float* sonar_data = reinterpret_cast<const float*>(msg->image.data.data());
  auto image_col = vertices_.size();

  for (uint32_t i = start_row; i < end_row; i++)
  {
    auto c = color_map_->lookup(sonar_data[i*msg->rx_angles.size()+beam_number]);
    auto image_row = i-start_row;
    auto image_cell = &image_->data.at((image_col*max_ping_count_+image_row)*4);
    image_cell[0] = c.r*255;
    image_cell[1] = c.g*255;
    image_cell[2] = c.b*255;
    image_cell[3] = c.a*255;
  }

  texture_->addMessage(image_);
  texture_->update();

  float sample_length = (step_size*msg->ping_info.sound_speed/msg->sample_rate)/2.0;

  auto y_angle = msg->rx_angles[beam_number];
  auto z_angle = msg->tx_angles[beam_number];

  float cosy = cos(y_angle);
  float cosz = cos(z_angle);
  float siny = sin(y_angle);
  float sinz = sin(z_angle);

  float startx = cosy*cosz*start_range;
  float starty = siny*start_range;
  float startz = sinz*start_range; 

  float dx = cosy*cosz*sample_length;
  float dy = siny*sample_length;
  float dz = sinz*sample_length;

  vertices_.resize(vertices_.size()+1);

  for(int i = 0; i < steps_per_beam; i++)
  {
    vertices_.back().push_back(transform.transformAffine(Ogre::Vector3(startx+i*dx, starty+i*dy, startz+i*dz)));
  }

  mesh_shape_->estimateVertexCount(vertices_.size()*vertices_.front().size());
  mesh_shape_->beginTriangles();

  for(int i = 0; i < vertices_.size(); i++)
  {
    int c1 = i*steps_per_beam;
    int c2 = c1 + steps_per_beam;
    float u = i/(max_ping_count_-1.0);
    for(int j = 0; j < steps_per_beam; j++)
    {
      mesh_shape_->addVertex(vertices_[i][j]);
      mesh_shape_->getManualObject()->textureCoord(u, j/float(steps_per_beam-1));
      if(i < vertices_.size()-1 && j < steps_per_beam - 1)
      {
        mesh_shape_->addTriangle(c1+j, c1+j+1, c2+j+1);
        mesh_shape_->addTriangle(c2+j+1, c2+j, c1+j);
      }
    }
  }

  mesh_shape_->endTriangles();

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


bool SonarImageCurtain::full() const
{
  return vertices_.size() >= max_ping_count_;
}

} // namespace rviz_sonar_image
