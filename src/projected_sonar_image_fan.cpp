#include <rviz_sonar_image/projected_sonar_image_fan.h>
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

ProjectedSonarImageFan::ProjectedSonarImageFan( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, std::shared_ptr<ColorMap> color_map )
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

ProjectedSonarImageFan::~ProjectedSonarImageFan()
{
  delete mesh_shape_;
  delete texture_;
  scene_manager_->destroySceneNode(frame_node_);
}

void ProjectedSonarImageFan::setMessage(const marine_acoustic_msgs::ProjectedSonarImage::ConstPtr& msg, uint32_t start_row, uint32_t end_row)
{
  mesh_shape_->clear();

  int max_steps = 128;
  int step_size = 1;
  start_row = std::min<uint32_t>(start_row, msg->ranges.size());
  end_row = std::min<uint32_t>(end_row, msg->ranges.size());
  if(!end_row>start_row)
    return;

  auto sample_count = end_row-start_row;
  auto steps_per_beam = sample_count;

  if (steps_per_beam > max_steps)
  {
    step_size = ceil(sample_count/float(max_steps));
    steps_per_beam /= step_size;
  }

  struct DirectionTexture
  {
    geometry_msgs::Vector3 direction;
    float texture_coordinate;
  };

  std::vector<DirectionTexture> directions;
  for(int i = 0; i < msg->image.beam_count; i+= 10)
  {
    DirectionTexture dt;
    dt.direction = msg->beam_directions[i];
    if(msg->image.beam_count > 1)
      dt.texture_coordinate = i/float(msg->image.beam_count-1);
    else
      dt.texture_coordinate = 0.5;
    directions.push_back(dt);
  }

  mesh_shape_->estimateVertexCount(directions.size()*(steps_per_beam+1));
  mesh_shape_->beginTriangles();

  std::vector<int> column_sizes;
  for(auto direction: directions)
  {
    column_sizes.push_back(0);
    auto i = start_row;
    while(i < end_row)
    {
      column_sizes.back()++;
      auto range = msg->ranges[i];
      mesh_shape_->addVertex(Ogre::Vector3(direction.direction.x*range, direction.direction.y*range, direction.direction.z*range));

      mesh_shape_->getManualObject()->textureCoord(direction.texture_coordinate, (i-start_row) /float(end_row-start_row-1));
      if(i != end_row-1 && i+step_size >= end_row)
        i = end_row-1;
      else
        i+=step_size;
    }
  }

  int c1 = 0;
  for(int i = 0; i < directions.size()-1; i++)
  {
    int c2 = c1 + column_sizes[i];
    for(int j = 0; j < column_sizes[i]-1; j++)
    {
      mesh_shape_->addTriangle(c1+j, c1+j+1, c2+j+1);
      mesh_shape_->addTriangle(c2+j+1, c2+j, c1+j);
    }
    c1 = c2;
  }
  mesh_shape_->endTriangles();

  sensor_msgs::Image::Ptr image(new sensor_msgs::Image);
  image->header.stamp = msg->header.stamp;
  image->width = msg->image.beam_count;
  image->height = sample_count;
  // TODO, support other than floats
  image->encoding = "rgba8";
  image->step = image->width*4;

  switch(msg->image.dtype)
  {
    case marine_acoustic_msgs::SonarImageData::DTYPE_UINT8:
    {
      const uint8_t* sonar_data = reinterpret_cast<const uint8_t*>(msg->image.data.data());
      for (uint32_t i = start_row*image->width; i < end_row*image->width; i++)
      {
        auto c = color_map_->lookup(sonar_data[i]);
        image->data.push_back(c.r*255);
        image->data.push_back(c.g*255);
        image->data.push_back(c.b*255);
        image->data.push_back(c.a*255);
      }
      break;
    }
    case marine_acoustic_msgs::SonarImageData::DTYPE_UINT16:
    {
      const uint16_t* sonar_data = reinterpret_cast<const uint16_t*>(msg->image.data.data());
      for (uint32_t i = start_row*image->width; i < end_row*image->width; i++)
      {
        auto c = color_map_->lookup(sonar_data[i]);
        image->data.push_back(c.r*255);
        image->data.push_back(c.g*255);
        image->data.push_back(c.b*255);
        image->data.push_back(c.a*255);
      }
      break;
    }
    case marine_acoustic_msgs::SonarImageData::DTYPE_UINT32:
    {
      const uint32_t* sonar_data = reinterpret_cast<const uint32_t*>(msg->image.data.data());
      for (uint32_t i = start_row*image->width; i < end_row*image->width; i++)
      {
        auto c = color_map_->lookup(sonar_data[i]);
        image->data.push_back(c.r*255);
        image->data.push_back(c.g*255);
        image->data.push_back(c.b*255);
        image->data.push_back(c.a*255);
      }
      break;
    }
    case marine_acoustic_msgs::SonarImageData::DTYPE_FLOAT32:
    {
      const float* sonar_data = reinterpret_cast<const float*>(msg->image.data.data());
      for (uint32_t i = start_row*image->width; i < end_row*image->width; i++)
      {
        auto c = color_map_->lookup(sonar_data[i]);
        image->data.push_back(c.r*255);
        image->data.push_back(c.g*255);
        image->data.push_back(c.b*255);
        image->data.push_back(c.a*255);
      }
      break;
    }
    default:
      ROS_WARN_STREAM("Unimplemented type: " << msg->image.dtype);
  }

  //std::cerr << image->width << " x " << image->height << " image " << image->data.size() << "bytes" << std::endl;

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

void ProjectedSonarImageFan::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void ProjectedSonarImageFan::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

} // namespace rviz_sonar_image
