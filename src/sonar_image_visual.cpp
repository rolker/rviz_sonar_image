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

  double sample_length = (msg->ping_info.sound_speed/msg->sample_rate)/2.0;
  auto sample_count = end_row-start_row;
  auto steps_per_beam = sample_count;

  if (steps_per_beam > max_steps)
  {
    step_size = ceil(sample_count/float(max_steps));
    steps_per_beam /= step_size;
  }

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
    for(int i = 0; i < msg->image.beam_count; i++)
    {
      AnglesTexture at;
      at.y_angle = msg->rx_angles[i];
      at.z_angle = msg->tx_angles[i];
      if(msg->image.beam_count > 1)
        at.texture_coordinate = i/float(msg->image.beam_count-1);
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

  mesh_shape_->estimateVertexCount(angles.size()*(steps_per_beam+1));
  mesh_shape_->beginTriangles();

  std::vector<int> column_sizes;
  for(auto angle: angles)
  {
    float cosy = cos(angle.y_angle);
    float cosz = cos(angle.z_angle);
    float siny = sin(angle.y_angle);
    float sinz = sin(angle.z_angle);

    float dx = cosy*cosz*sample_length;
    float dy = siny*sample_length;
    float dz = sinz*sample_length;

    column_sizes.push_back(0);
    auto i = start_row;
    while(i < end_row)
    {
      column_sizes.back()++;
      mesh_shape_->addVertex(Ogre::Vector3((msg->sample0+i)*dx, (msg->sample0+i)*dy, (msg->sample0+i)*dz));

      mesh_shape_->getManualObject()->textureCoord(angle.texture_coordinate, (i-start_row) /float(end_row-start_row-1));
      if(i != end_row-1 && i+step_size >= end_row)
        i = end_row-1;
      else
        i+=step_size;
    }
  }

  int c1 = 0;
  for(int i = 0; i < angles.size()-1; i++)
  {
    int c2 = c1 + column_sizes[i];
    for(int j = 0; j < column_sizes[i]-1; j++)
    {
      mesh_shape_->addTriangle(c1+j, c1+j+1, c2+j+1);
      mesh_shape_->addTriangle(c2+j+1, c2+j, c1+j);
    }
    c1 = c2;
  }
  c1 += column_sizes.back();
  mesh_shape_->addVertex(Ogre::Vector3(0,0,0));
  mesh_shape_->getManualObject()->textureCoord(0,0);
  mesh_shape_->addVertex(Ogre::Vector3(10,0,0));
  mesh_shape_->getManualObject()->textureCoord(1,0);
  mesh_shape_->addVertex(Ogre::Vector3(10,10,0));
  mesh_shape_->getManualObject()->textureCoord(1,1);
  mesh_shape_->addVertex(Ogre::Vector3(0,10,0));
  mesh_shape_->getManualObject()->textureCoord(0,1);
  mesh_shape_->addTriangle(c1+0, c1+1, c1+2);
  mesh_shape_->addTriangle(c1+2, c1+3, c1+0);


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
    case acoustic_msgs::SonarImageData::DTYPE_UINT16:
    {
      //image->encoding = "mono16";
      //image->step = image->width*2;
      // for (uint32_t i = start_row*image->width; i < end_row*image->width; i++)
      // {
      //   image->data.push_back(msg->image.data[2*i]);
      //   image->data.push_back(msg->image.data[2*i+1]);
      // }
      for(int i = 0; i < image->width; i++)
      {
        float g = i/float(image->width);
        for(int j = 0; j < image->height; j++)
        {
          float r = j/float(image->height);
          image->data.push_back(r*255);
          image->data.push_back(g*255);
          image->data.push_back(128);
          image->data.push_back(255);
        }
      }
      break;
    }
    case acoustic_msgs::SonarImageData::DTYPE_FLOAT32:
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

void SonarImageVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void SonarImageVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

} // namespace rviz_sonar_image
