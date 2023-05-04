#include <rviz_sonar_image/projected_sonar_image_display.h>
#include <rviz_sonar_image/projected_sonar_image_fan.h>
#include <rviz_sonar_image/projected_sonar_image_curtain.h>
#include <rviz_sonar_image/color_map.h>

namespace rviz_sonar_image
{

ProjectedSonarImageDisplay::ProjectedSonarImageDisplay()
  :color_map_(std::make_shared<ColorMap>())
{

}

ProjectedSonarImageDisplay::~ProjectedSonarImageDisplay()
{

}

void ProjectedSonarImageDisplay::onInitialize()
{
  MFDClass::onInitialize();

}

void ProjectedSonarImageDisplay::reset()
{
  MFDClass::reset();
  fans_.clear();
}

void ProjectedSonarImageDisplay::processMessage(const marine_acoustic_msgs::ProjectedSonarImage::ConstPtr& msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  switch(msg->image.dtype)
  {
  case marine_acoustic_msgs::SonarImageData::DTYPE_UINT8:
    color_map_->setRange(0,255);
    break;
  case marine_acoustic_msgs::SonarImageData::DTYPE_UINT16:
    color_map_->setRange(0, 1000);
    break;
  case marine_acoustic_msgs::SonarImageData::DTYPE_UINT32:
    color_map_->setRange(0, 4000000000);
    break;
  // QUESTION(lindzey): Should this at least generate a warning? In other parts
  //    of the code, using an unsupported DTYPE is an error.
  default:
    color_map_->setRange(-80, -20);
  }

  uint32_t sector_size = 4096;

  if(curtains_.empty() ||  (!curtains_.back().empty() && curtains_.back().front()->full()))
  {
    curtains_.push_back(std::vector<std::shared_ptr<ProjectedSonarImageCurtain> >());
    while (curtains_.size() > curtain_length_ && !curtains_.empty())
    {
      curtains_.pop_front();
    }
  }

  int i = 0;
  uint32_t start_row = 0;
  while (start_row < msg->ranges.size())
  {
    uint32_t end_row = std::min<uint32_t>(start_row+sector_size, msg->ranges.size());
    if(i >= fans_.size())
      fans_.push_back(std::make_shared<ProjectedSonarImageFan>(context_->getSceneManager(), scene_node_, color_map_));
    fans_[i]->setMessage(msg, start_row, end_row);
    fans_[i]->setFramePosition( position );
    fans_[i]->setFrameOrientation( orientation );

    if(msg->image.beam_count > 0)
      curtain_beam_ = msg->image.beam_count/2;

    if(curtain_beam_ >= 0 && curtain_length_ > 0)
    {
      if(i >= curtains_.back().size())
        curtains_.back().push_back(std::make_shared<ProjectedSonarImageCurtain>(context_->getSceneManager(), scene_node_, color_map_));
      curtains_.back()[i]->addMessage(msg, start_row, end_row, curtain_beam_, position, orientation);
    }
    i++;
    start_row += sector_size-1;
  }
}

} // namespace rviz_sonar_image

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_sonar_image::ProjectedSonarImageDisplay, rviz::Display)
