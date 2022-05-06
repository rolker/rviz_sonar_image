#include <rviz_sonar_image/sonar_image_display.h>
#include <rviz_sonar_image/sonar_image_visual.h>
#include <rviz_sonar_image/sonar_image_curtain.h>
#include <rviz_sonar_image/color_map.h>

namespace rviz_sonar_image
{

SonarImageDisplay::SonarImageDisplay()
  :color_map_(std::make_shared<ColorMap>())
{

}

SonarImageDisplay::~SonarImageDisplay()
{

}

void SonarImageDisplay::onInitialize()
{
  MFDClass::onInitialize();

}

void SonarImageDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void SonarImageDisplay::processMessage(const acoustic_msgs::RawSonarImage::ConstPtr& msg)
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

  uint32_t sector_size = 4096;

  std::vector<std::shared_ptr<SonarImageVisual> > visuals_for_ribbon;

  if(curtains_.empty() ||  (!curtains_.back().empty() && curtains_.back().front()->full()))
  {
    curtains_.push_back(std::vector<std::shared_ptr<SonarImageCurtain> >());
    while (curtains_.size() > curtain_length_ && !curtains_.empty())
    {
      curtains_.pop_front();
    }
  }

  int i = 0;
  uint32_t start_row = 0;
  while (start_row < msg->samples_per_beam)
  {
    uint32_t end_row = std::min(start_row+sector_size, msg->samples_per_beam);
    if(i >= visuals_.size())
      visuals_.push_back(std::make_shared<SonarImageVisual>(context_->getSceneManager(), scene_node_, color_map_));
    visuals_[i]->setMessage(msg, start_row, end_row);
    visuals_[i]->setFramePosition( position );
    visuals_[i]->setFrameOrientation( orientation );

    if(curtain_beam_ >= 0 && curtain_length_ > 0)
    {
      if(i >= curtains_.back().size())
        curtains_.back().push_back(std::make_shared<SonarImageCurtain>(context_->getSceneManager(), scene_node_, color_map_));
      curtains_.back()[i]->addMessage(msg, start_row, end_row, curtain_beam_, position, orientation);
    }

    if(ribbon_beam_ >= 0 && ribbon_length_ > 0)
    {
      visuals_for_ribbon.push_back(std::make_shared<SonarImageVisual>(context_->getSceneManager(), scene_node_, color_map_));
      visuals_for_ribbon.back()->setMessage(msg, start_row, end_row, ribbon_beam_);
      visuals_for_ribbon.back()->setFramePosition( position );
      visuals_for_ribbon.back()->setFrameOrientation( orientation );
    }
    i++;
    start_row += sector_size-1;
  }
  if(!visuals_for_ribbon.empty())
    ribbon_visuals_.push_back(visuals_for_ribbon);
  if(ribbon_visuals_.size() > ribbon_length_ && !ribbon_visuals_.empty())
    ribbon_visuals_.pop_front();
  if(visuals_.size() >= i)
    visuals_.resize(i-1);

}

} // namespace rviz_sonar_image

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_sonar_image::SonarImageDisplay, rviz::Display)
