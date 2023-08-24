#include <rviz_sonar_image/sonar_image_display.h>
#include <rviz_sonar_image/sonar_image_fan.h>
#include <rviz_sonar_image/sonar_image_curtain.h>
#include <rviz_sonar_image/color_map.h>

namespace rviz_sonar_image
{

SonarImageDisplay::SonarImageDisplay()
  :color_map_(std::make_shared<ColorMap>())
{

  alpha_property_ =
      new rviz::FloatProperty("Alpha", 1.0f, "The amount of transparency to apply to the curtain.", this,
                        SLOT(updateAlpha()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  colormap_minimum_property_ = new rviz::FloatProperty("Colormap minimum value", -80.0f, "The value representing the bottom of the color map", this, SLOT(updateColormapRange()));
  colormap_minimum_property_->setMin(-200.0f);
  colormap_minimum_property_->setMax(4096.0f);

  colormap_maximum_property_ = new rviz::FloatProperty("Colormap maximum value", -20.0f, "The value representing the top of the color map", this, SLOT(updateColormapRange()));
  colormap_maximum_property_->setMin(-200.0f);
  colormap_maximum_property_->setMax(4096.0f);
  updateColormapRange();
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
  fans_.clear();
}

void SonarImageDisplay::updateAlpha()
{
  for(auto cv: curtains_)
    for(auto c: cv)
      c->updateAlpha(alpha_property_->getFloat());
}

void SonarImageDisplay::updateColormapRange()
{
  color_map_->setRange(colormap_minimum_property_->getFloat(), colormap_maximum_property_->getFloat());
}

void SonarImageDisplay::processMessage(const marine_acoustic_msgs::RawSonarImage::ConstPtr& msg)
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

  // if(msg->image.dtype == marine_acoustic_msgs::SonarImageData::DTYPE_UINT16)
  // {
  //   color_map_->setRange(0, 1000);
  // }
  // else
  //   color_map_->setRange(-80, -20);

  uint32_t sector_size = 4096;

  if(curtain_beam_ >= 0 && curtains_.empty()) // first time?
    if(msg->image.beam_count > 1) // only default to showing curtain for single beam
      curtain_beam_ = -1;

  if(curtain_beam_ >= 0 && (curtains_.empty() ||  (!curtains_.back().empty() && curtains_.back().front()->full())))
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
    if(i >= fans_.size())
      fans_.push_back(std::make_shared<SonarImageFan>(context_->getSceneManager(), scene_node_, color_map_));
    fans_[i]->setMessage(msg, start_row, end_row);
    fans_[i]->setFramePosition( position );
    fans_[i]->setFrameOrientation( orientation );

    // if(msg->image.beam_count > 0)
    //   curtain_beam_ = msg->image.beam_count/2;

    if(curtain_beam_ >= 0 && curtain_length_ > 0)
    {
      if(i >= curtains_.back().size())
      {
        curtains_.back().push_back(std::make_shared<SonarImageCurtain>(context_->getSceneManager(), scene_node_, color_map_));
        updateAlpha();
      }
      curtains_.back()[i]->addMessage(msg, start_row, end_row, curtain_beam_, position, orientation);
    }
    i++;
    start_row += sector_size-1;
  }
}

} // namespace rviz_sonar_image

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_sonar_image::SonarImageDisplay, rviz::Display)
