#include "rviz_sonar_image/projected_sonar_image_display.h"
#include "rviz_sonar_image/projected_sonar_image_fan.h"
#include "rviz_sonar_image/projected_sonar_image_curtain.h"
#include "rviz_sonar_image/color_map.h"
#include "rviz_common/logging.hpp"

namespace rviz_sonar_image
{

ProjectedSonarImageDisplay::ProjectedSonarImageDisplay()
  :color_map_(std::make_shared<ColorMap>())
{
  alpha_property_ =
      new rviz_common::properties::FloatProperty("Alpha", 1.0f, "The amount of transparency to apply to the curtain.", this,
                        SLOT(updateAlpha()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  colormap_minimum_property_ = new rviz_common::properties::FloatProperty("Colormap minimum value", -80.0f, "The value representing the bottom of the color map", this, SLOT(updateColormapRange()));
  colormap_minimum_property_->setMin(-200.0f);
  colormap_minimum_property_->setMax(4096.0f);

  colormap_maximum_property_ = new rviz_common::properties::FloatProperty("Colormap maximum value", -20.0f, "The value representing the top of the color map", this, SLOT(updateColormapRange()));
  colormap_maximum_property_->setMin(-200.0f);
  colormap_maximum_property_->setMax(4096.0f);
  updateColormapRange();

  minimum_data_value_property_ = new rviz_common::properties::FloatProperty("Minimum data value", 0.0f, "The minimum data value received", this);
  minimum_data_value_property_->setReadOnly(true);
  minimum_data_value_property_->setShouldBeSaved(false);

  maximum_data_value_property_ = new rviz_common::properties::FloatProperty("Maximum data value", 0.0f, "The maximum data value received", this);
  maximum_data_value_property_->setReadOnly(true);
  maximum_data_value_property_->setShouldBeSaved(false);


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

void ProjectedSonarImageDisplay::updateAlpha()
{
  for(auto cv: curtains_)
    for(auto c: cv)
      c->updateAlpha(alpha_property_->getFloat());
}

void ProjectedSonarImageDisplay::updateColormapRange()
{
  color_map_->setRange(colormap_minimum_property_->getFloat(), colormap_maximum_property_->getFloat());
}


void ProjectedSonarImageDisplay::processMessage(const marine_acoustic_msgs::msg::ProjectedSonarImage::ConstSharedPtr msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    RVIZ_COMMON_LOG_DEBUG_STREAM( "Error transforming from frame " << msg->header.frame_id << " to frame " << qPrintable( fixed_frame_ ));
    return;
  }

  // switch(msg->image.dtype)
  // {
  // case marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8:
  //   color_map_->setRange(0,255);
  //   break;
  // case marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT16:
  //   color_map_->setRange(0, 1000);
  //   break;
  // case marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT32:
  //   color_map_->setRange(0, 4000000000);
  //   break;
  // // QUESTION(lindzey): Should this at least generate a warning? In other parts
  // //    of the code, using an unsupported DTYPE is an error.
  // default:
  //   color_map_->setRange(-80, -20);
  // }

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
      auto range = curtains_.back()[i]->getDataValueRange();
      minimum_data_value_ = std::min(minimum_data_value_, range.first);
      maximum_data_value_ = std::max(maximum_data_value_, range.second);

      minimum_data_value_property_->setFloat(minimum_data_value_);
      maximum_data_value_property_->setFloat(maximum_data_value_);

      colormap_minimum_property_->setMin(minimum_data_value_);
      colormap_maximum_property_->setMin(minimum_data_value_);
      colormap_minimum_property_->setMax(maximum_data_value_);
      colormap_maximum_property_->setMax(maximum_data_value_);

    }
    i++;
    start_row += sector_size-1;
  }
}

} // namespace rviz_sonar_image

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_sonar_image::ProjectedSonarImageDisplay, rviz_common::Display)
