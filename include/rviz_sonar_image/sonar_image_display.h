#ifndef RVIZ_SONAR_IMAGE_SONAR_IMAGE_DISPLAY_H
#define RVIZ_SONAR_IMAGE_SONAR_IMAGE_DISPLAY_H

#include "rviz_common/message_filter_display.hpp"
#include "marine_acoustic_msgs/msg/raw_sonar_image.hpp"
#include "rviz_common/properties/float_property.hpp"

#include "rviz_sonar_image/sonar_image_fan.h"
#include "rviz_sonar_image/sonar_image_curtain.h"
#include "rviz_sonar_image/color_map.h"

namespace rviz_sonar_image
{


class SonarImageDisplay: public rviz_common::MessageFilterDisplay<marine_acoustic_msgs::msg::RawSonarImage>
{
Q_OBJECT
public:
  SonarImageDisplay();
  ~SonarImageDisplay();

protected:
  void onInitialize() override;
  void reset() override;

private slots:

  void updateAlpha();
  void updateColormapRange();

private:
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::FloatProperty* colormap_minimum_property_;
  rviz_common::properties::FloatProperty* colormap_maximum_property_;

  rviz_common::properties::FloatProperty* minimum_data_value_property_;
  rviz_common::properties::FloatProperty* maximum_data_value_property_;

  void processMessage(marine_acoustic_msgs::msg::RawSonarImage::ConstSharedPtr msg) override;

  std::vector<std::shared_ptr<SonarImageFan> > fans_;

  std::list<std::vector<std::shared_ptr<SonarImageCurtain> > > curtains_;
  int curtain_length_ = 3;
  int curtain_beam_ = 0;

  std::shared_ptr<ColorMap> color_map_;

  float minimum_data_value_ = std::numeric_limits<float>::max();
  float maximum_data_value_ = std::numeric_limits<float>::lowest();
};

} // namespace rviz_sonar_image

#endif
