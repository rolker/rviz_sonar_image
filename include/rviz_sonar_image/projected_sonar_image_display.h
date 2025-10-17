#ifndef RVIZ_SONAR_IMAGE_PROJECTED_SONAR_IMAGE_DISPLAY_H
#define RVIZ_SONAR_IMAGE_PROJECTED_SONAR_IMAGE_DISPLAY_H

//#ifndef Q_MOC_RUN
#include "rviz_common/message_filter_display.hpp"
#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"
//#endif
#include "rviz_sonar_image/projected_sonar_image_fan.h"
#include "rviz_sonar_image/projected_sonar_image_curtain.h"
#include "rviz_sonar_image/color_map.h"

namespace rviz_sonar_image
{

// class ProjectedSonarImageCurtain;
// class ProjectedSonarImageFan;
// class ColorMap;

class ProjectedSonarImageDisplay: public rviz_common::MessageFilterDisplay<marine_acoustic_msgs::msg::ProjectedSonarImage>
{
Q_OBJECT
public:
  ProjectedSonarImageDisplay();
  ~ProjectedSonarImageDisplay();

protected:
  void onInitialize() override;
  void reset() override;

private:
  void processMessage(const marine_acoustic_msgs::msg::ProjectedSonarImage::ConstSharedPtr msg) override;

  std::vector<std::shared_ptr<ProjectedSonarImageFan> > fans_;

  std::list<std::vector<std::shared_ptr<ProjectedSonarImageCurtain> > > curtains_;
  int curtain_length_ = 3;
  int curtain_beam_ = 0;

  std::shared_ptr<ColorMap> color_map_;
};

} // namespace rviz_sonar_image

#endif
