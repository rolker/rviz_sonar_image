#ifndef RVIZ_SONAR_IMAGE_PROJECTED_SONAR_IMAGE_DISPLAY_H
#define RVIZ_SONAR_IMAGE_PROJECTED_SONAR_IMAGE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <marine_acoustic_msgs/ProjectedSonarImage.h>
#endif

namespace rviz_sonar_image
{

class ProjectedSonarImageCurtain;
class ProjectedSonarImageFan;
class ColorMap;

class ProjectedSonarImageDisplay: public rviz::MessageFilterDisplay<marine_acoustic_msgs::ProjectedSonarImage>
{
Q_OBJECT
public:
  ProjectedSonarImageDisplay();
  ~ProjectedSonarImageDisplay();

protected:
  void onInitialize() override;
  void reset() override;

private:
  void processMessage(const marine_acoustic_msgs::ProjectedSonarImage::ConstPtr& msg) override;

  std::vector<std::shared_ptr<ProjectedSonarImageFan> > fans_;

  std::list<std::vector<std::shared_ptr<ProjectedSonarImageCurtain> > > curtains_;
  int curtain_length_ = 3;
  int curtain_beam_ = 0;

  std::shared_ptr<ColorMap> color_map_;
};

} // namespace rviz_sonar_image

#endif
