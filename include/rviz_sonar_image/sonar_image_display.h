#ifndef RVIZ_SONAR_IMAGE_SONAR_IMAGE_DISPLAY_H
#define RVIZ_SONAR_IMAGE_SONAR_IMAGE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <acoustic_msgs/RawSonarImage.h>
#endif

namespace rviz_sonar_image
{

class SonarImageVisual;
class ColorMap;

class SonarImageDisplay: public rviz::MessageFilterDisplay<acoustic_msgs::RawSonarImage>
{
Q_OBJECT
public:
  SonarImageDisplay();
  ~SonarImageDisplay();

protected:
  void onInitialize() override;
  void reset() override;

private:
  void processMessage(const acoustic_msgs::RawSonarImage::ConstPtr& msg) override;

  std::vector<std::shared_ptr<SonarImageVisual> > visuals_;

  std::list<std::vector<std::shared_ptr<SonarImageVisual> > > ribbon_visuals_;
  int ribbon_length_ = 250;
  int ribbon_beam_ = 0;

  std::shared_ptr<ColorMap> color_map_;
};

} // namespace rviz_sonar_image

#endif
