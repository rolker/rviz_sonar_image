#ifndef RVIZ_SONAR_IMAGE_SONAR_IMAGE_DISPLAY_H
#define RVIZ_SONAR_IMAGE_SONAR_IMAGE_DISPLAY_H

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <acoustic_msgs/RawSonarImage.h>
#endif

namespace rviz_sonar_image
{

class SonarImageCurtain;
class SonarImageFan;
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

  std::vector<std::shared_ptr<SonarImageFan> > fans_;

  std::list<std::vector<std::shared_ptr<SonarImageCurtain> > > curtains_;
  int curtain_length_ = 3;
  int curtain_beam_ = 0;

  std::shared_ptr<ColorMap> color_map_;
};

} // namespace rviz_sonar_image

#endif
