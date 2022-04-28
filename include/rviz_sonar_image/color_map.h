#ifndef RVIZ_SONAR_IMAGE_COLOR_MAP_H
#define RVIZ_SONAR_IMAGE_COLOR_MAP_H

#include <OGRE/OgreColourValue.h>

namespace rviz_sonar_image
{

class ColorMap
{
public:
  ColorMap();
  void setRange(float min, float max);
  void setAlphaRange(float min, float max);
  Ogre::ColourValue lookup(float value);
private:
  float min_ = -70.0;
  float max_ = 0.0;
  float min_alpha_ = 0.0;
  float max_alpha_ = 0.8;
  std::vector<Ogre::ColourValue> map_;

};

} // namespace rviz_sonar_image

#endif
