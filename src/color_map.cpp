#include <rviz_sonar_image/color_map.h>

namespace rviz_sonar_image
{

ColorMap::ColorMap()
{
  map_.push_back(Ogre::ColourValue(0.3, 0.3, 0.3));
  map_.push_back(Ogre::ColourValue(0.02, 0.4, 0.95));
  map_.push_back(Ogre::ColourValue(0.13, 0.09, 0.71));
  map_.push_back(Ogre::ColourValue(0.15, 0.65, 0.54));
  map_.push_back(Ogre::ColourValue(0.07, 0.61, 0.41));
  map_.push_back(Ogre::ColourValue(0.63, 0.82, 0.24));
  map_.push_back(Ogre::ColourValue(0.99, 0.70, 0.18));
  map_.push_back(Ogre::ColourValue(0.98, 0.37, 0.60));
  map_.push_back(Ogre::ColourValue(0.99, 0.19, 0.38));
  map_.push_back(Ogre::ColourValue(0.99, 0.19, 0.38));
  map_.push_back(Ogre::ColourValue(0.86, 0.16, 0.20));
  map_.push_back(Ogre::ColourValue(0.65, 0.20, 0.20));
  map_.push_back(Ogre::ColourValue(0.60, 0.04, 0.06));
}

void ColorMap::setRange(float min, float max)
{
  min_ = min;
  max_ = max;
}

void ColorMap::setAlphaRange(float min, float max)
{
  min_alpha_ = min;
  max_alpha_ = max;
}


Ogre::ColourValue ColorMap::lookup(float value)
{
  if (value <= min_)
    return Ogre::ColourValue(1.0, 1.0, 1.0, 1.0);
    //return map_.front();
  if (value >= max_)
    return map_.back();

  float p = (map_.size()-1)*(value - min_)/(max_-min_);
  float alpha = 1.0; //min_alpha_ + p *(max_alpha_-min_alpha_);
  int pi = floor(p);
  float p1 = p-pi;
  float p0 = 1.0-p1;
  return Ogre::ColourValue(map_[pi].r*p0+map_[pi+1].r*p1, map_[pi].g*p0+map_[pi+1].g*p1, map_[pi].b*p0+map_[pi+1].b*p1, alpha);
}

} // namespace rviz_sonar_image
