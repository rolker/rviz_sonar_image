#ifndef RVIZ_SONAR_IMAGE_COLOR_MAP_H
#define RVIZ_SONAR_IMAGE_COLOR_MAP_H

#include <OGRE/OgreColourValue.h>

#include <marine_colormap/palette.hpp>
#include <marine_colormap/transfer.hpp>

namespace rviz_sonar_image
{

// Thin adapter over the shared marine_colormap library: maps a sonar value (dB)
// through the canonical "thermal" palette and a transfer function to an Ogre
// colour. Preserves the prior behaviour -- white below the floor, opaque, linear
// interpolation, default range -70..0 dB -- while sourcing the palette and
// transfer from the shared lib. Colours now track marine_colormap's canonical
// thermal, which drops the stop this map previously carried twice.
class ColorMap
{
public:
  ColorMap();
  void setRange(float min, float max);
  void setAlphaRange(float min, float max);
  Ogre::ColourValue lookup(float value);

private:
  const marine_colormap::Palette * palette_;
  marine_colormap::TransferParams params_;
};

} // namespace rviz_sonar_image

#endif
