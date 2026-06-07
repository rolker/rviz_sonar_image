#include <rviz_sonar_image/color_map.h>

#include <marine_colormap/colormap.hpp>

namespace rviz_sonar_image
{

ColorMap::ColorMap()
: palette_(marine_colormap::find_palette("thermal"))
{
  params_.min = -70.0f;
  params_.max = 0.0f;
  // Preserve the prior "white below the floor" sentinel.
  params_.has_below_color = true;
  params_.below_color = marine_colormap::Rgba{1.0f, 1.0f, 1.0f, 1.0f};
}

void ColorMap::setRange(float min, float max)
{
  params_.min = min;
  params_.max = max;
}

void ColorMap::setAlphaRange(float min, float max)
{
  // Previously dormant (alpha was hard-coded to 1.0). Wire it to the shared
  // transfer's alpha ramp so a caller that sets it gets a value-dependent alpha;
  // unset, alpha stays opaque as before.
  params_.alpha_ramp = true;
  params_.alpha_min = min;
  params_.alpha_max = max;
}

Ogre::ColourValue ColorMap::lookup(float value)
{
  // palette_ is set from the guaranteed built-in "thermal" and never reassigned,
  // so this is defensive: never crash the render loop if a lib change ever made
  // the lookup fail -- fall back to opaque black.
  if (palette_ == nullptr) {
    return Ogre::ColourValue(0.0f, 0.0f, 0.0f, 1.0f);
  }
  const marine_colormap::Rgba c = marine_colormap::lookup(value, *palette_, params_);
  return Ogre::ColourValue(c.r, c.g, c.b, c.a);
}

} // namespace rviz_sonar_image
