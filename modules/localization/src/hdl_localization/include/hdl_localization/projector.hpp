#pragma once

#include <cmath>

namespace hdl_localization {
namespace projector {

constexpr double RADIANS_PER_DEGREE = M_PI / 180.0;
constexpr double DEGREES_PER_RADIAN = 180.0 / M_PI;

constexpr double WGS84_A = 6378137.0;
constexpr double WGS84_E = 0.006694379983166804;
constexpr double WGS84_EP2 = WGS84_E / (1.0 - WGS84_E);
constexpr double SQRT_E = std::sqrt(1 - WGS84_E);
constexpr double UTM_K0 = 0.9996;
constexpr double UTM_E = (1 - SQRT_E) / (1 + SQRT_E);
constexpr double UTM_E2 = UTM_E * UTM_E;
constexpr double UTM_E3 = UTM_E2 * UTM_E;
constexpr double UTM_E4 = UTM_E3 * UTM_E;
constexpr double UTM_E5 = UTM_E4 * UTM_E;

constexpr double M1 = 1 - WGS84_E / 4 - 3 * WGS84_E * WGS84_E / 64 - 5 * WGS84_E * WGS84_E * WGS84_E / 256;
constexpr double M2 = 3 * WGS84_E / 8 + 3 * WGS84_E * WGS84_E / 32 + 45 * WGS84_E * WGS84_E * WGS84_E / 1024;
constexpr double M3 = 15 * WGS84_E * WGS84_E / 256 + 45 * WGS84_E * WGS84_E * WGS84_E / 1024;
constexpr double M4 = 35 * WGS84_E * WGS84_E * WGS84_E / 3072;

constexpr double P2 = (3.0 / 2.0 * UTM_E - 27.0 / 32.0 * UTM_E3 + 269.0 / 512.0 * UTM_E5);
constexpr double P3 = (21.0 / 16.0 * UTM_E2 - 55.0 / 32.0 * UTM_E4);
constexpr double P4 = (151.0 / 96.0 * UTM_E3 - 417.0 / 128.0 * UTM_E5);
constexpr double P5 = (1097.0 / 512.0 * UTM_E4);

struct PointLL {
  double lat = 0.0;
  double lon = 0.0;
};

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

inline double mod_angle(double value) {
  double a = std::fmod(value + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

inline int latlon_to_zone(double lon) {
  return static_cast<int>((lon + 180.0) / 6.0) + 1;
}

inline double zone_to_center_lon(int zone) {
  return (zone - 1) * 6.0 - 180.0 + 3.0;
}

inline Point2D origin_from_latlon(double lat, double lon, int zone, bool& northern) {
  const double lat_rad = lat * RADIANS_PER_DEGREE;
  const double lat_sin = std::sin(lat_rad);
  const double lat_cos = std::cos(lat_rad);
  const double lat_tan = lat_sin / lat_cos;
  const double lat_tan2 = lat_tan * lat_tan;
  const double lat_tan4 = lat_tan2 * lat_tan2;

  const double lon_rad = lon * RADIANS_PER_DEGREE;
  const double center_lon_rad = zone_to_center_lon(zone) * RADIANS_PER_DEGREE;

  const double n = WGS84_A / std::sqrt(1 - WGS84_E * lat_sin * lat_sin);
  const double c = WGS84_EP2 * lat_cos * lat_cos;

  const double a = lat_cos * mod_angle(lon_rad - center_lon_rad);
  const double a2 = a * a;
  const double a3 = a2 * a;
  const double a4 = a3 * a;
  const double a5 = a4 * a;
  const double a6 = a5 * a;
  const double m = WGS84_A * (M1 * lat_rad - M2 * std::sin(2 * lat_rad) + M3 * std::sin(4 * lat_rad) - M4 * std::sin(6 * lat_rad));

  const double easting =
      UTM_K0 * n * (a + a3 / 6.0 * (1 - lat_tan2 + c) + a5 / 120.0 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * WGS84_EP2)) + 500000.0;
  double northing =
      UTM_K0 * (m + n * lat_tan * (a2 / 2.0 + a4 / 24.0 * (5 - lat_tan2 + 9 * c + 4 * c * c) + a6 / 720.0 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * WGS84_EP2)));

  northern = lat >= 0.0;
  if (!northern) {
    northing += 10000000.0;
  }

  return {easting, northing};
}

inline PointLL to_latlon(double easting, double northing, int zone, bool northern) {
  const double x = easting - 500000.0;
  double y = northing;
  if (!northern) {
    y -= 10000000.0;
  }

  const double m = y / UTM_K0;
  const double mu = m / (WGS84_A * M1);
  const double p_rad = mu + P2 * std::sin(2 * mu) + P3 * std::sin(4 * mu) + P4 * std::sin(6 * mu) + P5 * std::sin(8 * mu);
  const double p_sin = std::sin(p_rad);
  const double p_sin2 = p_sin * p_sin;
  const double p_cos = std::cos(p_rad);
  const double p_tan = p_sin / p_cos;
  const double p_tan2 = p_tan * p_tan;
  const double p_tan4 = p_tan2 * p_tan2;
  const double ep_sin = 1 - WGS84_E * p_sin2;
  const double n = WGS84_A / std::sqrt(ep_sin);
  const double r = (1 - WGS84_E) / ep_sin;
  const double c = WGS84_EP2 * p_cos * p_cos;
  const double c2 = c * c;
  const double d = x / (n * UTM_K0);
  const double d2 = d * d;
  const double d3 = d2 * d;
  const double d4 = d3 * d;
  const double d5 = d4 * d;
  const double d6 = d5 * d;

  const double latitude =
      p_rad - (p_tan / r) * (d2 / 2.0 - d4 / 24.0 * (5 + 3 * p_tan2 + 10 * c - 4 * c2 - 9 * WGS84_EP2) +
                             d6 / 720.0 * (61 + 90 * p_tan2 + 298 * c + 45 * p_tan4 - 252 * WGS84_EP2 - 3 * c2));
  double longitude = (d - d3 / 6.0 * (1 + 2 * p_tan2 + c) + d5 / 120.0 * (5 - 2 * c + 28 * p_tan2 - 3 * c2 + 8 * WGS84_EP2 + 24 * p_tan4)) / p_cos;
  longitude = mod_angle(longitude + zone_to_center_lon(zone) * RADIANS_PER_DEGREE);

  return {latitude * DEGREES_PER_RADIAN, longitude * DEGREES_PER_RADIAN};
}

class Projector {
public:
  void set_origin_ll(double lat, double lon) {
    zone_ = latlon_to_zone(lon);
    const auto origin = origin_from_latlon(lat, lon, zone_, northern_);
    origin_x_ = origin.x;
    origin_y_ = origin.y;
    initialized_ = true;
  }

  bool initialized() const { return initialized_; }

  PointLL reverse(double x, double y) const {
    return to_latlon(x + origin_x_, y + origin_y_, zone_, northern_);
  }

private:
  bool initialized_ = false;
  int zone_ = 0;
  bool northern_ = true;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
};

}  // namespace projector
}  // namespace hdl_localization
