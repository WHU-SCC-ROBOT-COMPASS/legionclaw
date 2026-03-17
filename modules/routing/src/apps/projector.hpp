#pragma once
#include <cmath>
#include <iostream>
namespace legionclaw
{
    namespace routing
    {
        namespace myprojector
        {
            const static double RADIANS_PER_DEGREE = M_PI / 180.0;
            const static double DEGREES_PER_RADIAN = 180.0 / M_PI;

            // WGS84 and UTM Parameters
            const static double WGS84_A = 6378137.0;            // major axis
            const static double WGS84_B = 6356752.31424518;     // minor axis
            const static double WGS84_E = 0.006694379983166804; // first eccentricity^2
            const static double WGS84_E2 = WGS84_E * WGS84_E;
            const static double WGS84_E3 = WGS84_E2 * WGS84_E;
            const static double WGS84_EP2 = WGS84_E / (1.0 - WGS84_E); // second eccentricity ^2
            const static double SQRT_E = sqrt(1 - WGS84_E);
            const static double UTM_K0 = 0.9996;
            const static double UTM_E = (1 - SQRT_E) / (1 + SQRT_E);
            const static double UTM_E2 = UTM_E * UTM_E;
            const static double UTM_E3 = UTM_E2 * UTM_E;
            const static double UTM_E4 = UTM_E3 * UTM_E;
            const static double UTM_E5 = UTM_E4 * UTM_E;

            const static double M1 = (1 - WGS84_E / 4 - 3 * WGS84_E2 / 64 - 5 * WGS84_E3 / 256);
            const static double M2 = (3 * WGS84_E / 8 + 3 * WGS84_E2 / 32 + 45 * WGS84_E3 / 1024);
            const static double M3 = (15 * WGS84_E2 / 256 + 45 * WGS84_E3 / 1024);
            const static double M4 = (35 * WGS84_E3 / 3072);

            const static double P2 = (3. / 2 * UTM_E - 27. / 32 * UTM_E3 + 269. / 512 * UTM_E5);
            const static double P3 = (21. / 16 * UTM_E2 - 55. / 32 * UTM_E4);
            const static double P4 = (151. / 96 * UTM_E3 - 417. / 128 * UTM_E5);
            const static double P5 = (1097. / 512 * UTM_E4);

            // UTM Parameters

            inline double
            ModAngleInRadians(double value)
            {
                double a = std::fmod(value + M_PI, 2.0 * M_PI);
                if (a < 0.0)
                {
                    a += (2.0 * M_PI);
                }
                return a - M_PI;
            }

            inline bool InBound(double x, double min, double max)
            {
                return (min <= x) && (x <= max);
            }

            class Pointll
            {
            public:
                double lat;
                double lon;
                Pointll() = default;
                ~Pointll() = default;
                Pointll(double input_lat, double input_lon) : lat(input_lat),
                                                              lon(input_lon)
                {
                }
                Pointll(Pointll &p) : lat(p.lat),
                                      lon(p.lon)
                {
                }
                Pointll(Pointll &&p) : lat(p.lat),
                                       lon(p.lon)
                {
                }
                Pointll& operator=(const Pointll& p)
                {
                    lat=p.lat;
                    lon=p.lon;
                    return *this;
                }
            };

            class Point2d
            {
            public:
                double x;
                double y;
                Point2d() = default;
                ~Point2d() = default;
                Point2d(double input_x, double input_y) : x(input_x),
                                                          y(input_y)
                {
                }
                Point2d(Point2d &p) : x(p.x),
                                      y(p.y)
                {
                }
                Point2d(Point2d &&p) : x(p.x),
                                       y(p.y)
                {
                }
                Point2d& operator=(const Point2d& p)
                {
                    x=p.x;
                    y=p.y;
                    return *this;
                }
            };

            class Point3d
            {
            public:
                double x;
                double y;
                double z;
                Point3d() = default;
                ~Point3d() = default;
                Point3d(double input_x, double input_y, double input_z) : x(input_x),
                                                                          y(input_y),
                                                                          z(input_z)
                {
                }
                Point3d(Point3d &p) : x(p.x),
                                      y(p.y),
                                      z(p.z)
                {
                }
                Point3d(Point3d &&p) : x(p.x),
                                       y(p.y),
                                       z(p.z)
                {
                }
                Point3d& operator=(const Point3d& p)
                {
                    x=p.x;
                    y=p.y;
                    z=p.z;
                    return *this;
                }
            };

            inline int LatLon2Zone(double lat, double lon)
            {
                // if (56 <= lat < 64 && 3 <= lon < 12)
                // {
                //     return 32;
                // }
                // if (72 <= lat < 84 && lon >= 0)
                // {
                //     if (lon < 9)
                //     {
                //         return 31;
                //     }
                //     else if (lon < 21)
                //     {
                //         return 33;
                //     }
                //     else if (lon < 33)
                //     {
                //         return 35;
                //     }
                //     else if (lon < 42)
                //     {
                //         return 37;
                //     }
                // }
                return int((lon + 180) / 6) + 1;
            }

            inline double Zone2CenterLon(int zone)
            {
                return (zone - 1) * 6 - 180 + 3;
            }

            inline bool CheckValidZone(int zone)
            {
                if ((1 <= zone) && (zone <= 60))
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            inline Point2d OriginFromLatLon(double lat, double lon, int zone, bool &northern)
            {
                if (!InBound(lat, -80.0, 84.0))
                {
                    std::cerr << "latitude out of range (must be between 80 deg S and 84 deg N" << std::endl;
                    return Point2d(0, 0);
                }
                if (!InBound(lon, -180.0, 180.0))
                {
                    std::cerr << "longitude out of range (must be between 180 deg W and 180 deg E)" << std::endl;
                    return Point2d(0, 0);
                }
                if (!CheckValidZone(zone))
                {
                    std::cerr << "Zone number out of range (must be between 1 and 60)" << std::endl;
                    return Point2d(0, 0);
                }
                double lat_rad = lat * RADIANS_PER_DEGREE;
                double lat_sin = sin(lat_rad);
                double lat_cos = cos(lat_rad);

                double lat_tan = lat_sin / lat_cos;
                double lat_tan2 = lat_tan * lat_tan;
                double lat_tan4 = lat_tan2 * lat_tan2;

                double lon_rad = lon * RADIANS_PER_DEGREE;
                double center_lon = Zone2CenterLon(zone);
                double center_lon_rad = center_lon * RADIANS_PER_DEGREE;

                double n = WGS84_A / sqrt(1 - WGS84_E * lat_sin * lat_sin);
                double c = WGS84_EP2 * lat_cos * lat_cos;

                double a = lat_cos * ModAngleInRadians(lon_rad - center_lon_rad);
                double a2 = a * a;
                double a3 = a2 * a;
                double a4 = a3 * a;
                double a5 = a4 * a;
                double a6 = a5 * a;
                double m = WGS84_A * (M1 * lat_rad - M2 * sin(2 * lat_rad) + M3 * sin(4 * lat_rad) - M4 * sin(6 * lat_rad));
                double easting = UTM_K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) + a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * WGS84_EP2)) + 500000;
                double northing = UTM_K0 * (m + n * lat_tan * (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) + a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * WGS84_EP2)));
                northern = true;
                if (lat < 0)
                {
                    northern = false;
                    northing += 10000000.0;
                }
                return Point2d(easting, northing);
            }

            inline Point2d FromLatLon(double lat, double lon, int zone, bool northern)
            {
                if (!InBound(lat, -80.0, 84.0))
                {
                    std::cerr << "latitude out of range (must be between 80 deg S and 84 deg N" << std::endl;
                    return Point2d(0, 0);
                }
                if (!InBound(lon, -180.0, 180.0))
                {
                    std::cerr << "longitude out of range (must be between 180 deg W and 180 deg E)" << std::endl;
                    return Point2d(0, 0);
                }
                if (!CheckValidZone(zone))
                {
                    std::cerr << "Zone number out of range (must be between 1 and 60)" << std::endl;
                    return Point2d(0, 0);
                }
                double lat_rad = lat * RADIANS_PER_DEGREE;
                double lat_sin = sin(lat_rad);
                double lat_cos = cos(lat_rad);

                double lat_tan = lat_sin / lat_cos;
                double lat_tan2 = lat_tan * lat_tan;
                double lat_tan4 = lat_tan2 * lat_tan2;

                double lon_rad = lon * RADIANS_PER_DEGREE;
                double center_lon = Zone2CenterLon(zone);
                double center_lon_rad = center_lon * RADIANS_PER_DEGREE;

                double n = WGS84_A / sqrt(1 - WGS84_E * lat_sin * lat_sin);
                double c = WGS84_EP2 * lat_cos * lat_cos;

                double a = lat_cos * ModAngleInRadians(lon_rad - center_lon_rad);
                double a2 = a * a;
                double a3 = a2 * a;
                double a4 = a3 * a;
                double a5 = a4 * a;
                double a6 = a5 * a;
                double m = WGS84_A * (M1 * lat_rad - M2 * sin(2 * lat_rad) + M3 * sin(4 * lat_rad) - M4 * sin(6 * lat_rad));
                double easting = UTM_K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) + a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * WGS84_EP2)) + 500000;
                double northing = UTM_K0 * (m + n * lat_tan * (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) + a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * WGS84_EP2)));
                if (!northern)
                {
                    northing += 10000000.0;
                }
                return Point2d(easting, northing);
            }

            inline Pointll ToLatLon(double easting, double northing, int zone, bool northern)
            {
                if (!InBound(easting, 100000, 1000000))
                {
                    std::cerr << "easting out of range (must be between 100,000 m and 1000,000 m)" << std::endl;
                    return Pointll(0, 0);
                }
                if (!InBound(northing, -10000000, 10000000))
                {
                    std::cerr << "northing out of range (must be between -10,000,000 m and 10,000,000 m)" << std::endl;
                    return Pointll(0, 0);
                }
                if (!CheckValidZone(zone))
                {
                    std::cerr << "Zone number out of range (must be between 1 and 60)" << std::endl;
                    return Pointll(0, 0);
                }
                double x = easting - 500000;
                double y = northing;
                if (!northern)
                {
                    y -= 10000000;
                }

                double m = y / UTM_K0;
                double mu = m / (WGS84_A * M1);
                double p_rad = (mu + P2 * sin(2 * mu) + P3 * sin(4 * mu) + P4 * sin(6 * mu) + P5 * sin(8 * mu));
                double p_sin = sin(p_rad);
                double p_sin2 = p_sin * p_sin;
                double p_cos = cos(p_rad);
                double p_tan = p_sin / p_cos;
                double p_tan2 = p_tan * p_tan;
                double p_tan4 = p_tan2 * p_tan2;
                double ep_sin = 1 - WGS84_E * p_sin2;
                double ep_sin_sqrt = sqrt(1 - WGS84_E * p_sin2);
                double n = WGS84_A / ep_sin_sqrt;
                double r = (1 - WGS84_E) / ep_sin;
                double c = WGS84_EP2 * p_cos * p_cos;
                double c2 = c * c;
                double d = x / (n * UTM_K0);
                double d2 = d * d;
                double d3 = d2 * d;
                double d4 = d3 * d;
                double d5 = d4 * d;
                double d6 = d5 * d;
                double latitude = (p_rad - (p_tan / r) * (d2 / 2 - d4 / 24 * (5 + 3 * p_tan2 + 10 * c - 4 * c2 - 9 * WGS84_EP2)) + d6 / 720 * (61 + 90 * p_tan2 + 298 * c + 45 * p_tan4 - 252 * WGS84_EP2 - 3 * c2));
                double longitude = (d - d3 / 6 * (1 + 2 * p_tan2 + c) + d5 / 120 * (5 - 2 * c + 28 * p_tan2 - 3 * c2 + 8 * WGS84_EP2 + 24 * p_tan4)) / p_cos;
                longitude = ModAngleInRadians(longitude + Zone2CenterLon(zone) * RADIANS_PER_DEGREE);
                double longitude_deg = longitude * DEGREES_PER_RADIAN;
                double latitude_deg = latitude * DEGREES_PER_RADIAN;
                return Pointll(latitude_deg, longitude_deg);
            }

            class Projector
            {
            private:
                bool init_ = false;
                int zone_;
                bool northern_;
                double origin_x_;
                double origin_y_;

            public:
                Projector() = default;
                ~Projector() = default;
                void set_origin_ll(double lat, double lon)
                {
                    zone_ = LatLon2Zone(lat, lon);
                    Point2d origin_point = OriginFromLatLon(lat, lon, zone_, northern_);
                    origin_x_ = origin_point.x;
                    origin_y_ = origin_point.y;
                    init_ = true;
                }

                Point2d get_origin_xy()
                {
                    return Point2d(origin_x_, origin_y_);
                }

                Point2d forward(double lat, double lon)
                {
                    Point2d p = FromLatLon(lat, lon, zone_, northern_);
                    Point2d p_out;
                    p_out.x = p.x - origin_x_;
                    p_out.y = p.y - origin_y_;
                    return p_out;
                }

                Pointll reverse(double x, double y)
                {
                    x += origin_x_;
                    y += origin_y_;
                    Pointll p_out = ToLatLon(x, y, zone_, northern_);
                    return p_out;
                }
            };

        } // myprojector
    }     //routing
} //legion
