/**
 * @file    enum.h
 * @author  dabai-legion
 * @date    2024-01-04
 * @version 1.0.0
 * @par     Copyright(c)
 * @license GNU General Public License (GPL)
 */

#pragma once

namespace motion_manager
{
namespace common
{
    enum DetectedObjectLabel
    {
        DO_UNKNOWN = 0,
        DO_CAR = 1,
        DO_TRUCK = 2,
        DO_BUS = 3,
        DO_TRAILER = 4,
        DO_MOTORCYCLE = 5,
        DO_BICYCLE = 6,
        DO_PEDESTRIAN = 7,
    };
    enum ShapeType
    {
        S_BOUNDING_BOX = 0,
        S_CYLINDER = 1,
        S_POLYGON = 2,
    };
} // namespace common
} // namespace legion
