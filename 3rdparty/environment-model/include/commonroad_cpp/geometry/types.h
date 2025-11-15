#pragma once

#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/box.hpp>

/*
 * Note:
 * The point_type_alias definition below is required because Boost.Geometry
 * already defines a point_type, but we also want to define a point_type.
 * Our point_type is going to shadow Boost's, however that doesn't work with
 * the BOOST_GEOMETRY_REGISTER_BOX macro, so we need to jump through a few hoops
 * here. Same for box_type vs box.
 *
 * TODO Use non-conflicting names or introduce namespace for our geometry types
 */
using point_type_alias = boost::geometry::model::d2::point_xy<double>;

using polygon_type = boost::geometry::model::polygon<point_type_alias>;

using multi_polygon_type = boost::geometry::model::multi_polygon<polygon_type>;

/**
 * Box type (used for bounding boxes)
 *
 * Note: We don't use Boost's box model because the include is quite heavy and
 * boxes are (in)directly used almost everywhere.
 */
struct box_type {
    point_type_alias ll; //**< lower left corner */
    point_type_alias ur; //**< upper right corner */

    /**
     * Getter for lower left corner
     */
    point_type_alias min_corner() const noexcept { return ll; }

    /**
     * Getter for upper right corner
     */
    point_type_alias max_corner() const noexcept { return ur; }
};

// Register the box type for Boost

BOOST_GEOMETRY_REGISTER_BOX(box_type, point_type_alias, ll, ur)

// See above
using box = box_type;
using point_type = point_type_alias;
