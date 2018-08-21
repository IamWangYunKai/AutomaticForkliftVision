#ifndef _MAP_MISC_H_
#define _MAP_MISC_H_

#include <string>
#include <robokit/foundation/utils/geometry.h>

namespace rbk
{
    namespace foundation
    {
        /// define the range structure
        struct Range {
            int x_min;
            int x_max;
            int y_min;
            int y_max;
        };

        /************************************************************************/
        /* 1.Markers in map							                            */
        /************************************************************************/
        enum MarkerType {
            Marker_None = 0,
            Marker_Eraser,
            Marker_Line,
            Marker_Goal,
            Marker_Dock,
            Marker_ForbiddenArea,
            Marker_ForbiddenLine,
            Marker_HomePoint,
            Marker_HomeArea,
        };

        /************************************************************************/
        /* 2.Cairns in map                                                      */
        /************************************************************************/
        enum CairnType {
            None = 0,
            ForbiddenArea,
            ForbiddenLine,
            RobotHome,
            Dock,
            Goal,
            GoalWithHeading,
            SimBoxObstacle
        };

        /************************************************************************/
        /* 3.Objects in map                                                     */
        /************************************************************************/
        typedef struct {
            CairnType type;
            GeoPoint center;
            double dir;
            std::string tagname;
            GeoLine line;
        } MapObject;
    }
}
#endif //~_MAP_MISC_H_
