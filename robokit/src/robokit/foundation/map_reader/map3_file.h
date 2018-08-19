#ifndef _MAP3_FILE_H_
#define _MAP3_FILE_H_

#include <vector>
#include <robokit/foundation/utils/geometry.h>

namespace rbk
{
    namespace foundation
    {
        class Map3File {
        public:
            Map3File();
            ~Map3File();
            void setResolution(int);
            void setPointList(std::vector<GeoPoint>);
            void write(std::string);
            GeoPoint getMinPoint();
            GeoPoint getMaxPoint();
            int getWidth();
            int getHeight();
        private:
            std::vector<GeoPoint> m_point_list;
            GeoPoint m_min_pos;
            GeoPoint m_max_pos;
            int m_resolution;
        };
    }
}
#endif // ~_MAP3_FILE_H_
