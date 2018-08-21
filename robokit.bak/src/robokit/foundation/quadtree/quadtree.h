#ifndef _QUADTREE_H_
#define _QUADTREE_H_

#include <robokit/foundation/quadtree/quadtree_basic.h>
#include <robokit/foundation/utils/geometry.h>
#include <map>
#include <vector>
#include <string>

namespace rbk
{
    namespace foundation
    {
        /// Define the class
        class QuadTree {
        public:
            /// Constructor
            QuadTree(/*MapReader* map_reader*/);

            /// Destroyer
            ~QuadTree();

            /// Initialize
            void Initialize(int resulotion, double x_max, double x_min, double y_max, double y_min, std::vector<GeoPoint> points, std::vector<GeoLine> lines);

            void setResolution(int resulotion);

            void setMapRange(double x_max, double x_min, double y_max, double y_min);
            /// Input: Points

            void setMapName(std::string name);

            void setPoints(std::vector<GeoPoint> points);

            ///Input Lines:
            void setLines(std::vector<GeoLine> lines);

            /// Reload
            void Reload();

            /// Is tree loaded
            bool IsLoaded() const;

            void getMapName();

            stg_matrix_t* GetQuadTreeMatrix();

            /// Release
            void Release();

			// add lines
			void addLines(void*, std::vector<GeoLine>);

			// remove lines
			void removeLines(void*);
        private:

            /// Optimize the matrix
            void OptimizeMatrix(stg_matrix_t* matrix);

            void DivideCells(stg_matrix_t* matrix, stg_cell_t* cell);

            void DivideSingleCell(stg_matrix_t* matrix, stg_cell_t* cell);
            /// Optimize the cells
            void OptimizeCells(stg_matrix_t* matrix, stg_cell_t* cell);

            /// Optimize single cell
            void OptimizeSingleCell(stg_matrix_t* matrix, stg_cell_t* cell);

        private:
            stg_matrix_t* m_matrix;

            stg_point_t*	m_matrix_pointlist;

            stg_line_t*		m_matrix_linelist;

            std::vector<GeoPoint> m_points;

            std::vector<GeoLine> m_lines;

            int m_resolution;

            double m_xmax;

            double m_xmin;

            double m_ymax;

            double m_ymin;

            std::string m_map_name;
        };
    }
}
#endif	// ~_QUADTREE_H_
