#ifndef _QUADTREE_BASIC_H_
#define _QUADTREE_BASIC_H_

#include <math.h>
#include <map>
#include <vector>
#include <string>

#if defined (__cplusplus)
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define PRECISION 100000.0
    ///** TRUE iff A and B are equal to within PRECISION */
    //#define EQ(A,B) ((floor(A*PRECISION+0.5))==(floor(B*PRECISION+0.5)))
    ///** TRUE iff A is less than B, subject to PRECISION */
    //#define LT(A,B) ((floor(A*PRECISION+0.5))<(floor(B*PRECISION+0.5)))
    ///** TRUE iff A is greater than B, subject to PRECISION */
    //#define GT(A,B) ((floor(A*PRECISION+0.5))>(floor(B*PRECISION+0.5)))
    ///** TRUE iff A is greater than or equal B, subject to PRECISION */
    //#define GTE(A,B) ((floor(A*PRECISION+0.5))>=(floor(B*PRECISION+0.5)))
    ///** TRUE iff A is less than or equal to B, subject to PRECISION */
    //#define LTE(A,B) ((floor(A*PRECISION+0.5))<=(floor(B*PRECISION+0.5)))

                /** TRUE iff A and B are equal to within PRECISION */
#define EQ(A,B) (int(A*PRECISION)==int(B*PRECISION))
            /** TRUE iff A is less than B, subject to PRECISION */
#define LT(A,B) (int(A*PRECISION)<int(B*PRECISION))
            /** TRUE iff A is greater than B, subject to PRECISION */
#define GT(A,B) (int(A*PRECISION)>int(B*PRECISION))
            /** TRUE iff A is greater than or equal B, subject to PRECISION */
#define GTE(A,B) (int(A*PRECISION)>=int(B*PRECISION))
            /** TRUE iff A is less than or equal to B, subject to PRECISION */
#define LTE(A,B) (int(A*PRECISION)<=int(B*PRECISION))

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/** define a point on the plane */
    typedef double stg_meters_t;
    typedef struct
    {
        stg_meters_t x, y;
    } stg_point_t;

    /** specify a line from (x1,y1) to (x2,y2), all in meters
    */
    typedef struct
    {
        stg_meters_t x1, y1, x2, y2;
    } stg_line_t;

    /** A node in the occupancy quad-tree */
    typedef struct stg_cell
    {
        double x, y;
        double off_x, off_y;	// need to be optimized with neighbouring cells
        double size;

        // bounding box
        double xmin, ymin, xmax, ymax;

        struct stg_cell* children[4];
        struct stg_cell* parent;
        std::vector<std::string> object_list;
    } stg_cell_t;

    /** Occupancy quadtree structure */
    typedef struct stg_matrix
    {
        double ppm;				// pixels per meter (1/resolution)
        double width, height;

        stg_cell_t* root;
        std::map<std::string, std::vector<stg_cell_t*> >* cell_map;
    } stg_matrix_t;

    /************************************************************************/
    /*					Cell related operations                             */
    /************************************************************************/
    /// Create a new cell with parent
    stg_cell_t* stg_cell_create(stg_cell_t* parent, double x, double y, double size);

    /// Frees the cell
    void stg_cell_delete(stg_cell_t* cell);

    /// Recursively free the quadtree of cells (rh)
    void stg_cell_and_descendents_delete(stg_cell_t* cell);

    /// Get the smallest cell that contains the point x,y. cell need not be the root of the tree
    stg_cell_t* stg_cell_locate(stg_cell_t* cell, double x, double y);

    /// Find or create the smallest cell for the given point
    stg_cell_t* stg_find_or_create_leaf_cell(stg_cell_t *startcell, double x, double y, double leaf_cell_size);

    /************************************************************************/
    /*					Matrix related operations                           */
    /************************************************************************/
    /// Create a new matrix structure
    stg_matrix_t* stg_matrix_create(double ppm, double width, double height);

    /// Frees all memory allocated by the matrix; first the cells, then the cell array.
    void stg_matrix_destroy(stg_matrix_t* matrix);

    /// Project a set of points into the matrix.
    void stg_matrix_points(stg_matrix_t *matrix, double x, double y, double a,
        stg_point_t *points, size_t npoints, void *object);

    /// Call stg_matrix_line for each of [num_lines] lines
    void stg_matrix_lines(stg_matrix_t* matrix, stg_line_t* lines, int num_lines, void* object);

    /// Append to the [object] pointer to the cells on the edge of a rectangle
    void stg_matrix_rectangle(stg_matrix_t* matrix, double px, double py, double pth,
        double dx, double dy, void* object);

    /// Optimize the created matrix with forces by neighbouring cells
    void stg_matrix_optimize(stg_matrix_t* matrix);

    /// Remove the points of the object
    void stg_cell_remove_object(stg_cell_t* cell);

    /// Remove the points of the object in matrix
    void stg_matrix_remove_object(stg_matrix_t* matrix, void* object);

#if defined (__cplusplus)
}
#endif

#endif	// ~_QUADTREE_BASIC_H_
