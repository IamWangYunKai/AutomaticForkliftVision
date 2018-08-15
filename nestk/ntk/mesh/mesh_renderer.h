/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#ifndef MESH_RENDERER_H
#define MESH_RENDERER_H

#include "mesh.h"

// Qt Opengl headers include windows.h on windows without preventing
// preprocessor namespace pollution.
// FIXME: Factor this out.
#ifdef _WIN32
#   define NOMINMAX
#   define VC_EXTRALEAN
#   define WIN32_LEAN_AND_MEAN
#endif
#include <QtOpenGL/QGLPixelBuffer>
#ifdef _WIN32
#   undef WIN32_LEAN_AND_MEAN
#   undef VC_EXTRALEAN
#   undef NOMINMAX
#endif

namespace ntk
{

  class Pose3D;

  struct VertexBufferObject
  {
    VertexBufferObject() : initialized(false), vertex_id(-1), model_view_matrix(4,4) {}
    bool initialized;
    GLuint vertex_id;
    GLuint faces_id;
    GLuint texture_id;
    int color_offset;
    int texture_offset;
    int nb_vertices;
    int nb_faces;
    bool has_texcoords;
    bool has_color;
    bool has_faces;
    cv::Mat_<GLfloat> model_view_matrix;
  };

  /*!
   * Project a mesh on an image using the given pose.
   */
  class MeshRenderer
  {
  public:
    // Flags
    enum { NORMAL = 0, WIREFRAME = 0x1, RANDOM_COLORS = 0x2, TRANSPARENCY = 0x4, LIGHTING = 0x8, OUTLINE = 0x10 };

  public:
    MeshRenderer(int image_width, int image_height, float transparency = 1.0);
    ~MeshRenderer();

  public:
    void setMesh(const Mesh& mesh);
    void setPose(const Pose3D& pose,
                 float* near_plane_in_meters = 0, /* automatic */
                 float* far_plane_in_meters = 0); /* automatic */

  public:
    const cv::Mat1f& depthBuffer() const { return m_depth_buffer; }
    const cv::Mat4b& colorBuffer() const { return m_color_buffer; }

  public:
    void renderToImage(cv::Mat4b& image, int flags);
    void setTransparency(float f) { m_transparency = f; }
    void setClipPlanes(float near_plane_z, float far_plane_z) { m_last_near_plane = near_plane_z; m_last_far_plane = far_plane_z; }

  protected:
    void computeDepthBuffer();
    void estimateOptimalPlanes(const Pose3D& pose, float* near_plane, float* far_plane);
    void clearVertexBufferObject();

  private:
    const Mesh* m_mesh;
    QGLContext* m_context;
    QGLPixelBuffer* m_pbuffer;
    VertexBufferObject m_vertex_buffer_object;
    GLuint m_list_index;
    cv::Mat1f m_depth_buffer;
    cv::Mat4b m_color_buffer;
    float m_last_near_plane;
    float m_last_far_plane;
    float m_transparency;
  };

} // ntk

#endif // MESH_RENDERER_H
