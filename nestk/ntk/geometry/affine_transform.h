/*
  ==============================================================================

   This file is part of the JUCE library - "Jules' Utility Class Extensions"
   Copyright 2004-7 by Raw Material Software ltd.

  ------------------------------------------------------------------------------

   JUCE can be redistributed and/or modified under the terms of the
   GNU General Public License, as published by the Free Software Foundation;
   either version 2 of the License, or (at your option) any later version.

   JUCE is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with JUCE; if not, visit www.gnu.org/licenses or write to the
   Free Software Foundation, Inc., 59 Temple Place, Suite 330,
   Boston, MA 02111-1307 USA

  ------------------------------------------------------------------------------

   If you'd like to release a closed-source product which uses JUCE, commercial
   licenses are also available: visit www.rawmaterialsoftware.com/juce for
   more information.

  ==============================================================================
*/

#ifndef NTK_GEOMETRY_AFFINE_TRANSFORM
#define NTK_GEOMETRY_AFFINE_TRANSFORM

#include <ntk/core.h>
#include <ntk/geometry/polygon.h>
#include <ntk/utils/xml_serializable.h>

#include <QMatrix>

namespace ntk
{

//==============================================================================
/*!
    Represents a 2D affine-transformation matrix.

    An affine transformation is a transformation such as a rotation, scale, shear,
    resize or translation.

    These are used for various 2D transformation tasks, e.g. with Path objects.

    @see Path, Point, Line
*/
class AffineTransform
{
public:
    //==============================================================================
    /** Creates an identity transform. */
    AffineTransform() throw();

    /** Creates a copy of another transform. */
    AffineTransform (const AffineTransform& other) throw();

    /** Creates a transform from a set of raw matrix values.

        The resulting matrix is:

            (mat00 mat01 mat02)
            (mat10 mat11 mat12)
            (  0     0     1  )
    */
    AffineTransform (const float mat00, const float mat01, const float mat02,
                     const float mat10, const float mat11, const float mat12) throw();

    /** Copies from another AffineTransform object */
    const AffineTransform& operator= (const AffineTransform& other) throw();

    /** Compares two transforms. */
    bool operator== (const AffineTransform& other) const throw();

    /** Compares two transforms. */
    bool operator!= (const AffineTransform& other) const throw();

    /** A ready-to-use identity transform, which you can use to append other
        transformations to.

        e.g. @code
        AffineTransform myTransform = AffineTransform::identity.rotated (.5f)
                                                               .scaled (2.0f);

        @endcode
    */
    static const AffineTransform identity;

    //==============================================================================
    /** Transforms a 2D co-ordinate using this matrix. */
    void transformPoint (float& x,
                         float& y) const throw();

    /** Transforms a 2D co-ordinate using this matrix. */
    void transformPoint (double& x,
                         double& y) const throw();

    //==============================================================================
    /** Returns a new transform which is the same as this one followed by a translation. */
    const AffineTransform translated (const float deltaX,
                                      const float deltaY) const throw();

    /** Returns a new transform which is a translation. */
    static const AffineTransform translation (const float deltaX,
                                              const float deltaY) throw();

    /** Returns a transform which is the same as this one followed by a rotation.

        The rotation is specified by a number of radians to rotate clockwise, centred around
        the origin (0, 0).
    */
    const AffineTransform rotated (const float angleInRadians) const throw();

    /** Returns a transform which is the same as this one followed by a rotation about a given point.

        The rotation is specified by a number of radians to rotate clockwise, centred around
        the co-ordinates passed in.
    */
    const AffineTransform rotated (const float angleInRadians,
                                   const float pivotX,
                                   const float pivotY) const throw();

    /** Returns a new transform which is a rotation about (0, 0). */
    static const AffineTransform rotation (const float angleInRadians) throw();

    /** Returns a new transform which is a rotation about a given point. */
    static const AffineTransform rotation (const float angleInRadians,
                                           const float pivotX,
                                           const float pivotY) throw();

    /** Returns a transform which is the same as this one followed by a re-scaling.

        The scaling is centred around the origin (0, 0).
    */
    const AffineTransform scaled (const float factorX,
                                  const float factorY) const throw();

    /** Returns a new transform which is a re-scale about the origin. */
    static const AffineTransform scale (const float factorX,
                                        const float factorY) throw();

    /** Returns a transform which is the same as this one followed by a shear.

        The shear is centred around the origin (0, 0).
    */
    const AffineTransform sheared (const float shearX,
                                   const float shearY) const throw();

    /** Returns a matrix which is the inverse operation of this one.

        Some matrices don't have an inverse - in this case, the method will just return
        an identity transform.
    */
    const AffineTransform inverted() const throw();

    //==============================================================================
    /** Returns the result of concatenating another transformation after this one. */
    const AffineTransform followedBy (const AffineTransform& other) const throw();

    /** Returns true if this transform has no effect on points. */
    bool isIdentity() const throw();

    /** Returns true if this transform maps to a singularity - i.e. if it has no inverse. */
    bool isSingularity() const throw();

    //==============================================================================
    /* The transform matrix is:

        (mat00 mat01 mat02)
        (mat10 mat11 mat12)
        (  0     0     1  )
    */
    float mat00, mat01, mat02;
    float mat10, mat11, mat12;

private:
    //==============================================================================
    const AffineTransform followedBy (const float mat00, const float mat01, const float mat02,
                                      const float mat10, const float mat11, const float mat12) const throw();
};

cv::Point2f apply_transform(const ntk::AffineTransform& transform, const cv::Point2f& point);

void apply_transform(const ntk::AffineTransform& transform,
                     double x, double y,
                     double* dx, double* dy);

ntk::Polygon2d apply_transform(const ntk::AffineTransform& transform,
                               const cv::Rect_<float>& rect);

} // ntk

namespace ntk
{

class AffineTransformXmlSerializer : public XmlSerializable
{
  public:
    typedef ntk::AffineTransform value_type;
    typedef AffineTransformXmlSerializer this_type;

  public:
    static const XmlSerializableConstPtr get(const value_type& value)
    { return XmlSerializableConstPtr(new this_type(value)); }

    static XmlSerializablePtr get(value_type& value)
    { return XmlSerializablePtr(new this_type(value)); }

    virtual void fillXmlElement(XMLNode& element) const
    {
      setXmlAttribute(element, "mat00", value.mat00);
      setXmlAttribute(element, "mat01", value.mat01);
      setXmlAttribute(element, "mat02", value.mat02);
      setXmlAttribute(element, "mat10", value.mat10);
      setXmlAttribute(element, "mat11", value.mat11);
      setXmlAttribute(element, "mat12", value.mat12);
    }

    virtual void loadFromXmlElement(const XMLNode& element)
    {
      loadFromXmlAttribute(element, "mat00", value.mat00);
      loadFromXmlAttribute(element, "mat01", value.mat01);
      loadFromXmlAttribute(element, "mat02", value.mat02);
      loadFromXmlAttribute(element, "mat10", value.mat10);
      loadFromXmlAttribute(element, "mat11", value.mat11);
      loadFromXmlAttribute(element, "mat12", value.mat12);
    }

  private:
    AffineTransformXmlSerializer(const value_type& value)
        : value(const_cast<value_type&>(value))
    {}

  private:
    value_type& value;
};

inline QMatrix toQt(const ntk::AffineTransform& transform)
{
  return QMatrix(transform.mat00, transform.mat01,
                 transform.mat10, transform.mat11,
                 transform.mat02, transform.mat12);
}

} // ntk

#endif   // NTK_GEOMETRY_AFFINE_TRANSFORM
