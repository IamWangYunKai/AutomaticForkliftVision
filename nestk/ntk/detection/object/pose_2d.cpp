//
// Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2009
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "pose_2d.h"
#include "object_database.h"

using namespace ntk;

namespace ntk
{

  Pose2D :: Pose2D(const ntk::AffineTransform& affine_transform)
    : m_affine_transform(affine_transform)
  {
  }

  void Pose2D ::
  fillXmlElement(XMLNode& element) const
  {
    addXmlChild(element, "affine_transform", *AffineTransformXmlSerializer::get(m_affine_transform));
  }

  void Pose2D ::
  loadFromXmlElement(const XMLNode& element)
  {
    loadFromXmlChild(element, "affine_transform", *AffineTransformXmlSerializer::get(m_affine_transform));
  }

  const NtkDebug& operator<<(const NtkDebug& os, const Pose2D& pose)
  {
    os << "tx=" << pose.affineTransform().mat02
       << " ty=" << pose.affineTransform().mat12;
    return os;
  }

}
