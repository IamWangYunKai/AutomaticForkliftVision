//
// object_match.cc
//
// Author: Nicolas Burrus <nicolas.burrus@ensta.fr>, (C) 2007
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//

#include "sift_object_match.h"
#include "sift_point_match.h"
#include "object_database.h"
#include "object_detector.h"
#include "visual_object.h"
#include <ntk/ntk.h>

using namespace ntk;

namespace ntk
{

  void SiftObjectMatch ::
  debug(const NtkDebug& os) const
  {
    os << "ObjectMatch [ # = " << score() << "] [model="
        << model().name() << "] [nbpoints=" << point_matches.size() << "]"
        << " [nbpoints_with_depth=" << nbMatchesWithDepth() << "]"
        << *pose();
        // << projectedBoundingRect()
        // << hough_point;
  }

  int SiftObjectMatch :: nbMatchesWithDepth() const
  {
    int r = 0;
    foreach_const_it(it, point_matches, SiftPointMatchConstPtrSet)
    {
      if ((*it)->modelPoint().location().has_depth)
        ++r;
    }
    return r;
  }

  void SiftObjectMatch :: fillXmlElement(XMLNode& element) const
  {
    super::fillXmlElement(element);
    foreach_const_it(it, point_matches, SiftPointMatchConstPtrSet)
      addXmlChild(element, "point_match", **it);
  }

  void SiftObjectMatch :: loadFromXmlElement(const XMLNode& element, XmlContext* context)
  {
    super::loadFromXmlElement(element, context);
    for (int i = 0; i < element.nChildNode(); ++i)
    {
      XMLNode e = element.getChildNode(i);
      if (e.getNameAsString() == "point_match")
      {
        SiftPointMatch* match = new SiftPointMatch();
        match->loadFromXmlElement(e, context);
        point_matches.insert(SiftPointMatchConstPtr(match));
      }
    }
  }

} // end of avs
