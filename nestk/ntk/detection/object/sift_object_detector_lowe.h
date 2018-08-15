//
// object_finder_simple.h
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

#ifndef FROL_sift_object_finder_lowe_H
# define FROL_sift_object_finder_lowe_H

# include "sift_object_detector.h"
# include "sift_object_match_lowe.h"

namespace ntk
{
  
  double computeLowePfa(const SiftParameters& params,
                        double nb_points_in_object_view,
                        double nb_points_in_db,
                        double nb_points_in_region,
                        double nb_matches);

  class SiftObjectDetectorLowe : public SiftObjectDetector
  {
    typedef SiftObjectDetector super;

    public:
      SiftObjectDetectorLowe(const SiftParameters& params);
      virtual ~SiftObjectDetectorLowe();

    public:
      virtual void fillXmlElement(XMLNode& element) const;
      virtual void loadFromXmlElement(const XMLNode& element);

    public:
      virtual unsigned nbObjectMatches() const { return m_objects.size(); }
      virtual const SiftObjectMatchLowe& objectMatch(unsigned idx) const;
      virtual SiftObjectMatchLowe& objectMatch(unsigned idx);
      virtual void keepOnlyBestMatch();

    public:
      virtual void initializeFindObjects();
      virtual void findObjects();

    protected:
      double computePFA(const SiftObjectMatchLowe& match) const;
      void filterIdenticalMatches();
      void filterMultipleInstanceClusters(SiftHough::clusters_type& clusters);

    private:
      std::vector<SiftObjectMatchLowePtr> m_objects;
  };  

} // end of avs

#endif // ndef FROL_sift_object_finder_lowe_H
