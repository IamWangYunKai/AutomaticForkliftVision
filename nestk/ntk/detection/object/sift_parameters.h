//
// parameters.h
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

#ifndef FROL_sift_parameters_H
# define FROL_sift_parameters_H

# include <ntk/core.h>

# include "located_feature.h"

namespace ntk
{

  struct SiftHoughParameters
  {
    SiftHoughParameters();
    
    double delta_scale;
    double delta_orientation;
    double delta_location;
  };
  
  struct SiftParameters
  {
    SiftParameters();
    
    std::string sift_database_type;
    double max_dist_ratio;
    double p_good_match;
    SiftHoughParameters hough;
    double pfa_threshold;
    LocatedFeature::FeatureType feature_type;
    double depth_margin;
    double max_reprojection_error_percent;
  };

} // end of avs

#endif // ndef FROL_sift_parameters_H
