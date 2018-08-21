#ifndef _LOCAL_MAP_REGION_H_
#define  _LOCAL_MAP_REGION_H_

#include <robokit/foundation/utils/geometry.h>

namespace rbk {
	namespace foundation {
		class LocalRegion {
		public:
			LocalRegion() {}
			~LocalRegion() {}
			bool contain(rbk::foundation::GeoPoint p) {
				int j = 3;
				bool oddNodes = false;

				for (int i = 0; i < 4; i++) {
					if (m_point[i].y() < p.y() && m_point[j].y() >= p.y() || m_point[j].y() < p.y() && m_point[i].y() >= p.y()) {
						if (m_point[i].x() + (p.y() - m_point[i].y()) / (m_point[j].y() - m_point[i].y() + 0.00000001)*(m_point[j].x() - m_point[i].x()) < p.x()) {
							oddNodes = !oddNodes;
						}
					}
					j = i;
				}
				return oddNodes;
			}
			rbk::foundation::GeoPoint m_point[4];
			double m_dir;
		};
	}
}



#endif