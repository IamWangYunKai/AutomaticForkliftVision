#ifndef _MEANSHIFT_H_
#define _MEANSHIFT_H_
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/eigen.h>
#include <pcl/common/common.h>
//#include "common.h"


class MeanShift
{
public:
	typedef std::vector<VecPoint> VecVecPoint;
 
	static const float NEAREST_ZERO;
	static const float C;  //!楂樻柉鏍稿嚱鏁颁腑鐨勫父鏁?
 
	MeanShift() : m_size(0), R(0.0f){}
 
	 /** \brief 璁剧疆杈撳叆鐐逛簯
	   * \param[in]  pPntCloud 杈撳叆鐐逛簯
	   */
	bool setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pPntCloud);
 
 
	 /** \brief 鑾峰彇鑱氱被鍚庣殑鐐逛簯  */
	VecVecPoint & getOutputCloud()
	{
		return vv_pnt;
	}
 
	/** \brief 璁剧疆K杩戦偦绠楁硶鐨勬悳绱㈠崐寰?
	   * \param[in]  radius 鍗婂緞
	   */
	bool setKNNRadius(const float radius)
	{
		R = radius;
        return true;
	}
 
	 /** \brief 鎵цMeanShift鑱氱被绠楁硶	   */
	bool process();
 
	 /** \brief 淇濆瓨鑱氱被鍚庣殑鐐逛簯鏂囦欢
	   * \param[in] dir_name 淇濆瓨鐨勭洰褰?
	   * \param[in] prex_name 鏂囦欢鍚嶅墠缂€
	   */
	bool SaveFile(const char *dir_name, const char *prex_name);
 
private:
	size_t m_size;  //!瑕佸鐞嗙偣鐨勪釜鏁?
	pcl::PointCloud<pcl::PointXYZ>::Ptr mp_pointcloud;  //!PCL褰㈠紡鐨勭偣浜戯紝涓昏鏄负浜嗕娇鐢‵LANN鐨凨D鏍?
	VecPoint mv_pntcld;  //!鐐逛簯
	VecPoint mv_local_mode;  //!姣忎釜鐐圭殑灞€閮ㄦā寮?
	VecVecPoint vv_pnt;  //!鑱氱被鍚庣殑鐐逛簯
 
	float R;  //!K杩戦偦绠楁硶鏃跺搴旂殑鎼滅储鍗婂緞
 
	 /** \brief 瀵规瘡涓偣鎵цMeanShift
	   * \param[in]  in_pnt 杈撳叆鐨勭偣
	   * \param[out] out_pnt 杈撳嚭鐨勭偣
	   */
	inline bool execMeanShiftEachPoint(const pcl::PointXYZ &in_pnt, Point &out_pnt);
 
	 /** \brief 灏嗗叿鏈夌浉鍚屽眬閮ㄦā寮忕殑鐐瑰綊涓轰竴绫?
	   * \param[in] v_localmode 鍚勪釜鐐瑰搴旂殑灞€閮ㄦā寮?
	   * \param[out] vv_pnt 褰掑苟鍚庣殑鐐?
	   */
	bool mergeSameLocalModePoint(const VecPoint &v_localmode, VecVecPoint &vv_pnt);
 
	inline float gauss(float x)
	{
		return C * sqrt(x) * exp(-0.5 * x);
	}
 
};
 
#endif