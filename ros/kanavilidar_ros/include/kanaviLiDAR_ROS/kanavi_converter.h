/*
 * Copyright (c) 2022, Kanavi-Mobility.co.,ltd.
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __Kanavi_CONVERTER_H__
#define __Kanavi_CONVERTER_H__

/**
 * @file kanavi_converter.h
 * @author twchong (twchong@Kanavi.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "kanaviLiDAR_ros.h"
#include <pcl/point_types_conversion.h>

typedef pcl::PointCloud<pcl::PointXYZRGB>	PointCloudT;

class kanavi_converter
{
public:
	kanavi_converter(/* args */);
	~kanavi_converter();

	void setDatagram(const lidarDatagram &datagram);
	void setReverse(const bool &checked=false);
	PointCloudT getPointCloud();
private:
	//func.
	void calculateAngular(int model);
	void generatePointCloud(const lidarDatagram &datagram, PointCloudT &cloud_);
	pcl::PointXYZRGB length2point(float len, float v_sin, float v_cos, float h_sin, float h_cos);
	void HSV2RGB(float *fR, float *fG, float *fB, float fH, float fS, float fV);

	/* data */
	lidarDatagram g_datagram;
	bool checked_setAngular;

	std::vector<float> v_sin;
	std::vector<float> v_cos;
	std::vector<float> h_sin;
	std::vector<float> h_cos;

	//point cloud
	PointCloudT cloud;

	bool g_checked_HorizontalReverse;
};

#endif // __Kanavi_CONVERTER_H__