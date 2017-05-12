/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <kfusion/cyclical_buffer.h>
#include <kfusion/precomp.hpp>

bool
kfusion::cuda::CyclicalBuffer::checkForShift (cv::Ptr<cuda::TsdfVolume> volume, const Affine3f &cam_pose, const double distance_camera_target, const bool perform_shift, const bool last_shift, const bool record_mode)
{
    bool result = false;
    //mcwrap_.setCameraDist(distance_camera_target);
 	cv::Vec3f targetPoint(0,0, distance_camera_target);
 	targetPoint = cam_pose * targetPoint;
    targetPoint[1] = cam_pose.translation()[1];
	cv::Vec3f center_cube;
	center_cube[0] = buffer_.origin_metric.x + buffer_.volume_size.x/2.0f;
	center_cube[1] = buffer_.origin_metric.y + buffer_.volume_size.y/2.0f;
	center_cube[2] = buffer_.origin_metric.z + buffer_.volume_size.z/2.0f;
	double dist = norm(targetPoint, center_cube, cv::NORM_L2);
	//printf("   dist: %f   \n", dist);
	if (dist > distance_threshold_)
		result = true;

	// perform shifting operations
	if (result || last_shift || perform_shift)
	{
		performShift (volume, targetPoint, cam_pose, last_shift, record_mode);
		return true;
	}

	return (result);
}


void
kfusion::cuda::CyclicalBuffer::performShift (cv::Ptr<cuda::TsdfVolume> volume, const cv::Vec3f& target_point, const Affine3f &cam_pose, const bool last_shift, const bool record_mode)
{
	//ScopeTime* time = new ScopeTime("Whole Cube shift");
	// compute new origin and offsets
	Vec3i offset;
	Vec3i minBounds;
	Vec3i maxBounds;
	if(!last_shift)
		computeAndSetNewCubeMetricOrigin (volume, target_point, offset);

	if(!no_reconstruct_)
	{
		DeviceArray<Point> cloud;

		// calculate mininum and maximum slice bounds
		if(last_shift)
		{
			minBounds[0] = minBounds[1] = minBounds[2] = 0;
			maxBounds[0] = maxBounds[1] = maxBounds[2] = 512;
		}
		else
		{
			calcBounds(offset, minBounds, maxBounds);
		}

		cloud = volume->fetchSliceAsCloud(cloud_buffer_device_, &buffer_, minBounds, maxBounds, global_shift_ );

		cloud_slice_ = cv::Mat(1, (int)cloud.size(), CV_32FC4);
		cloud.download(cloud_slice_.ptr<Point>());

		Point* tsdf_ptr = cloud_slice_.ptr<Point>();
		if(cloud.size() > 0)
		{
			std::cout << "####    Performing slice number: " << slice_count_ << " with " << cloud.size() << " TSDF values  ####" << std::endl;

            global_tsdf_->addSliceToInQueue(tsdf_ptr, cloud_slice_.cols, last_shift);
            std::ofstream sliceFile;
            std::string fileName("slice_" + std::to_string(slice_count_) + ".3d");
            sliceFile.open (fileName);
            for (int i = 0; i < cloud.size(); i++)
            {
//                std::cout << "extracted: (" << tsdf_ptr->x << ", " << tsdf_ptr->y << ", " << tsdf_ptr->z << ", " << tsdf_ptr->w << ")" << std::endl;
                sliceFile << tsdf_ptr->x
                          << " " << tsdf_ptr->y
                          << " " << tsdf_ptr->z
                          << " " << 255 << " " << 0 << " " << 0
                          << std::endl;
                tsdf_ptr++;
            }
            sliceFile.close();
            slice_count_++;
		}
	}

	if(!last_shift)
	{
		// clear buffer slice and update the world model
		volume->clearSlice(&buffer_, offset);

		// shift buffer addresses
		shiftOrigin (volume, offset);

        // TODO: integrate existing GlobalTSDFData here

        // Calculate bounding box
        Vec3i min(280, 280, 280);
        Vec3i max(330, 330, 330);

        lvr::BoundingBox<cVertex> bbox = lvr::BoundingBox<cVertex>(min[0] + global_shift_[0], min[1] + global_shift_[1], min[2] + global_shift_[2],
                                                                   max[0] + global_shift_[0], max[1] + global_shift_[1], max[2] + global_shift_[2]);

        std::cout << "offset: " << offset << " globalShift: " << global_shift_ << std::endl;
        std::cout << "minBounds: " << minBounds << " maxBounds: " << maxBounds << " diff: " << maxBounds - minBounds << std::endl;
        std::cout << "min: " << min << " max: " << max << " diff: " << max - min << std::endl;

        int center_of_bb_x = (global_tsdf_->getBoundingBox().getXSize() / 2) / buffer_.voxels_size.x;
        int center_of_bb_y = (global_tsdf_->getBoundingBox().getXSize() / 2) / buffer_.voxels_size.y;
        int center_of_bb_z = (global_tsdf_->getBoundingBox().getXSize() / 2) / buffer_.voxels_size.z;
        std::ofstream boundingFile;
        std::string fileName("bounding_" + std::to_string(slice_count_) + ".3d");
        boundingFile.open (fileName);

        int stepSize = 10;
        for (int z = bbox.getMin().z + center_of_bb_z; z <= bbox.getMax().z + center_of_bb_z; z += stepSize)
        {
            for (int y = bbox.getMin().y + center_of_bb_y; y <= bbox.getMax().y + center_of_bb_y; y += stepSize)
            {
                for (int x = bbox.getMin().x + center_of_bb_x; x <= bbox.getMax().x + center_of_bb_x; x += stepSize)
                {
                    boundingFile << x
                                 << " " << y
                                 << " " << z
                                 << " " << 0 << " " << 255 << " " << 255
                                 << std::endl;
                }
            }
        }
        boundingFile.close();

        // get data from global tsdf in bounding box
        std::pair<Point*, size_t> data = global_tsdf_->getData(bbox);
		if (data.second > 0 )
        {
            // prepare data for integration in device buffer
            DeviceArray<Point> integrationCloud(data.second);
            integrationCloud.upload(data.first, data.second);

            cv::Mat integration_cloud_slice(1, (int)integrationCloud.size(), CV_32FC4);
            integrationCloud.download(integration_cloud_slice.ptr<Point>());

            Point* integration_ptr = integration_cloud_slice.ptr<Point>();
            if(integrationCloud.size() > 0 && integration_ptr != NULL)
            {
                std::cout << "integrationClout size: " << integrationCloud.size() << std::endl;
                std::ofstream integrationCloudFile;
                std::string fileName("integration_cloud_" + std::to_string(slice_count_) + ".3d");
                integrationCloudFile.open(fileName);
                for (int i = 0; i < integrationCloud.size(); i++)
                {
                    float distance = integration_ptr->w;
                    if (distance != 0.f)
                    {
                        integrationCloudFile << integration_ptr->x
                                             << " " << integration_ptr->y
                                             << " " << integration_ptr->z;
                        if (distance >= 0.f && distance <= 1.f)
                        {
                            integrationCloudFile << " " << 255
                                                 << " " << 0
                                                 << " " << 0
                                                 << std::endl;
                        }
                        else if (distance >= -1.f && distance < 0.f)
                        {
                            integrationCloudFile << " " << 0
                                                 << " " << 255
                                                 << " " << 0
                                                 << std::endl;
                        }
                    }
                    integration_ptr++;
                }

//                int yStepSize = max[0] - min[0] + 1;
//                int zStepSize = (max[0] - min[1] + 1) * yStepSize;
//                for(int x = 0; x < 512; ++x)
//                {
//                    for(int y = 0; y < 512; ++y)
//                    {
//                        for (int z = 0; z < 512; ++z)
//                        {
//                            bool inBounds = (
//                                    (x >= min[0] && x <= max[0]) &&
//                                    (y >= min[1] && y <= max[1]) &&
//                                    (z >= min[2] && z <= max[2])
//                            );
//
//                            if (inBounds)
//                            {
//                                Point* localPtr = (integration_ptr + (z - min[2]) * zStepSize + (y - min[1]) * yStepSize + (x - min[0]));
//                                float distance = localPtr->w;
//                                if (distance >= -1.f && distance <= 1.f && distance != 0.f)
//                                {
//                                    integrationCloudFile << localPtr->x
//                                                         << " " << localPtr->y
//                                                         << " " << localPtr->z;
//                                    if (distance >= 0.f && distance <= 1.f)
//                                    {
//                                        integrationCloudFile << " " << 255
//                                                             << " " << 0
//                                                             << " " << 0
//                                                             << std::endl;
//                                    }
//                                    else if (distance >= -1.f && distance < 0.f)
//                                    {
//                                        integrationCloudFile << " " << 0
//                                                             << " " << 255
//                                                             << " " << 0
//                                                             << std::endl;
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }
                integrationCloudFile.close();
            }

            volume->integrateSlice(&buffer_, integrationCloud, min, max, global_shift_);
        }
	}
    if (last_shift)
    {
        lvr::BoundingBox<cVertex> bbox = lvr::BoundingBox<cVertex>(0, 0, 0, 512, 512, 512);

        int center_of_bb_x = (global_tsdf_->getBoundingBox().getXSize() / 2) / buffer_.voxels_size.x;
        int center_of_bb_y = (global_tsdf_->getBoundingBox().getXSize() / 2) / buffer_.voxels_size.y;
        int center_of_bb_z = (global_tsdf_->getBoundingBox().getXSize() / 2) / buffer_.voxels_size.z;

        static char fileName[26];
        time_t now = time(0);
        strftime(fileName, sizeof(fileName), "origin_%Y%m%d_%H%M%S.3d", localtime(&now));

        std::ofstream originFile;
        originFile.open(fileName);

        int stepSize = 10;
        for (int z = bbox.getMin().z + center_of_bb_z; z <= bbox.getMax().z + center_of_bb_z; z += stepSize)
        {
            for (int y = bbox.getMin().y + center_of_bb_y; y <= bbox.getMax().y + center_of_bb_y; y += stepSize)
            {
                for (int x = bbox.getMin().x + center_of_bb_x; x <= bbox.getMax().x + center_of_bb_x; x += stepSize)
                {
                    originFile << x
                               << " " << y
                               << " " << z
                               << " " << 0 << " " << 255 << " " << 255
                               << std::endl;
                }
            }
        }
        originFile.close();

        global_tsdf_->saveMesh(options_->getOutput());
    }
}

void
kfusion::cuda::CyclicalBuffer::computeAndSetNewCubeMetricOrigin (cv::Ptr<cuda::TsdfVolume> volume, const cv::Vec3f& target_point, Vec3i& offset)
{
	// compute new origin for the cube, based on the target point
	float3 new_cube_origin_meters;
	new_cube_origin_meters.x = target_point[0] - buffer_.volume_size.x/2.0f;
	new_cube_origin_meters.y = target_point[1] - buffer_.volume_size.y/2.0f;
	new_cube_origin_meters.z = target_point[2] - buffer_.volume_size.z/2.0f;
	//printf("The old cube's metric origin was    (%f, %f, %f).\n", buffer_.origin_metric.x, buffer_.origin_metric.y, buffer_.origin_metric.z);
	//printf("The new cube's metric origin is now (%f, %f, %f).\n", new_cube_origin_meters.x, new_cube_origin_meters.y, new_cube_origin_meters.z);

	// deduce each shift in indices
	offset[0] = calcIndex((new_cube_origin_meters.x - buffer_.origin_metric.x) * ( buffer_.voxels_size.x / (float) (buffer_.volume_size.x) ));
	offset[1] = calcIndex((new_cube_origin_meters.y - buffer_.origin_metric.y) * ( buffer_.voxels_size.y / (float) (buffer_.volume_size.y) ));
	offset[2] = calcIndex((new_cube_origin_meters.z - buffer_.origin_metric.z) * ( buffer_.voxels_size.z / (float) (buffer_.volume_size.z) ));

	//printf("The shift indices are (X:%d, Y:%d, Z:%d).\n", offset[0], offset[1], offset[2]);
	// update the cube's metric origin
	buffer_.origin_metric = new_cube_origin_meters;
	volume->setPose(Affine3f().translate(Vec3f(new_cube_origin_meters.x, new_cube_origin_meters.y,  new_cube_origin_meters.z)));
}

void kfusion::cuda::CyclicalBuffer::calcBounds(Vec3i& offset, Vec3i& minBounds, Vec3i& maxBounds)
{

	//Compute slice bounds
	int newX = buffer_.origin_GRID.x + offset[0];
	int newY = buffer_.origin_GRID.y + offset[1];
	int newZ = buffer_.origin_GRID.z + offset[2];

	//X
	if (newX >= 0)
	{
		minBounds[0] = buffer_.origin_GRID.x;
		maxBounds[0] = newX;
	}
	else
	{
		minBounds[0] = newX + buffer_.voxels_size.x;
		maxBounds[0] = buffer_.origin_GRID.x + buffer_.voxels_size.x;
	}

	if (minBounds[0] > maxBounds[0])
	  std::swap (minBounds[0], maxBounds[0]);

	//Y
	if (newY >= 0)
	{
		minBounds[1] = buffer_.origin_GRID.y;
		maxBounds[1] = newY;
	}
	else
	{
		minBounds[1] = newY + buffer_.voxels_size.y;
		maxBounds[1] = buffer_.origin_GRID.y + buffer_.voxels_size.y;
	}

	if(minBounds[1] > maxBounds[1])
	  std::swap (minBounds[1], maxBounds[1]);

	//Z
	if (newZ >= 0)
	{
		minBounds[2] = buffer_.origin_GRID.z;
		maxBounds[2] = newZ;
	}
	else
	{
	  minBounds[2] = newZ + buffer_.voxels_size.z;
	  maxBounds[2] = buffer_.origin_GRID.z + buffer_.voxels_size.z;
	}

	if (minBounds[2] > maxBounds[2])
	  std::swap(minBounds[2], maxBounds[2]);

	minBounds[0] -= buffer_.origin_GRID.x;
	maxBounds[0] -= buffer_.origin_GRID.x;

	minBounds[1] -= buffer_.origin_GRID.y;
	maxBounds[1] -= buffer_.origin_GRID.y;

	minBounds[2] -= buffer_.origin_GRID.z;
	maxBounds[2] -= buffer_.origin_GRID.z;

	if (minBounds[0] < 0) // We are shifting Left
	{
	  minBounds[0] += buffer_.voxels_size.x;
	  maxBounds[0] += buffer_.voxels_size.x;
	}


	if (minBounds[1] < 0) // We are shifting up
	{
	  minBounds[1] += buffer_.voxels_size.y;
	  maxBounds[1] += buffer_.voxels_size.y;
	}

	if (minBounds[2] < 0) // We are shifting forward
	{
	  minBounds[2] += buffer_.voxels_size.z;
	  maxBounds[2] += buffer_.voxels_size.z;
	}
	for(int i = 0; i < 3; i++)
	{
		if(maxBounds[i] > 0)
		{
			if(minBounds[i] == 0)
			{
				maxBounds[i] += 1;
				minBounds[i] += 1;

			}
			if(maxBounds[i] == 512)
			{
				minBounds[i] -= 1;
				maxBounds[i] -= 1;
				//offset[i] -=1;
			}
		}
	}


	//cout << "minBounds: " << minBounds[0] <<  ", " << minBounds[1] <<  ", " << minBounds[2] << endl;
	//cout << "maxBounds: " << maxBounds[0] <<  ", " << maxBounds[1] <<  ", " << maxBounds[2] << endl;
}
