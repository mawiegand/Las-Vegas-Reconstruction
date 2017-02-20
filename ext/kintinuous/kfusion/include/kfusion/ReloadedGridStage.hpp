/*
 * Software License Agreement (BSD License)
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
/**
 * ReloadedGridStage.hpp
 *
 *  @date 24.01.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#ifndef RELOADEDGRIDSTAGE_HPP__
#define RELOADEDGRIDSTAGE_HPP__

#include "AbstractStage.hpp"
#include "BlockingQueue.hpp"
#include <boost/any.hpp>
#include <lvr/reconstruction/FastReconstruction.hpp>
#include <lvr/reconstruction/TSDFGrid.hpp>
#include <lvr/reconstruction/GlobalTsdfGrid.hpp>
#include <lvr/reconstruction/PointsetSurface.hpp>
#include <lvr/reconstruction/FastKinFuBox.hpp>
#include <lvr/io/PointBuffer.hpp>
#include <lvr/io/DataStruct.hpp>
#include <lvr/geometry/HalfEdgeVertex.hpp>
#include <lvr/geometry/HalfEdgeKinFuMesh.hpp>
#include <lvr/geometry/BoundingBox.hpp>
#include <kfusion/types.hpp>
#include <kfusion/cuda/tsdf_volume.hpp>
#include <kfusion/cuda/projective_icp.hpp>

using namespace lvr;
using namespace kfusion;
using namespace std;

typedef Vertex<float> fVertex;
typedef ColorVertex<float, unsigned char> cVertex;
typedef FastKinFuBox<ColorVertex<float, unsigned char>, lvr::Normal<float> > cFastBox;
typedef GlobalTsdfGrid<cVertex, cFastBox, kfusion::Point> GGrid;
typedef FastReconstruction<ColorVertex<float, unsigned char>, lvr::Normal<float>, cFastBox> cFastReconstruction;
typedef HalfEdgeKinFuMesh<cVertex, lvr::Normal<float> > HMesh;
typedef HMesh *MeshPtr;

class ReloadedGridStage : public AbstractStage
{
public:

    // default constructor
    ReloadedGridStage(double voxel_size = 3.0 / 512.0, Options *options = NULL);

    void firstStep();
    void step();
    void lastStep();

private:
    double voxel_size_;
    Options *options_;
    size_t slice_count_;
    BoundingBox<cVertex> bbox_;
    bool last_shift;
    GGrid *global_tsdf_;
};

#endif // STAGE
