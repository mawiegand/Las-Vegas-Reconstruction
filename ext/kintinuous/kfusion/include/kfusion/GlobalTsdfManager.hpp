/**
 * GlobalTsdfManager.hpp
 *
 *  @date 14.06.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#ifndef GLOBALTSDFMANAGER_HPP
#define GLOBALTSDFMANAGER_HPP

#include <kfusion/Options.hpp>
#include <kfusion/BlockingQueue.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <lvr/geometry/ColorVertex.hpp>
#include <lvr/geometry/BoundingBox.hpp>
#include <lvr/reconstruction/FastKinFuBox.hpp>
#include <lvr/reconstruction/GlobalTsdfGrid.hpp>

typedef lvr::ColorVertex<float, unsigned char> cVertex;
typedef lvr::FastKinFuBox<lvr::ColorVertex<float, unsigned char>, lvr::Normal<float> > cFastBox;
typedef lvr::GlobalTsdfGrid<cVertex, cFastBox, kfusion::Point> GGrid;

template<typename VectorT, typename TsdfT>
class GlobalTsdfManager
{
private:
    kfusion::Options* m_options;
    const double m_camera_target_distance;

    boost::shared_ptr<boost::thread> m_writerThread;
    boost::shared_ptr<BlockingQueue> m_sliceInQueue;

    GGrid* m_globalTsdfGrid;

    void writeSliceData();
public:
    GlobalTsdfManager(float cellSize, bool isVoxelsize, kfusion::Options* options, double camera_target_distance);
    pair<float*, size_t> getData(VectorT minBounds, VectorT maxBounds);
    bool addSliceToInQueue(TsdfT* tsdf, size_t size, bool last_shift);
    void saveMesh(string filename);
    ~GlobalTsdfManager();
};

#include <GlobalTsdfManager.tcc>

#endif //GLOBALTSDFMANAGER_HPP
