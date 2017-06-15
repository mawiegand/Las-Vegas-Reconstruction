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
#include <lvr/reconstruction/GlobalTsdfGrid.hpp>

typedef lvr::ColorVertex<float, unsigned char> cVertex;
typedef lvr::FastKinFuBox<lvr::ColorVertex<float, unsigned char>, lvr::Normal<float> > cFastBox;
typedef lvr::GlobalTsdfGrid<cVertex, cFastBox, kfusion::Point> GGrid;

template<typename VertexT, typename BoxT, typename TsdfT>
class GlobalTsdfManager
{
private:
    kfusion::Options* m_options;

    boost::shared_ptr<boost::thread> m_writerThread;
    boost::shared_ptr<BlockingQueue> m_sliceInQueue;

    GGrid* m_globalTsdfGrid;

    void writeSliceData();
public:
    GlobalTsdfManager(float cellSize, bool isVoxelsize, kfusion::Options* options);
    pair<float*, size_t> getData(lvr::BoundingBox<VertexT> bb);
    bool addSliceToInQueue(TsdfT* tsdf, size_t size, bool last_shift);
    void saveMesh(string filename);
    ~GlobalTsdfManager();
};

#include <GlobalTsdfManager.tcc>

#endif //GLOBALTSDFMANAGER_HPP
