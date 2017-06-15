/**
 * GlobalTsdfManager.tcc
 *
 *  @date 14.06.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#include <kfusion/GlobalTsdfManager.hpp>

#define DEBUG 0

template<typename VectorT, typename TsdfT>
GlobalTsdfManager<VectorT, TsdfT>::GlobalTsdfManager(float cellSize, bool isVoxelsize, kfusion::Options* options) :
        m_options(options)
{
    this->m_sliceInQueue = boost::shared_ptr<BlockingQueue>(new BlockingQueue());
    this->m_writerThread = boost::shared_ptr<boost::thread>(new boost::thread(
            boost::bind(&GlobalTsdfManager<VectorT, TsdfT>::writeSliceData, this)
    ));

    lvr::BoundingBox<cVertex> bbox_ = lvr::BoundingBox<cVertex>(0.0, 0.0, 0.0, 300.0, 300.0, 300.0);
    bbox_.expand(300.0, 300.0, 300.0);

    this->m_globalTsdfGrid = new GGrid(cellSize, bbox_, isVoxelsize, options);
}

template<typename VectorT, typename TsdfT>
bool GlobalTsdfManager<VectorT, TsdfT>::addSliceToInQueue(TsdfT *tsdf, size_t size, bool last_shift)
{
    m_sliceInQueue->Add(pair<pair<TsdfT*, size_t>, bool>(pair<TsdfT*, size_t>(tsdf, size), last_shift));
}

template<typename VectorT, typename TsdfT>
void GlobalTsdfManager<VectorT, TsdfT>::writeSliceData()
{
    auto slice_work = boost::any_cast<pair<pair<TsdfT*, size_t>, bool> >(m_sliceInQueue->Take());
    pair<TsdfT*, size_t > slice = slice_work.first;
    this->m_globalTsdfGrid->integrateSliceData(slice.first, slice.second);
    if (!slice_work.second) {
        writeSliceData();
    }
}

template<typename VectorT, typename TsdfT>
pair<float*, size_t> GlobalTsdfManager<VectorT, TsdfT>::getData(VectorT minBounds, VectorT maxBounds)
{
    lvr::BoundingBox<cVertex> boundingBox = lvr::BoundingBox<cVertex>(
            minBounds[0], minBounds[1], minBounds[2],
            maxBounds[0], maxBounds[1], maxBounds[2]
    );
    return m_globalTsdfGrid->getData(boundingBox);
}

template<typename VectorT, typename TsdfT>
void GlobalTsdfManager<VectorT, TsdfT>::saveMesh(string filename)
{
    // wait for writerThread to integrate last slice before starting reconstruction
    m_writerThread->join();

    m_globalTsdfGrid->saveMesh(filename);
}

template<typename VectorT, typename TsdfT>
GlobalTsdfManager<VectorT, TsdfT>::~GlobalTsdfManager()
{
    delete m_globalTsdfGrid;
}
