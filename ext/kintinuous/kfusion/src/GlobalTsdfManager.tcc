/**
 * GlobalTsdfManager.tcc
 *
 *  @date 14.06.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#include <kfusion/GlobalTsdfManager.hpp>

#define DEBUG 0

template<typename VertexT, typename BoxT, typename TsdfT>
GlobalTsdfManager<VertexT, BoxT, TsdfT>::GlobalTsdfManager(float cellSize, bool isVoxelsize, kfusion::Options* options) :
        m_options(options)
{
    this->m_sliceInQueue = boost::shared_ptr<BlockingQueue>(new BlockingQueue());
    this->m_writerThread = boost::shared_ptr<boost::thread>(new boost::thread(
            boost::bind(&GlobalTsdfManager<VertexT, BoxT, TsdfT>::writeSliceData, this)
    ));

    lvr::BoundingBox<cVertex> bbox_ = lvr::BoundingBox<cVertex>(0.0, 0.0, 0.0, 300.0, 300.0, 300.0);
    bbox_.expand(300.0, 300.0, 300.0);

    this->m_globalTsdfGrid = new GGrid(cellSize, bbox_, isVoxelsize, options);
}

template<typename VertexT, typename BoxT, typename TsdfT>
bool GlobalTsdfManager<VertexT, BoxT, TsdfT>::addSliceToInQueue(TsdfT *tsdf, size_t size, bool last_shift)
{
    m_sliceInQueue->Add(pair<pair<TsdfT*, size_t>, bool>(pair<TsdfT*, size_t>(tsdf, size), last_shift));
}

template<typename VertexT, typename BoxT, typename TsdfT>
void GlobalTsdfManager<VertexT, BoxT, TsdfT>::writeSliceData()
{
    auto slice_work = boost::any_cast<pair<pair<TsdfT*, size_t>, bool> >(m_sliceInQueue->Take());
    pair<TsdfT*, size_t > slice = slice_work.first;
    this->m_globalTsdfGrid->integrateSliceData(slice.first, slice.second);
    if (!slice_work.second) {
        writeSliceData();
    }
}

template<typename VertexT, typename BoxT, typename TsdfT>
pair<float*, size_t> GlobalTsdfManager<VertexT, BoxT, TsdfT>::getData(lvr::BoundingBox<VertexT> bb)
{
    return m_globalTsdfGrid->getData(bb);
}

template<typename VertexT, typename BoxT, typename TsdfT>
void GlobalTsdfManager<VertexT, BoxT, TsdfT>::saveMesh(string filename)
{
    // wait for writerThread to integrate last slice before starting reconstruction
    m_writerThread->join();

    m_globalTsdfGrid->saveMesh(filename);
}

template<typename VertexT, typename BoxT, typename TsdfT>
GlobalTsdfManager<VertexT, BoxT, TsdfT>::~GlobalTsdfManager()
{

}
