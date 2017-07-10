/**
 * GlobalTsdfManager.tcc
 *
 *  @date 14.06.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#include <kfusion/GlobalTsdfManager.hpp>

#include <chrono>
#include <iostream>

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
    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    this->m_globalTsdfGrid->integrateSliceData(slice.first, slice.second);
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    std::cout << "~~~~ Finished writeSliceData() after " << duration << " microseconds. ~~~~" << std::endl;
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
    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    pair<float*, size_t> data = m_globalTsdfGrid->getData(boundingBox);
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    std::cout << "~~~~ Finished getData() after " << duration << " microseconds. ~~~~" << std::endl;

    return data;
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
