/**
 * GlobalTsdfGrid.hpp
 *
 *  @date 27.01.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#ifndef GLOBALTSDFGRID_HPP
#define GLOBALTSDFGRID_HPP

#include "HashGrid.hpp"
#include "../../../ext/kintinuous/kfusion/include/kfusion/BlockingQueue.hpp"
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

namespace lvr
{
    template<typename VertexT, typename BoxT, typename TsdfT>
    class GlobalTsdfGrid : public HashGrid<VertexT, BoxT>
    {
    private:
        boost::shared_ptr<boost::thread> m_writerThread;
        boost::shared_ptr<BlockingQueue> m_sliceInQueue;

        size_t m_maxBufferIndexX;
        size_t m_maxBufferIndexY;
        size_t m_maxBufferIndexZ;
        /* count of buffer elements */
        size_t m_globalBufferSize;
        /* count of buffer elements != 0 */
        size_t m_insertedBufferElements;
        float* m_globalBuffer;

        void writeSliceData();
        bool integrateSliceData(TsdfT *tsdf, size_t size);
    public:
        // Typedef to alias iterators for box maps
        typedef typename unordered_map<size_t, BoxT *>::iterator box_map_it;

        GlobalTsdfGrid(size_t bufferSizeX, size_t bufferSizeY, size_t bufferSizeZ,
                       float cellSize, BoundingBox<VertexT> bb, bool isVoxelsize);
        pair<float*, size_t> getData(BoundingBox<VertexT> bb);
        bool addSliceToInQueue(TsdfT *tsdf, size_t size, bool last_shift);
        virtual void addLatticePoint(int index_x, int index_y, int index_z, float distance = 0);
        void saveMesh(string filename);
        void exportGlobalTSDFValues();
        ~GlobalTsdfGrid();
    };
}

#include "GlobalTsdfGrid.tcc"

#endif //GLOBALTSDFGRID_HPP
