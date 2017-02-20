/**
 * GlobalTsdfGrid.hpp
 *
 *  @date 27.01.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#ifndef GLOBALTSDFGRID_HPP
#define GLOBALTSDFGRID_HPP

#include "HashGrid.hpp"

namespace lvr
{
    template<typename VertexT, typename BoxT, typename TsdfT>
    class GlobalTsdfGrid : public HashGrid<VertexT, BoxT>
    {
    public:
        // Typedef to alias iterators for box maps
        typedef typename unordered_map<size_t, BoxT *>::iterator box_map_it;

        GlobalTsdfGrid(float cellSize, BoundingBox<VertexT> bb, bool isVoxelsize);
        void getData(BoundingBox<VertexT> bb);
        bool addSliceData(TsdfT *tsdf, size_t size);
        virtual void addLatticePoint(int index_x, int index_y, int index_z, float distance = 0);
        ~GlobalTsdfGrid();
    };
}

#include "GlobalTsdfGrid.tcc"

#endif //GLOBALTSDFGRID_HPP
