/**
 * GlobalTsdfGrid.hpp
 *
 *  @date 27.01.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#ifndef GLOBALTSDFGRID_HPP
#define GLOBALTSDFGRID_HPP

#include "HashGrid.hpp"
#include <kfusion/Options.hpp>
#include <lvr/geometry/ColorVertex.hpp>
#include <lvr/geometry/HalfEdgeKinFuMesh.hpp>

namespace lvr
{
    typedef ColorVertex<float, unsigned char> cVertex;
    typedef HalfEdgeKinFuMesh<cVertex, lvr::Normal<float> > HMesh;
    typedef HMesh *MeshPtr;

    template<typename VertexT, typename BoxT, typename TsdfT>
    class GlobalTsdfGrid : public HashGrid<VertexT, BoxT>
    {
    private:
        kfusion::Options *options_;

        size_t m_maxBufferIndexX;
        size_t m_maxBufferIndexY;
        size_t m_maxBufferIndexZ;
        /* count of buffer elements */
        size_t m_globalBufferSize;
        /* count of buffer elements != 0 */
        size_t m_insertedBufferElements;
        float* m_globalBuffer;

        void transferBufferToHashGrid();
        void transformMeshBack(MeshPtr mesh, const double camera_target_distance);
        void optimizeMesh(MeshPtr meshPtr);
    public:
        // Typedef to alias iterators for box maps
        typedef typename unordered_map<size_t, BoxT *>::iterator box_map_it;

        GlobalTsdfGrid(float cellSize, BoundingBox<VertexT> bb, bool isVoxelsize,
                       kfusion::Options* options);
        pair<float*, size_t> getData(BoundingBox<VertexT> bb);
        bool integrateSliceData(TsdfT *tsdf, size_t size);
        virtual void addLatticePoint(int index_x, int index_y, int index_z, float distance = 0);
        void saveMesh(string filename, const double camera_target_distance);
        void exportGlobalTSDFValues();
        ~GlobalTsdfGrid();
    };
}

#include "GlobalTsdfGrid.tcc"

#endif //GLOBALTSDFGRID_HPP
