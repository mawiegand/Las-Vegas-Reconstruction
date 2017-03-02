/**
 * GlobalTsdfGrid.hpp
 *
 *  @date 27.01.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#include "GlobalTsdfGrid.hpp"

#include <lvr/geometry/ColorVertex.hpp>
#include <lvr/reconstruction/FastKinFuBox.hpp>
#include <lvr/reconstruction/FastReconstruction.hpp>
#include <lvr/geometry/HalfEdgeKinFuMesh.hpp>

namespace lvr
{
    typedef ColorVertex<float, unsigned char> cVertex;
    typedef FastKinFuBox<ColorVertex<float, unsigned char>, lvr::Normal<float> > cFastBox;
    typedef FastReconstruction<ColorVertex<float, unsigned char>, lvr::Normal<float>, cFastBox> cFastReconstruction;
    typedef HalfEdgeKinFuMesh<cVertex, lvr::Normal<float> > HMesh;
    typedef HMesh *MeshPtr;

    template<typename VertexT, typename BoxT, typename TsdfT>
    GlobalTsdfGrid<VertexT, BoxT, TsdfT>::GlobalTsdfGrid(float cellSize, BoundingBox<VertexT> bb, bool isVoxelsize) :
            HashGrid<VertexT, BoxT>(cellSize, bb, isVoxelsize)
    {

    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    TsdfT* GlobalTsdfGrid<VertexT, BoxT, TsdfT>::getData(BoundingBox<VertexT> bb)
    {
        std::cout << "get data from global TSDF" << std::endl;

        int center_of_bb_x = (this->m_boundingBox.getXSize() / 2) / this->m_voxelsize;
        int center_of_bb_y = (this->m_boundingBox.getYSize() / 2) / this->m_voxelsize;
        int center_of_bb_z = (this->m_boundingBox.getZSize() / 2) / this->m_voxelsize;

        VertexT bbMin = bb.getMin();
        VertexT bbMax = bb.getMin();

        // calculate tsdf size
        size_t tsdfSize = (bbMax.x - bbMin.x) * (bbMax.y - bbMin.y) * (bbMax.z - bbMin.z);
        TsdfT tsdf[tsdfSize];

        size_t tsdfIndex = 0;
        // for each value in bounding box
        // TODO: determine required order by cyclical buffer
        for (int x = bbMin.x; x < bbMax.x; x++)
        {
            for (int y = bbMin.y; y < bbMax.y; y++)
            {
                for (int z = bbMin.z; z < bbMax.z; z++)
                {
                    // calculate hash value
                    size_t hash_value = this->hashValue(x, y, z);
                    // TODO: catch index out of bounce
                    int grid_index = this->m_qpIndices[hash_value];
                    QueryPoint<VertexT> qp = this->m_queryPoints[grid_index];
                    VertexT position = qp.m_position;
                    tsdf[tsdfIndex].x = position.x - center_of_bb_x;
                    tsdf[tsdfIndex].y = position.y - center_of_bb_y;
                    tsdf[tsdfIndex].z = position.z - center_of_bb_z;
                    tsdf[tsdfIndex].w = qp.m_distance;
                    tsdfIndex++;
                }
            }
        }

        return &tsdf;
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    bool GlobalTsdfGrid<VertexT, BoxT, TsdfT>::addSliceData(TsdfT *tsdf, size_t size)
    {
        cout << timestamp << "Started adding data to global TSDF " << "Values: " << size << endl;

        int center_of_bb_x = (this->m_boundingBox.getXSize() / 2) / this->m_voxelsize;
        int center_of_bb_y = (this->m_boundingBox.getYSize() / 2) / this->m_voxelsize;
        int center_of_bb_z = (this->m_boundingBox.getZSize() / 2) / this->m_voxelsize;

        //#pragma omp parallllell for
        int grid_index = 0;
        size_t last_size = this->m_queryPoints.size();
        this->m_queryPoints.resize(size + last_size);
        for (size_t i = 0; i < size; i++)
        {
            grid_index = i + last_size;
            // shift tsdf onto global grid
            int global_x = tsdf[i].x + center_of_bb_x;
            int global_y = tsdf[i].y + center_of_bb_y;
            int global_z = tsdf[i].z + center_of_bb_z;
            VertexT position(global_x, global_y, global_z);
            QueryPoint<VertexT> qp = QueryPoint<VertexT>(position, tsdf[i].w);
            this->m_queryPoints[grid_index] = qp;
            size_t hash_value = this->hashValue(global_x, global_y, global_z);
            this->m_qpIndices[hash_value] = grid_index;
        }
        this->m_globalIndex = grid_index + 1;
        // Iterator over all points, calc lattice indices and add lattice points to the grid
        for (size_t i = 0; i < size; i++)
        {
            int global_x = tsdf[i].x + center_of_bb_x;
            int global_y = tsdf[i].y + center_of_bb_y;
            int global_z = tsdf[i].z + center_of_bb_z;
            //#pragma omp task
            addLatticePoint(global_x, global_y, global_z, tsdf[i].w);
        }
        cout << timestamp << "Finished adding data to global TSDF" << endl;

        return true;
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    void GlobalTsdfGrid<VertexT, BoxT, TsdfT>::addLatticePoint(int index_x, int index_y, int index_z, float distance)
    {
        size_t hash_value;

        // Some iterators for hash map accesses
        box_map_it it;
        box_map_it neighbor_it;

        // Values for current and global indices. Current refers to a
        // already present query point, global index is id that the next
        // created query point will get
        unsigned int current_index = 0;

        int dx, dy, dz;

        // Get min and max vertex of the point clouds bounding box
        VertexT v_min = this->m_boundingBox.getMin();
        VertexT v_max = this->m_boundingBox.getMax();

        hash_value = this->hashValue(index_x, index_y, index_z);

        //Calculate box center .. useless
        VertexT box_center((index_x), (index_y), (index_z));

        //Create new box
        BoxT *box = new BoxT(box_center);
        vector<size_t> boxQps;
        boxQps.resize(8);
        vector<size_t> cornerHashs;
        cornerHashs.resize(8);
        std::vector<int> missingCorner;
        //Setup the box itself
        for (int k = 0; k < 8; k++)
        {
            //Find point in Grid
            dx = TSDFCreateTable[k][0];
            dy = TSDFCreateTable[k][1];
            dz = TSDFCreateTable[k][2];
            size_t corner_hash = this->hashValue(index_x + dx, index_y + dy, index_z + dz);
            auto qp_index_it = this->m_qpIndices.find(corner_hash);
            //If point exist, save index in box
            if (qp_index_it != this->m_qpIndices.end())
            {
                box->setVertex(k, qp_index_it->second);
                boxQps[k] = qp_index_it->second;
            }
            else
            {
                delete box;
                return;
            }
            cornerHashs[k] = corner_hash;
        }
        //Set pointers to the neighbors of the current box
        int neighbor_index = 0;
        size_t neighbor_hash = 0;

        for (int a = -1; a < 2; a++)
        {
            for (int b = -1; b < 2; b++)
            {
                for (int c = -1; c < 2; c++)
                {
                    //Calculate hash value for current neighbor cell
                    neighbor_hash = this->hashValue(index_x + a, index_y + b, index_z + c);

                    //Try to find this cell in the grid
                    neighbor_it = this->m_cells.find(neighbor_hash);

                    //If it exists, save pointer in box
                    if (neighbor_it != this->m_cells.end())
                    {
                        box->setNeighbor(neighbor_index, (*neighbor_it).second);
                        (*neighbor_it).second->setNeighbor(26 - neighbor_index, box);
                    }

                    neighbor_index++;
                }
            }
        }

        this->m_cells[hash_value] = box;
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    void GlobalTsdfGrid<VertexT, BoxT, TsdfT>::saveMesh(string filename)
    {
        //global_tsdf_->saveGrid("global_tsdf.grid");

        // create mesh
        MeshPtr meshPtr = new HMesh();
        cFastReconstruction *fast_recon = new cFastReconstruction(this);
        fast_recon->getMesh(*meshPtr);
        std::cout << "Global amount of vertices: " << meshPtr->meshSize() << std::endl;
        std::cout << "Global amount of faces: " << meshPtr->getFaces().size() << std::endl;
        meshPtr->finalize();
        ModelPtr m(new Model(meshPtr->meshBuffer()));

        // save mesh
        ModelFactory::saveModel(m, filename + ".ply");
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    GlobalTsdfGrid<VertexT, BoxT, TsdfT>::~GlobalTsdfGrid()
    {

    }
}