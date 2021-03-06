/**
 * GlobalTsdfGrid.tcc
 *
 *  @date 27.01.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#include "GlobalTsdfGrid.hpp"

#include <lvr/reconstruction/FastKinFuBox.hpp>
#include <lvr/reconstruction/FastReconstruction.hpp>

#define TSDFDEBUG 0

namespace lvr
{
    typedef FastKinFuBox<ColorVertex<float, unsigned char>, lvr::Normal<float> > cFastBox;
    typedef FastReconstruction<ColorVertex<float, unsigned char>, lvr::Normal<float>, cFastBox> cFastReconstruction;

    template<typename VertexT, typename BoxT, typename TsdfT>
    GlobalTsdfGrid<VertexT, BoxT, TsdfT>::GlobalTsdfGrid(float cellSize, BoundingBox<VertexT> bb, bool isVoxelsize,
                                                         kfusion::Options* options) :
            HashGrid<VertexT, BoxT>(cellSize, bb, isVoxelsize),
            options_(options)
    {
        this->m_maxBufferIndexX = this->options_->getMaxBufferIndexX();
        this->m_maxBufferIndexY = this->options_->getMaxBufferIndexY();
        this->m_maxBufferIndexZ = this->options_->getMaxBufferIndexZ();
        this->m_globalBufferSize = (this->m_maxBufferIndexX + 1) * (this->m_maxBufferIndexY + 1) * (this->m_maxBufferIndexZ + 1);
        this->m_globalBuffer = new float[this->m_globalBufferSize];
        this->m_insertedBufferElements = 0;
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    pair<float*, size_t> GlobalTsdfGrid<VertexT, BoxT, TsdfT>::getData(BoundingBox<VertexT> bb)
    {
        std::cout << "get data from global TSDF" << std::endl;

        size_t centerOfX = this->m_maxBufferIndexX / 2;
        size_t centerOfY = this->m_maxBufferIndexY / 2;
        size_t centerOfZ = this->m_maxBufferIndexZ / 2;

        size_t gloablStepSizeY = this->m_maxBufferIndexX + 1;
        size_t gloablStepSizeZ = (this->m_maxBufferIndexY + 1) * gloablStepSizeY;

        VertexT bbMin = bb.getMin();
        VertexT bbMax = bb.getMax();

        size_t yStep = abs(bbMax.x - bbMin.x + 1);
        size_t zStep = abs(bbMax.y - bbMin.y + 1) * yStep;

        // calculate tsdf size
        size_t tsdfSize = abs(bbMax.x - bbMin.x + 1) * abs(bbMax.y - bbMin.y + 1) * abs(bbMax.z - bbMin.z + 1);
        float* tsdf = new float[tsdfSize];

#if TSDFDEBUG
        std::ofstream file;
        static char fileName[23];
        time_t now = time(0);
        strftime(fileName, sizeof(fileName), "get_%Y%m%d_%H%M%S.3d", localtime(&now));
        file.open(fileName);
#endif

        cout << timestamp << "Started getting data from global TSDF Values: " << tsdfSize << endl;

        size_t globalOffsetX = (centerOfX + bbMin.x);

//        #pragma omp parallel for
//        for (size_t z = (size_t)(centerOfZ + bbMin.z); z <= (size_t) (centerOfZ + bbMax.z); ++z)
        for (size_t z = (centerOfZ + bbMin.z); z <= (centerOfZ + bbMax.z); ++z)
        {
            for (size_t y = (centerOfY + bbMin.y); y <= (centerOfY + bbMax.y); ++y)
            {
                size_t hash = z * gloablStepSizeZ + y * gloablStepSizeY + globalOffsetX;
                memcpy(
                        &tsdf[(size_t)((z - centerOfZ - bbMin.z) * zStep + (y - centerOfY - bbMin.y) * yStep)],
                        &(this->m_globalBuffer[hash]),
                        yStep * sizeof(float)
                );

#if TSDFDEBUG
                for (size_t x = (centerOfX + bbMin.x); x <= (centerOfX + bbMax.x); ++x)
                {
                    float distance = tsdf[(size_t)((z - centerOfZ - bbMin.z) * zStep + (y - centerOfY - bbMin.y) * yStep + (x - centerOfX - bbMin.x))];
                    if (distance != 0.f)
                    {
                        file << x - centerOfX << " " << y - centerOfY << " " << z - centerOfZ;
                        if (distance > 0.f)
                        {
                            file << " " << 0 << " " << 0 << " " << 255;
                        }
                        else if (distance < 0.f)
                        {
                            file << " " << 255 << " " << 0 << " " << 0;
                        }
                        file << std::endl;
                    }
                }
#endif
            }
        }
        cout << timestamp << "Finished getting data from global TSDF" << endl;

#if TSDFDEBUG
        file.close();
#endif

        return pair<float*, size_t>(tsdf, tsdfSize);
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    bool GlobalTsdfGrid<VertexT, BoxT, TsdfT>::integrateSliceData(TsdfT *tsdf, size_t size)
    {
#if TSDFDEBUG
        int center_of_bb_x = (this->m_boundingBox.getXSize() / 2) / this->m_voxelsize;
        int center_of_bb_y = (this->m_boundingBox.getYSize() / 2) / this->m_voxelsize;
        int center_of_bb_z = (this->m_boundingBox.getZSize() / 2) / this->m_voxelsize;

        std::ofstream file;
        static char fileName[23];
        time_t now = time(0);
        strftime(fileName, sizeof(fileName), "int_%Y%m%d_%H%M%S.3d", localtime(&now));
        file.open(fileName);
#endif

        cout << timestamp << "Started adding data to global TSDF " << "Values: " << size << endl;

        size_t centerOfX = this->m_maxBufferIndexX / 2;
        size_t centerOfY = this->m_maxBufferIndexY / 2;
        size_t centerOfZ = this->m_maxBufferIndexZ / 2;

        size_t yStepSize = this->m_maxBufferIndexX + 1;
        size_t zStepSize = (this->m_maxBufferIndexY + 1) * yStepSize;

        for (size_t i = 0; i < size; i++)
        {
            // shift tsdf onto global grid
            size_t globalX = tsdf[i].x + centerOfX;
            size_t globalY = tsdf[i].y + centerOfY;
            size_t globalZ = tsdf[i].z + centerOfZ;

            if (globalX > this->m_maxBufferIndexX || globalY > this->m_maxBufferIndexY || globalZ > this->m_maxBufferIndexZ)
            {
                std::cout << "Value out of buffer index range! Skipping integration of value with ("
                          << globalX << ", " << globalY << ", " << globalZ << "). Max index is ("
                          << this->m_maxBufferIndexX << ", " << this->m_maxBufferIndexY << ", "
                          << this->m_maxBufferIndexZ << ")." << std::endl;
                continue;
            }

            size_t bufferIndex = globalZ * zStepSize + globalY * yStepSize + globalX;
            this->m_globalBuffer[bufferIndex] = tsdf[i].w;

            this->m_insertedBufferElements++;
            if (tsdf[i].w != 0.f)
            {
#if TSDFDEBUG
                file << tsdf[i].x << " " << tsdf[i].y << " " << tsdf[i].z;
                if (tsdf[i].w > 0.f)
                {
                    file << " " << 0 << " " << 0 << " " << 255;
                }
                else if (tsdf[i].w < 0.f)
                {
                    file << " " << 255 << " " << 0 << " " << 0;
                }
                file << std::endl;
#endif
            }
        }
        cout << timestamp << "Finished adding data to global TSDF" << endl;

#if TSDFDEBUG
        file.close();
#endif

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
    void GlobalTsdfGrid<VertexT, BoxT, TsdfT>::transferBufferToHashGrid()
    {
        cout << timestamp << "Transforming data" << "Values: " << this->m_globalBufferSize << endl;

        int center_of_bb_x = (this->m_boundingBox.getXSize() / 2) / this->m_voxelsize;
        int center_of_bb_y = (this->m_boundingBox.getYSize() / 2) / this->m_voxelsize;
        int center_of_bb_z = (this->m_boundingBox.getZSize() / 2) / this->m_voxelsize;

        size_t centerOfBufferX = this->m_maxBufferIndexX / 2;
        size_t centerOfBufferY = this->m_maxBufferIndexY / 2;
        size_t centerOfBufferZ = this->m_maxBufferIndexZ / 2;

        size_t gloablStepSizeY = this->m_maxBufferIndexX + 1;
        size_t gloablStepSizeZ = (this->m_maxBufferIndexY + 1) * gloablStepSizeY;

        size_t last_size = this->m_queryPoints.size();
        int grid_index = last_size;
        this->m_queryPoints.resize(this->m_insertedBufferElements + last_size);
        for (size_t z = 0; z <= this->m_maxBufferIndexZ; ++z)
        {
            for (size_t y = 0; y <= this->m_maxBufferIndexY; ++y)
            {
                for (size_t x = 0; x <= this->m_maxBufferIndexX; ++x)
                {
                    size_t hash = z * gloablStepSizeZ + y * gloablStepSizeY + x;
                    float tsdfValue = this->m_globalBuffer[hash];
                    if (tsdfValue != 0.f)
                    {
                        // shift tsdf onto global grid
                        int global_x = x - centerOfBufferX + center_of_bb_x;
                        int global_y = y - centerOfBufferY + center_of_bb_y;
                        int global_z = z - centerOfBufferZ + center_of_bb_z;
                        VertexT position(global_x, global_y, global_z);
                        QueryPoint<VertexT> qp = QueryPoint<VertexT>(position, tsdfValue);
                        this->m_queryPoints[grid_index] = qp;
                        size_t hash_value = this->hashValue(global_x, global_y, global_z);
                        this->m_qpIndices[hash_value] = grid_index;
                        grid_index++;
                    }
                }
            }
        }
        this->m_globalIndex = grid_index + 1;
        for (size_t z = 0; z <= this->m_maxBufferIndexZ; ++z)
        {
            for (size_t y = 0; y <= this->m_maxBufferIndexY; ++y)
            {
                for (size_t x = 0; x <= this->m_maxBufferIndexX; ++x)
                {
                    size_t hash = z * gloablStepSizeZ + y * gloablStepSizeY + x;
                    float tsdfValue = this->m_globalBuffer[hash];
                    if (tsdfValue != 0.f)
                    {
                        // shift tsdf onto global grid
                        int global_x = x - centerOfBufferX + center_of_bb_x;
                        int global_y = y - centerOfBufferY + center_of_bb_y;
                        int global_z = z - centerOfBufferZ + center_of_bb_z;
                        addLatticePoint(global_x, global_y, global_z, tsdfValue);
                    }
                }
            }
        }
        cout << timestamp << "Finished transforming data" << endl;
        delete[] this->m_globalBuffer;
        this->m_globalBuffer = NULL;
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    void GlobalTsdfGrid<VertexT, BoxT, TsdfT>::optimizeMesh(MeshPtr meshPtr)
    {
        if (meshPtr == NULL)
        {
            return;
        }
        cout << timestamp << "Started Mesh Optimization." << endl;
        meshPtr->fillHoles(options_->getFillHoles());
        if(options_->getDepth())
        {
            meshPtr->setDepth(options_->getDepth());
        }
        meshPtr->setClassifier(options_->getClassifier());
        meshPtr->optimizePlanes(options_->getPlaneIterations(),
                                options_->getNormalThreshold(),
                                options_->getMinPlaneSize(),
                                options_->getSmallRegionThreshold(), false);
//        meshPtr->retesselateInHalfEdge(options_->getLineFusionThreshold(), options_->textures(), 0);
//            if(tmp_pointer == NULL)
//                return;
        meshPtr->restorePlanes(options_->getMinPlaneSize());
        cout << timestamp << "Finished Mesh Optimization" << endl;

        std::cout << "Global amount of vertices after Optimization: " << meshPtr->meshSize() << std::endl;
        std::cout << "Global amount of faces after Optimization: " << meshPtr->getFaces().size() << std::endl;
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    void GlobalTsdfGrid<VertexT, BoxT, TsdfT>::saveMesh(string filename, const double camera_target_distance)
    {
        //global_tsdf_->saveGrid("global_tsdf.grid");

        transferBufferToHashGrid();

#if TSDFDEBUG
        exportGlobalTSDFValues();
#endif

        // create mesh
        MeshPtr meshPtr = new HMesh();
        cFastReconstruction *fast_recon = new cFastReconstruction(this);
        fast_recon->getMesh(*meshPtr);
        transformMeshBack(meshPtr, camera_target_distance);
        std::cout << "Global amount of vertices: " << meshPtr->meshSize() << std::endl;
        std::cout << "Global amount of faces: " << meshPtr->getFaces().size() << std::endl;

        if (options_->optimizePlanes())
        {
            optimizeMesh(meshPtr);
        }

        cout << timestamp << "Started saving Mesh." << endl;
        meshPtr->finalize();
        ModelPtr m(new Model(meshPtr->meshBuffer()));

        // save mesh
        ModelFactory::saveModel(m, filename + ".ply");
        cout << timestamp << "Finished saving Mesh." << endl;
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    void GlobalTsdfGrid<VertexT, BoxT, TsdfT>::transformMeshBack(MeshPtr mesh, const double camera_target_distance)
    {
        for(auto vert : mesh->getVertices())
        {
            // calc in voxel
            vert->m_position.x *= this->m_voxelsize;
            vert->m_position.y *= this->m_voxelsize;
            vert->m_position.z *= this->m_voxelsize;
            //offset for cube coord to center coord
            vert->m_position.x -= 1.5;
            vert->m_position.y -= 1.5;
            vert->m_position.z -= 1.5 - camera_target_distance;

            //offset for cube coord to center coord
            vert->m_position.x -= 150;
            vert->m_position.y -= 150;
            vert->m_position.z -= 150;
        }
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    void GlobalTsdfGrid<VertexT, BoxT, TsdfT>::exportGlobalTSDFValues()
    {
        std::cout << "Started exporting global TSDF Values. Amount of points: " << this->m_qpIndices.size() << std::endl;

        int center_of_bb_x = (this->m_boundingBox.getXSize() / 2) / this->m_voxelsize;
        int center_of_bb_y = (this->m_boundingBox.getYSize() / 2) / this->m_voxelsize;
        int center_of_bb_z = (this->m_boundingBox.getZSize() / 2) / this->m_voxelsize;

        static char fileName[26];
        time_t now = time(0);
        strftime(fileName, sizeof(fileName), "global_%Y%m%d_%H%M%S.3d", localtime(&now));

        std::ofstream globalFile;
        globalFile.open(fileName);
        for ( std::pair<size_t, size_t> index : this->m_qpIndices)
        {
            int gridIndex = index.second;
            QueryPoint<VertexT> qp = this->m_queryPoints[gridIndex];
            VertexT position = qp.m_position;
            globalFile << position.x - center_of_bb_x
                       << " " << position.y - center_of_bb_y
                       << " " << position.z - center_of_bb_z;
            if (qp.m_distance > 0.f )
            {
                globalFile << " " << 0 << " " << 255 << " " << 0;
            }
            else
            {
                globalFile << " " << 0 << " " << 255 << " " << 255;
            }
            globalFile << std::endl;
        }
        globalFile.close();

        std::cout << timestamp << "Finished exporting global TSDF values" << endl;
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    GlobalTsdfGrid<VertexT, BoxT, TsdfT>::~GlobalTsdfGrid()
    {
        if (this->m_globalBuffer != NULL)
        {
            delete[] this->m_globalBuffer;
        }
    }
}
