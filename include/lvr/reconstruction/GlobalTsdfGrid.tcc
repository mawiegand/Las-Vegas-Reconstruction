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
        m_sliceInQueue = boost::shared_ptr<BlockingQueue>(new BlockingQueue());
        m_writerThread = boost::shared_ptr<boost::thread>(new boost::thread(
                boost::bind(&GlobalTsdfGrid<VertexT, BoxT, TsdfT>::writeSliceData, this)
                ));
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    pair<TsdfT*, size_t> GlobalTsdfGrid<VertexT, BoxT, TsdfT>::getData(BoundingBox<VertexT> bb)
    {
        std::cout << "get data from global TSDF" << std::endl;

        int center_of_bb_x = (this->m_boundingBox.getXSize() / 2) / this->m_voxelsize;
        int center_of_bb_y = (this->m_boundingBox.getYSize() / 2) / this->m_voxelsize;
        int center_of_bb_z = (this->m_boundingBox.getZSize() / 2) / this->m_voxelsize;

        VertexT bbMin = bb.getMin();
        VertexT bbMax = bb.getMax();

        // calculate tsdf size
        int stepSize = 5;
        size_t tsdfSize = ((int) abs(bbMax.x - bbMin.x + 1) / stepSize) * ((int) abs(bbMax.y - bbMin.y + 1) / stepSize) * ((int) abs(bbMax.z - bbMin.z + 1) / stepSize);
        cout << timestamp << "Started getting data from global TSDF Values: " << tsdfSize << endl;
        cv::Mat tsdfValues(1, tsdfSize, CV_32FC4);

        TsdfT* tsdf = tsdfValues.ptr<TsdfT>();

        if (tsdf != NULL)
        {
            size_t tsdfIndex = 0;

            // TODO: determine required order by cyclical buffer
            /* TODO: check hash loop
            // calculate hash values of bounding box
            size_t minHashValue = this->hashValue(bbMin.x, bbMin.y, bbMin.z);
            size_t maxHashValue = this->hashValue(bbMax.x, bbMax.y, bbMax.z);
            // for each value in bounding box
            for (size_t hash = minHashValue; hash <= maxHashValue; hash++)
            {
                // TODO: catch index out of bounce
                int gridIndex = this->m_qpIndices[hash];
                QueryPoint<VertexT> qp = this->m_queryPoints[gridIndex];
                VertexT position = qp.m_position;
                tsdf[tsdfIndex].x = position.x - center_of_bb_x;
                tsdf[tsdfIndex].y = position.y - center_of_bb_y;
                tsdf[tsdfIndex].z = position.z - center_of_bb_z;
                tsdf[tsdfIndex].w = qp.m_distance;
                tsdfIndex++;
            }*/

//            /* test sphere */
//            bool debug = true;
//            VertexT center((bbMax.x - bbMin.x) / 2, (bbMax.y - bbMin.y) / 2, (bbMax.z - bbMin.z) / 2);
//            float radius = 2.f;
//
//            std::ofstream sphereFile;
//            if (debug)
//            {
//                static char sphereFileName[26];
//                time_t now = time(0);
//                strftime(sphereFileName, sizeof(sphereFileName), "sphere_%Y%m%d_%H%M%S.3d", localtime(&now));
//                sphereFile.open(sphereFileName);
//            }
//
//            //#pragma omp parallel for
//            for (int z = (int) (bbMin.z + stepSize - 1); z <= (int) bbMax.z; z += stepSize)
//            {
//                for (int y = (int) (bbMin.y + stepSize - 1); y <= (int) bbMax.y; y += stepSize)
//                {
//                    for (int x = (int) (bbMin.x + stepSize - 1); x <= (int) bbMax.x; x += stepSize)
//                    {
//                        float distance = sqrt(pow((x - center.x), 2) * this->m_voxelsize
//                                              + pow((y - center.y), 2) * this->m_voxelsize
//                                              + pow((z - center.z), 2) * this->m_voxelsize)
//                                         - radius;
//                        tsdf[tsdfIndex].x = x;
//                        tsdf[tsdfIndex].y = y;
//                        tsdf[tsdfIndex].z = z;
//                        tsdf[tsdfIndex].w = (distance >= -1.f && distance <= 1.f) ? distance : 0.f;
//
//                        if (debug)
//                        {
//                            if (distance <= 1.f)
//                            {
//                                sphereFile << x
//                                           << " " << y
//                                           << " " << z;
//                            }
//                            if (distance >= 0.f && distance <= 1.f)
//                            {
//                                sphereFile << " " << 255
//                                           << " " << 0
//                                           << " " << 0
//                                           << std::endl;
//                            }
//                            else if (distance >= -1.f && distance < 0.f)
//                            {
//                                sphereFile << " " << 0
//                                           << " " << 255
//                                           << " " << 0
//                                           << std::endl;
//                            }
//                            else if (distance < -1.f)
//                            {
//                                sphereFile << " " << 255
//                                           << " " << 255
//                                           << " " << 255
//                                           << std::endl;
//                            }
//                        }
//                        tsdfIndex++;
//                    }
//                }
//            }
//            if (debug)
//            {
//                sphereFile.close();
//            }
//            /* test sphere */

            if (this->m_qpIndices.size() > 0)
            {
                static char fileName[23];
                time_t now = time(0);
                strftime(fileName, sizeof(fileName), "get_%Y%m%d_%H%M%S.3d", localtime(&now));

                std::ofstream file;
                file.open(fileName);

                //#pragma omp parallel for
                for(int x = (int) (bbMin.x + center_of_bb_x  + stepSize - 1); x <= (int) (bbMax.x + center_of_bb_x); x += stepSize)
                {
                    for (int y = (int) (bbMin.y + center_of_bb_y + stepSize - 1); y <= (int) (bbMax.y + center_of_bb_y); y += stepSize)
                    {
                        for (int z = (int) (bbMin.z + center_of_bb_z + stepSize - 1); z <= (int) (bbMax.z + center_of_bb_z); z += stepSize)
                        {
                            // TODO: catch index out of bounce
                            size_t hash = this->hashValue(x, y, z);
                            try
                            {
                                size_t gridIndex = this->m_qpIndices.at(hash);
                                QueryPoint<VertexT> qp = this->m_queryPoints[gridIndex];
                                VertexT position = qp.m_position;
                                tsdf[tsdfIndex].x = position.x - center_of_bb_x;
                                tsdf[tsdfIndex].y = position.y - center_of_bb_y;
                                tsdf[tsdfIndex].z = position.z - center_of_bb_z;
                                tsdf[tsdfIndex].w = qp.m_distance;
                            }
                            catch (const std::out_of_range &oor)
                            {
                                tsdf[tsdfIndex].x = x - center_of_bb_x;
                                tsdf[tsdfIndex].y = y - center_of_bb_y;
                                tsdf[tsdfIndex].z = z - center_of_bb_z;
                                tsdf[tsdfIndex].w = 0.f;
                            }
                            file << tsdf[tsdfIndex].x
                                 << " " << tsdf[tsdfIndex].y
                                 << " " << tsdf[tsdfIndex].z;
                            if (tsdf[tsdfIndex].w > 0.f )
                            {
                                file << " " << 0 << " " << 0 << " " << 255;
                            }
                            else
                            {
                                file << " " << 255 << " " << 0 << " " << 0;
                            }
                            file << std::endl;
                            tsdfIndex++;
                        }
                    }
                }
                file.close();
            }
        }
        cout << timestamp << "Finished getting data from global TSDF" << endl;

        return pair<TsdfT*, size_t>(tsdf, tsdfSize);
    }

    template<typename VertexT, typename BoxT, typename TsdfT>
    bool GlobalTsdfGrid<VertexT, BoxT, TsdfT>::addSliceToInQueue(TsdfT *tsdf, size_t size, bool last_shift)
    {
        m_sliceInQueue->Add(pair<pair<TsdfT*, size_t>, bool>(pair<TsdfT*, size_t>(tsdf, size), last_shift));
    };

    template<typename VertexT, typename BoxT, typename TsdfT>
    void GlobalTsdfGrid<VertexT, BoxT, TsdfT>::writeSliceData()
    {
        auto slice_work = boost::any_cast<pair<pair<TsdfT*, size_t>, bool> >(m_sliceInQueue->Take());
        pair<TsdfT*, size_t > slice = slice_work.first;
        integrateSliceData(slice.first, slice.second);
        if (!slice_work.second) {
            writeSliceData();
        }
    };

    template<typename VertexT, typename BoxT, typename TsdfT>
    bool GlobalTsdfGrid<VertexT, BoxT, TsdfT>::integrateSliceData(TsdfT *tsdf, size_t size)
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
        // wait for writerThread to integrate last slice before starting reconstruction
        m_writerThread->join();

        //global_tsdf_->saveGrid("global_tsdf.grid");

        // if debugging enabled
        exportGlobalTSDFValues();

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
    }
}