/**
 * GlobalTsdfGrid.hpp
 *
 *  @date 27.01.2017
 *  @author Marcel Wiegand <marcel@bluepanel.org>
 */

#include "GlobalTsdfGrid.hpp"

namespace lvr
{
	template<typename VertexT, typename BoxT, typename TsdfT>
	GlobalTsdfGrid<VertexT, BoxT, TsdfT>::GlobalTsdfGrid(float cellSize,  BoundingBox<VertexT> bb, bool isVoxelsize) :
			HashGrid<VertexT, BoxT>(cellSize, bb, isVoxelsize)
	{

	}

	template<typename VertexT, typename BoxT, typename TsdfT>
	void GlobalTsdfGrid<VertexT, BoxT, TsdfT>::getData(BoundingBox<VertexT> bb)
	{
		std::cout << "get data from global TSDF" << std::endl;
		return;
	}

	template<typename VertexT, typename BoxT, typename TsdfT>
	bool GlobalTsdfGrid<VertexT, BoxT, TsdfT>::addData(BoundingBox<VertexT> bb, TsdfT* tsdf, size_t size,
						    int shiftX, int shiftY, int shiftZ,
						    int backShiftX, int backShiftY, int backShiftZ)
	{
		std::cout << "adding data to global TSDF" << std::endl;
		return true;
	}

	template<typename VertexT, typename BoxT, typename TsdfT>
	GlobalTsdfGrid<VertexT, BoxT, TsdfT>::~GlobalTsdfGrid()
	{

	}
}