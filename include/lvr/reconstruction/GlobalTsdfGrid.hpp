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
		GlobalTsdfGrid(float cellSize,  BoundingBox<VertexT> bb, bool isVoxelsize);
		void getData(BoundingBox<VertexT> bb); // params: boundingBox; return data;
		bool addData(BoundingBox<VertexT> bb, TsdfT* tsdf, size_t size,
			     int shiftX, int shiftY, int shiftZ,
			     int backShiftX, int backShiftY, int backShiftZ); // params: boundingBox, data; return bool
		~GlobalTsdfGrid();
	};
}

#include "GlobalTsdfGrid.tcc"

#endif //GLOBALTSDFGRID_HPP
