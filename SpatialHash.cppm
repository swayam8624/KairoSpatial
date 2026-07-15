module;

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

export module Kairo.Foundation.Spatial.SpatialHash;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Spatial.Types;

export namespace std
{
    template<>
    struct hash<kairo::foundation::spatial::GridCoord3>
    {
        [[nodiscard]]
        std::size_t operator()(
            const kairo::foundation::spatial::GridCoord3& coord) const noexcept
        {
            const std::int64_t x = static_cast<std::int64_t>(coord.x) * 73856093;
            const std::int64_t y = static_cast<std::int64_t>(coord.y) * 19349663;
            const std::int64_t z = static_cast<std::int64_t>(coord.z) * 83492791;
            return static_cast<std::size_t>(x ^ y ^ z);
        }
    };
}

export namespace kairo::foundation::spatial
{
    class SpatialHash final
    {
    public:
        explicit SpatialHash(
            float cellSize)
            : m_CellSize(cellSize)
        {
            if (cellSize <= 0.0f)
            {
                throw std::invalid_argument("SpatialHash failed: cell size must be positive.");
            }
        }

        [[nodiscard]]
        GridCoord3 CellOf(
            const Vec3f& position) const noexcept
        {
            return
            {
                static_cast<int>(std::floor(position.x / m_CellSize)),
                static_cast<int>(std::floor(position.y / m_CellSize)),
                static_cast<int>(std::floor(position.z / m_CellSize))
            };
        }

        void Insert(
            SpatialID id,
            const Vec3f& position)
        {
            Insert(id, SpatialAABB::FromPoint(position));
        }

        void Insert(
            SpatialID id,
            const SpatialAABB& bounds)
        {
            if (!bounds.IsValid())
            {
                throw std::invalid_argument("SpatialHash::Insert failed: invalid bounds.");
            }

            Remove(id);

            const GridCoord3 minCell = CellOf(bounds.Min);
            const GridCoord3 maxCell = CellOf(bounds.Max);
            m_Bounds[id] = bounds;

            for (int z = minCell.z; z <= maxCell.z; ++z)
            {
                for (int y = minCell.y; y <= maxCell.y; ++y)
                {
                    for (int x = minCell.x; x <= maxCell.x; ++x)
                    {
                        GridCoord3 coord{ x, y, z };
                        m_Cells[coord].push_back(id);
                        m_IDCells[id].push_back(coord);
                    }
                }
            }
        }

        bool Remove(
            SpatialID id)
        {
            auto foundCells = m_IDCells.find(id);
            if (foundCells == m_IDCells.end())
            {
                return false;
            }

            for (const GridCoord3& coord : foundCells->second)
            {
                auto cell = m_Cells.find(coord);
                if (cell == m_Cells.end())
                {
                    continue;
                }

                std::vector<SpatialID>& values = cell->second;
                values.erase(
                    std::remove(values.begin(), values.end(), id),
                    values.end());

                if (values.empty())
                {
                    m_Cells.erase(coord);
                }
            }

            m_IDCells.erase(id);
            m_Bounds.erase(id);
            return true;
        }

        [[nodiscard]]
        std::vector<SpatialID> QueryCell(
            const GridCoord3& coord) const
        {
            auto found = m_Cells.find(coord);
            std::vector<SpatialID> result = found == m_Cells.end()
                ? std::vector<SpatialID>{}
                : found->second;

            std::sort(result.begin(), result.end());
            return result;
        }

        [[nodiscard]]
        std::vector<SpatialID> QueryAABB(
            const SpatialAABB& bounds) const
        {
            if (!bounds.IsValid())
            {
                throw std::invalid_argument("SpatialHash::QueryAABB failed: invalid bounds.");
            }

            std::unordered_set<SpatialID> unique;
            const GridCoord3 minCell = CellOf(bounds.Min);
            const GridCoord3 maxCell = CellOf(bounds.Max);

            for (int z = minCell.z; z <= maxCell.z; ++z)
            {
                for (int y = minCell.y; y <= maxCell.y; ++y)
                {
                    for (int x = minCell.x; x <= maxCell.x; ++x)
                    {
                        auto found = m_Cells.find({ x, y, z });
                        if (found == m_Cells.end())
                        {
                            continue;
                        }

                        for (SpatialID id : found->second)
                        {
                            auto boundsIt = m_Bounds.find(id);
                            if (boundsIt != m_Bounds.end() &&
                                Intersects(boundsIt->second, bounds))
                            {
                                unique.insert(id);
                            }
                        }
                    }
                }
            }

            std::vector<SpatialID> result(unique.begin(), unique.end());
            std::sort(result.begin(), result.end());
            return result;
        }

        [[nodiscard]]
        std::vector<SpatialID> QueryRadius(
            const Vec3f& center,
            float radius) const
        {
            if (radius < 0.0f)
            {
                return {};
            }

            SpatialAABB query =
                SpatialAABB::FromCenterExtent(
                    center,
                    Vec3f{ radius, radius, radius });

            std::vector<SpatialID> candidates =
                QueryAABB(query);

            candidates.erase(
                std::remove_if(
                    candidates.begin(),
                    candidates.end(),
                    [&](SpatialID id)
                    {
                        return !OverlapsSphere(m_Bounds.at(id), center, radius);
                    }),
                candidates.end());

            return candidates;
        }

        void Clear()
        {
            m_Cells.clear();
            m_IDCells.clear();
            m_Bounds.clear();
        }

    private:
        float m_CellSize = 1.0f;
        std::unordered_map<GridCoord3, std::vector<SpatialID>> m_Cells;
        std::unordered_map<SpatialID, std::vector<GridCoord3>> m_IDCells;
        std::unordered_map<SpatialID, SpatialAABB> m_Bounds;
    };
}
