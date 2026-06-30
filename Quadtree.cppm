module;

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

export module Kairo.Foundation.Spatial.Quadtree;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Spatial.Types;

export namespace kairo::foundation::spatial
{
    struct SpatialRect final
    {
        Vec2f Min = Vec2f::Zero();
        Vec2f Max = Vec2f::Zero();

        [[nodiscard]]
        bool IsValid() const noexcept
        {
            return Min.x <= Max.x && Min.y <= Max.y;
        }

        [[nodiscard]]
        bool Contains(
            const SpatialRect& other) const noexcept
        {
            return other.Min.x >= Min.x &&
                other.Max.x <= Max.x &&
                other.Min.y >= Min.y &&
                other.Max.y <= Max.y;
        }

        [[nodiscard]]
        bool Intersects(
            const SpatialRect& other) const noexcept
        {
            return Min.x <= other.Max.x &&
                Max.x >= other.Min.x &&
                Min.y <= other.Max.y &&
                Max.y >= other.Min.y;
        }
    };

    struct QuadtreeItem final
    {
        SpatialID ID = SpatialInvalidID;
        SpatialRect Bounds;
    };

    class Quadtree final
    {
    public:
        explicit Quadtree(
            SpatialRect worldBounds)
            : m_WorldBounds(worldBounds)
        {
            if (!worldBounds.IsValid())
            {
                throw std::invalid_argument("Quadtree failed: invalid world bounds.");
            }
        }

        void Insert(
            SpatialID id,
            SpatialRect bounds)
        {
            if (!bounds.IsValid())
            {
                throw std::invalid_argument("Quadtree::Insert failed: invalid bounds.");
            }

            Remove(id);
            m_Items.push_back({ id, bounds });
        }

        bool Remove(
            SpatialID id)
        {
            const auto found =
                std::remove_if(
                    m_Items.begin(),
                    m_Items.end(),
                    [id](const QuadtreeItem& item)
                    {
                        return item.ID == id;
                    });

            const bool removed = found != m_Items.end();
            m_Items.erase(found, m_Items.end());
            return removed;
        }

        [[nodiscard]]
        std::vector<SpatialID> QueryAABB(
            SpatialRect query) const
        {
            std::vector<SpatialID> result;

            for (const QuadtreeItem& item : m_Items)
            {
                if (item.Bounds.Intersects(query))
                {
                    result.push_back(item.ID);
                }
            }

            return result;
        }

        [[nodiscard]]
        const std::vector<QuadtreeItem>& Items() const noexcept
        {
            return m_Items;
        }

        void Clear()
        {
            m_Items.clear();
        }

    private:
        SpatialRect m_WorldBounds;
        std::vector<QuadtreeItem> m_Items;
    };
}
