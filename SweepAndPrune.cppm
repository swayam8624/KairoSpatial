// TEACHING MODULE: Sweep-and-prune broadphase indexes projected AABB intervals and tests candidate overlaps in other axes. Temporal coherence can help; dense overlapping scenes may be expensive.
module;

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

export module Kairo.Foundation.Spatial.SweepAndPrune;

import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Spatial.Types;

export namespace kairo::foundation::spatial
{
    struct SAPEndpoint final
    {
        float Value = 0.0f;
        SpatialID ID = SpatialInvalidID;
        bool IsMin = true;
    };

    struct SAPProxy final
    {
        SpatialID ID = SpatialInvalidID;
        SpatialAABB Bounds = SpatialAABB::Empty();
        bool Active = true;
    };

    class SweepAndPrune final
    {
    public:
        /// TEACHING API: void Insert( — Add new active AABB proxy; broadphase only tracks box intervals, not original mesh/contact.
        void Insert(
            SpatialID id,
            const SpatialAABB& bounds)
        {
            if (!bounds.IsValid())
            {
                throw std::invalid_argument("SweepAndPrune::Insert failed: invalid bounds.");
            }

            m_Proxies.push_back({ id, bounds, true });
        }

        /// TEACHING API: void UpdateProxy( — Replace AABB coordinates for a stable ID; next sweep must recompute valid overlap candidates.
        void UpdateProxy(
            SpatialID id,
            const SpatialAABB& bounds)
        {
            if (!bounds.IsValid())
            {
                throw std::invalid_argument("SweepAndPrune::UpdateProxy failed: invalid bounds.");
            }

            for (SAPProxy& proxy : m_Proxies)
            {
                if (proxy.ID == id && proxy.Active)
                {
                    proxy.Bounds = bounds;
                    return;
                }
            }

            throw std::out_of_range("SweepAndPrune::UpdateProxy failed: id not found.");
        }

        bool Remove(
            SpatialID id)
        {
            for (SAPProxy& proxy : m_Proxies)
            {
                if (proxy.ID == id && proxy.Active)
                {
                    proxy.Active = false;
                    return true;
                }
            }

            return false;
        }

        [[nodiscard]]
        /// TEACHING API: std::vector<SpatialPair> ComputePairs( — Sort projected interval endpoints, check active interval pairs and other axes; broadphase complexity worsens when all bounds overlap.
        std::vector<SpatialPair> ComputePairs() const
        {
            std::vector<const SAPProxy*> sorted;
            sorted.reserve(m_Proxies.size());

            for (const SAPProxy& proxy : m_Proxies)
            {
                if (proxy.Active)
                {
                    sorted.push_back(&proxy);
                }
            }

            std::stable_sort(
                sorted.begin(),
                sorted.end(),
                [](const SAPProxy* lhs, const SAPProxy* rhs)
                {
                    return lhs->Bounds.Min.x < rhs->Bounds.Min.x;
                });

            std::vector<SpatialPair> pairs;

            for (std::size_t i = 0; i < sorted.size(); ++i)
            {
                const SAPProxy& a = *sorted[i];

                for (std::size_t j = i + 1; j < sorted.size(); ++j)
                {
                    const SAPProxy& b = *sorted[j];

                    if (b.Bounds.Min.x > a.Bounds.Max.x)
                    {
                        break;
                    }

                    if (Intersects(a.Bounds, b.Bounds))
                    {
                        pairs.push_back(OrderedPair(a.ID, b.ID));
                    }
                }
            }

            return pairs;
        }

        [[nodiscard]]
        const std::vector<SAPProxy>& Proxies() const noexcept
        {
            return m_Proxies;
        }

        void Clear()
        {
            m_Proxies.clear();
        }

    private:
        std::vector<SAPProxy> m_Proxies;
    };
}
