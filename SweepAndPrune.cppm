// TEACHING MODULE: Sweep-and-prune is an alternative broadphase to the
// dynamic tree used by PhysicsEngine. This implementation stores active
// AABB proxies, sorts them by x-min on *each* ComputePairs call, then
// checks y/z overlap via full Intersects for still-overlapping x ranges.
// It does not persist an incrementally sorted endpoint list across frames;
// temporal coherence is a potential optimization, not a current guarantee.
//
// Under sparse x-axis arrangements, inner loops break early. If most boxes
// overlap along x, many candidate checks remain (quadratic worst case).
// Sorted bounding candidates are not actual shape contact manifolds.
// SpatialID is the caller's identity; a SAPProxy owns no physics body.
// See DynamicAABBTree.cppm, PhysicsEngine/Broadphase.cppm and SpatialTests.
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
    // Endpoint record available for interval-oriented algorithms.
    // Current ComputePairs instead sorts pointers to SAPProxy bounds by
    // minimum x; it does not construct an endpoint event sweep.
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
        // The API checks bounds but does not reject duplicate SpatialID
        // values here. The calling system must maintain identity policy.
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

        // Logically marks the first active matching ID inactive; the
        // record stays in m_Proxies until Clear. Iteration skips it.
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
        // Each call filters active proxies and sorts their x minima.
        // Compare this exact cost model with DynamicAABBTree::ComputePairs
        // before interpreting a sparse/dense benchmark.
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

                    // Because minima are sorted, no later proxy can
                    // overlap a's x interval after this condition.
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
