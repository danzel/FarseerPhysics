/*
* Farseer Physics Engine based on Box2D.XNA port:
* Copyright (c) 2010 Ian Qvist
* 
* Box2D.XNA port of Box2D:
* Copyright (c) 2009 Brandon Furtwangler, Nathan Furtwangler
*
* Original source Box2D:
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org 
* 
* This software is provided 'as-is', without any express or implied 
* warranty.  In no event will the authors be held liable for any damages 
* arising from the use of this software. 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it 
* freely, subject to the following restrictions: 
* 1. The origin of this software must not be misrepresented; you must not 
* claim that you wrote the original software. If you use this software 
* in a product, an acknowledgment in the product documentation would be 
* appreciated but is not required. 
* 2. Altered source versions must be plainly marked as such, and must not be 
* misrepresented as being the original software. 
* 3. This notice may not be removed or altered from any source distribution. 
*/

using System;
using FarseerPhysics.Dynamics;
using Microsoft.Xna.Framework;

namespace FarseerPhysics.Collision
{
    internal struct Pair : IComparable<Pair>
    {
        public int ProxyIdA;
        public int ProxyIdB;

        #region IComparable<Pair> Members

        public int CompareTo(Pair other)
        {
            if (ProxyIdA < other.ProxyIdA)
            {
                return -1;
            }
            if (ProxyIdA == other.ProxyIdA)
            {
                if (ProxyIdB < other.ProxyIdB)
                {
                    return -1;
                }
                if (ProxyIdB == other.ProxyIdB)
                {
                    return 0;
                }
            }

            return 1;
        }

        #endregion
    }

    /// <summary>
    /// The broad-phase is used for computing pairs and performing volume queries and ray casts.
    /// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
    /// It is up to the client to consume the new pairs and to track subsequent overlap.
    /// </summary>
    public class DynamicTreeBroadPhase : IBroadPhase
    {
        private int[] _moveBuffer;
        private int _moveCapacity;
        private int _moveCount;

        private Pair[] _pairBuffer;
        private int _pairCapacity;
        private int _pairCount;
        private int _proxyCount;
        private Func<int, bool> _queryCallback;
        private int _queryProxyId;
        private bool _queryingStaticTree;
        /// <summary>
        /// Tree for Static fixtures
        /// </summary>
        private DynamicTree<FixtureProxy> _treeStatic = new DynamicTree<FixtureProxy>();
        /// <summary>
        /// Tree for Dynamic fixtures
        /// </summary>
        private DynamicTree<FixtureProxy> _treeDynamic = new DynamicTree<FixtureProxy>();

        public DynamicTreeBroadPhase()
        {
            _queryCallback = new Func<int, bool>(QueryCallback);

            _pairCapacity = 16;
            _pairBuffer = new Pair[_pairCapacity];

            _moveCapacity = 16;
            _moveBuffer = new int[_moveCapacity];
        }

        #region IBroadPhase Members

        /// <summary>
        /// Get the number of proxies.
        /// </summary>
        /// <value>The proxy count.</value>
        public int ProxyCount
        {
            get { return _proxyCount; }
        }

        /// <summary>
        /// Create a proxy with an initial AABB. Pairs are not reported until
        /// UpdatePairs is called.
        /// </summary>
        /// <param name="proxy">The user data.</param>
        /// <returns></returns>
        public int AddProxy(ref FixtureProxy proxy)
        {
            bool isStatic = proxy.Fixture.Body.IsStatic;
            var tree = isStatic ? _treeStatic : _treeDynamic;
            int externalProxyId = GetExternalId(tree.AddProxy(ref proxy.AABB, proxy), isStatic);
            ++_proxyCount;
            BufferMove(externalProxyId);
            return externalProxyId;
        }

        /// <summary>
        /// Destroy a proxy. It is up to the client to remove any pairs.
        /// </summary>
        /// <param name="proxyId">The proxy id.</param>
        public void RemoveProxy(int proxyId)
        {
            UnBufferMove(proxyId);
            --_proxyCount;
            var tree = IsStaticProxyId(proxyId) ? _treeStatic : _treeDynamic;
            tree.RemoveProxy(GetInternalId(proxyId));
        }

        public void MoveProxy(int proxyId, ref AABB aabb, Vector2 displacement)
        {
            var tree = IsStaticProxyId(proxyId) ? _treeStatic : _treeDynamic;
            var internalProxyId = GetInternalId(proxyId);
            bool buffer = tree.MoveProxy(internalProxyId, ref aabb, displacement);
            if (buffer)
            {
                BufferMove(proxyId);
            }
        }

        /// <summary>
        /// Get the AABB for a proxy.
        /// </summary>
        /// <param name="proxyId">The proxy id.</param>
        /// <param name="aabb">The aabb.</param>
        public void GetFatAABB(int proxyId, out AABB aabb)
        {
            var tree = IsStaticProxyId(proxyId) ? _treeStatic : _treeDynamic;
            tree.GetFatAABB(GetInternalId(proxyId), out aabb);
        }

        /// <summary>
        /// Get user data from a proxy. Returns null if the id is invalid.
        /// </summary>
        /// <param name="proxyId">The proxy id.</param>
        /// <returns></returns>
        public FixtureProxy GetProxy(int proxyId)
        {
            var tree = IsStaticProxyId(proxyId) ? _treeStatic : _treeDynamic;
            return tree.GetUserData(GetInternalId(proxyId));
        }

        /// <summary>
        /// Test overlap of fat AABBs.
        /// </summary>
        /// <param name="proxyIdA">The proxy id A.</param>
        /// <param name="proxyIdB">The proxy id B.</param>
        /// <returns></returns>
        public bool TestOverlap(int proxyIdA, int proxyIdB)
        {
            AABB aabbA, aabbB;
            (IsStaticProxyId(proxyIdA) ? _treeStatic : _treeDynamic).GetFatAABB(GetInternalId(proxyIdA), out aabbA);
            (IsStaticProxyId(proxyIdB) ? _treeStatic : _treeDynamic).GetFatAABB(GetInternalId(proxyIdB), out aabbB);
            return AABB.TestOverlap(ref aabbA, ref aabbB);
        }

        /// <summary>
        /// Update the pairs. This results in pair callbacks. This can only add pairs.
        /// </summary>
        /// <param name="callback">The callback.</param>
        public void UpdatePairs(BroadphaseDelegate callback)
        {
            // Reset pair buffer
            _pairCount = 0;

            // Perform tree queries for all moving proxies.
            for (int j = 0; j < _moveCount; ++j)
            {
                _queryProxyId = _moveBuffer[j];
                if (_queryProxyId == -1)
                {
                    continue;
                }

                bool isStatic = IsStaticProxyId(_queryProxyId);

                // We have to query the tree with the fat AABB so that
                // we don't fail to create a pair that may touch later.
                AABB fatAABB;
                (isStatic ? _treeStatic : _treeDynamic).GetFatAABB(GetInternalId(_queryProxyId), out fatAABB);

                // Query tree, create pairs and add them pair buffer.
                if (!isStatic)
                {
                    _queryingStaticTree = true;
                    _treeStatic.Query(_queryCallback, ref fatAABB);
                }
                _queryingStaticTree = false;
                _treeDynamic.Query(_queryCallback, ref fatAABB);
            }

            // Reset move buffer
            _moveCount = 0;

            // Sort the pair buffer to expose duplicates.
            Array.Sort(_pairBuffer, 0, _pairCount);

            // Send the pairs back to the client.
            int i = 0;
            while (i < _pairCount)
            {
                Pair primaryPair = _pairBuffer[i];
                FixtureProxy userDataA = (IsStaticProxyId(primaryPair.ProxyIdA) ? _treeStatic : _treeDynamic).GetUserData(GetInternalId(primaryPair.ProxyIdA));
                FixtureProxy userDataB = (IsStaticProxyId(primaryPair.ProxyIdB) ? _treeStatic : _treeDynamic).GetUserData(GetInternalId(primaryPair.ProxyIdB));

                callback(ref userDataA, ref userDataB);
                ++i;

                // Skip any duplicate pairs.
                while (i < _pairCount)
                {
                    Pair pair = _pairBuffer[i];
                    if (pair.ProxyIdA != primaryPair.ProxyIdA || pair.ProxyIdB != primaryPair.ProxyIdB)
                    {
                        break;
                    }
                    ++i;
                }
            }
        }

        /// <summary>
        /// Query an AABB for overlapping proxies. The callback class
        /// is called for each proxy that overlaps the supplied AABB.
        /// </summary>
        /// <param name="callback">The callback.</param>
        /// <param name="aabb">The aabb.</param>
        public void Query(Func<int, bool> callback, ref AABB aabb)
        {
            throw new Exception();
            //FIXME: This passes internalProxyIds to the callback, but should pass externalProxyIds
            _treeDynamic.Query(callback, ref aabb);
            _treeStatic.Query(callback, ref aabb);
        }

        /// <summary>
        /// Ray-cast against the proxies in the tree. This relies on the callback
        /// to perform a exact ray-cast in the case were the proxy contains a shape.
        /// The callback also performs the any collision filtering. This has performance
        /// roughly equal to k * log(n), where k is the number of collisions and n is the
        /// number of proxies in the tree.
        /// </summary>
        /// <param name="callback">A callback class that is called for each proxy that is hit by the ray.</param>
        /// <param name="input">The ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).</param>
        public void RayCast(Func<RayCastInput, int, float> callback, ref RayCastInput input)
        {
            _treeDynamic.RayCast(callback, ref input); //FIXME: Need to RayCast both trees at the same time
        }

        public void TouchProxy(int proxyId)
        {
            BufferMove(proxyId);
        }

        #endregion

        /// <summary>
        /// Compute the height of the embedded tree.
        /// </summary>
        /// <returns></returns>
        public int ComputeHeight()
        {
            return Math.Max(_treeDynamic.ComputeHeight(), _treeStatic.ComputeHeight());
        }

        private void BufferMove(int externalProxyId)
        {
            if (_moveCount == _moveCapacity)
            {
                int[] oldBuffer = _moveBuffer;
                _moveCapacity *= 2;
                _moveBuffer = new int[_moveCapacity];
                Array.Copy(oldBuffer, _moveBuffer, _moveCount);
            }

            _moveBuffer[_moveCount] = externalProxyId;
            ++_moveCount;
        }

        private void UnBufferMove(int proxyId)
        {
            for (int i = 0; i < _moveCount; ++i)
            {
                if (_moveBuffer[i] == proxyId)
                {
                    _moveBuffer[i] = -1;
                    return;
                }
            }
        }

        private bool QueryCallback(int proxyId)
        {
            var externalProxyId = GetExternalId(proxyId, _queryingStaticTree);

            // A proxy cannot form a pair with itself.
            if (externalProxyId == _queryProxyId)
            {
                return true;
            }

            // Grow the pair buffer as needed.
            if (_pairCount == _pairCapacity)
            {
                Pair[] oldBuffer = _pairBuffer;
                _pairCapacity *= 2;
                _pairBuffer = new Pair[_pairCapacity];
                Array.Copy(oldBuffer, _pairBuffer, _pairCount);
            }

            _pairBuffer[_pairCount].ProxyIdA = Math.Min(externalProxyId, _queryProxyId);
            _pairBuffer[_pairCount].ProxyIdB = Math.Max(externalProxyId, _queryProxyId);
            ++_pairCount;

            return true;
        }

        #region ProxyId Mangling
        //We encode whether a proxy is for a static or dynamic body in the lowest bit
        //This allows us to skip doing static-static collisions
        //internalProxyId are the IDs without this encoding applied.
        //externalProxyId are the IDs with this encoding applied.

        private int GetExternalId(int internalProxyId, bool isStatic)
        {
            return (internalProxyId << 1) | (isStatic ? 1 : 0);
        }

        private bool IsStaticProxyId(int externalProxyId)
        {
            return (externalProxyId & 1) == 1;
        }

        private int GetInternalId(int externalProxyId)
        {
            return externalProxyId >> 1;
        }

        #endregion
    }
}