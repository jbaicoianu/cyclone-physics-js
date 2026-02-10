elation.require(["physics.common"], function() {
  elation.extend("physics.octree", function(args) {
    if (!args) args = {};
    this.maxDepth = args.maxDepth || 6;
    this.maxObjects = args.maxObjects || 8;

    // Node pool to minimize GC pressure on per-frame rebuilds
    this._nodePool = [];
    this._nodePoolIndex = 0;

    // Planes (infinite extent) are stored separately
    this.planes = [];

    // Root node
    this.root = null;

    // Pair deduplication
    this._pairSet = new Set();
    this._pairs = [];

    this._getNode = function() {
      if (this._nodePoolIndex < this._nodePool.length) {
        var node = this._nodePool[this._nodePoolIndex++];
        node.objects.length = 0;
        node.children = null;
        return node;
      }
      var node = {
        minX: 0, minY: 0, minZ: 0,
        maxX: 0, maxY: 0, maxZ: 0,
        depth: 0,
        objects: [],
        children: null,
      };
      this._nodePool.push(node);
      this._nodePoolIndex++;
      return node;
    };

    this.build = function(bodies, dt) {
      // Reset pool
      this._nodePoolIndex = 0;
      this.planes.length = 0;

      // Separate planes from finite bodies, compute AABBs
      var finiteBodies = [];
      for (var i = 0; i < bodies.length; i++) {
        var body = bodies[i];
        if (!body.collider) continue;

        // Ensure reusable AABB object exists on the body
        if (!body._aabb) {
          body._aabb = {
            min: new THREE.Vector3(),
            max: new THREE.Vector3(),
          };
        }

        var aabb = body.collider.getWorldAABB(body._aabb, dt);
        if (aabb === null) {
          // Infinite extent (plane) — handle separately
          this.planes.push(body);
        } else {
          finiteBodies.push(body);
        }
      }

      if (finiteBodies.length === 0) {
        this.root = null;
        return;
      }

      // Compute world bounds as the union of all AABBs
      var first = finiteBodies[0]._aabb;
      var wMinX = first.min.x, wMinY = first.min.y, wMinZ = first.min.z;
      var wMaxX = first.max.x, wMaxY = first.max.y, wMaxZ = first.max.z;
      for (var i = 1; i < finiteBodies.length; i++) {
        var aabb = finiteBodies[i]._aabb;
        if (aabb.min.x < wMinX) wMinX = aabb.min.x;
        if (aabb.min.y < wMinY) wMinY = aabb.min.y;
        if (aabb.min.z < wMinZ) wMinZ = aabb.min.z;
        if (aabb.max.x > wMaxX) wMaxX = aabb.max.x;
        if (aabb.max.y > wMaxY) wMaxY = aabb.max.y;
        if (aabb.max.z > wMaxZ) wMaxZ = aabb.max.z;
      }

      // Add small padding
      var pad = 0.1;
      wMinX -= pad; wMinY -= pad; wMinZ -= pad;
      wMaxX += pad; wMaxY += pad; wMaxZ += pad;

      // Create root node
      this.root = this._getNode();
      this.root.minX = wMinX; this.root.minY = wMinY; this.root.minZ = wMinZ;
      this.root.maxX = wMaxX; this.root.maxY = wMaxY; this.root.maxZ = wMaxZ;
      this.root.depth = 0;

      // Insert each finite body into the tree
      for (var i = 0; i < finiteBodies.length; i++) {
        this._insert(this.root, finiteBodies[i], finiteBodies[i]._aabb);
      }
    };

    this._insert = function(node, body, aabb) {
      if (node.children !== null) {
        // Internal node: insert into every child whose bounds overlap the body's AABB
        for (var i = 0; i < 8; i++) {
          var child = node.children[i];
          if (aabb.min.x <= child.maxX && aabb.max.x >= child.minX &&
              aabb.min.y <= child.maxY && aabb.max.y >= child.minY &&
              aabb.min.z <= child.maxZ && aabb.max.z >= child.minZ) {
            this._insert(child, body, aabb);
          }
        }
        return;
      }

      // Leaf node: add object
      node.objects.push(body);

      // If leaf is full and below max depth, subdivide
      if (node.objects.length > this.maxObjects && node.depth < this.maxDepth) {
        this._subdivide(node);
        // Redistribute existing objects into children
        var objects = node.objects;
        node.objects = [];
        for (var i = 0; i < objects.length; i++) {
          this._insert(node, objects[i], objects[i]._aabb);
        }
      }
    };

    this._subdivide = function(node) {
      var midX = (node.minX + node.maxX) * 0.5;
      var midY = (node.minY + node.maxY) * 0.5;
      var midZ = (node.minZ + node.maxZ) * 0.5;
      var d = node.depth + 1;

      node.children = [];
      // 8 children, splitting at midpoint along each axis
      // Ordered by (x, y, z) low/high bit pattern
      for (var i = 0; i < 8; i++) {
        var child = this._getNode();
        child.depth = d;
        child.minX = (i & 1) ? midX : node.minX;
        child.maxX = (i & 1) ? node.maxX : midX;
        child.minY = (i & 2) ? midY : node.minY;
        child.maxY = (i & 2) ? node.maxY : midY;
        child.minZ = (i & 4) ? midZ : node.minZ;
        child.maxZ = (i & 4) ? node.maxZ : midZ;
        node.children.push(child);
      }
    };

    this.getPotentialPairs = function() {
      this._pairSet.clear();
      this._pairs.length = 0;

      // Traverse tree and generate intra-leaf pairs
      if (this.root !== null) {
        this._collectPairs(this.root);
      }

      // Add plane pairs: each plane paired with every finite body
      // Collect all finite bodies from the tree
      if (this.planes.length > 0) {
        var finiteBodies = [];
        if (this.root !== null) {
          this._collectBodies(this.root, finiteBodies);
        }
        for (var i = 0; i < this.planes.length; i++) {
          for (var j = 0; j < finiteBodies.length; j++) {
            this._pairs.push([this.planes[i], finiteBodies[j]]);
          }
          // Plane-plane pairs
          for (var j = i + 1; j < this.planes.length; j++) {
            this._pairs.push([this.planes[i], this.planes[j]]);
          }
        }
      }

      return this._pairs;
    };

    this._collectPairs = function(node) {
      if (node.children !== null) {
        for (var i = 0; i < 8; i++) {
          this._collectPairs(node.children[i]);
        }
        return;
      }

      // Leaf node: generate all intra-leaf pairs
      var objects = node.objects;
      for (var i = 0; i < objects.length; i++) {
        for (var j = i + 1; j < objects.length; j++) {
          var id1 = objects[i].id;
          var id2 = objects[j].id;
          // Deduplicate: bodies spanning cell boundaries appear in multiple leaves
          var key;
          if (id1 < id2) {
            key = id1 + ',' + id2;
          } else {
            key = id2 + ',' + id1;
          }
          if (!this._pairSet.has(key)) {
            this._pairSet.add(key);
            this._pairs.push([objects[i], objects[j]]);
          }
        }
      }
    };

    this._collectBodies = function(node, out) {
      if (node.children !== null) {
        for (var i = 0; i < 8; i++) {
          this._collectBodies(node.children[i], out);
        }
        return;
      }
      for (var i = 0; i < node.objects.length; i++) {
        out.push(node.objects[i]);
      }
    };

    // Deduplicate the collected bodies (they may appear in multiple leaves)
    this._getUniqueFiniteBodies = function() {
      var seen = new Set();
      var result = [];
      if (this.root !== null) {
        this._collectBodiesUnique(this.root, seen, result);
      }
      return result;
    };

    this._collectBodiesUnique = function(node, seen, out) {
      if (node.children !== null) {
        for (var i = 0; i < 8; i++) {
          this._collectBodiesUnique(node.children[i], seen, out);
        }
        return;
      }
      for (var i = 0; i < node.objects.length; i++) {
        var body = node.objects[i];
        if (!seen.has(body.id)) {
          seen.add(body.id);
          out.push(body);
        }
      }
    };
  });
});
