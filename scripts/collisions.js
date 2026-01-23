'use strict';
/**
 * colliders 
 */
elation.require(['physics.common', 'utils.math'], function() {

  elation.extend("physics.colliders.helperfuncs", new function() {
    this.sphere_sphere = function() {
      // closure scratch variables
      var thispos = new THREE.Vector3(),
          otherpos = new THREE.Vector3(),
          thisvel = new THREE.Vector3(),
          othervel = new THREE.Vector3(),
          midline = new THREE.Vector3(),
          scaledVelocity = new THREE.Vector3(),
          intersectionPoint = new THREE.Vector3();

      return function(obj1, obj2, contacts, dt) {
        if (!contacts) contacts = [];

        // Work in world space
        obj1.body.localToWorldPos(thispos.set(0,0,0));
        obj2.body.localToWorldPos(otherpos.set(0,0,0));

        let dynamic = (obj1.body.velocity.lengthSq() > 0 || obj2.body.velocity.lengthSq() > 0); // TODO - this should either be a flag on rigid bodies, or a configurable threshold based on velocity
        if (!dynamic) {
          midline.subVectors(otherpos, thispos),
          size = midline.length();
          midline.divideScalar(size);

          var normal = midline.clone(); // allocate normal
          var point = thispos.clone().add(midline.multiplyScalar(obj1.radius)); // allocate point

          if (!(size <= 0 || size >= obj1.radius + obj2.radius)) {
            var penetration = (obj1.radius + obj2.radius - size);
            // Collision point is on the outer shell of obj1
            contact = new elation.physics.contact({
              normal: normal,
              point: point,
              penetration: -penetration,
              bodies: [obj1.body, obj2.body]
            });
            contacts.push(contact);
            //console.log('crash a sphere-sphere', contact);
          }
        } else {
          let r = obj1.radius + obj2.radius;
          // FIXME - probably need to transform velocity into world coordinates as well
          obj1.body.localToWorldDir(thisvel.copy(obj1.body.velocity));
          obj2.body.localToWorldDir(othervel.copy(obj2.body.velocity));


          let v = scaledVelocity.copy(thisvel).sub(othervel).multiplyScalar(dt);

          midline.copy(thispos).sub(otherpos).normalize();

          //if (midline.dot(scaledVelocity) > 0) return; // moving away, can't collide

          let endpos = midline.copy(thispos).add(v);

          let intersection = elation.physics.colliders.helperfuncs.line_sphere(thispos, endpos, otherpos, r, intersectionPoint);
          if (intersection && intersection.t <= r) {
            let t = intersection.t;
            thispos.add(scaledVelocity.copy(obj1.body.velocity).multiplyScalar(t * dt));
            otherpos.add(scaledVelocity.copy(obj2.body.velocity).multiplyScalar(t * dt));
//console.log(intersection, thispos, v, otherpos, r, obj1, obj2);
            let normal = otherpos.clone().sub(thispos).normalize(); // allocate normal

            var contact = new elation.physics.contact_dynamic({
              normal: normal,
              point: normal.clone().multiplyScalar(obj1.radius).add(thispos), // allocate point
              penetrationTime: intersection.t,
              bodies: [obj1.body, obj2.body],
            });
            contacts.push(contact);
            return contacts;
          }
        }
        return contacts;
      }
    }();

    this.sphere_plane = function() {
      // closure scratch variables
      var pos = new THREE.Vector3();

      return function(sphere, plane, contacts, dt) {
        if (!contacts) contacts = [];
        var contact = false;
        var position = sphere.body.localToWorldPos(pos.set(0,0,0));
        var norm = plane.body.localToWorldDir(plane.normal.clone()); // allocate normal
        var distance = norm.dot(pos) - sphere.radius - plane.offset;
        if (distance < 0) {
          var sepspeed = sphere.body.velocity.dot(plane.normal);
          if (sepspeed <= 0) {
            var point = position.clone().sub(norm.clone().multiplyScalar(distance + sphere.radius));  // allocate point
            contact = new elation.physics.contact({
              normal: norm,
              point: point,
              penetration: -distance,
              bodies: [sphere.body, plane.body]
            });
            contacts.push(contact);
            //console.log('crash a sphere-plane!', contact);
          }
        }
        return contacts;
      }
    }();

    this.box_sphere = function() {
      // closure scratch variables
      var center = new THREE.Vector3(),      // center of sphere, box-space coordinates (scaled)
          centerWorld = new THREE.Vector3(), // center of sphere, world-space coordinates
          diff = new THREE.Vector3(),
          closest = new THREE.Vector3(),
          closestWorld = new THREE.Vector3(),
          invQuat = new THREE.Quaternion();

      return function(box, sphere, contacts, dt) {
        if (!contacts) contacts = [];

        // Get sphere position in world space
        if (sphere.offset) {
          sphere.body.localToWorldPos(centerWorld.copy(sphere.offset));
        } else {
          sphere.body.localToWorldPos(centerWorld.set(0,0,0));
        }

        // Transform sphere center to box's SCALED local space
        // (subtract position, apply inverse rotation, but do NOT divide by scale)
        // This matches the coordinate system of box.min/max which are pre-scaled
        center.copy(centerWorld).sub(box.body.position);
        if (box.body.orientation) {
          invQuat.copy(box.body.orientation).invert();
          center.applyQuaternion(invQuat);
        }

        // box.min/max are already in scaled local space, and so is center now
        // sphere.radius is in world units which matches scaled local space

        // Early out if any of the axes are separating
        if ((center.x + sphere.radius < box.min.x || center.x - sphere.radius > box.max.x) ||
            (center.y + sphere.radius < box.min.y || center.y - sphere.radius > box.max.y) ||
            (center.z + sphere.radius < box.min.z || center.z - sphere.radius > box.max.z)) {
          return false;
        }

        // Find closest point on box (in scaled local space)
        closest.x = elation.utils.math.clamp(center.x, box.min.x, box.max.x);
        closest.y = elation.utils.math.clamp(center.y, box.min.y, box.max.y);
        closest.z = elation.utils.math.clamp(center.z, box.min.z, box.max.z);

        // Check distance (all in scaled local space = world units)
        diff.subVectors(closest, center);
        var dist = diff.lengthSq();
        if (dist > sphere.radius * sphere.radius) {
          return 0;
        }

        // Transform closest point back to world space
        closestWorld.copy(closest);
        if (box.body.orientation) {
          closestWorld.applyQuaternion(box.body.orientation);
        }
        closestWorld.add(box.body.position);

        var contact = new elation.physics.contact({
          point: closestWorld.clone(), // allocate point
          normal: centerWorld.clone().sub(closestWorld).normalize(), // allocate normal
          penetration: -(sphere.radius - Math.sqrt(dist)),
          bodies: [box.body, sphere.body]
        });
        contacts.push(contact);

        return contacts;
      }
    }();

    this.box_plane = function() {
      // closure scratch variables
      var worldpos = new THREE.Vector3();

      return function(box, plane, contacts, dt) {
        if (!contacts) contacts = [];
        var vertices = [
              [box.min.x, box.min.y, box.min.z],
              [box.min.x, box.min.y, box.max.z],
              [box.min.x, box.max.y, box.min.z],
              [box.min.x, box.max.y, box.max.z],

              [box.max.x, box.min.y, box.min.z],
              [box.max.x, box.min.y, box.max.z],
              [box.max.x, box.max.y, box.min.z],
              [box.max.x, box.max.y, box.max.z],
            ];

        for (var i = 0; i < vertices.length; i++) {
          // Pass world position of vertex to vertex_plane collider
          // No allocations needed here, since they're done in vertex_plane
          box.body.localToWorldPos(worldpos.set(vertices[i][0], vertices[i][1], vertices[i][2]));
          var contact = this.vertex_plane(worldpos, plane);
          if (contact) {
            contact.bodies = [box.body, plane.body];
            contacts.push(contact);
          }
        }

        return contacts;
      }
    }();

    this.vertex_vertex = function(v1, v2, contacts, dt) {
      if (!contacts) contacts = [];
      let distance = v1.distanceTo(v2);
      if (distance < 1e6) {
        var contact = new elation.physics.contact({
          point: v1.clone(), // allocate point
          //normal: normal.clone(), // allocate normal
          penetration: distance
        });
        contacts.push(contact);
      }
      return contacts;
    };

    this.vertex_sphere = function() {
      let localvert = new THREE.Vector3();
      return function(vertex, sphere, contacts, dt) {
        if (!contacts) contacts = [];
        sphere.body.worldToLocal(localvert.copy(vertex));

        let distance = localvert.length();
        if (distance <= sphere.body.radius) {
          var contact = new elation.physics.contact({
            point: vertex.clone(), // allocate point
            normal: sphere.body.localToWorldDir(localvert.clone().divideScalar(distance)), // allocate normal
            penetration: distance
          });
          contacts.push(contact);
        }
        return contacts;
      };
    }();

    this.vertex_box = function() {
      var relpos = new THREE.Vector3();
      return function(vertex, box, contacts, dt) {
        if (!contacts) contacts = [];
        // Get point in box-local coordinates
        box.body.worldToLocalPos(relpos.copy(vertex));

        // check x axis
        var min_depth = box.halfsize.x - Math.abs(relpos.x);
        if (min_depth < 0) return false;
        // normal = ...

        // check y axis
        var depth = box.halfsize.y - Math.abs(relpos.y);
        if (depth < 0) return false;
        else if (depth < min_depth) {
          min_depth = depth;
          //normal = ...
        }

        // check z axis
        depth = box.halfsize.z - Math.abs(relpos.z);
        if (depth < 0) return false;
        else if (depth < min_depth) {
          min_depth = depth;
          //normal = ...
        }

        var contact = new elation.physics.contact({
          point: vertex.clone(), // allocate point
          //normal: normal.clone(), // allocate normal
          penetration: min_depth
        });
        contacts.push(contact);
        return contacts;
      }
    }();

    this.vertex_triangle = function(vertex, triangle, contacts, dt) {
      if (triangle.containsPoint(vertex)) {
        var contact = new elation.physics.contact({
          point: vertex.clone(), // allocate point
          normal: triangle.normal.clone(), // allocate normal, FIXME - transform into world coords
          penetration: 0,
          bodies: [vertex.body, triangle.body]
        });
        contacts.push(contact);
      }
      return contacts;
    };
    this.vertex_plane = function(vertex, plane) {
      // FIXME - Only one contact possible...should this return a single-element array to be consistent?
      var contact = false;
      var distance = vertex.dot(plane.normal);
      if (distance <= plane.offset) {
        contact = new elation.physics.contact({
          normal: plane.normal.clone(), // allocate normal
          //point: plane.normal.clone().multiplyScalar((distance - plane.offset) / 2).add(vertex), // allocate point
          point: vertex.clone().sub(plane.normal.clone().multiplyScalar(distance)),
          penetration: plane.offset - distance
        });
        //console.log('crash a vertex-plane', contact.point.toArray(), contact.normal.toArray());
      }
      return contact;
    }
    this.vertex_capsule = (function() {
      const closest = new THREE.Vector3();
      return function(vertex, capsule) {
        let capsuleDims = capsule.getDimensions();
        let scaledRadius = capsuleDims.scaledRadius;
        elation.physics.colliders.helperfuncs.closest_point_on_line(capsuleDims.start, capsuleDims.end, vertex, closest);
        let distSq = closest.distanceToSquared(vertex);
        if (distSq <= scaledRadius * scaledRadius) {
          let dist = Math.sqrt(distSq);
          let normal = closest.clone().sub(vertex).divideScalar(dist);
          let point = closest.clone();
          point.x += normal.x * scaledRadius;
          point.y += normal.y * scaledRadius;
          point.z += normal.z * scaledRadius;
          let contact = new elation.physics.contact({
            normal: normal,
            point: point,
            penetration: dist - scaledRadius,
          });
          return contact;
        }
      }
    })();

    this.box_box_old = function() {
      // closure scratch variables
      var diff = new THREE.Vector3(),
          thispos = new THREE.Vector3(),
          otherpos = new THREE.Vector3(),
          matrix1 = new THREE.Matrix4(),
          matrix2 = new THREE.Matrix4(),
          axis = new THREE.Vector3(),
          axis2 = new THREE.Vector3(),
          corner = new THREE.Vector3(),
          smallestPenetration, smallestIndex, best;

      var axes = [
          new THREE.Vector3(1,0,0),
          new THREE.Vector3(0,1,0),
          new THREE.Vector3(0,0,1)
        ];

      // static helper functions

      var tmpaxis = new THREE.Vector3();
      function transformToAxis(box, axis) {
        return (box.halfsize.x * Math.abs(axis.dot(box.body.localToWorldDir(tmpaxis.set(1,0,0))))) +
               (box.halfsize.y * Math.abs(axis.dot(box.body.localToWorldDir(tmpaxis.set(0,1,0))))) +
               (box.halfsize.z * Math.abs(axis.dot(box.body.localToWorldDir(tmpaxis.set(0,0,1)))));
      }
      function penetrationOnAxis(box1, box2, axis, diff) {
        var oneProject = transformToAxis(box1, axis),
            twoProject = transformToAxis(box2, axis),
            distance = Math.abs(diff.dot(axis));

  //console.log(axis.toArray(), oneProject, twoProject, distance, oneProject + twoProject - distance);
        return oneProject + twoProject - distance;
      }
      function testOverlap(box1, box2, axis, diff, index) {
        if (axis.lengthSq() < 0.0001) return true;
        axis.normalize();
        var penetration = penetrationOnAxis(box1, box2, axis, diff);
        if (penetration < 0) return false;
        if (penetration < smallestPenetration) {
          smallestPenetration = penetration;
          smallestIndex = index;
        }
        return true;
      }
      function getAxis(obj, index, taxis) {
        if (!taxis) taxis = axis;
        matrix1.makeRotationFromQuaternion(obj.body.orientationWorld);
        var m1 = matrix1.elements;
        var offset = index * 4;
        taxis.set(m1[offset], m1[offset+1], m1[offset+2]);
        return taxis;
      }
      function fillPointFaceBoxBox(box1, box2, toCenter, best, penetration) {
        var point = new THREE.Vector3(); // allocate point
        var normal = new THREE.Vector3(0,1,0); // allocate normal

        getAxis(box1, best, normal);
        if (normal.dot(toCenter) < 0) {
          normal.multiplyScalar(-1);
        }

        point.copy(box2.halfsize);
        if (getAxis(box2, 0, axis).dot(normal) < 0) point.x = -point.x;
        if (getAxis(box2, 1, axis).dot(normal) < 0) point.y = -point.y;
        if (getAxis(box2, 2, axis).dot(normal) < 0) point.z = -point.z;
      
        var contact = new elation.physics.contact({
          point: box2.body.localToWorldPos(point),
          normal: normal.normalize(),
          penetration: -penetration,
          restitution: box1.body.restitution * box2.body.restitution,
          bodies: [box1.body, box2.body]
        });

        return contact;
      }

      return function(box1, box2, contacts, dt) {
        if (!contacts) contacts = [];
        box1.body.localToWorldPos(thispos.set(0,0,0));
        box2.body.localToWorldPos(otherpos.set(0,0,0));
        diff.subVectors(otherpos, thispos);
        
        matrix1.makeRotationFromQuaternion(box1.body.orientationWorld);
        matrix2.makeRotationFromQuaternion(box2.body.orientationWorld);
        var m1 = matrix1.elements,
            m2 = matrix2.elements;

        smallestPenetration = Infinity;
        smallestIndex = false;

        // box1's primary axes
        if (!testOverlap(box1, box2, getAxis(box1, 0, axis), diff, 0)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 1, axis), diff, 1)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 2, axis), diff, 2)) return false;

        // box 2's primary axes
        if (!testOverlap(box1, box2, getAxis(box2, 0, axis), diff, 3)) return false;
        if (!testOverlap(box1, box2, getAxis(box2, 1, axis), diff, 4)) return false;
        if (!testOverlap(box1, box2, getAxis(box2, 2, axis), diff, 5)) return false;

        var bestSingleAxis = smallestIndex;

        // perpendicular axes
        if (!testOverlap(box1, box2, getAxis(box1, 0, axis).cross(getAxis(box2, 0, axis2)), diff, 6)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 0, axis).cross(getAxis(box2, 1, axis2)), diff, 7)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 0, axis).cross(getAxis(box2, 2, axis2)), diff, 8)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 1, axis).cross(getAxis(box2, 0, axis2)), diff, 9)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 1, axis).cross(getAxis(box2, 1, axis2)), diff, 10)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 1, axis).cross(getAxis(box2, 2, axis2)), diff, 11)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 2, axis).cross(getAxis(box2, 0, axis2)), diff, 12)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 2, axis).cross(getAxis(box2, 1, axis2)), diff, 13)) return false;
        if (!testOverlap(box1, box2, getAxis(box1, 2, axis).cross(getAxis(box2, 2, axis2)), diff, 14)) return false;
        
        // Separating axis theorem returned positive overlap, generate contacts
        if (false) {
          // check box1's vertices against box2
          for (var i = 0; i < 8; i++) {
            box1.body.localToWorldPos(box1.getCorner(i, corner));
            var contact = elation.physics.colliders.helperfuncs.vertex_box(corner, box2);
            if (contact) {
              contact.bodies = [box1, box2];
              contacts.push(contact);
            }
          }
          // check box2's vertices against box1
          for (var i = 0; i < 8; i++) {
            box1.body.localToWorldPos(box1.getCorner(i, corner));
            var contact = elation.physics.colliders.helperfuncs.vertex_box(corner, box2);
            if (contact) {
              contact.bodies = [box1, box2];
              contacts.push(contact);
            }
          }
          // check box1's edges against box2's edges
          /*
          for (var i = 0; i < 12; i++) {
            var edge = box1.getEdge(i);
            var edge = box1.getEdge(i);
          }
          */
          //console.log(contacts);
          return contacts;
        } else {
          var contact = false;
          if (smallestIndex < 3) {
            contact = fillPointFaceBoxBox(box1, box2, diff, smallestIndex, smallestPenetration);
          } else if (smallestIndex < 6) {
            contact = fillPointFaceBoxBox(box1, box2, diff.multiplyScalar(-1), smallestIndex - 3, smallestPenetration);
          } else {
            console.log('uh oh hard part', smallestIndex, smallestPenetration, box1, box2);
          }

          if (contact) {
            contacts.push(contact);
          }
          return contacts;
        }
      }
    }();

    this.box_box = (function() {
      // closure scratch variables - reused to avoid per-frame allocations
      var box1Axes = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
      var box2Axes = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
      var crossAxis = new THREE.Vector3();
      var diff = new THREE.Vector3();
      var box1Center = new THREE.Vector3();
      var box2Center = new THREE.Vector3();
      var contactNormal = new THREE.Vector3();
      var contactPoint = new THREE.Vector3();
      var tmpVec = new THREE.Vector3();
      var tmpVec2 = new THREE.Vector3();
      var edge1Point = new THREE.Vector3();
      var edge2Point = new THREE.Vector3();
      var closestOnEdge1 = new THREE.Vector3();
      var closestOnEdge2 = new THREE.Vector3();
      // Reusable projection results to avoid allocation
      var proj1 = { min: 0, max: 0 };
      var proj2 = { min: 0, max: 0 };

      // Get world-space axes for a box using orientationWorld for hierarchy support
      function getWorldAxes(box, axes) {
        var orient = box.body.orientationWorld || box.body.orientation;
        axes[0].set(1, 0, 0).applyQuaternion(orient);
        axes[1].set(0, 1, 0).applyQuaternion(orient);
        axes[2].set(0, 0, 1).applyQuaternion(orient);
        return axes;
      }

      // Get box center in world space
      function getBoxCenter(box, out) {
        // For standard physics bodies, position IS the center
        // If there's an offset in box.min/max, account for it
        out.addVectors(box.min, box.max).multiplyScalar(0.5);
        if (out.lengthSq() > 0) {
          // There's an offset - transform it to world space
          var orient = box.body.orientationWorld || box.body.orientation;
          out.applyQuaternion(orient);
        }
        out.add(box.body.position);
        return out;
      }

      // Project box onto axis, storing result in outProj to avoid allocation
      function projectBox(box, boxCenter, boxAxes, axis, outProj) {
        var halfExtents = box.halfsize; // pre-scaled at collider creation time
        var centerProj = boxCenter.dot(axis);

        // Project each axis extent onto the test axis
        var extent =
          halfExtents.x * Math.abs(boxAxes[0].dot(axis)) +
          halfExtents.y * Math.abs(boxAxes[1].dot(axis)) +
          halfExtents.z * Math.abs(boxAxes[2].dot(axis));

        outProj.min = centerProj - extent;
        outProj.max = centerProj + extent;
      }

      // Test overlap on a single axis, return penetration or false if separating
      function testAxis(box1, box2, box1Center, box2Center, box1Axes, box2Axes, axis) {
        // Skip degenerate axes (near-zero length from parallel edges)
        var lenSq = axis.lengthSq();
        if (lenSq < 1e-8) return Infinity; // Treat as non-separating

        // Normalize the axis
        var invLen = 1 / Math.sqrt(lenSq);
        axis.x *= invLen;
        axis.y *= invLen;
        axis.z *= invLen;

        projectBox(box1, box1Center, box1Axes, axis, proj1);
        projectBox(box2, box2Center, box2Axes, axis, proj2);

        // Check for separation
        if (proj1.max < proj2.min || proj2.max < proj1.min) {
          return false; // Separating axis found
        }

        // Return overlap amount
        return Math.min(proj1.max - proj2.min, proj2.max - proj1.min);
      }

      // Find closest points between two line segments
      // Returns parameter t for point on segment 1: P1 + t * D1
      function closestPointsOnSegments(p1, d1, halfLen1, p2, d2, halfLen2, outPoint1, outPoint2) {
        // Direction from p1 to p2
        tmpVec2.subVectors(p2, p1);

        var d1d1 = d1.dot(d1);
        var d2d2 = d2.dot(d2);
        var d1d2 = d1.dot(d2);
        var d1r = d1.dot(tmpVec2);
        var d2r = d2.dot(tmpVec2);

        var denom = d1d1 * d2d2 - d1d2 * d1d2;

        var t, s;
        if (Math.abs(denom) < 1e-8) {
          // Parallel segments - use midpoint
          t = 0;
          s = d2r / d2d2;
        } else {
          t = (d1d2 * d2r - d2d2 * d1r) / denom;
          s = (d1d1 * d2r - d1d2 * d1r) / denom;
        }

        // Clamp to segment bounds
        t = Math.max(-halfLen1, Math.min(halfLen1, t));
        s = Math.max(-halfLen2, Math.min(halfLen2, s));

        // Compute closest points
        outPoint1.copy(d1).multiplyScalar(t).add(p1);
        outPoint2.copy(d2).multiplyScalar(s).add(p2);
      }

      // Get edge center and direction for a box edge
      // edgeAxisIndex: which axis the edge runs along (0=X, 1=Y, 2=Z)
      // signs: which quadrant of the perpendicular plane (-1 or +1 for each of the other two axes)
      function getBoxEdge(boxCenter, boxAxes, halfsize, edgeAxisIndex, sign1, sign2, outPoint, outDir, outHalfLen) {
        var ax1 = (edgeAxisIndex + 1) % 3;
        var ax2 = (edgeAxisIndex + 2) % 3;
        var hs = [halfsize.x, halfsize.y, halfsize.z];

        outPoint.copy(boxCenter);
        outPoint.addScaledVector(boxAxes[ax1], sign1 * hs[ax1]);
        outPoint.addScaledVector(boxAxes[ax2], sign2 * hs[ax2]);

        outDir.copy(boxAxes[edgeAxisIndex]);
        return hs[edgeAxisIndex]; // half-length along edge
      }

      return function(box1, box2, contacts, dt) {
        if (!contacts) contacts = [];

        // Get box centers and axes in world space
        getBoxCenter(box1, box1Center);
        getBoxCenter(box2, box2Center);
        getWorldAxes(box1, box1Axes);
        getWorldAxes(box2, box2Axes);

        // Vector from box1 center to box2 center
        diff.subVectors(box2Center, box1Center);

        var minPenetration = Infinity;
        var minAxisIndex = -1;
        var penetration;

        // Test box1's 3 face axes
        for (var i = 0; i < 3; i++) {
          penetration = testAxis(box1, box2, box1Center, box2Center, box1Axes, box2Axes,
                                  tmpVec.copy(box1Axes[i]));
          if (penetration === false) return false;
          if (penetration < minPenetration) {
            minPenetration = penetration;
            minAxisIndex = i;
            contactNormal.copy(tmpVec);
          }
        }

        // Test box2's 3 face axes
        for (var i = 0; i < 3; i++) {
          penetration = testAxis(box1, box2, box1Center, box2Center, box1Axes, box2Axes,
                                  tmpVec.copy(box2Axes[i]));
          if (penetration === false) return false;
          if (penetration < minPenetration) {
            minPenetration = penetration;
            minAxisIndex = 3 + i;
            contactNormal.copy(tmpVec);
          }
        }

        // Test 9 edge-edge cross product axes
        for (var i = 0; i < 3; i++) {
          for (var j = 0; j < 3; j++) {
            crossAxis.crossVectors(box1Axes[i], box2Axes[j]);
            penetration = testAxis(box1, box2, box1Center, box2Center, box1Axes, box2Axes,
                                    tmpVec.copy(crossAxis));
            if (penetration === false) return false;
            if (penetration < minPenetration) {
              minPenetration = penetration;
              minAxisIndex = 6 + i * 3 + j;
              contactNormal.copy(tmpVec);
            }
          }
        }

        // Ensure normal points from box1 to box2
        if (contactNormal.dot(diff) < 0) {
          contactNormal.negate();
        }

        // Calculate contact point based on collision type
        if (minAxisIndex < 6) {
          // Face-face or face-vertex contact (axes 0-5 are face normals)
          // Contact point: project other box's center onto the contact face, clamped to face bounds
          // Then offset by half penetration into the contact

          // Get the face normal axis index (0-2 for the face box)
          var faceAxisIndex, faceBox, faceCenter, faceAxes, faceHalfsize, otherCenter;
          if (minAxisIndex < 3) {
            // box1's face
            faceAxisIndex = minAxisIndex;
            faceBox = box1;
            faceCenter = box1Center;
            faceAxes = box1Axes;
            faceHalfsize = box1.halfsize;
            otherCenter = box2Center;
          } else {
            // box2's face
            faceAxisIndex = minAxisIndex - 3;
            faceBox = box2;
            faceCenter = box2Center;
            faceAxes = box2Axes;
            faceHalfsize = box2.halfsize;
            otherCenter = box1Center;
          }

          // The two tangent axis indices (perpendicular to face normal)
          var tangent1 = (faceAxisIndex + 1) % 3;
          var tangent2 = (faceAxisIndex + 2) % 3;

          // Project other box's center onto the face plane
          // First, get vector from face center to other center
          tmpVec.subVectors(otherCenter, faceCenter);

          // Project onto the face's tangent axes (not the normal) and clamp to face bounds
          var hs = [faceHalfsize.x, faceHalfsize.y, faceHalfsize.z];

          // Start at face center, offset by face normal to reach the face surface
          var normalSign = tmpVec.dot(faceAxes[faceAxisIndex]) > 0 ? 1 : -1;
          contactPoint.copy(faceCenter);
          contactPoint.addScaledVector(faceAxes[faceAxisIndex], normalSign * hs[faceAxisIndex]);

          // Project onto the two tangent axes and clamp
          var proj1 = tmpVec.dot(faceAxes[tangent1]);
          proj1 = Math.max(-hs[tangent1], Math.min(hs[tangent1], proj1));
          contactPoint.addScaledVector(faceAxes[tangent1], proj1);

          var proj2 = tmpVec.dot(faceAxes[tangent2]);
          proj2 = Math.max(-hs[tangent2], Math.min(hs[tangent2], proj2));
          contactPoint.addScaledVector(faceAxes[tangent2], proj2);

          // Move contact point to midway through penetration (into the face)
          contactPoint.addScaledVector(contactNormal, -minPenetration * 0.5);

        } else {
          // Edge-edge contact (axes 6-14 are cross products)
          // Find the two edges and compute closest points

          var edgeIndex = minAxisIndex - 6;
          var edge1Axis = Math.floor(edgeIndex / 3); // 0, 1, or 2
          var edge2Axis = edgeIndex % 3;             // 0, 1, or 2

          // Determine which of the 4 parallel edges on each box to use
          // Pick the edges closest to each other
          var hs1 = [box1.halfsize.x, box1.halfsize.y, box1.halfsize.z];
          var hs2 = [box2.halfsize.x, box2.halfsize.y, box2.halfsize.z];

          // For edge on box1 along axis edge1Axis, the other two axes determine position
          var ax1_1 = (edge1Axis + 1) % 3;
          var ax1_2 = (edge1Axis + 2) % 3;
          var sign1_1 = diff.dot(box1Axes[ax1_1]) > 0 ? 1 : -1;
          var sign1_2 = diff.dot(box1Axes[ax1_2]) > 0 ? 1 : -1;

          // For edge on box2 along axis edge2Axis
          var ax2_1 = (edge2Axis + 1) % 3;
          var ax2_2 = (edge2Axis + 2) % 3;
          var sign2_1 = diff.dot(box2Axes[ax2_1]) < 0 ? 1 : -1;
          var sign2_2 = diff.dot(box2Axes[ax2_2]) < 0 ? 1 : -1;

          // Get edge 1 (on box1)
          edge1Point.copy(box1Center);
          edge1Point.addScaledVector(box1Axes[ax1_1], sign1_1 * hs1[ax1_1]);
          edge1Point.addScaledVector(box1Axes[ax1_2], sign1_2 * hs1[ax1_2]);
          var halfLen1 = hs1[edge1Axis];

          // Get edge 2 (on box2)
          edge2Point.copy(box2Center);
          edge2Point.addScaledVector(box2Axes[ax2_1], sign2_1 * hs2[ax2_1]);
          edge2Point.addScaledVector(box2Axes[ax2_2], sign2_2 * hs2[ax2_2]);
          var halfLen2 = hs2[edge2Axis];

          // Find closest points on the two edges
          closestPointsOnSegments(
            edge1Point, box1Axes[edge1Axis], halfLen1,
            edge2Point, box2Axes[edge2Axis], halfLen2,
            closestOnEdge1, closestOnEdge2
          );

          // Contact point is midway between the two closest points
          contactPoint.addVectors(closestOnEdge1, closestOnEdge2).multiplyScalar(0.5);
        }

        var contact = new elation.physics.contact({
          point: contactPoint.clone(),
          normal: contactNormal.clone(),
          penetration: -minPenetration,
          bodies: [box1.body, box2.body]
        });

        contacts.push(contact);
        return contacts;
      };
    })();

    /**
     * Box-Triangle collision using SAT (Separating Axis Theorem)
     * Tests 13 axes: 1 triangle normal + 3 box faces + 9 edge cross products
     */
    this.box_triangle = (function() {
      // Closure scratch variables
      var boxAxes = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
      var triEdges = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
      var triNormal = new THREE.Vector3();
      var boxCenter = new THREE.Vector3();
      var triCenter = new THREE.Vector3();
      var diff = new THREE.Vector3();
      var contactNormal = new THREE.Vector3();
      var contactPoint = new THREE.Vector3();
      var tmpVec = new THREE.Vector3();
      var tmpVec2 = new THREE.Vector3();
      var crossAxis = new THREE.Vector3();
      var triVerts = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];

      // Project box onto axis
      function projectBox(boxCenter, boxAxes, halfsize, axis, out) {
        var centerProj = boxCenter.dot(axis);
        var extent =
          halfsize.x * Math.abs(boxAxes[0].dot(axis)) +
          halfsize.y * Math.abs(boxAxes[1].dot(axis)) +
          halfsize.z * Math.abs(boxAxes[2].dot(axis));
        out.min = centerProj - extent;
        out.max = centerProj + extent;
      }

      // Project triangle onto axis
      function projectTriangle(v0, v1, v2, axis, out) {
        var p0 = v0.dot(axis);
        var p1 = v1.dot(axis);
        var p2 = v2.dot(axis);
        out.min = Math.min(p0, p1, p2);
        out.max = Math.max(p0, p1, p2);
      }

      // Test if projections overlap, return overlap amount or false
      function testOverlap(proj1, proj2) {
        if (proj1.max < proj2.min || proj2.max < proj1.min) {
          return false; // Separating axis found
        }
        return Math.min(proj1.max - proj2.min, proj2.max - proj1.min);
      }

      var proj1 = { min: 0, max: 0 };
      var proj2 = { min: 0, max: 0 };

      return function(box, triangle, contacts, dt) {
        if (!contacts) contacts = [];

        // Get triangle world points
        var worldpoints = triangle.getWorldPoints();
        triVerts[0].copy(worldpoints.p1);
        triVerts[1].copy(worldpoints.p2);
        triVerts[2].copy(worldpoints.p3);

        // Triangle edges
        triEdges[0].subVectors(triVerts[1], triVerts[0]);
        triEdges[1].subVectors(triVerts[2], triVerts[1]);
        triEdges[2].subVectors(triVerts[0], triVerts[2]);

        // Triangle normal (from cached world normal)
        triNormal.copy(worldpoints.normal);

        // Triangle center
        triCenter.copy(triVerts[0]).add(triVerts[1]).add(triVerts[2]).divideScalar(3);

        // Get box center and axes in world space
        // Box center from min/max (handles offset colliders)
        boxCenter.addVectors(box.min, box.max).multiplyScalar(0.5);
        if (boxCenter.lengthSq() > 0) {
          var orient = box.body.orientationWorld || box.body.orientation;
          boxCenter.applyQuaternion(orient);
        }
        boxCenter.add(box.body.position);

        // Box world axes
        var orient = box.body.orientationWorld || box.body.orientation;
        boxAxes[0].set(1, 0, 0).applyQuaternion(orient);
        boxAxes[1].set(0, 1, 0).applyQuaternion(orient);
        boxAxes[2].set(0, 0, 1).applyQuaternion(orient);

        // Vector from box center to triangle center
        diff.subVectors(triCenter, boxCenter);

        var minPenetration = Infinity;
        var minAxisType = -1; // 0=triNormal, 1-3=boxAxes, 4-12=edge cross
        var minAxisIndex = -1;
        var penetration;

        // Test 1: Triangle normal
        tmpVec.copy(triNormal);
        if (tmpVec.lengthSq() > 1e-8) {
          tmpVec.normalize();
          projectBox(boxCenter, boxAxes, box.halfsize, tmpVec, proj1);
          projectTriangle(triVerts[0], triVerts[1], triVerts[2], tmpVec, proj2);
          penetration = testOverlap(proj1, proj2);
          if (penetration === false) return false;
          if (penetration < minPenetration) {
            minPenetration = penetration;
            minAxisType = 0;
            contactNormal.copy(tmpVec);
          }
        }

        // Test 2-4: Box face normals
        for (var i = 0; i < 3; i++) {
          tmpVec.copy(boxAxes[i]);
          projectBox(boxCenter, boxAxes, box.halfsize, tmpVec, proj1);
          projectTriangle(triVerts[0], triVerts[1], triVerts[2], tmpVec, proj2);
          penetration = testOverlap(proj1, proj2);
          if (penetration === false) return false;
          if (penetration < minPenetration) {
            minPenetration = penetration;
            minAxisType = 1;
            minAxisIndex = i;
            contactNormal.copy(tmpVec);
          }
        }

        // Test 5-13: Cross products of triangle edges with box edges
        for (var i = 0; i < 3; i++) {
          for (var j = 0; j < 3; j++) {
            crossAxis.crossVectors(triEdges[i], boxAxes[j]);
            var lenSq = crossAxis.lengthSq();
            if (lenSq < 1e-8) continue; // Skip degenerate (parallel) axes

            tmpVec.copy(crossAxis).normalize();
            projectBox(boxCenter, boxAxes, box.halfsize, tmpVec, proj1);
            projectTriangle(triVerts[0], triVerts[1], triVerts[2], tmpVec, proj2);
            penetration = testOverlap(proj1, proj2);
            if (penetration === false) return false;
            if (penetration < minPenetration) {
              minPenetration = penetration;
              minAxisType = 2;
              minAxisIndex = i * 3 + j;
              contactNormal.copy(tmpVec);
            }
          }
        }

        // Ensure normal points from box toward triangle
        if (contactNormal.dot(diff) < 0) {
          contactNormal.negate();
        }

        // Calculate contact point
        if (minAxisType === 0) {
          // Triangle normal was minimum - contact is on triangle face
          // Find closest point on triangle to box center, clamped
          elation.physics.colliders.helperfuncs.closest_point_on_triangle(
            boxCenter, triVerts[0], triVerts[1], triVerts[2], contactPoint
          );
          // Move toward box by half penetration
          contactPoint.addScaledVector(contactNormal, -minPenetration * 0.5);

        } else if (minAxisType === 1) {
          // Box face normal was minimum - contact is on box face
          // Project triangle center onto box face, clamped to face bounds
          var faceAxisIndex = minAxisIndex;
          var tangent1 = (faceAxisIndex + 1) % 3;
          var tangent2 = (faceAxisIndex + 2) % 3;
          var hs = [box.halfsize.x, box.halfsize.y, box.halfsize.z];

          // Start at box face
          var normalSign = diff.dot(boxAxes[faceAxisIndex]) > 0 ? 1 : -1;
          contactPoint.copy(boxCenter);
          contactPoint.addScaledVector(boxAxes[faceAxisIndex], normalSign * hs[faceAxisIndex]);

          // Project triangle center onto face tangent axes
          tmpVec.subVectors(triCenter, boxCenter);
          var proj1Val = tmpVec.dot(boxAxes[tangent1]);
          proj1Val = Math.max(-hs[tangent1], Math.min(hs[tangent1], proj1Val));
          contactPoint.addScaledVector(boxAxes[tangent1], proj1Val);

          var proj2Val = tmpVec.dot(boxAxes[tangent2]);
          proj2Val = Math.max(-hs[tangent2], Math.min(hs[tangent2], proj2Val));
          contactPoint.addScaledVector(boxAxes[tangent2], proj2Val);

          // Move into contact by half penetration
          contactPoint.addScaledVector(contactNormal, -minPenetration * 0.5);

        } else {
          // Edge-edge contact
          var triEdgeIdx = Math.floor(minAxisIndex / 3);
          var boxEdgeIdx = minAxisIndex % 3;

          // Get triangle edge endpoints
          var triEdgeStart = triVerts[triEdgeIdx];
          var triEdgeEnd = triVerts[(triEdgeIdx + 1) % 3];

          // Get box edge - find the edge closest to the triangle
          var hs = [box.halfsize.x, box.halfsize.y, box.halfsize.z];
          var ax1 = (boxEdgeIdx + 1) % 3;
          var ax2 = (boxEdgeIdx + 2) % 3;

          // Pick signs based on direction to triangle
          var sign1 = diff.dot(boxAxes[ax1]) > 0 ? 1 : -1;
          var sign2 = diff.dot(boxAxes[ax2]) > 0 ? 1 : -1;

          // Box edge center and half-length
          tmpVec.copy(boxCenter);
          tmpVec.addScaledVector(boxAxes[ax1], sign1 * hs[ax1]);
          tmpVec.addScaledVector(boxAxes[ax2], sign2 * hs[ax2]);
          var boxEdgeHalfLen = hs[boxEdgeIdx];

          // Find closest points on the two edges
          // Use line-line closest point calculation
          var d1 = boxAxes[boxEdgeIdx];
          var d2 = triEdges[triEdgeIdx].clone().normalize();
          var triEdgeLen = triEdges[triEdgeIdx].length();

          tmpVec2.subVectors(triEdgeStart, tmpVec);

          var d1d1 = 1; // d1 is unit length
          var d2d2 = 1; // d2 is normalized
          var d1d2 = d1.dot(d2);
          var d1r = d1.dot(tmpVec2);
          var d2r = d2.dot(tmpVec2);

          var denom = d1d1 * d2d2 - d1d2 * d1d2;
          var t, s;
          if (Math.abs(denom) < 1e-8) {
            t = 0;
            s = 0;
          } else {
            t = (d1d2 * d2r - d2d2 * d1r) / denom;
            s = (d1d1 * d2r - d1d2 * d1r) / denom;
          }

          // Clamp to edge bounds
          t = Math.max(-boxEdgeHalfLen, Math.min(boxEdgeHalfLen, t));
          s = Math.max(0, Math.min(triEdgeLen, s));

          // Compute contact point as midpoint
          var boxEdgePoint = tmpVec.clone().addScaledVector(d1, t);
          var triEdgePoint = triEdgeStart.clone().addScaledVector(d2, s);
          contactPoint.addVectors(boxEdgePoint, triEdgePoint).multiplyScalar(0.5);
        }

        var contact = new elation.physics.contact({
          point: contactPoint.clone(),
          normal: contactNormal.clone(),
          penetration: -minPenetration,
          bodies: [box.body, triangle.body],
          triangle: triangle
        });

        contacts.push(contact);
        return contacts;
      };
    })();

    /**
     * Triangle-Box collision (reverses box-triangle)
     */
    this.triangle_box = function(triangle, box, contacts, dt) {
      return this.box_triangle(box, triangle, contacts, dt);
    };

    /**
     * Mesh-Box collision - iterates over mesh triangles
     */
    this.mesh_box = (function() {
      var boxBoundingSphere = { radius: 0 };
      var sphereContacts = [];

      return function(mesh, box, contacts, dt) {
        if (!contacts) contacts = [];

        // Lazy triangle extraction - if mesh has geometry but no triangles, try extracting now
        if (mesh.triangles.length === 0 && ((mesh.mesh && mesh.mesh.geometry) || mesh.modeldata)) {
          mesh.triangles = mesh.extractTriangles(mesh.mesh);
        }

        // Broad phase: check box against mesh bounding sphere
        // Compute box bounding sphere radius
        var boxDiag = Math.sqrt(
          box.halfsize.x * box.halfsize.x +
          box.halfsize.y * box.halfsize.y +
          box.halfsize.z * box.halfsize.z
        );

        // Quick bounding sphere check
        var boxCenter = box.body.positionWorld;
        var meshCenter = mesh.body.positionWorld;
        var centerDist = boxCenter.distanceTo(meshCenter);
        // Compute scaled bounding radius dynamically from current scaleWorld
        var meshScale = mesh.body.scaleWorld;
        var scaledMeshRadius = (mesh.localRadius || mesh.radius) * Math.max(meshScale.x, meshScale.y, meshScale.z);
        var maxDist = boxDiag + scaledMeshRadius;

        if (centerDist > maxDist) {
          return false; // Too far apart
        }

        // Narrow phase: test box against each triangle
        var localcontacts = [];
        var boxMaxDistSq = Math.pow(boxDiag + box.body.velocity.length(), 2);

        for (var i = 0; i < mesh.triangles.length; i++) {
          var triangle = mesh.triangles[i];
          var worldpoints = triangle.getWorldPoints();

          // Quick distance check to triangle center
          var distToCenter = worldpoints.center.distanceToSquared(boxCenter);
          var triRadius = worldpoints.radius || 0;

          if (distToCenter <= boxMaxDistSq + triRadius * triRadius + boxDiag * boxDiag) {
            elation.physics.colliders.helperfuncs.box_triangle(box, triangle, localcontacts, dt);
          }
        }

        // Find the deepest contact
        if (localcontacts.length > 0) {
          var closest = localcontacts[0];
          for (var i = 1; i < localcontacts.length; i++) {
            if (localcontacts[i].penetration < closest.penetration) {
              closest = localcontacts[i];
            }
          }

          // Update bodies to use mesh root
          closest.bodies[1] = mesh.getRoot();
          contacts.push(closest);
        }

        return contacts;
      };
    })();

    /**
     * Box-Mesh collision (reverses mesh-box)
     */
    this.box_mesh = function(box, mesh, contacts, dt) {
      return this.mesh_box(mesh, box, contacts, dt);
    };

    /* cylinder helpers */
    this.cylinder_sphere = function() {
      // closure scratch variables
      var spherepos = new THREE.Vector3();
      var up = new THREE.Vector3();
      var capline = new THREE.Vector3();

      return function(cylinder, sphere, contacts, dt) {
        if (!contacts) contacts = [];
        // Transform sphere position into cylinder's coordinate space
        // TODO - account for offset
        cylinder.body.worldToLocalPos(sphere.body.localToWorldPos(spherepos.set(0,0,0)));
        var halfh = cylinder.height / 2,
            rCylinder = cylinder.radius;
            rSphere = sphere.radius;
        //var type = 'none';

        if (spherepos.y + rSphere - cylinder.offset.y < -halfh || spherepos.y - rSphere - cylinder.offset.y > halfh) {
          // far enough above that we definitely won't hit
          return false;
        }
        var lsq = spherepos.x * spherepos.x + spherepos.z * spherepos.z;
        var rTotal = rSphere + rCylinder;
        if (lsq > rTotal * rTotal) {
          // Outside of cylinder radius
          return false;
        }
        var contact = false;
        if (spherepos.y - cylinder.offset.y > -halfh && spherepos.y - cylinder.offset.y < halfh) {
          // Colliding with side of cylinder (center of sphere is between cylinder ends)
          var penetration = (Math.sqrt(lsq) - rSphere - rCylinder) / 2;
          var normal = spherepos.clone(); // allocate normal
          normal.y = 0;
          normal.normalize();
          var point = normal.clone().multiplyScalar(rCylinder + penetration); // allocate point
          point.y = spherepos.y;

          contact = new elation.physics.contact({
            normal: cylinder.body.localToWorldDir(normal).normalize(), 
            point: cylinder.body.localToWorldPos(point),
            penetration: penetration,
            bodies: [cylinder.body, sphere.body]
          });
          contacts.push(contact);
          //type = 'side';
        } else {
          // Colliding with end caps of cylinder

          up.set(0,1,0);
          spherepos.sub(cylinder.offset);
          capline.crossVectors(up, spherepos).cross(up).normalize();
          var d = spherepos.dot(capline);
          var sign = (Math.abs(spherepos.y) / spherepos.y);
          if (d < cylinder.radius) {
            //type = 'endcap';
            // straight-on collision with end cap
            var point = capline.clone().multiplyScalar(d); // allocate point
            point.y = sign * cylinder.height / 2;
            var penetration = spherepos.distanceTo(point) - sphere.radius;
            
            contact = new elation.physics.contact({
              normal: cylinder.body.localToWorldDir(up.clone().multiplyScalar(sign)).normalize(), // allocate normal
              point: cylinder.body.localToWorldPos(point).add(cylinder.offset),
              penetration: penetration,
              bodies: [cylinder.body, sphere.body]
            });
            contacts.push(contact);
          } else {
            //type = 'edge';
            capline.multiplyScalar(cylinder.radius);
            capline.y = sign * cylinder.height / 2;
            var normal = new THREE.Vector3().subVectors(capline, spherepos); // allocate normal
            var penetration = sphere.radius - normal.length();
            normal.divideScalar(-penetration);
            contact = new elation.physics.contact({
              normal: cylinder.body.localToWorldDir(normal).normalize().negate(), 
              point: cylinder.body.localToWorldPos(capline.clone()).add(cylinder.offset), // allocate point
              penetration: penetration,
              bodies: [cylinder.body, sphere.body]
            });
            contacts.push(contact);
          }
          //console.log(d, spherepos.toArray(), capline.toArray());
        }
        //console.log(type, contact.penetration, contact);
        return contacts;
      }
    }();
    this.sphere_cylinder = function(sphere, cylinder, contacts, dt) {
      return this.cylinder_sphere(cylinder, sphere, contacts, dt);
    }
    this.sphere_triangle = function(sphere, triangle, contacts, dt) {
      return this.triangle_sphere(triangle, sphere, contacts, dt);
    }
    this.cylinder_box = (function() {
      // Closure scratch variables
      var cylAxisStart = new THREE.Vector3();
      var cylAxisEnd = new THREE.Vector3();
      var cylAxisStartLocal = new THREE.Vector3();
      var cylAxisEndLocal = new THREE.Vector3();
      var closestOnAxis = new THREE.Vector3();
      var closestOnBox = new THREE.Vector3();
      var diff = new THREE.Vector3();
      var invQuat = new THREE.Quaternion();
      var tmpVec = new THREE.Vector3();
      var lineDir = new THREE.Vector3();
      var capCenter = new THREE.Vector3();
      var capNormal = new THREE.Vector3();

      // Closest point on line segment to a point
      function closestPointOnSegment(segStart, segEnd, point, out) {
        lineDir.subVectors(segEnd, segStart);
        var lenSq = lineDir.lengthSq();
        if (lenSq < 1e-8) {
          out.copy(segStart);
          return 0;
        }
        var t = tmpVec.subVectors(point, segStart).dot(lineDir) / lenSq;
        t = Math.max(0, Math.min(1, t));
        out.copy(segStart).addScaledVector(lineDir, t);
        return t;
      }

      // Closest point on AABB to a point
      function closestPointOnAABB(point, boxMin, boxMax, out) {
        out.x = Math.max(boxMin.x, Math.min(boxMax.x, point.x));
        out.y = Math.max(boxMin.y, Math.min(boxMax.y, point.y));
        out.z = Math.max(boxMin.z, Math.min(boxMax.z, point.z));
      }

      // Check if a point is inside AABB
      function pointInAABB(point, boxMin, boxMax) {
        return point.x >= boxMin.x && point.x <= boxMax.x &&
               point.y >= boxMin.y && point.y <= boxMax.y &&
               point.z >= boxMin.z && point.z <= boxMax.z;
      }

      return function(cylinder, box, contacts, dt) {
        if (!contacts) contacts = [];

        var halfHeight = cylinder.height / 2;
        // Scale radius by body's world scale (max of X/Z since cylinder is Y-aligned)
        var cylScale = cylinder.body.scaleWorld;
        var cylRadius = cylinder.radius * Math.max(cylScale.x, cylScale.z);

        // Get cylinder axis endpoints in world space
        cylAxisStart.set(0, -halfHeight, 0);
        cylAxisEnd.set(0, halfHeight, 0);
        if (cylinder.offset) {
          cylAxisStart.add(cylinder.offset);
          cylAxisEnd.add(cylinder.offset);
        }
        cylinder.body.localToWorldPos(cylAxisStart);
        cylinder.body.localToWorldPos(cylAxisEnd);

        // Transform to box's local space (where box is axis-aligned)
        cylAxisStartLocal.copy(cylAxisStart).sub(box.body.position);
        cylAxisEndLocal.copy(cylAxisEnd).sub(box.body.position);
        if (box.body.orientation) {
          invQuat.copy(box.body.orientation).invert();
          cylAxisStartLocal.applyQuaternion(invQuat);
          cylAxisEndLocal.applyQuaternion(invQuat);
        }

        var bestContact = null;
        var bestPenetration = -Infinity;

        // === Test 1: Barrel vs Box ===
        // Find closest point pair between cylinder axis and box
        // Use iterative approach like capsule_box
        closestPointOnSegment(cylAxisStartLocal, cylAxisEndLocal,
          tmpVec.addVectors(box.min, box.max).multiplyScalar(0.5), closestOnAxis);

        for (var iter = 0; iter < 3; iter++) {
          closestPointOnAABB(closestOnAxis, box.min, box.max, closestOnBox);
          closestPointOnSegment(cylAxisStartLocal, cylAxisEndLocal, closestOnBox, closestOnAxis);
        }
        closestPointOnAABB(closestOnAxis, box.min, box.max, closestOnBox);

        // Check distance for barrel collision
        diff.subVectors(closestOnAxis, closestOnBox);
        var dist = diff.length();

        if (dist < cylRadius) {
          var penetration = -(cylRadius - dist);
          if (penetration > bestPenetration) {
            bestPenetration = penetration;

            // Transform contact back to world space
            var contactPointWorld = closestOnBox.clone();
            if (box.body.orientation) {
              contactPointWorld.applyQuaternion(box.body.orientation);
            }
            contactPointWorld.add(box.body.position);

            var normal;
            if (dist > 1e-6) {
              // Normal from cylinder toward box (bodies[0] toward bodies[1])
              normal = closestOnBox.clone().sub(closestOnAxis).normalize();
              if (box.body.orientation) {
                normal.applyQuaternion(box.body.orientation);
              }
            } else {
              // Degenerate case - find direction from cylinder center to box center
              normal = box.body.position.clone().sub(cylinder.body.position).normalize();
            }

            bestContact = {
              point: contactPointWorld,
              normal: normal,
              penetration: penetration
            };
          }
        }

        // === Test 2: Caps vs Box ===
        // Check each cap (disk) against the box
        var caps = [
          { center: cylAxisStartLocal.clone(), sign: -1 },
          { center: cylAxisEndLocal.clone(), sign: 1 }
        ];

        // Cylinder's local Y axis in box's local space
        capNormal.subVectors(cylAxisEndLocal, cylAxisStartLocal).normalize();

        for (var c = 0; c < 2; c++) {
          var cap = caps[c];
          var capCenterLocal = cap.center;

          // Find closest point on box to cap center
          closestPointOnAABB(capCenterLocal, box.min, box.max, closestOnBox);

          // Project that point onto the cap plane
          diff.subVectors(closestOnBox, capCenterLocal);
          var distAlongNormal = diff.dot(capNormal);

          // Point on cap plane closest to the box point
          tmpVec.copy(capNormal).multiplyScalar(distAlongNormal);
          var projectedPoint = closestOnBox.clone().sub(tmpVec);

          // Check if projected point is within disk radius
          var distFromAxis = projectedPoint.distanceTo(capCenterLocal);

          if (distFromAxis <= cylRadius) {
            // The projected point is within the disk
            // For a cap collision, we need the box to be on the "inside" of the cap
            // cap.sign=-1 for bottom cap (capNormal points up), box inside if distAlongNormal > 0
            // cap.sign=1 for top cap (capNormal points up), box inside if distAlongNormal < 0
            // So collision when: distAlongNormal * cap.sign < 0
            var insideAmount = -distAlongNormal * cap.sign;

            if (insideAmount > 0 && insideAmount < cylRadius) {
              // Penetration is how far the box has crossed the cap plane
              var penetration = -insideAmount;

              if (penetration > bestPenetration) {
                bestPenetration = penetration;

                var contactPointWorld = closestOnBox.clone();
                if (box.body.orientation) {
                  contactPointWorld.applyQuaternion(box.body.orientation);
                }
                contactPointWorld.add(box.body.position);

                // Normal from cylinder cap toward box (bodies[0] toward bodies[1])
                // Normal should point outward from cap (the direction the cap faces)
                // capNormal points from bottom to top, so cap's outward = capNormal * cap.sign
                var normal = capNormal.clone().multiplyScalar(cap.sign);
                if (box.body.orientation) {
                  normal.applyQuaternion(box.body.orientation);
                }

                bestContact = {
                  point: contactPointWorld,
                  normal: normal,
                  penetration: penetration
                };
              }
            }
          } else {
            // Box might be hitting the rim (edge of cap)
            // Find closest point on rim circle to box
            var toBox = projectedPoint.clone().sub(capCenterLocal);
            if (toBox.lengthSq() > 1e-8) {
              toBox.normalize().multiplyScalar(cylRadius);
              var rimPoint = capCenterLocal.clone().add(toBox);

              closestPointOnAABB(rimPoint, box.min, box.max, closestOnBox);
              diff.subVectors(closestOnBox, rimPoint);
              dist = diff.length();

              // Rim collision - only when rim and box are actually close
              // Use a small fixed tolerance, not a fraction of radius
              if (dist < 0.1) {
                var penetration = -dist;
                if (penetration > bestPenetration) {
                  bestPenetration = penetration;

                  var contactPointWorld = closestOnBox.clone();
                  if (box.body.orientation) {
                    contactPointWorld.applyQuaternion(box.body.orientation);
                  }
                  contactPointWorld.add(box.body.position);

                  // Normal from cylinder rim toward box (bodies[0] toward bodies[1])
                  var normal = diff.clone().normalize();
                  if (box.body.orientation) {
                    normal.applyQuaternion(box.body.orientation);
                  }

                  bestContact = {
                    point: contactPointWorld,
                    normal: normal,
                    penetration: penetration
                  };
                }
              }
            }
          }
        }

        if (bestContact) {
          var contact = new elation.physics.contact({
            point: bestContact.point,
            normal: bestContact.normal,
            penetration: bestContact.penetration,
            bodies: [cylinder.body, box.body]
          });
          contacts.push(contact);
          return contacts;
        }

        return false;
      };
    })();

    this.box_cylinder = function(box, cylinder, contacts, dt) {
      return this.cylinder_box(cylinder, box, contacts, dt);
    };

    this.cylinder_cylinder = (function() {
      // Closure scratch variables
      var axis1Start = new THREE.Vector3();
      var axis1End = new THREE.Vector3();
      var axis2Start = new THREE.Vector3();
      var axis2End = new THREE.Vector3();
      var closestOn1 = new THREE.Vector3();
      var closestOn2 = new THREE.Vector3();
      var diff = new THREE.Vector3();
      var d1 = new THREE.Vector3();
      var d2 = new THREE.Vector3();
      var r = new THREE.Vector3();

      // Closest points between two line segments (using full segment vectors)
      function closestPointsBetweenSegments(p1, q1, p2, q2, out1, out2) {
        d1.subVectors(q1, p1);
        d2.subVectors(q2, p2);
        r.subVectors(p1, p2);

        var a = d1.dot(d1);
        var e = d2.dot(d2);
        var f = d2.dot(r);

        var s, t;

        if (a < 1e-8 && e < 1e-8) {
          out1.copy(p1);
          out2.copy(p2);
          return;
        }
        if (a < 1e-8) {
          s = 0;
          t = Math.max(0, Math.min(1, f / e));
        } else {
          var c = d1.dot(r);
          if (e < 1e-8) {
            t = 0;
            s = Math.max(0, Math.min(1, -c / a));
          } else {
            var b = d1.dot(d2);
            var denom = a * e - b * b;

            if (Math.abs(denom) > 1e-8) {
              s = Math.max(0, Math.min(1, (b * f - c * e) / denom));
            } else {
              s = 0;
            }

            t = (b * s + f) / e;

            if (t < 0) {
              t = 0;
              s = Math.max(0, Math.min(1, -c / a));
            } else if (t > 1) {
              t = 1;
              s = Math.max(0, Math.min(1, (b - c) / a));
            }
          }
        }

        out1.copy(p1).addScaledVector(d1, s);
        out2.copy(p2).addScaledVector(d2, t);
      }

      return function(cylinder1, cylinder2, contacts, dt) {
        if (!contacts) contacts = [];

        // Get cylinder 1 axis in world space
        var halfHeight1 = cylinder1.height / 2;
        axis1Start.set(0, -halfHeight1, 0);
        axis1End.set(0, halfHeight1, 0);
        if (cylinder1.offset) {
          axis1Start.add(cylinder1.offset);
          axis1End.add(cylinder1.offset);
        }
        cylinder1.body.localToWorldPos(axis1Start);
        cylinder1.body.localToWorldPos(axis1End);

        // Get cylinder 2 axis in world space
        var halfHeight2 = cylinder2.height / 2;
        axis2Start.set(0, -halfHeight2, 0);
        axis2End.set(0, halfHeight2, 0);
        if (cylinder2.offset) {
          axis2Start.add(cylinder2.offset);
          axis2End.add(cylinder2.offset);
        }
        cylinder2.body.localToWorldPos(axis2Start);
        cylinder2.body.localToWorldPos(axis2End);

        // Find closest points between the two axes
        closestPointsBetweenSegments(
          axis1Start, axis1End,
          axis2Start, axis2End,
          closestOn1, closestOn2
        );

        // Check barrel-to-barrel collision
        diff.subVectors(closestOn1, closestOn2);
        var dist = diff.length();
        var combinedRadius = cylinder1.radius + cylinder2.radius;

        if (dist < combinedRadius) {
          var penetration = -(combinedRadius - dist);

          var normal;
          if (dist > 1e-6) {
            normal = diff.clone().divideScalar(dist); // Points from cyl2 toward cyl1
          } else {
            // Axes are intersecting - use perpendicular to both
            d1.subVectors(axis1End, axis1Start);
            d2.subVectors(axis2End, axis2Start);
            normal = d1.clone().cross(d2);
            if (normal.lengthSq() < 1e-8) {
              normal.set(1, 0, 0);
            }
            normal.normalize();
          }

          // Contact point between the two surfaces
          var contactPoint = closestOn2.clone().addScaledVector(normal, cylinder2.radius);

          var contact = new elation.physics.contact({
            point: contactPoint,
            normal: normal,
            penetration: penetration,
            bodies: [cylinder2.body, cylinder1.body]
          });
          contacts.push(contact);
          return contacts;
        }

        // TODO: Check cap-to-cap and cap-to-barrel collisions

        return contacts;
      };
    })();
    this.cylinder_plane = function() {
      var up = new THREE.Vector3();
      var planenorm = new THREE.Vector3();
      var dir = new THREE.Vector3();
      var centerpoint = new THREE.Vector3();
      var point = new THREE.Vector3();
      var tolerance = 1e-6;

      var checkPoint = function(point, cylinder, plane, contacts) {
        var contact = elation.physics.colliders.helperfuncs.vertex_plane(point, plane);
        if (contact) {
          contact.bodies = [cylinder.body, plane.body];
          contacts.push(contact);
        }
      }
      var checkEndPoints = function(centerpoint, offset, cylinder, plane, contacts) {
        point.addVectors(centerpoint, offset);
        checkPoint(point, cylinder, plane, contacts);
        point.subVectors(centerpoint, offset);
        checkPoint(point, cylinder, plane, contacts);
      }

      return function(cylinder, plane, contacts, dt) {
        if (!contacts) contacts = [];
        cylinder.body.localToWorldDir(up.set(0,1,0));
        plane.body.localToWorldDir(planenorm.copy(plane.normal));
        var dot = planenorm.dot(up);
        dir.crossVectors(planenorm, up).cross(up).normalize().multiplyScalar(cylinder.radius);
        
        // TODO - handle cases where cylinder is parallel or perpendicular to plane

  /*
        if (Math.abs(dot) <= tolerance) { // parallel to plane - generate two contacts, one at each end
          console.log('parallel!');
        } else if (Math.abs(Math.abs(dot) - 1) <= tolerance) { // perpendicular to plane - generate three contacts at 120 degree increments
          console.log('perpendicular!');
        } else {
  */
          // top point
          cylinder.body.localToWorldPos(centerpoint.set(0,cylinder.height/2, 0));
          checkEndPoints(centerpoint, dir, cylinder, plane, contacts);

          // bottom point
          cylinder.body.localToWorldPos(centerpoint.set(0,-cylinder.height/2, 0));
          checkEndPoints(centerpoint, dir, cylinder, plane, contacts);
          //console.log('up:', up.toArray(), 'planenorm:', planenorm.toArray(), 'top:', top.toArray(), 'bottom:', centerpoint.toArray(), 'dir:', dir.toArray());
  //      }
        return contacts;
      }
    }();

    /* capsule helpers */
    this.capsule_sphere = function() {
      // closure scratch variables
      const spherepos = new THREE.Vector3(),
            point = new THREE.Vector3(),
            normal = new THREE.Vector3(),
            closest = new THREE.Vector3();

      return function(capsule, sphere, contacts, dt) {
        const capsuleScaledRadius = capsule.radius * Math.max(capsule.body.scale.x, capsule.body.scale.z),
              combinedRadius = capsuleScaledRadius + sphere.radius,
              capsuleDims = capsule.getDimensions();
        sphere.body.localToWorldPos(point.set(0,0,0));


        elation.physics.colliders.helperfuncs.closest_point_on_line(capsuleDims.start, capsuleDims.end, point, closest);

        normal.subVectors(closest, point);
        const distance = normal.length();

        if (distance <= combinedRadius) {
          normal.divideScalar(distance);
          let contact = new elation.physics.contact({
            normal: normal.clone(), // allocate normal
            point: closest.clone().add(normal.multiplyScalar(capsuleScaledRadius)), // allocate point
            penetration: combinedRadius - distance,
            bodies: [capsule.body, sphere.body]
          });
          contacts.push(contact);
        }
        return contacts;
      }
    }();
    this.capsule_capsule = function() {
      // closure scratch variables
      const p1 = new THREE.Vector3(),
            p2 = new THREE.Vector3();

      return function(capsule1, capsule2, contacts, dt) {
        const capsule1Dims = capsule1.getDimensions(),
              capsule2Dims = capsule2.getDimensions();

        let distSquared = elation.physics.colliders.helperfuncs.distancesquared_between_lines(capsule1Dims.start, capsule1Dims.end, capsule2Dims.start, capsule2Dims.end, p1, p2);

        const capsule1ScaledRadius = capsule1.radius * Math.max(capsule1.body.scale.x, capsule1.body.scale.z),
              capsule2ScaledRadius = capsule2.radius * Math.max(capsule2.body.scale.x, capsule2.body.scale.z);

        if (distSquared <= Math.pow(capsule1ScaledRadius + capsule2ScaledRadius, 2)) {
          console.log('CAPSULE COLLIDE', capsule1, capsule2);
          let normal = new THREE.Vector3().subVectors(p2, p1),
              point = p1.clone();
              dist = Math.sqrt(distSquared);
          normal.divideScalar(dist);
          point.x += normal.x * capsule1ScaledRadius;
          point.y += normal.y * capsule1ScaledRadius;
          point.z += normal.z * capsule1ScaledRadius;

          let contact = new elation.physics.contact({
            normal: normal,
            point: point,
            penetration: dist - (capsule1ScaledRadius + capsule2ScaledRadius),
            bodies: [capsule1.body, capsule2.body]
          });
          contacts.push(contact);
          return contacts;
        }
      }
    }();
    this.capsule_box = (function() {
      // closure scratch variables
      var startWorld = new THREE.Vector3();
      var endWorld = new THREE.Vector3();
      var startLocal = new THREE.Vector3();
      var endLocal = new THREE.Vector3();
      var closestOnCapsule = new THREE.Vector3();
      var closestOnBox = new THREE.Vector3();
      var closestOnBoxWorld = new THREE.Vector3();
      var closestOnCapsuleWorld = new THREE.Vector3();
      var diff = new THREE.Vector3();
      var lineDir = new THREE.Vector3();
      var invQuat = new THREE.Quaternion();
      var tmpVec = new THREE.Vector3();

      // Find closest point on line segment to a point
      function closestPointOnSegment(segStart, segEnd, point, out) {
        lineDir.subVectors(segEnd, segStart);
        var lenSq = lineDir.lengthSq();
        if (lenSq < 1e-8) {
          out.copy(segStart);
          return 0;
        }
        var t = tmpVec.subVectors(point, segStart).dot(lineDir) / lenSq;
        t = Math.max(0, Math.min(1, t));
        out.copy(segStart).addScaledVector(lineDir, t);
        return t;
      }

      // Find closest point on AABB to a point
      function closestPointOnAABB(point, boxMin, boxMax, out) {
        out.x = Math.max(boxMin.x, Math.min(boxMax.x, point.x));
        out.y = Math.max(boxMin.y, Math.min(boxMax.y, point.y));
        out.z = Math.max(boxMin.z, Math.min(boxMax.z, point.z));
      }

      return function(capsule, box, contacts, dt) {
        if (!contacts) contacts = [];

        // Get capsule axis endpoints in world space
        startWorld.set(0, 0, 0);
        endWorld.set(0, capsule.length, 0);
        if (capsule.offset) {
          startWorld.add(capsule.offset);
          endWorld.add(capsule.offset);
        }
        capsule.body.localToWorldPos(startWorld);
        capsule.body.localToWorldPos(endWorld);

        // Get scaled capsule radius (account for non-uniform scale)
        var capsuleScaledRadius = capsule.radius * Math.max(capsule.body.scale.x, capsule.body.scale.z);

        // Transform capsule endpoints to box's scaled local space
        // (same coordinate space as box.min/max)
        startLocal.copy(startWorld).sub(box.body.position);
        endLocal.copy(endWorld).sub(box.body.position);
        if (box.body.orientation) {
          invQuat.copy(box.body.orientation).invert();
          startLocal.applyQuaternion(invQuat);
          endLocal.applyQuaternion(invQuat);
        }

        // Find closest point pair between capsule axis and box
        // Use iterative approach: alternate between finding closest on box and closest on line
        // Start with midpoint of capsule
        closestPointOnSegment(startLocal, endLocal, tmpVec.addVectors(box.min, box.max).multiplyScalar(0.5), closestOnCapsule);

        // Iterate to converge on closest pair (usually 2-3 iterations is enough)
        for (var iter = 0; iter < 3; iter++) {
          closestPointOnAABB(closestOnCapsule, box.min, box.max, closestOnBox);
          closestPointOnSegment(startLocal, endLocal, closestOnBox, closestOnCapsule);
        }

        // Final closest point on box
        closestPointOnAABB(closestOnCapsule, box.min, box.max, closestOnBox);

        // Check distance
        diff.subVectors(closestOnCapsule, closestOnBox);
        var distSq = diff.lengthSq();

        if (distSq > capsuleScaledRadius * capsuleScaledRadius) {
          return false; // No collision
        }

        var dist = Math.sqrt(distSq);

        // Transform points back to world space
        closestOnBoxWorld.copy(closestOnBox);
        closestOnCapsuleWorld.copy(closestOnCapsule);
        if (box.body.orientation) {
          closestOnBoxWorld.applyQuaternion(box.body.orientation);
          closestOnCapsuleWorld.applyQuaternion(box.body.orientation);
        }
        closestOnBoxWorld.add(box.body.position);
        closestOnCapsuleWorld.add(box.body.position);

        // Calculate normal (from box toward capsule)
        var normal;
        if (dist > 1e-6) {
          normal = closestOnCapsuleWorld.clone().sub(closestOnBoxWorld).normalize();
        } else {
          // Capsule axis passes through box - use box face normal
          // Find which face the capsule is closest to
          var minDist = Infinity;
          normal = new THREE.Vector3(0, 1, 0);

          var distToMinX = closestOnCapsule.x - box.min.x;
          var distToMaxX = box.max.x - closestOnCapsule.x;
          var distToMinY = closestOnCapsule.y - box.min.y;
          var distToMaxY = box.max.y - closestOnCapsule.y;
          var distToMinZ = closestOnCapsule.z - box.min.z;
          var distToMaxZ = box.max.z - closestOnCapsule.z;

          if (distToMinX < minDist) { minDist = distToMinX; normal.set(-1, 0, 0); }
          if (distToMaxX < minDist) { minDist = distToMaxX; normal.set(1, 0, 0); }
          if (distToMinY < minDist) { minDist = distToMinY; normal.set(0, -1, 0); }
          if (distToMaxY < minDist) { minDist = distToMaxY; normal.set(0, 1, 0); }
          if (distToMinZ < minDist) { minDist = distToMinZ; normal.set(0, 0, -1); }
          if (distToMaxZ < minDist) { minDist = distToMaxZ; normal.set(0, 0, 1); }

          // Transform normal to world space
          if (box.body.orientation) {
            normal.applyQuaternion(box.body.orientation);
          }
        }

        // Contact point is on the box surface
        var contact = new elation.physics.contact({
          point: closestOnBoxWorld.clone(),
          normal: normal,
          penetration: -(capsuleScaledRadius - dist),
          bodies: [box.body, capsule.body]
        });
        contacts.push(contact);

        return contacts;
      };
    })();

    /**
     * Box-Capsule collision (reverses capsule-box)
     */
    this.box_capsule = function(box, capsule, contacts, dt) {
      return this.capsule_box(capsule, box, contacts, dt);
    };

    this.capsule_cylinder = (function() {
      // Closure scratch variables
      var capsuleStart = new THREE.Vector3();
      var capsuleEnd = new THREE.Vector3();
      var cylAxisStart = new THREE.Vector3();
      var cylAxisEnd = new THREE.Vector3();
      var closestOnCapsule = new THREE.Vector3();
      var closestOnCylinder = new THREE.Vector3();
      var diff = new THREE.Vector3();
      var tmpVec = new THREE.Vector3();
      var d1 = new THREE.Vector3();
      var d2 = new THREE.Vector3();
      var r = new THREE.Vector3();

      // Closest points between two line segments (using full segment vectors)
      // Based on Real-Time Collision Detection by Christer Ericson
      function closestPointsBetweenSegments(p1, q1, p2, q2, out1, out2) {
        d1.subVectors(q1, p1); // Segment 1 direction (unnormalized)
        d2.subVectors(q2, p2); // Segment 2 direction (unnormalized)
        r.subVectors(p1, p2);

        var a = d1.dot(d1); // Squared length of segment 1
        var e = d2.dot(d2); // Squared length of segment 2
        var f = d2.dot(r);

        var s, t;

        // Check if either or both segments are points
        if (a < 1e-8 && e < 1e-8) {
          // Both segments are points
          out1.copy(p1);
          out2.copy(p2);
          return;
        }
        if (a < 1e-8) {
          // First segment is a point
          s = 0;
          t = Math.max(0, Math.min(1, f / e));
        } else {
          var c = d1.dot(r);
          if (e < 1e-8) {
            // Second segment is a point
            t = 0;
            s = Math.max(0, Math.min(1, -c / a));
          } else {
            // General case
            var b = d1.dot(d2);
            var denom = a * e - b * b;

            if (Math.abs(denom) > 1e-8) {
              s = Math.max(0, Math.min(1, (b * f - c * e) / denom));
            } else {
              s = 0; // Parallel segments, pick arbitrary s
            }

            // Compute t for the closest point on segment 2
            t = (b * s + f) / e;

            // Clamp t and recompute s if needed
            if (t < 0) {
              t = 0;
              s = Math.max(0, Math.min(1, -c / a));
            } else if (t > 1) {
              t = 1;
              s = Math.max(0, Math.min(1, (b - c) / a));
            }
          }
        }

        out1.copy(p1).addScaledVector(d1, s);
        out2.copy(p2).addScaledVector(d2, t);
      }

      return function(capsule, cylinder, contacts, dt) {
        if (!contacts) contacts = [];

        // Get capsule axis in world space
        capsuleStart.set(0, 0, 0);
        capsuleEnd.set(0, capsule.length, 0);
        if (capsule.offset) {
          capsuleStart.add(capsule.offset);
          capsuleEnd.add(capsule.offset);
        }
        capsule.body.localToWorldPos(capsuleStart);
        capsule.body.localToWorldPos(capsuleEnd);

        // Get cylinder axis in world space
        var halfHeight = cylinder.height / 2;
        cylAxisStart.set(0, -halfHeight, 0);
        cylAxisEnd.set(0, halfHeight, 0);
        if (cylinder.offset) {
          cylAxisStart.add(cylinder.offset);
          cylAxisEnd.add(cylinder.offset);
        }
        cylinder.body.localToWorldPos(cylAxisStart);
        cylinder.body.localToWorldPos(cylAxisEnd);

        // Calculate scaled radii
        var capsuleScale = capsule.body.scaleWorld;
        var capsuleScaledRadius = capsule.radius * Math.max(capsuleScale.x, capsuleScale.z);
        var cylinderScale = cylinder.body.scaleWorld;
        var cylinderScaledRadius = cylinder.radius * Math.max(cylinderScale.x, cylinderScale.z);

        // Find closest points between the two axes
        closestPointsBetweenSegments(
          capsuleStart, capsuleEnd,
          cylAxisStart, cylAxisEnd,
          closestOnCapsule, closestOnCylinder
        );

        // Check barrel collision
        diff.subVectors(closestOnCapsule, closestOnCylinder);
        var dist = diff.length();
        var combinedRadius = capsuleScaledRadius + cylinderScaledRadius;

        if (dist < combinedRadius) {
          var penetration = -(combinedRadius - dist);

          var normal;
          if (dist > 1e-6) {
            normal = diff.clone().divideScalar(dist); // Points from cylinder toward capsule
          } else {
            // Axes are intersecting - use perpendicular to both
            d1.subVectors(capsuleEnd, capsuleStart);
            d2.subVectors(cylAxisEnd, cylAxisStart);
            normal = d1.clone().cross(d2);
            if (normal.lengthSq() < 1e-8) {
              normal.set(1, 0, 0); // Fallback
            }
            normal.normalize();
          }

          // Contact point is on the cylinder surface toward the capsule
          var contactPoint = closestOnCylinder.clone().addScaledVector(normal, cylinderScaledRadius);

          var contact = new elation.physics.contact({
            point: contactPoint,
            normal: normal,
            penetration: penetration,
            bodies: [cylinder.body, capsule.body]
          });
          contacts.push(contact);
          return contacts;
        }

        // TODO: Check capsule sphere ends against cylinder caps

        return contacts;
      };
    })();

    this.cylinder_capsule = function(cylinder, capsule, contacts, dt) {
      return this.capsule_cylinder(capsule, cylinder, contacts, dt);
    };
    this.triangle_sphere = function() {
      // closure scratch variables
      const sphereClosestPointToPlane = new THREE.Vector3(),
            endpos = new THREE.Vector3(),
            scaledVelocity = new THREE.Vector3(),
            intersectionPoint = new THREE.Vector3(),
            triangleClosestPoint = new THREE.Vector3();

      // Reference: http://www.peroxide.dk/papers/collision/collision.pdf

      return function(triangle, sphere, contacts, dt) {
        const spherepos = sphere.body.positionWorld;
        const spherevel = sphere.body.velocity;
        const worldpoints = triangle.getWorldPoints();
        const p1 = worldpoints.p1,
              p2 = worldpoints.p2,
              p3 = worldpoints.p3,
              normal = worldpoints.normal;

        let velNormal = spherevel.dot(normal);

        // Check if we're already in contact
        elation.physics.colliders.helperfuncs.closest_point_on_triangle(sphere.body.positionWorld, p1, p2, p3, triangleClosestPoint);
        let triangleDistSquared = triangleClosestPoint.distanceToSquared(sphere.body.positionWorld)
        if (triangleDistSquared < sphere.radius * sphere.radius && velNormal <= 0) {
          // Compute collision normal pointing from triangle toward sphere (not the face normal)
          let collisionNormal = sphere.body.positionWorld.clone().sub(triangleClosestPoint); // allocate normal
          let dist = Math.sqrt(triangleDistSquared);
          if (dist > 1e-6) {
            collisionNormal.divideScalar(dist);
          } else {
            // Degenerate case - sphere center is exactly on triangle, use face normal
            // but ensure it points toward the sphere (away from the triangle's front face)
            collisionNormal.copy(normal);
            // Check which side the sphere came from using velocity
            if (velNormal > 0) {
              collisionNormal.negate();
            }
          }
          let contact = new elation.physics.contact({
            normal: collisionNormal,
            point: triangleClosestPoint.clone(), // allocate point
            penetration: dist - sphere.radius,
            bodies: [triangle.body, sphere.body],
            triangle: triangle
          });
          contacts.push(contact);
          return contacts;
        }

        sphereClosestPointToPlane.x = (normal.x * -sphere.radius) + spherepos.x;
        sphereClosestPointToPlane.y = (normal.y * -sphere.radius) + spherepos.y;
        sphereClosestPointToPlane.z = (normal.z * -sphere.radius) + spherepos.z;
        let sphereOffset = sphere.offset;
        if (sphereOffset) {
          if (sphereOffset._target) sphereOffset = sphereOffset._target;
          sphereClosestPointToPlane.add(sphereOffset);
        }

        scaledVelocity.x = spherevel.x * dt;
        scaledVelocity.y = spherevel.y * dt;
        scaledVelocity.z = spherevel.z * dt;

        endpos.x = sphereClosestPointToPlane.x + scaledVelocity.x;
        endpos.y = sphereClosestPointToPlane.y + scaledVelocity.y;
        endpos.z = sphereClosestPointToPlane.z + scaledVelocity.z;

        // Find intersection between the ray our sphere is travelling and the plane upon which our triangle rests
        let intersectionPlane = elation.physics.colliders.helperfuncs.line_plane(sphereClosestPointToPlane, endpos, p1, p2, p3, intersectionPoint);
        if (intersectionPlane && triangle.containsPoint(intersectionPlane.point)) {
          // If the intersection point is inside of our triangle, we've collided with the triangle's face
          // Compute collision normal pointing from triangle toward sphere (based on approach direction)
          // The sphere is approaching from the direction opposite to its velocity
          let collisionNormal = normal.clone(); // allocate normal
          // If velocity is going WITH the normal (same direction), the sphere is on the back side
          // and we need to flip the normal to point toward the sphere
          if (velNormal > 0) {
            collisionNormal.negate();
          }
          let contact = new elation.physics.contact_dynamic({
            normal: collisionNormal,
            point: intersectionPlane.point.clone(), // allocate point
            penetrationTime: intersectionPlane.t,
            bodies: [triangle.body, sphere.body],
            triangle: triangle
          });
          contacts.push(contact);
          return contacts;
        }
        // Check sphere against edges of triangle
        endpos.x = spherepos.x + scaledVelocity.x;
        endpos.y = spherepos.y + scaledVelocity.y;
        endpos.z = spherepos.z + scaledVelocity.z;
        // FIXME - this is probably more efficient if we find the closest point on each edge first, use that to pick the closest edge, and then perform only one cylinder intersection test
        let intersections = [
          elation.physics.colliders.helperfuncs.line_cylinder(spherepos, endpos, p1, p2, sphere.radius),
          elation.physics.colliders.helperfuncs.line_cylinder(spherepos, endpos, p2, p3, sphere.radius),
          elation.physics.colliders.helperfuncs.line_cylinder(spherepos, endpos, p3, p1, sphere.radius),
        ];

        //console.log(intersections, sphere.body.position, endpos);
        let closestIntersectionDist = Infinity;
        let closestIntersection = null;
        for (let i = 0; i < intersections.length; i++) {
          if (intersections[i] != undefined && intersections[i] < closestIntersectionDist) {
            closestIntersectionDist = intersections[i];
            closestIntersection = i;
          }
        }

        if (closestIntersection !== null) {
          const intersectionPoint = endpos.clone().sub(spherepos).multiplyScalar(closestIntersectionDist).add(spherepos); // allocate point

          elation.physics.colliders.helperfuncs.closest_point_on_triangle(intersectionPoint, p1, p2, p3, triangleClosestPoint);
          let collisionNormal = new THREE.Vector3( // allocate normal
            intersectionPoint.x - triangleClosestPoint.x,
            intersectionPoint.y - triangleClosestPoint.y,
            intersectionPoint.z - triangleClosestPoint.z
          );
          collisionNormal.normalize();

          intersectionPoint.x += collisionNormal.x * -sphere.radius;
          intersectionPoint.y += collisionNormal.y * -sphere.radius;
          intersectionPoint.z += collisionNormal.z * -sphere.radius;

          let contact = new elation.physics.contact_dynamic({
            normal: collisionNormal,
            point: intersectionPoint,
            penetrationTime: closestIntersectionDist,
            bodies: [triangle.body, sphere.body],
            triangle: triangle
          });

          contacts.push(contact);
          return contacts;
        }
      }
    }();
    this.triangle_capsule = function() {
      // Scratch variables
      const capsuleLine = new THREE.Vector3(),
            capsuleStart = new THREE.Vector3(),
            capsuleEnd = new THREE.Vector3(),
            closestPoint = new THREE.Vector3(),
            intersectionPoint = new THREE.Vector3(),
            //normal = new THREE.Vector3(),
            capsuleNormal = new THREE.Vector3(),
            sphereCenter = new THREE.Vector3(),
            localSphere = new elation.physics.rigidbody();

      return function(triangle, capsule, contacts, dt) {
        //triangle.body.localToWorldDir(normal.copy(triangle.normal));

        const capsuleDims = capsule.getDimensions();
        const scaledRadius = capsuleDims.scaledRadius;
        capsuleNormal.subVectors(capsuleDims.end, capsuleDims.start).normalize();
        localSphere.position.copy(capsule.body.position);
        localSphere.positionWorld.copy(capsule.body.positionWorld);

        const worldpoints = triangle.getWorldPoints();
        const p1 = worldpoints.p1,
              p2 = worldpoints.p2,
              p3 = worldpoints.p3,
              normal = worldpoints.normal;

        // Find the closest point of the capsule to the triangle
        let t = normal.dot(capsuleLine.subVectors(p1, capsuleDims.start).divideScalar(Math.abs(normal.dot(capsuleNormal))));
        intersectionPoint.copy(capsuleNormal).multiplyScalar(t).add(capsuleDims.start);
        elation.physics.colliders.helperfuncs.closest_point_on_triangle(intersectionPoint, p1, p2, p3, closestPoint);
        elation.physics.colliders.helperfuncs.closest_point_on_line(capsuleDims.start, capsuleDims.end, closestPoint, localSphere.position);

        localSphere.positionWorld.copy(localSphere.position);

        // Perform a sphere/triangle intersection test with our sphere
        // Use the scaled radius from getDimensions
        if (!localSphere.collider) {
            localSphere.setCollider('sphere', { radius: scaledRadius });
        } else {
          localSphere.collider.radius = scaledRadius;
        }
        localSphere.orientation.copy(capsule.body.orientation);
        localSphere.orientationWorld.copy(capsule.body.orientationWorld);
        localSphere.velocity.copy(capsule.body.velocity);
        let localcontacts = [];
        elation.physics.colliders.helperfuncs.triangle_sphere(triangle, localSphere.collider, localcontacts, dt);
        if (localcontacts.length > 0) {
          let contact = localcontacts[0];
          contact.bodies[1] = capsule.body;
          contacts.push(contact);
          return contacts;
        }
      }
    }();
    this.mesh_sphere = function() {
      return function(mesh, sphere, contacts, dt) {
        // Lazy triangle extraction - if mesh has geometry but no triangles, try extracting now
        if (mesh.triangles.length === 0 && mesh.mesh && mesh.mesh.geometry) {
          mesh.triangles = mesh.extractTriangles(mesh.mesh);
        }

        var localcontacts = [], spherecontacts = [];

        // Update bounding sphere radius dynamically from current scale
        var meshScale = mesh.body.scaleWorld;
        mesh.boundingSphere.radius = (mesh.localRadius || mesh.radius) * Math.max(meshScale.x, meshScale.y, meshScale.z);

        elation.physics.colliders.helperfuncs.sphere_sphere(sphere, mesh.boundingSphere, spherecontacts, dt);
        if (spherecontacts.length == 0) return;

        let spherepos = sphere.body.positionWorld,
            sphereMaxDist = sphere.radius * sphere.radius + sphere.body.velocity.lengthSq();
        for (var i = 0; i < mesh.triangles.length; i++) {
          let triangle = mesh.triangles[i],
              worldpoints = triangle.getWorldPoints(),
              distToCenter = worldpoints.center.distanceToSquared(spherepos);
          if (distToCenter <= sphereMaxDist + worldpoints.radius * worldpoints.radius) {
            elation.physics.colliders.helperfuncs.triangle_sphere(mesh.triangles[i], sphere, localcontacts, dt);
          }
        }

        if (localcontacts.length > 0) {
          let closestStatic = false,
              closestDynamic = false;
          for (let i = 0; i < localcontacts.length; i++) {
            let contact = localcontacts[i];
            if (contact instanceof elation.physics.contact_dynamic) {
              if (!closestDynamic || closestDynamic.penetrationTime > contact.penetrationTime) {
                closestDynamic = contact;
              }
            } else {
              if (!closestStatic || closestStatic.penetration > contact.penetration) {
                closestStatic = contact;
              }
            }
          }

          // Handle static contacts first, since they're equivalent to penetrationTime=0
          let closest = closestStatic || closestDynamic;
          if (closest.bodies[0] === sphere) {
            closest.bodies[0] = sphere.body;
            closest.bodies[1] = mesh.getRoot();
          } else if (closest.bodies[1] === sphere) {
            closest.bodies[0] = mesh.getRoot();
            closest.bodies[1] = sphere.body;
          }
          contacts.push(closest);
        }

        return contacts;
      }
    }();
    this.mesh_capsule = function() {
      return function(mesh, capsule, contacts, dt) {
        // Lazy triangle extraction - if mesh has geometry but no triangles, try extracting now
        if (mesh.triangles.length === 0 && mesh.mesh && mesh.mesh.geometry) {
          mesh.triangles = mesh.extractTriangles(mesh.mesh);
        }

        var localcontacts = [], spherecontacts = [];

        // Update bounding sphere radius dynamically from current scale
        var meshScale = mesh.body.scaleWorld;
        mesh.boundingSphere.radius = (mesh.localRadius || mesh.radius) * Math.max(meshScale.x, meshScale.y, meshScale.z);

        elation.physics.colliders.helperfuncs.capsule_sphere(capsule, mesh.boundingSphere, spherecontacts, dt);
        if (spherecontacts.length == 0) return;

        // Get scaled capsule dimensions for distance culling
        let capsuleDims = capsule.getDimensions();
        let scaledLength = capsuleDims.start.distanceTo(capsuleDims.end);
        let scaledRadius = capsuleDims.scaledRadius;

        let capsulepos = capsule.body.positionWorld,
            capsuleMaxDist = Math.pow(scaledLength + scaledRadius, 2);
        for (var i = 0; i < mesh.triangles.length; i++) {
          let triangle = mesh.triangles[i],
              worldpoints = triangle.getWorldPoints(),
              distToCenter = worldpoints.center.distanceToSquared(capsulepos);
          if (distToCenter <= capsuleMaxDist + worldpoints.radius * worldpoints.radius) {
            elation.physics.colliders.helperfuncs.triangle_capsule(triangle, capsule, localcontacts, dt);
          }
        }

        if (localcontacts.length > 0) {
          let closestStatic = false,
              closestDynamic = false;
          for (let i = 0; i < localcontacts.length; i++) {
            let contact = localcontacts[i];
            if (contact instanceof elation.physics.contact_dynamic) {
              if (!closestDynamic || closestDynamic.penetrationTime > contact.penetrationTime) {
                closestDynamic = contact;
              }
            } else {
              if (!closestStatic || closestStatic.penetration > contact.penetration) {
                closestStatic = contact;
              }
            }
          }

          // Handle static contacts first, since they're equivalent to penetrationTime=0
          let closest = closestStatic || closestDynamic;
          if (closest.bodies[0] === capsule) {
            closest.bodies[0] = capsule.body;
            closest.bodies[1] = mesh.getRoot();
          } else if (closest.bodies[1] === capsule) {
            closest.bodies[0] = mesh.getRoot();
            closest.bodies[1] = capsule.body;
          }
          contacts.push(closest);
        }

        return contacts;
      }
    }();

    this.triangle_cylinder = function() {
      // Scratch variables
      const cylAxisStart = new THREE.Vector3(),
            cylAxisEnd = new THREE.Vector3(),
            closestOnAxis = new THREE.Vector3(),
            closestOnTriangle = new THREE.Vector3(),
            diff = new THREE.Vector3(),
            lineDir = new THREE.Vector3(),
            tmpVec = new THREE.Vector3(),
            capCenter = new THREE.Vector3(),
            capNormal = new THREE.Vector3(),
            edgeStart = new THREE.Vector3(),
            edgeEnd = new THREE.Vector3(),
            closestOnEdge = new THREE.Vector3();

      // Closest point on line segment to a point
      function closestPointOnSegment(segStart, segEnd, point, out) {
        lineDir.subVectors(segEnd, segStart);
        var lenSq = lineDir.lengthSq();
        if (lenSq < 1e-8) {
          out.copy(segStart);
          return 0;
        }
        var t = tmpVec.subVectors(point, segStart).dot(lineDir) / lenSq;
        t = Math.max(0, Math.min(1, t));
        out.copy(segStart).addScaledVector(lineDir, t);
        return t;
      }

      return function(triangle, cylinder, contacts, dt) {
        if (!contacts) contacts = [];

        var halfHeight = cylinder.height / 2;
        // Scale radius by body's world scale (max of X/Z since cylinder is Y-aligned)
        var cylScale = cylinder.body.scaleWorld;
        var cylRadius = cylinder.radius * Math.max(cylScale.x, cylScale.z);

        // Get cylinder axis endpoints in world space
        cylAxisStart.set(0, -halfHeight, 0);
        cylAxisEnd.set(0, halfHeight, 0);
        if (cylinder.offset) {
          cylAxisStart.add(cylinder.offset);
          cylAxisEnd.add(cylinder.offset);
        }
        cylinder.body.localToWorldPos(cylAxisStart);
        cylinder.body.localToWorldPos(cylAxisEnd);

        // Get triangle world points
        const worldpoints = triangle.getWorldPoints();
        const p1 = worldpoints.p1,
              p2 = worldpoints.p2,
              p3 = worldpoints.p3,
              triNormal = worldpoints.normal;

        var bestContact = null;
        var bestPenetration = -Infinity;

        // === Test 1: Barrel vs Triangle ===
        // Find closest point on triangle to cylinder axis
        // First, find where the axis intersects the triangle plane
        capNormal.subVectors(cylAxisEnd, cylAxisStart).normalize();
        var denom = triNormal.dot(capNormal);

        if (Math.abs(denom) > 1e-6) {
          // Axis is not parallel to triangle
          var t = triNormal.dot(tmpVec.subVectors(p1, cylAxisStart)) / denom;
          tmpVec.copy(cylAxisStart).addScaledVector(capNormal, t);
        } else {
          // Axis is parallel - use midpoint projected onto triangle plane
          tmpVec.copy(cylAxisStart).add(cylAxisEnd).multiplyScalar(0.5);
        }

        // Find closest point on triangle to this intersection point
        elation.physics.colliders.helperfuncs.closest_point_on_triangle(tmpVec, p1, p2, p3, closestOnTriangle);

        // Find closest point on cylinder axis to that triangle point
        closestPointOnSegment(cylAxisStart, cylAxisEnd, closestOnTriangle, closestOnAxis);

        // Iterate to refine (like capsule_box)
        for (var iter = 0; iter < 2; iter++) {
          elation.physics.colliders.helperfuncs.closest_point_on_triangle(closestOnAxis, p1, p2, p3, closestOnTriangle);
          closestPointOnSegment(cylAxisStart, cylAxisEnd, closestOnTriangle, closestOnAxis);
        }
        elation.physics.colliders.helperfuncs.closest_point_on_triangle(closestOnAxis, p1, p2, p3, closestOnTriangle);

        // Check barrel collision
        diff.subVectors(closestOnAxis, closestOnTriangle);
        var dist = diff.length();

        if (dist < cylRadius && dist > 1e-6) {
          var penetration = -(cylRadius - dist);
          if (penetration > bestPenetration) {
            bestPenetration = penetration;

            // Normal from cylinder toward triangle (bodies[0] toward bodies[1])
            var normal = closestOnTriangle.clone().sub(closestOnAxis).normalize();

            // Contact point on cylinder surface
            var contactPoint = closestOnAxis.clone().addScaledVector(normal, cylRadius);

            bestContact = {
              point: contactPoint,
              normal: normal,
              penetration: penetration
            };
          }
        }

        // === Test 2: Caps vs Triangle ===
        var caps = [
          { center: cylAxisStart.clone(), sign: -1 },
          { center: cylAxisEnd.clone(), sign: 1 }
        ];

        for (var c = 0; c < 2; c++) {
          var cap = caps[c];

          // Distance from cap center to triangle plane
          var distToPlane = triNormal.dot(tmpVec.subVectors(cap.center, p1));

          // Cap normal points outward from cylinder
          var capDir = capNormal.clone().multiplyScalar(cap.sign);

          // Check if the cap is facing toward the triangle (cap normal and triangle normal opposing)
          var facing = capDir.dot(triNormal);

          if (Math.abs(distToPlane) < cylRadius && facing < 0) {
            // Cap might intersect triangle
            // Project cap center onto triangle plane
            tmpVec.copy(cap.center).addScaledVector(triNormal, -distToPlane);

            // Find closest point on triangle to this projected point
            elation.physics.colliders.helperfuncs.closest_point_on_triangle(tmpVec, p1, p2, p3, closestOnTriangle);

            // Check if this point is within the cap disk radius
            diff.subVectors(closestOnTriangle, cap.center);
            var distAlongAxis = diff.dot(capNormal) * cap.sign;
            var radialDist = diff.clone().addScaledVector(capNormal, -diff.dot(capNormal)).length();

            if (radialDist <= cylRadius && distAlongAxis >= 0 && distAlongAxis < cylRadius) {
              var penetration = -distAlongAxis;
              if (penetration > bestPenetration) {
                bestPenetration = penetration;

                // Normal points from cylinder cap toward triangle
                // capDir is the outward normal of the cap, which points toward the triangle
                var normal = capDir.clone();

                bestContact = {
                  point: closestOnTriangle.clone(),
                  normal: normal,
                  penetration: penetration
                };
              }
            }
          }
        }

        // === Test 3: Cylinder barrel vs Triangle edges ===
        var edges = [
          [p1, p2],
          [p2, p3],
          [p3, p1]
        ];

        for (var e = 0; e < 3; e++) {
          edgeStart.copy(edges[e][0]);
          edgeEnd.copy(edges[e][1]);

          // Find closest points between cylinder axis and triangle edge
          // Using iterative approach
          closestPointOnSegment(cylAxisStart, cylAxisEnd, edgeStart, closestOnAxis);
          for (var iter = 0; iter < 3; iter++) {
            closestPointOnSegment(edgeStart, edgeEnd, closestOnAxis, closestOnEdge);
            closestPointOnSegment(cylAxisStart, cylAxisEnd, closestOnEdge, closestOnAxis);
          }
          closestPointOnSegment(edgeStart, edgeEnd, closestOnAxis, closestOnEdge);

          diff.subVectors(closestOnAxis, closestOnEdge);
          dist = diff.length();

          if (dist < cylRadius && dist > 1e-6) {
            var penetration = -(cylRadius - dist);
            if (penetration > bestPenetration) {
              bestPenetration = penetration;

              // Normal from cylinder toward edge
              var normal = closestOnEdge.clone().sub(closestOnAxis).normalize();
              var contactPoint = closestOnAxis.clone().addScaledVector(normal, cylRadius);

              bestContact = {
                point: contactPoint,
                normal: normal,
                penetration: penetration
              };
            }
          }
        }

        if (bestContact) {
          var contact = new elation.physics.contact({
            point: bestContact.point,
            normal: bestContact.normal,
            penetration: bestContact.penetration,
            bodies: [cylinder.body, triangle.body]
          });
          contacts.push(contact);
        }

        return contacts;
      };
    }();

    this.cylinder_triangle = function(cylinder, triangle, contacts, dt) {
      return this.triangle_cylinder(triangle, cylinder, contacts, dt);
    };

    this.mesh_cylinder = function() {
      return function(mesh, cylinder, contacts, dt) {
        if (!contacts) contacts = [];

        // Lazy triangle extraction - if mesh has geometry but no triangles, try extracting now
        if (mesh.triangles.length === 0 && mesh.mesh && mesh.mesh.geometry) {
          mesh.triangles = mesh.extractTriangles(mesh.mesh);
        }

        var localcontacts = [], spherecontacts = [];

        // Update bounding sphere radius dynamically from current scale
        var meshScale = mesh.body.scaleWorld;
        mesh.boundingSphere.radius = (mesh.localRadius || mesh.radius) * Math.max(meshScale.x, meshScale.y, meshScale.z);

        // Broad phase: check against mesh bounding sphere
        elation.physics.colliders.helperfuncs.cylinder_sphere(cylinder, mesh.boundingSphere, spherecontacts, dt);
        if (spherecontacts.length == 0) return contacts;

        // Get scaled cylinder dimensions for triangle culling
        var cylScale = cylinder.body.scaleWorld;
        var scaledHeight = cylinder.height * cylScale.y;
        var scaledRadius = cylinder.radius * Math.max(cylScale.x, cylScale.z);

        let cylPos = cylinder.body.positionWorld,
            cylMaxDist = Math.pow(scaledHeight / 2 + scaledRadius, 2);

        // Test each triangle
        for (var i = 0; i < mesh.triangles.length; i++) {
          let triangle = mesh.triangles[i],
              worldpoints = triangle.getWorldPoints(),
              distToCenter = worldpoints.center.distanceToSquared(cylPos);

          if (distToCenter <= cylMaxDist + worldpoints.radius * worldpoints.radius) {
            elation.physics.colliders.helperfuncs.triangle_cylinder(triangle, cylinder, localcontacts, dt);
          }
        }

        if (localcontacts.length > 0) {
          let closestStatic = false,
              closestDynamic = false;

          for (let i = 0; i < localcontacts.length; i++) {
            let contact = localcontacts[i];
            if (contact instanceof elation.physics.contact_dynamic) {
              if (!closestDynamic || closestDynamic.penetrationTime > contact.penetrationTime) {
                closestDynamic = contact;
              }
            } else {
              if (!closestStatic || closestStatic.penetration > contact.penetration) {
                closestStatic = contact;
              }
            }
          }

          // Handle static contacts first, since they're equivalent to penetrationTime=0
          let closest = closestStatic || closestDynamic;
          if (closest.bodies[0] === cylinder.body) {
            closest.bodies[1] = mesh.getRoot();
          } else if (closest.bodies[1] === cylinder.body) {
            closest.bodies[0] = mesh.getRoot();
            closest.bodies[1] = cylinder.body;
          }
          contacts.push(closest);
        }

        return contacts;
      };
    }();

    this.cylinder_mesh = function(cylinder, mesh, contacts, dt) {
      return this.mesh_cylinder(mesh, cylinder, contacts, dt);
    };

    this.closest_point_on_sphere = function(point, center, radius, closest) {
      if (!closest) closest = new THREE.Vector3();
      closest.copy(point).sub(center);
      closest.normalize().multiplyScalar(radius);
      return closest;
    }
    this.closest_point_on_line = function() {
      var line = new THREE.Vector3(),
          proj = new THREE.Vector3();

      return function(start, end, point, closest) {
        line.copy(end).sub(start);
        var lengthSq = line.lengthSq();
        if (lengthSq < 1e-6) { // zero-length line
          if (!closest) closest = new THREE.Vector3();
          closest.copy(start);
          return closest;
        }

        // Project point on to line
        proj.subVectors(point, start);
        var t = proj.dot(line) / lengthSq;
        if (t < 0) { // beyond the start
          if (!closest) closest = new THREE.Vector3();
          closest.copy(start);
          return closest;
        } else if (t > 1) { // beyond the end
          if (!closest) closest = new THREE.Vector3();
          closest.copy(end);
          return closest;
        }

        // Find point perpendicular to line segment
        if (!closest) closest = new THREE.Vector3();
        closest.copy(start).add(line.multiplyScalar(t));
        return closest;
      }
    }();
    this.closest_point_on_triangle = (function() {
      // Reference: Real Time Collision Detection by Christer Ericson
      let ab = new THREE.Vector3(),
          ac = new THREE.Vector3(),

          ap = new THREE.Vector3(),
          bp = new THREE.Vector3(),
          cp = new THREE.Vector3();
      return function(point, a, b, c, closest) {
        if (!closest) closest = new THREE.Vector3(); // Allocate return vector if reference isn't passed in

        ab.subVectors(b, a);
        ac.subVectors(c, a);

        // Check if point is in vertex region outside A
        ap.subVectors(point, a);
        let d1 = ab.dot(ap),
            d2 = ac.dot(ap);
        if (d1 <= 0 && d2 <= 0) return closest.copy(a);

        // Check if point is in vertex region outside B
        bp.subVectors(point, b);
        let d3 = ab.dot(bp),
            d4 = ac.dot(bp);
        if (d3 >= 0 && d4 <= d3) return closest.copy(b);

        // Check if point is in edge region of AB, if so return projection of point onto AB
        let vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0) {
          let v = d1 / (d1 - d3);
//console.log('p is in AB', point, ab, vc, v);
          return closest.copy(ab).multiplyScalar(v).add(a);
        }

        // Check if point is in vertex region outside C
        cp.subVectors(point, c);
        let d5 = ab.dot(cp),
            d6 = ac.dot(cp);
        if (d6 >= 0 && d5 <= d6) {
//console.log('p is in C', point, ab, cp, d5, d6);
          return closest.copy(c);
        }

        // Check if point is in edge region of AC, if so return projection of point onto AC
        let vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0) {
          let w = d2 / (d2 - d6);
//console.log('p is in AC', point, ac, cp, vb, d2, d6);
          return closest.copy(ac).multiplyScalar(w).add(a);
        }

        // Check if point is in edge region of BC, if so return projection of point onto BC
        let va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
          let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
//console.log('p is in BC', point, cp, va, w);
          return closest.copy(c).sub(b).multiplyScalar(w).add(b);
        }

        // point is inside face region. Compute Q through its barycentric coordinates
        let denom = 1 / (va + vb + vc),
            v = vb * denom,
            w = vc * denom;
//console.log('p is in face', point, denom, v, w);
        return closest.copy(ac).multiplyScalar(w).add(ab.multiplyScalar(v)).add(a);
      }
    })();
    this.closest_point_on_line_to_triangle = (function() {
      const closestAB = new THREE.Vector3(),
            closestBC = new THREE.Vector3(),
            closestCA = new THREE.Vector3();

      return function(start, end, a, b, c, closest) {
        let distAB = elation.physics.colliders.helperfuncs.distancesquared_between_lines(start, end, a, b),
            distBC = elation.physics.colliders.helperfuncs.distancesquared_between_lines(start, end, b, c),
            distCA = elation.physics.colliders.helperfuncs.distancesquared_between_lines(start, end, c, a);
        // TODO - finish implementing logic
      }
    })();
    this.distancesquared_between_lines = function() {
      const d1 = new THREE.Vector3(),
            d2 = new THREE.Vector3(),
            r = new THREE.Vector3(),
            closest1 = new THREE.Vector3(),
            closest2 = new THREE.Vector3(),
            d = new THREE.Vector3();
      return function(p1, q1, p2, q2, c1, c2) {
        if (!c1) c1 = closest1;
        if (!c2) c2 = closest2;

        d1.subVectors(q1, p1);
        d2.subVectors(q2, p2);
        r.subVectors(p1, p2);
        let a = d1.dot(d1),
            e = d2.dot(d2),
            f = d2.dot(r);
        let s = 0, t = 0;
        // Check if either or both segments degenerate into points (length is 0)
        if (a <= 1e-6 && e <= 1e-6) {
          // both segments degenerates into points
          c1.copy(p1);
          c2.copy(p2);
          d.subVectors(c1, c2);
          return d.dot(d);
        }
        if (a <= 1e-6) {
          // first segment degenerates into a point
          t = elation.utils.math.clamp(f / e, 0, 1);
        } else {
          let c = d1.dot(r);
          if (e < 1e-6) {
            // second segment degenerates into a point
            s = elation.utils.math.clamp(-c / a, 0, 1);
          } else {
            // general non-degenerate case
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            if (denom != 0) {
              // segments aren't parallel, compute closest point on L1 to L2 and clamp to segment S1
              s = elation.utils.math.clamp((b * f - c * e) / denom, 0, 1);
            } else {
              // segments are parallel, pick arbitrary s=0
              s = 0;
            }
            t = (b * s + f) / e;

            // if t in [0,1] done. Else clamp t, recompute s for new value of t and clamp to [0, 1]
            if (t < 0) {
              t = 0;
              s = elation.utils.math.clamp(-c / a, 0, 1);
            } else if (t > 1) {
              t = 1;
              s = elation.utils.math.clamp((b - c) / a, 0, 1);
            }
          }
        }

        c1.copy(d1).multiplyScalar(s).add(p1);
        c2.copy(d2).multiplyScalar(t).add(p2);
        d.subVectors(c1, c2);
        return d.dot(d);
      }
    }();
    this.distance_to_line = function(start, end, point) {
      return this.closest_point_on_line(start, end, point).distanceTo(point);
    }

    this.ray_sphere = (function() {
      // Closure scratch variables
      let m = new THREE.Vector3(),
          d = new THREE.Vector3();

      return function(raypos, dir, spherepos, radius, point) {
        m.copy(raypos).sub(spherepos);
        d.copy(dir);
        let t = d.length();
        d.divideScalar(t);
        let b = m.dot(d),
            c = m.dot(m) - radius * radius;
        // Exit if ray's origin is outside of sphere and ray is pointing away from sphere
        if (c > 0 && b > 0) return;
        let discr = b * b - c;
        // A negative discriminant corresponds to ray missing sphere
        if (discr < 0) return;
        // Ray now found to be intersecting sphere, compute smallest t value of intersection
        t = -b - Math.sqrt(discr);
        if (t < 0) t = 0;
        if (!point) point = new THREE.Vector3(); // Allocate return vector if reference isn't passed in
        point.copy(d).multiplyScalar(t).add(raypos);
        return {point, t};
      }
    })();
    this.line_sphere = (function() {
      // Closure scratch variables
      let m = new THREE.Vector3(),
          dir = new THREE.Vector3();
      return function(start, end, spherepos, radius, point, isRay) {
        m.copy(start).sub(spherepos);
        dir.copy(end).sub(start);
        let len = dir.length(),
            b = m.dot(dir.divideScalar(len)),
            c = m.dot(m) - radius * radius;
        // Exit if ray's origin is outside of sphere and ray is pointing away from sphere
        if (c > 0 && b > 0) return;
        let discr = b * b - c;
        // A negative discriminant corresponds to ray missing sphere
        if (discr < 0) return;
        // Ray now found to be intersecting sphere, compute smallest t value of intersection
        let t = -b - Math.sqrt(discr);
        // If t is negative, ray started insidde sphere so clamp t to zero
        if (t < 0) t = 0;
        // If the intersection happens past the length of our line segment, there's no collision
        if (t > len && !isRay) return;

        if (!point) point = new THREE.Vector3(); // Allocate return vector if reference isn't passed in
        point.copy(dir).multiplyScalar(t).add(start); // allocate point
        return {point: point, t: t / len};
      }
    })();
    this.line_plane = (function() {
      let ab = new THREE.Vector3(),
          planeNormal = new THREE.Vector3(),
          scratch = new THREE.Vector3();
      return function(start, end, p1, p2, p3, point) {
        ab.copy(end).sub(start);
        planeNormal.copy(p2).sub(p1).cross(scratch.copy(p3).sub(p1));
        let planeDistance = planeNormal.dot(p1);
        let t = (planeDistance - planeNormal.dot(start)) / planeNormal.dot(ab);
        if (t >= 0 && t <= 1) {
          if (!point) point = new THREE.Vector3(); // Allocate return vector if reference isn't passed in
          point.x = start.x + ab.x * t;
          point.y = start.y + ab.y * t;
          point.z = start.z + ab.z * t;
          return {point, t};
        }
      }
    })();
    this.line_cylinder = (function() {
      let d = new THREE.Vector3(),
          m = new THREE.Vector3(),
          n = new THREE.Vector3();
      return function(sa, sb, p, q, r, t) {
        d.subVectors(q, p);
        m.subVectors(sa, p);
        n.subVectors(sb, sa);

        let md = m.dot(d),
            nd = n.dot(d),
            dd = d.dot(d);

        // Test if segment is fully outside either endcap of cylinder
        if (md < 0 && md + nd < 0) return; // segment outside of 'p' side of cylinder
        if (md > dd && md + nd > dd) return; // segment outside of 'q' side of cylinder

        let nn = n.dot(n),
            mn = m.dot(n),
            a = dd * nn - nd * nd,
            k = m.dot(m) - r * r,
            c = dd * k - md * md;

        if (Math.abs(a) < 1e-6) {
          // segment runs parallel to cylinder axis
          if (c > 0) return; // 'a' and thus this segment lie outside cylinder
          // now known that segment intersects cylinder; figure out how it intersects
          if (md < 0) t = -mn / nn; // intersect segment against 'p' endcap
          else if (md > dd) t = (nd - mn) / nn; // intersect segment against 'q' endcap
          else t = 0; // 'a' lies inside cylinder
          //let point = n.clone().multiplyScalar(1 - t).add(sa);
          //return { point, t };
          return t;
        }
        let b = dd * mn - nd * md,
            discr = b * b - a * c;
        if (discr < 0) return; // no real roots; no intersection
        t = (-b - Math.sqrt(discr)) / a;
        if (t < 0 || t > 1) return; // intersection lies outside segment
        if (md + t * nd < 0) {
          // intersection outside cylinder on 'p' side
          if (nd <= 0) return; // segment pointing away from endcap
          t = -md / nd;
          // keep intersection if dot(S(t) - p, S(t) - p) <= r^2
          if (k + 2 * t * (mn + t * nn) <= 0) {
            //let point = n.clone().multiplyScalar(1 - t).add(sa);
            //return { point, t };
            return t;
          } else {
            return;
          }
        } else if (md + t * nd > dd) {
          // intersection outside cylinder on 'q' side
          if (nd >= 0) return; // segment pointing away from endcap
          t = (dd - md) / nd;
          // keep intersection if dot(S(t) - q, S(t) - q) <= r^2
          if (k + dd - 2 * md + t * (2 * (mn - nd) + t * nn) <= 0) {
            //let point = n.clone().multiplyScalar(1 - t).add(sa);
            //return { point, t };
            return t;
          } else {
            return;
          }
        }
        // segment intersects cylinder between the endcaps; t is correct
        //let point = n.clone().multiplyScalar(t).add(sa);
        //return { point, t };
        return t;
      }
    })();
  });


  /*
   * =========
   * colliders
   * =========
   */
 
  elation.extend("physics.colliders.sphere", function(body, args={}) {
    this.type = 'sphere';
    this.body = body;
    this.radius = args.radius || args;
    this.scale = args.scale || new THREE.Vector3(1,1,1),
    this.offset = args.offset || false;
    this.trigger = elation.utils.any(args.trigger, false);

    if (!(this.scale instanceof THREE.Vector3)) this.scale = new THREE.Vector3().copy(args.scale);
    if (this.offset !== false && !(this.offset instanceof THREE.Vector3)) this.offset = new THREE.Vector3().copy(args.offset);

    this.getContacts = function(other, contacts, dt) {
      if (!contacts) contacts = [];
      switch(other.type) {
        case 'sphere':
          contacts = elation.physics.colliders.helperfuncs.sphere_sphere(this, other, contacts, dt);
          break;
        case 'plane':
          contacts = elation.physics.colliders.helperfuncs.sphere_plane(this, other, contacts, dt);
          break;
        case 'box':
          contacts = elation.physics.colliders.helperfuncs.box_sphere(other, this, contacts, dt);
          break;
        case 'cylinder':
          contacts = elation.physics.colliders.helperfuncs.sphere_cylinder(this, other, contacts, dt);
          break;
        case 'capsule':
          contacts = elation.physics.colliders.helperfuncs.capsule_sphere(other, this, contacts, dt);
          break;
        case 'mesh':
          contacts = elation.physics.colliders.helperfuncs.mesh_sphere(other, this, contacts, dt);
          break;
        default:
          console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      // For static objects (mass=0), return zero matrix (infinite inertia = zero inverse)
      if (this.body.mass <= 0) {
        this.momentInverse.set(
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 1);
        return this.momentInverse;
      }
      var c = 5 / (2 * this.body.mass * this.radius * this.radius);
      this.momentInverse.set(
        c, 0, 0, 0,
        0, c, 0, 0,
        0, 0, c, 0,
        0, 0, 0, 1);
      return this.momentInverse;
    }
    this.toJSON = function() {
      return {
        type: this.type,
        radius: this.radius,
        scale: {x: this.scale.x, y: this.scale.y, z: this.scale.z},
        offset: {x: elation.utils.any(this.offset.x, 0), y: elation.utils.any(this.offset.y, 0), z: elation.utils.any(this.offset.z, 0)},
        trigger: this.trigger,
        isroot: this.isroot,
      };
    }
  });
  elation.extend("physics.colliders.plane", function(body, args) {
    this.type = 'plane';
    if (!args) args = {};
    this.body = body;
    this.normal = args.normal || new THREE.Vector3(0,1,0);
    this.offset = args.offset || 0;
    this.radius = Infinity;
    this.trigger = elation.utils.any(args.trigger, false);

    this.getContacts = function(other, contacts, dt) {
      if (!contacts) contacts = [];
      if (other instanceof elation.physics.colliders.sphere) {
        contacts = elation.physics.colliders.helperfuncs.sphere_plane(other, this, contacts, dt);
      } else if (other instanceof elation.physics.colliders.box) {
        contacts = elation.physics.colliders.helperfuncs.box_plane(other, this, contacts, dt);
      } else if (other instanceof elation.physics.colliders.cylinder) {
        contacts = elation.physics.colliders.helperfuncs.cylinder_plane(other, this, contacts, dt);
      } else {
        console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      // Planes are always static - return zero matrix (infinite inertia)
      this.momentInverse = new THREE.Matrix4();
      this.momentInverse.set(
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1);
      return this.momentInverse;
    }
    this.toJSON = function() {
      return {
        type: this.type,
        normal: this.normal,
        offset: this.offset,
        trigger: this.trigger,
      };
    }
  });
  elation.extend("physics.colliders.box", function(body, args) {
    this.type = 'box';
    if (!args) args = {};
    this.body = body;
    this.min = args.min || new THREE.Vector3(0,0,0);
    this.max = args.max || new THREE.Vector3(0,0,0);
    this.trigger = elation.utils.any(args.trigger, false);

    if (!(this.min instanceof THREE.Vector3)) this.min = new THREE.Vector3().copy(args.min);
    if (!(this.max instanceof THREE.Vector3)) this.max = new THREE.Vector3().copy(args.max);

    this.halfsize = new THREE.Vector3().copy(this.max).sub(this.min).divideScalar(2);
    this.offset = new THREE.Vector3().copy(this.max).add(this.min).divideScalar(2);

    this.getContacts = function(other, contacts, dt) {
      if (!contacts) contacts = [];
      if (other instanceof elation.physics.colliders.plane) {
        contacts = elation.physics.colliders.helperfuncs.box_plane(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.sphere) {
        contacts = elation.physics.colliders.helperfuncs.box_sphere(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.box) {
        contacts = elation.physics.colliders.helperfuncs.box_box(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.cylinder) {
        contacts = elation.physics.colliders.helperfuncs.cylinder_box(other, this, contacts, dt);
      } else if (other instanceof elation.physics.colliders.capsule) {
        contacts = elation.physics.colliders.helperfuncs.box_capsule(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.triangle) {
        contacts = elation.physics.colliders.helperfuncs.box_triangle(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.mesh) {
        contacts = elation.physics.colliders.helperfuncs.box_mesh(this, other, contacts, dt);
      } else {
        //console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      // For static objects (mass=0), return zero matrix (infinite inertia = zero inverse)
      if (this.body.mass <= 0) {
        this.momentInverse.set(
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 1);
        return this.momentInverse;
      }
      var diff = this.max.clone().sub(this.min);
      var xsq = diff.x*diff.x,
          ysq = diff.y*diff.y,
          zsq = diff.z*diff.z,
          m = 1/12 * this.body.mass;
      this.momentInverse.set(
        1 / (m * (ysq + zsq)), 0, 0, 0,
        0, 1 / (m * (xsq + zsq)), 0, 0,
        0, 0, 1 / (m * (xsq + ysq)), 0,
        0, 0, 0, 1);
      return this.momentInverse;
    }
    this.getCorner = function(i, v) {
      if (!v) v = new THREE.Vector3();
      switch (i) {
        case 0:
              v.set(this.min.x, this.min.y, this.min.z);
              break;
        case 1:
              v.set(this.min.x, this.min.y, this.max.z);
              break;
        case 2:
              v.set(this.min.x, this.max.y, this.min.z);
              break;
        case 3:
              v.set(this.min.x, this.max.y, this.max.z);
              break;
        case 4:
              v.set(this.max.x, this.min.y, this.min.z);
              break;
        case 5:
              v.set(this.max.x, this.min.y, this.max.z);
              break;
        case 6:
              v.set(this.max.x, this.max.y, this.min.z);
              break;
        case 7:
              v.set(this.max.x, this.max.y, this.max.z);
              break;
      }
      return v;
    }
    this.toJSON = function() {
      return {
        type: this.type,
        min: {x: this.min.x, y: this.min.y, z: this.min.z},
        max: {x: this.max.x, y: this.max.y, z: this.max.z},
        offset: {x: this.offset.x, y: this.offset.y, z: this.offset.z},
        trigger: this.trigger,
      };
    }
  });
  elation.extend("physics.colliders.cylinder", function(body, args) {
    this.type = 'cylinder';
    if (!args) args = {};
    this.body = body;
    this.radius = args.radius;
    this.height = args.height;
    this.offset = args.offset || new THREE.Vector3();
    this.trigger = elation.utils.any(args.trigger, false);

    if (!(this.offset instanceof THREE.Vector3)) this.offset = new THREE.Vector3().copy(args.offset);

    this.getContacts = function(other, contacts, dt) {
      if (!contacts) contacts = [];
      if (other instanceof elation.physics.colliders.plane) {
        contacts = elation.physics.colliders.helperfuncs.cylinder_plane(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.sphere) {
        contacts = elation.physics.colliders.helperfuncs.cylinder_sphere(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.box) {
        contacts = elation.physics.colliders.helperfuncs.cylinder_box(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.cylinder) {
        contacts = elation.physics.colliders.helperfuncs.cylinder_cylinder(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.capsule) {
        contacts = elation.physics.colliders.helperfuncs.cylinder_capsule(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.mesh) {
        contacts = elation.physics.colliders.helperfuncs.cylinder_mesh(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.triangle) {
        contacts = elation.physics.colliders.helperfuncs.cylinder_triangle(this, other, contacts, dt);
      } else {
        console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      // For static objects (mass=0), return zero matrix (infinite inertia = zero inverse)
      if (this.body.mass <= 0) {
        this.momentInverse.set(
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 1);
        return this.momentInverse;
      }
      // Solid cylinder moments of inertia (Y axis is the long axis):
      //   I_y = (1/2) * m * r²  (around long axis)
      //   I_x = I_z = (1/12) * m * (3r² + h²)  (perpendicular to long axis)
      // We need the INVERSE for momentInverse
      var rsq = this.radius * this.radius,
          hsq = this.height * this.height,
          m = this.body.mass;
      var Iy = 0.5 * m * rsq;
      var Ixz = (1/12) * m * (3 * rsq + hsq);
      this.momentInverse.set(
        1 / Ixz, 0, 0, 0,
        0, 1 / Iy, 0, 0,
        0, 0, 1 / Ixz, 0,
        0, 0, 0, 1);
      return this.momentInverse;
    }
    this.toJSON = function() {
      return {
        type: this.type,
        radius: this.radius,
        height: this.height,
        offset: this.offset,
        trigger: this.trigger,
      };
    }
  });
  elation.extend("physics.colliders.capsule", function(body, args) {
    this.type = 'capsule';
    if (!args) args = {};
    this.body = body;
    this.radius = args.radius;
    this.length = args.length;
    this.offset = args.offset || new THREE.Vector3();
    this.trigger = elation.utils.any(args.trigger, false);

    this.dimensions = {
      start: new THREE.Vector3(),
      end: new THREE.Vector3(),
      radius: new THREE.Vector3(),
    };

    this.getContacts = function(other, contacts, dt) {
      if (!contacts) contacts = [];
      if (other instanceof elation.physics.colliders.plane) {
        contacts = elation.physics.colliders.helperfuncs.capsule_plane(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.sphere) {
        contacts = elation.physics.colliders.helperfuncs.capsule_sphere(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.box) {
        contacts = elation.physics.colliders.helperfuncs.capsule_box(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.mesh) {
        contacts = elation.physics.colliders.helperfuncs.mesh_capsule(other, this, contacts, dt);
      } else if (other instanceof elation.physics.colliders.capsule) {
        contacts = elation.physics.colliders.helperfuncs.capsule_capsule(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.cylinder) {
        contacts = elation.physics.colliders.helperfuncs.capsule_cylinder(this, other, contacts, dt);
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      // For static objects (mass=0), return zero matrix (infinite inertia = zero inverse)
      if (this.body.mass <= 0) {
        this.momentInverse.set(
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 1);
        return this.momentInverse;
      }
      // Capsule approximated as cylinder with hemispherical caps
      // Moments of inertia (Y axis is the long axis):
      //   I_y = (1/2) * m * r²  (around long axis)
      //   I_x = I_z = (1/12) * m * (3r² + h²)  (perpendicular to long axis)
      // We need the INVERSE for momentInverse
      var rsq = this.radius * this.radius,
          hsq = this.length * this.length,
          m = this.body.mass;
      var Iy = 0.5 * m * rsq;
      var Ixz = (1/12) * m * (3 * rsq + hsq);
      this.momentInverse.set(
        1 / Ixz, 0, 0, 0,
        0, 1 / Iy, 0, 0,
        0, 0, 1 / Ixz, 0,
        0, 0, 0, 1);
      return this.momentInverse;
    }
    this.getDimensions = function() {
      // TODO - cache these
      this.body.localToWorldPos(this.dimensions.start.set(0, 0, 0).add(this.offset));
      this.body.localToWorldPos(this.dimensions.end.set(0, this.length, 0).add(this.offset));
      // Scale the radius by the body's world scale (use max of X/Z since capsule is Y-aligned)
      var scaleWorld = this.body.scaleWorld;
      this.dimensions.scaledRadius = this.radius * Math.max(scaleWorld.x, scaleWorld.z);
      // Keep the old radius vector for compatibility, but also apply scale
      this.body.localToWorldDir(this.dimensions.radius.set(0, this.radius, 0));
      return this.dimensions;
    }
    this.toJSON = function() {
      return {
        type: this.type,
        radius: this.radius,
        length: this.length,
        offset: this.offset,
        trigger: this.trigger,
      };
    }
  });
  elation.extend("physics.colliders.mesh", function(body, args) {
    this.type = 'mesh';
    if (!args) args = {};
    this.body = body;
    this.isroot = elation.utils.any(args.isroot, true);

    this.mesh = args.mesh;
    this.modeldata = args.modeldata;
    this.radius = 0;
    this.trigger = elation.utils.any(args.trigger, false);

    this.boundingSphere = new elation.physics.colliders.sphere(body, {});

    this.extractTriangles = function(mesh) {
      let triangles = [];
      let radiusSq = 0;

      this.body.updateState(); // ensure scaleWorld is up to date before processing triangles
      let doubleSided = false;
      if (!this.modeldata && this.mesh && this.mesh.geometry) {
        if (this.mesh.geometry instanceof THREE.BufferGeometry) {
          this.modeldata = {
            positions: this.mesh.geometry.attributes.position.array,
          };
          if (this.mesh.geometry.index) {
            this.modeldata.index = this.mesh.geometry.index.array;
          } else {
            let numverts = this.modeldata.positions.length / 3;
            this.modeldata.index = new Uint16Array(numverts);
            for (let i = 0; i < numverts; i++) {
              this.modeldata.index[i] = i;
            }
          }
        }
        doubleSided = this.mesh.material.side == THREE.DoubleSide;
      }
      if (this.modeldata) {
        if (this.modeldata.index) {
          let idxarr = this.modeldata.index;
          let posarr = this.modeldata.positions;
          for (var i = 0; i < idxarr.length / 3; i++) {
            let offset = i * 3,
                v1 = idxarr[offset] * 3,
                v2 = idxarr[offset+1] * 3,
                v3 = idxarr[offset+2] * 3,

                p1 = new THREE.Vector3((posarr[v1]), (posarr[v1 + 1]), (posarr[v1 + 2])),
                p2 = new THREE.Vector3((posarr[v2]), (posarr[v2 + 1]), (posarr[v2 + 2])),
                p3 = new THREE.Vector3((posarr[v3]), (posarr[v3 + 1]), (posarr[v3 + 2])),

  /*
                p1 = new THREE.Vector3((posarr[v1]), (posarr[v1 + 1]), (posarr[v1 + 2])),
                p2 = new THREE.Vector3((posarr[v2]), (posarr[v2 + 1]), (posarr[v2 + 2])),
                p3 = new THREE.Vector3((posarr[v3]), (posarr[v3 + 1]), (posarr[v3 + 2])),
  */
                triangle = new elation.physics.colliders.triangle(this.body, [p1, p2, p3]);

            if (!triangle.isDegenerate()) {
              triangles.push(triangle);

              if (doubleSided) {
                let triangle2 = new elation.physics.colliders.triangle(this.body, [p3, p2, p1]);
                triangles.push(triangle2);
              }

              let l1 = p1.lengthSq(),
                  l2 = p2.lengthSq(),
                  l3 = p3.lengthSq();
              if (l1 > radiusSq) radiusSq = l1;
              if (l2 > radiusSq) radiusSq = l2;
              if (l3 > radiusSq) radiusSq = l3;
            } else {
              console.log('Skipping degenerate triangle in collider', triangle, this);
            }
          }
        } else {
          let posarr = this.modeldata.positions;
          for (var i = 0; i < posarr.length / 9; i++) {
            let offset = i * 3 * 3,
                p1 = new THREE.Vector3(posarr[offset    ], posarr[offset + 1], posarr[offset + 2]),
                p2 = new THREE.Vector3(posarr[offset + 3], posarr[offset + 4], posarr[offset + 5]),
                p3 = new THREE.Vector3(posarr[offset + 6], posarr[offset + 7], posarr[offset + 8]),
                triangle = new elation.physics.colliders.triangle(this.body, [p1, p2, p3]);

            if (!triangle.isDegenerate()) {
              triangles.push(triangle);

              let l1 = p1.lengthSq(),
                  l2 = p2.lengthSq(),
                  l3 = p3.lengthSq();
              if (l1 > radiusSq) radiusSq = l1;
              if (l2 > radiusSq) radiusSq = l2;
              if (l3 > radiusSq) radiusSq = l3;
            } else {
              console.log('Skipping degenerate triangle in collider', triangle, this);
            }
          }
        }
      }
      // Store unscaled local radius - will be scaled dynamically when needed
      this.localRadius = Math.sqrt(radiusSq);
      this.radius = this.localRadius * Math.max(this.body.scaleWorld.x, this.body.scaleWorld.y, this.body.scaleWorld.z);
      this.boundingSphere.radius = this.radius;
      return triangles;
    }
    this.extractObjects = function(mesh) {
      // Build a hierarchy of rigidbodies for every mesh in this group
      let meshes = [],
          objects = {},
          bodies = {};

      // Extract all leaf nodes that are meshes
      mesh.traverse((n) => {
        if (n instanceof THREE.Mesh) {
          meshes.push(n);
        }
      });

      // Put all the parents of our mesh leaf nodes into a map
      for (let i = 0; i < meshes.length; i++) {
        let parents = [];
        let n = meshes[i];
        while (n && n !== mesh) {
          parents.unshift(n);
          n = n.parent;
        }

        let parent = this.body;
        for (let j = 0; j < parents.length; j++) {
          let obj = parents[j];
          if (!bodies[obj.uuid]) {
            bodies[obj.uuid] = new elation.physics.rigidbody();
            bodies[obj.uuid].id = obj.uuid;
            bodies[obj.uuid].position.copy(obj.position);
            bodies[obj.uuid].scale.copy(obj.scale);
            bodies[obj.uuid].orientation.copy(obj.quaternion);
            bodies[obj.uuid].object = this.body.object;
            parent.add(bodies[obj.uuid]);
            if (obj instanceof THREE.Mesh) {
              bodies[obj.uuid].setCollider('mesh', {mesh: obj, isroot: false });
              //elation.events.add(bodies[obj.uuid], 'physics_collide', (ev) => elation.events.fire({type: 'physics_collide', element: this.body, event: ev}));
            }
          }
          parent = bodies[obj.uuid];
        }
      }
    }

    if (this.mesh) {
      this.extractObjects(this.mesh);
    }
    this.triangles = this.extractTriangles(this.mesh);

    this.getContacts = function(other, contacts, dt) {
      if (!contacts) contacts = [];
      if (other instanceof elation.physics.colliders.sphere) {
        contacts = elation.physics.colliders.helperfuncs.mesh_sphere(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.capsule) {
        contacts = elation.physics.colliders.helperfuncs.mesh_capsule(this, other, contacts);
      } else if (other instanceof elation.physics.colliders.box) {
        contacts = elation.physics.colliders.helperfuncs.mesh_box(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.cylinder) {
        contacts = elation.physics.colliders.helperfuncs.mesh_cylinder(this, other, contacts, dt);
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      // Meshes are typically static geometry - return zero matrix (infinite inertia)
      // For dynamic meshes, we'd need to compute inertia from the actual geometry
      // which is expensive and rarely needed
      this.momentInverse.set(
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1);
      return this.momentInverse;
    }

    this.distanceTo = function(point) {
      return this.normal.dot(point) + this.offset;
    }
    this.getRoot = function() {
      if (this.isroot) return this.body;
      //return this.parent.getRoot();
      let parent = this.body.parent;
      while (parent) {
        if (parent.collider && parent.collider.isroot) return parent;
        parent = parent.parent;
      }
    }
    this.toJSON = function() {
      return {
        type: this.type,
        modeldata: this.modeldata,
        trigger: this.trigger,
        isroot: this.isroot,
      };
    }
  });
  elation.extend("physics.colliders.triangle", function(body, args) {
    this.type = 'triangle';
    if (!args) args = {};
    this.body = body;

    this.cache = {
      orientationWorld: new THREE.Quaternion(),
      scaleWorld: new THREE.Vector3(),
      scaleWorldTmp: new THREE.Vector3(),
      points: {
        p1: new THREE.Vector3(),
        p2: new THREE.Vector3(),
        p3: new THREE.Vector3(),
        normal: new THREE.Vector3(),
        center: new THREE.Vector3(),
        radius: 0
      },
    };

    this.v0 = new THREE.Vector3();
    this.v1 = new THREE.Vector3();

    this.normal = new THREE.Vector3(0,1,0);
    this.trigger = elation.utils.any(args.trigger, false);

    this.updatePoints = function(p1, p2, p3) {
      this.p1 = p1;
      this.p2 = p2;
      this.p3 = p3;

      this.v0.copy(this.p2).sub(this.p1);
      this.v1.copy(this.p3).sub(this.p1);

      this.normal.copy(this.v0).cross(this.v1).normalize();

      var origin = this.p1,
          normal = this.normal;
      this.offset = -(normal.x * origin.x + normal.y * origin.y + normal.z * origin.z);
    }

    this.updatePoints(args[0], args[1], args[2]);


    this.getContacts = function(other, contacts, dt) {
      if (!contacts) contacts = [];
      if (other instanceof elation.physics.colliders.sphere) {
        contacts = elation.physics.colliders.helperfuncs.triangle_sphere(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.capsule) {
        contacts = elation.physics.colliders.helperfuncs.triangle_capsule(this, other, contacts);
      } else if (other instanceof elation.physics.colliders.box) {
        contacts = elation.physics.colliders.helperfuncs.triangle_box(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.cylinder) {
        contacts = elation.physics.colliders.helperfuncs.triangle_cylinder(this, other, contacts, dt);
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      // Triangles are typically static geometry (part of meshes) - return zero matrix
      // This means they have infinite inertia and won't rotate from collisions
      this.momentInverse.set(
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1);
      return this.momentInverse;
    }

    this.containsPoint = (function() {
      const a = new THREE.Vector3(),
            b = new THREE.Vector3(),
            c = new THREE.Vector3();
      return function(point) {
        let worldpoints = this.getWorldPoints();
        a.copy(worldpoints.p1).sub(point);
        b.copy(worldpoints.p2).sub(point);
        c.copy(worldpoints.p3).sub(point);

        let ab = a.dot(b),
            ac = a.dot(c),
            bc = b.dot(c),
            cc = c.dot(c);

        if (bc * ac - cc * ab < 0) return false;

        let bb = b.dot(b);

        if (ab * bc - ac * bb < 0) return false;

        return true;
      }
    })();
    this.distanceTo = (function() {
      let triangleClosestPoint = new THREE.Vector3();
      return function(point) {
        //return this.normal.dot(point) + this.offset;
        // TODO - should use cached world points
        elation.physics.colliders.helperfuncs.closest_point_on_triangle(point, this.p1, this.p2, this.p3, triangleClosestPoint);
        return triangleClosestPoint.distanceTo(point);
      }
    })();
    this.isDegenerate = function() {
      return (this.p1.equals(this.p2) || this.p1.equals(this.p3) || this.p2.equals(this.p3));
    }
    this.getWorldPoints = function() {
      // check to see if world points are cached, and if the cache is still valid
      // TODO - it really makes more sense for this cache to be on the mesh, and for the mesh to update these cached values as needed
      // For root objects (no parent), scaleWorld might not be updated yet, so also compare against scale directly
      var bodyScale = this.body.parent ? this.body.scaleWorld : this.body.scale;
      if (!(this.cache.orientationWorld.equals(this.body.orientationWorld) && this.cache.scaleWorld.equals(bodyScale))) {
        this.cache.orientationWorld.copy(this.body.orientationWorld);
        this.cache.scaleWorld.copy(bodyScale);
        this.body.localToWorldPos(this.cache.points.p1.copy(this.p1));
        this.body.localToWorldPos(this.cache.points.p2.copy(this.p2));
        this.body.localToWorldPos(this.cache.points.p3.copy(this.p3));
        this.body.localToWorldDir(this.cache.points.normal.copy(this.normal));

        let circumsphere = this.getCircumcenter(this.cache.points, this.cache.points.center);
        this.cache.points.radius = circumsphere.radius;
      }
      return this.cache.points;
    }
    this.getArea = function() {
      let worldpoints = this.getWorldPoints()
          p1 = worldpoints.p1,
          p2 = worldpoints.p2,
          p3 = worldpoints.p3;

      return .5 * Math.abs(p1.x * (p2.y - p3.y) + p2.x * (p3.x - p1.y) + p3.x * (p1.y - p2.y));
    }
    this.getCircumcenter = function(points, center) {
      const ab = new THREE.Vector3(),
            ac = new THREE.Vector3(),
            abXac = new THREE.Vector3(),
            t1 = new THREE.Vector3(),
            t2 = new THREE.Vector3();
      if (!points) points = this.getWorldPoints();
      if (!center) center = new THREE.Vector3();

      const p1 = points.p1,
            p2 = points.p2,
            p3 = points.p3;

      ab.subVectors(p2, p1);
      ac.subVectors(p3, p1);
      abXac.crossVectors(ab, ac);

      t1.crossVectors(abXac, ab).multiplyScalar(ac.lengthSq());
      t2.crossVectors(ac, abXac).multiplyScalar(ab.lengthSq());
      center.addVectors(t1, t2).divideScalar(abXac.lengthSq() * 2);
      let radius = center.length();
      center.add(p1);

      return {center, radius};
    }
    this.toJSON = function() {
      return {
        type: this.type,
        p1: this.p1,
        p2: this.p2,
        p3: this.p3,
        trigger: this.trigger,
      };
    }
  });

  elation.extend("physics.contact", function(contactargs) {
    this.bodies = [];
    this.friction = contactargs.friction || 0;
    this.restitution = contactargs.restitution || 1;
    this.penetration = contactargs.penetration || 0;
    this.point = contactargs.point || new THREE.Vector3();
    this.normal = contactargs.normal || new THREE.Vector3(0,1,0);
    this.relativePositions = [];
    this.inertialMoments = [];
    this.impulses = [];
    this.contactToWorld = new THREE.Matrix4();
    this.worldToContact = new THREE.Matrix4();
    this.initialized = false;

    this._tmpmat = new THREE.Matrix4();

    if (contactargs.bodies) this.bodies = contactargs.bodies;
    if (contactargs.object1) this.bodies.push(contactargs.object1);
    if (contactargs.object2) this.bodies.push(contactargs.object2);

    /**
     * Resolve a collision using default physical simulation
     */
    this.resolve = function(t, a, b) {
      this.restitution = this.bodies[0].restitution * this.bodies[1].restitution;

      // Calculate friction from body materials (use geometric mean, or max if one is zero)
      var friction0 = this.bodies[0].material ? this.bodies[0].material.dynamicfriction : 0;
      var friction1 = this.bodies[1].material ? this.bodies[1].material.dynamicfriction : 0;
      if (friction0 > 0 && friction1 > 0) {
        this.friction = Math.sqrt(friction0 * friction1);
      } else {
        // If either has zero friction, use the max (allows one rough surface to create friction)
        this.friction = Math.max(friction0, friction1);
      }

      if (!this.initialized) {
        this.calculateInternals(t);
        this.initialized = true;
      }
      // Fire events for both objects, and combine them into one array
      var events = elation.events.fire({type: 'physics_collide', element: this.bodies[0], data: this});
      events.push.apply(events, elation.events.fire({type: 'physics_collide', element: this.bodies[1], data: this}));

      if (!elation.events.wasDefaultPrevented(events)) {
        // If no event handlers handled this event, use our default collision response
        this.applyPositionChange(t, a, b);

        // Apply velocity impulse if there's actual penetration
        // (penetration < 0 means objects are overlapping)
        if (this.penetration < 0) {
          this.applyVelocityChange(t, a, b);
          this.finalizeMovement(t, a, b);
        }
        events.push.apply(events, elation.events.fire({type: 'physics_collision_resolved', element: this.bodies[0], data: this}));
        events.push.apply(events, elation.events.fire({type: 'physics_collision_resolved', element: this.bodies[1], data: this}));
      }
    }
    /**
     * Generate a transform matrix which represents the collision's local coordinate space
     */
    this.calculateContactMatrix = function() {
      // Closure for scratch variables
      var c1 = new THREE.Vector3(),
          c2 = new THREE.Vector3(),
          binormal = new THREE.Vector3();
    
      return function() {
        // Check whether the Z-axis is nearer to the X or Y axis
        this.normal.normalize();
        //var c1 = this.normal.clone().cross(new THREE.Vector3(0,0,1));
        //var c2 = this.normal.clone().cross(new THREE.Vector3(0,1,0));
        var normal = this.normal;
        c1.set(0,0,1).cross(this.normal);
        c2.set(0,1,0).cross(this.normal);
        var tangent = (c1.lengthSq() > c2.lengthSq() ? c1 : c2);
        tangent.normalize().negate();
        binormal.copy(tangent).cross(this.normal);

        this.contactToWorld = new THREE.Matrix4().identity();
        if (tangent.lengthSq() > 0) {
          this.contactToWorld.set(
            tangent.x, this.normal.x, binormal.x, 0,
            tangent.y, this.normal.y, binormal.y, 0,
            tangent.z, this.normal.z, binormal.z, 0,
            0, 0, 0, 1
          );
        }
        this.worldToContact = new THREE.Matrix4().copy(this.contactToWorld).invert();
      }
    }();
    /**
     * Calculate velocity relative to contact point, taking into account angular velocity
     */
    this.calculateLocalVelocity = function(index, duration) {
      // TODO - optimize local scratch variables
      var velocity = new THREE.Vector3();
      var accvel= new THREE.Vector3();
      var body = this.bodies[index];

      // Velocity at contact point is the linear velocity + the linear component of angular velocity 
      velocity.crossVectors(body.angular, this.relativePositions[index]).add(body.velocity);
      velocity.applyMatrix4(this.worldToContact); // transform to contact-relative coordinate space

      // Calculate how much velocity is due to the previous frame's acceleration
      accvel.copy(body.lastacceleration).multiplyScalar(duration).applyMatrix4(this.worldToContact);
      accvel.y = 0;
      velocity.add(accvel);
      
      return velocity;
    }
    /**
     * Calculate what the new relative velocities should be after resolving the collision
     */
    this.calculateDesiredDeltaVelocity = function(duration) {
      // TODO - optimize local scratch variables
      var velocityFromAccel = 0;
      var lastaccel = new THREE.Vector3();

      if (this.bodies[0] && !this.bodies[0].state.sleeping) {
        velocityFromAccel -= lastaccel.copy(this.bodies[0].lastacceleration).multiplyScalar(duration).dot(this.normal);
      }
      if (this.bodies[1] && !this.bodies[1].state.sleeping) {
        velocityFromAccel += lastaccel.copy(this.bodies[1].lastacceleration).multiplyScalar(duration).dot(this.normal);
      }

      var restitution = this.restitution;

      if (Math.abs(this.velocity.y) < 0.01) { // FIXME - velocity threshold should be configurable
        restitution = 0;
      }

      this.desiredDeltaVelocity = -this.velocity.y - restitution * (this.velocity.y - velocityFromAccel);
      //if (this.desiredDeltaVelocity > 0) this.desiredDeltaVelocity *= -1;
    }
    this.calculateInternals = function(duration) {
      this.calculateContactMatrix();

      // Calculate relative position and inertial moment for the first body
      if (this.bodies[0].collider.offset) {
        this.relativePositions[0] = this.point.clone().sub(this.bodies[0].localToWorldPos(this.bodies[0].collider.offset.clone())); // allocate vector
      } else {
        this.relativePositions[0] = this.point.clone().sub(this.bodies[0].localToWorldPos()); // allocate vector
      }
      var mworld0 = this._tmpmat.makeRotationFromQuaternion(this.bodies[0].orientationWorld);
      this.inertialMoments[0] = this.bodies[0].collider.momentInverse.clone().multiply(mworld0); // allocate matrix

      // If we have a second body, figure out its position and inertial moment
      if (this.bodies[1]) {
        if (this.bodies[1].collider.offset) {
          this.relativePositions[1] = this.point.clone().sub(this.bodies[1].localToWorldPos(this.bodies[1].collider.offset.clone())); // allocate vector
        } else {
          this.relativePositions[1] = this.point.clone().sub(this.bodies[1].localToWorldPos()); // allocate vector
        }
        var mworld1 = this._tmpmat.makeRotationFromQuaternion(this.bodies[1].orientationWorld);
        this.inertialMoments[1] = this.bodies[1].collider.momentInverse.clone().multiply(mworld1); // allocate matrix
      }

      // calculate velocities
      this.velocity = this.calculateLocalVelocity(0, duration);
      if (this.bodies[1]) {
        this.velocity.sub(this.calculateLocalVelocity(1, duration));
      }
      this.calculateDesiredDeltaVelocity(duration);
    }
    this.applyVelocityChange = function() {
      // closure scratch variables
      var impulsiveForce = new THREE.Vector3();
      var impulsiveTorque = new THREE.Vector3();

      return function(duration, velocityChange, rotationChange) {
        var impulse = (this.friction == 0 ? this.calculateFrictionlessImpulse() : this.calculateFrictionImpulse());
        impulse.applyMatrix4(this.contactToWorld);

        if (this.bodies[0] && this.bodies[0].mass > 0) {
          rotationChange[0] = impulsiveTorque.crossVectors(this.relativePositions[0], impulse).applyMatrix4(this.inertialMoments[0]);
          velocityChange[0] = impulsiveForce.copy(impulse).multiplyScalar(1 / this.bodies[0].mass);
          this.impulses[0] = impulsiveForce.clone(); // allocation (FIXME - only needed for debug)
          this.bodies[0].addVelocity(impulsiveForce);
          this.bodies[0].addAngularVelocity(impulsiveTorque);

          // Rolling friction: a torque opposing rotation, proportional to normal force
          // This models energy loss from material deformation at the contact
          var Crr0 = this.bodies[0].material ? this.bodies[0].material.rollingfriction : 0;
          var Crr1 = this.bodies[1] && this.bodies[1].material ? this.bodies[1].material.rollingfriction : 0;
          var Crr = Math.max(Crr0, Crr1); // Use the higher rolling friction
          if (Crr > 0) {
            var normalForceMag = Math.abs(impulse.dot(this.normal));
            var angularSpeed = this.bodies[0].angular.length();
            if (angularSpeed > 0.001) {
              // Rolling friction torque magnitude = Crr * radius * normalForce
              // For simplicity, assume effective radius of ~0.5 for unit-sized objects
              var effectiveRadius = 0.5;
              var resistanceTorqueMag = Crr * effectiveRadius * normalForceMag;
              // Apply as angular impulse opposing rotation (divide by moment of inertia)
              // Approximate I = m/6 for a cube
              var approxI = this.bodies[0].mass / 6;
              var angularImpulseMag = resistanceTorqueMag; // This is already in torque*time units since normalForce is impulse
              // Limit to not reverse angular velocity
              angularImpulseMag = Math.min(angularImpulseMag, angularSpeed * approxI);
              // Apply opposing angular velocity
              var angularDamping = this.bodies[0].angular.clone().normalize().multiplyScalar(-angularImpulseMag / approxI);
              this.bodies[0].angular.add(angularDamping);
            }
          }
        }

        if (this.bodies[1] && this.bodies[1].mass > 0) {
          rotationChange[1] = impulsiveTorque.crossVectors(impulse, this.relativePositions[1]).applyMatrix4(this.inertialMoments[1]);
          velocityChange[1] = impulsiveForce.copy(impulse).multiplyScalar(-1 / this.bodies[1].mass);
          this.impulses[1] = impulsiveForce.clone();

          this.bodies[1].addVelocity(impulsiveForce);
          this.bodies[1].addAngularVelocity(impulsiveTorque);

          // Rolling friction for body 1
          var Crr0 = this.bodies[0].material ? this.bodies[0].material.rollingfriction : 0;
          var Crr1 = this.bodies[1].material ? this.bodies[1].material.rollingfriction : 0;
          var Crr = Math.max(Crr0, Crr1);
          if (Crr > 0) {
            var normalForceMag = Math.abs(impulse.dot(this.normal));
            var angularSpeed = this.bodies[1].angular.length();
            if (angularSpeed > 0.001) {
              var effectiveRadius = 0.5;
              var resistanceTorqueMag = Crr * effectiveRadius * normalForceMag;
              var approxI = this.bodies[1].mass / 6;
              var angularImpulseMag = Math.min(resistanceTorqueMag, angularSpeed * approxI);
              var angularDamping = this.bodies[1].angular.clone().normalize().multiplyScalar(-angularImpulseMag / approxI);
              this.bodies[1].angular.add(angularDamping);
            }
          }
        }
      }
    }();

    this.calculateFrictionlessImpulse = function() {
      // closure scratch variables
      var deltaVelWorld = new THREE.Vector3();

      return function() {
        var impulse = new THREE.Vector3();
        
        var deltaVelocity = 0;
        for (var i = 0; i < this.bodies.length; i++) {
          if (this.bodies[i].mass > 0) {
            deltaVelWorld.crossVectors(this.relativePositions[i], this.normal);
            deltaVelWorld.applyMatrix4(this.inertialMoments[i]);
            deltaVelWorld.cross(this.relativePositions[i]);
            deltaVelocity += deltaVelWorld.dot(this.normal) + 1 / this.bodies[i].mass;
          }
        }
        
        impulse.set(0, this.desiredDeltaVelocity / deltaVelocity, 0);

        return impulse;
      }
    }();

    this.calculateFrictionImpulse = function() {
      // Closure scratch variables
      var deltaVelWorld = new THREE.Vector3();
      var impulseContact = new THREE.Vector3();
      var tangent = new THREE.Vector3();

      return function() {
        var impulse = new THREE.Vector3();
        var inverseMass = 0;

        // Build a matrix that converts impulse in contact coords to velocity change in contact coords
        // This is a 3x3 matrix, but we'll compute the diagonal terms for a simplified approach

        // For each body, calculate the contribution to velocity change per unit impulse
        // in each of the three contact-space directions (tangent X, normal Y, tangent Z)
        var deltaVelX = 0, deltaVelY = 0, deltaVelZ = 0;

        for (var i = 0; i < this.bodies.length; i++) {
          if (this.bodies[i] && this.bodies[i].mass > 0) {
            inverseMass += 1 / this.bodies[i].mass;

            // Angular component for normal (Y) direction
            deltaVelWorld.crossVectors(this.relativePositions[i], this.normal);
            deltaVelWorld.applyMatrix4(this.inertialMoments[i]);
            deltaVelWorld.cross(this.relativePositions[i]);
            deltaVelY += deltaVelWorld.dot(this.normal);

            // For tangent directions, we need to use the contact matrix columns
            // Column 0 = tangent X direction in world space
            // Column 2 = tangent Z direction in world space
            var tangentX = new THREE.Vector3(
              this.contactToWorld.elements[0],
              this.contactToWorld.elements[1],
              this.contactToWorld.elements[2]
            );
            var tangentZ = new THREE.Vector3(
              this.contactToWorld.elements[8],
              this.contactToWorld.elements[9],
              this.contactToWorld.elements[10]
            );

            // Angular component for tangent X direction
            deltaVelWorld.crossVectors(this.relativePositions[i], tangentX);
            deltaVelWorld.applyMatrix4(this.inertialMoments[i]);
            deltaVelWorld.cross(this.relativePositions[i]);
            deltaVelX += deltaVelWorld.dot(tangentX);

            // Angular component for tangent Z direction
            deltaVelWorld.crossVectors(this.relativePositions[i], tangentZ);
            deltaVelWorld.applyMatrix4(this.inertialMoments[i]);
            deltaVelWorld.cross(this.relativePositions[i]);
            deltaVelZ += deltaVelWorld.dot(tangentZ);
          }
        }

        // Add linear (mass) contribution
        deltaVelX += inverseMass;
        deltaVelY += inverseMass;
        deltaVelZ += inverseMass;

        // Calculate impulse needed for each direction
        // Normal impulse (Y) - same as frictionless case
        var impulseY = this.desiredDeltaVelocity / deltaVelY;

        // Tangential impulses needed to stop sliding
        // velocity.x and velocity.z are the tangential velocities in contact space
        var impulseX = -this.velocity.x / deltaVelX;
        var impulseZ = -this.velocity.z / deltaVelZ;

        // Calculate the magnitude of tangential impulse
        var tangentMagnitude = Math.sqrt(impulseX * impulseX + impulseZ * impulseZ);

        // Maximum tangential impulse allowed by friction (Coulomb friction)
        var maxFriction = this.friction * Math.abs(impulseY);

        if (tangentMagnitude > maxFriction && tangentMagnitude > 1e-6) {
          // Sliding friction - scale tangential impulse to friction limit
          var scale = maxFriction / tangentMagnitude;
          impulseX *= scale;
          impulseZ *= scale;
        }
        // else: static friction - use full tangential impulse to stop sliding

        impulse.set(impulseX, impulseY, impulseZ);

        return impulse;
      }
    }();

    this.applyPositionChange = function() {
      // closure scratch variables
      var angularInertiaWorld = new THREE.Vector3();
      var projection = new THREE.Vector3();
      var angularLimit = 0.2;
      var euler = new THREE.Euler();
      var quat = new THREE.Quaternion();

      return function(duration, linearChange, angularChange, max) {
        // resolve penetration

        var linearInertia = [],
            linearMove = [],
            angularInertia = [],
            angularMove = [],
            totalInertia = 0,
            totalMass = 0;

        if (!linearChange) {
        }

        for (var i = 0; i < this.bodies.length; i++) {
          var body = this.bodies[i];
          if (body && body.mass > 0) {
            angularInertiaWorld.crossVectors(this.relativePositions[i], this.normal);
            angularInertiaWorld.applyMatrix4(this.inertialMoments[i]);
            angularInertiaWorld.cross(this.relativePositions[i]);
            angularInertia[i] = angularInertiaWorld.dot(this.normal);

            linearInertia[i] = (body.mass == 0 ? 0 : 1 / body.mass);
            totalInertia += linearInertia[i] + angularInertia[i];
            totalMass += body.mass;
          }
        }

        if (totalMass > 0 && totalInertia > 0) {
          for (var i = 0; i < this.bodies.length; i++) {
            if (this.bodies[i].mass == 0) continue;
            var sign = (i == 0 ? 1 : -1);
            angularMove[i] = sign * this.penetration * (angularInertia[i] / totalInertia);
            linearMove[i] = sign * this.penetration * (linearInertia[i] / totalInertia);

            // To avoid angular projections that are too great (when mass is large
            // but inertia tensor is small) limit the angular move.
            projection.copy(this.normal).multiplyScalar(-this.relativePositions[i].dot(this.normal));
            projection.add(this.relativePositions[i]);

            // Use the small angle approximation for the sine of the angle (i.e.
            // the magnitude would be sine(angularLimit) * projection.magnitude
            // but we approximate sine(angularLimit) to angularLimit).
            var maxMagnitude = angularLimit * projection.length();

            if (angularMove[i] < -maxMagnitude) {
              var totalMove = angularMove[i] + linearMove[i];
              angularMove[i] = -maxMagnitude;
              linearMove[i] = totalMove - angularMove[i];
            } else if (angularMove[i] > maxMagnitude) {
              var totalMove = angularMove[i] + linearMove[i];
              angularMove[i] = maxMagnitude;
              linearMove[i] = totalMove - angularMove[i];
            }

            // We have the linear amount of movement required by turning
            // the rigid body (in angularMove[i]). We now need to
            // calculate the desired rotation to achieve that.
            if (angularMove[i] == 0) {
              // Easy case - no angular movement means no rotation
              angularChange[i].set(0,0,0);
            } else {
              // Work out the direction we'd like to rotate in, and the direction we'd need to rotate to achieve that
              angularChange[i].crossVectors(this.relativePositions[i], this.normal).applyMatrix4(this.inertialMoments[i]);
              angularChange[i].multiplyScalar(angularMove[i] / angularInertia[i]);
            }

            // Velocity change is easier - it's just the linear movement along the contact normal
            linearChange[i].copy(this.normal).multiplyScalar(linearMove[i]);
            this.impulses[i] = linearChange[i];
            this.bodies[i].position.add(linearChange[i]);

            euler.set(angularChange[i].x, angularChange[i].y, angularChange[i].z);
            quat.setFromEuler(euler);
            this.bodies[i].orientation.multiply(quat);
            //var theta = angularChange[i].length();
            // FIXME - potential coordinate space confusion
            //q.setFromAxisAngle(angularChange[i].clone().divideScalar(theta), theta);
            //this.bodies[i].orientation.copy(q);
          }
        }
        //console.log('move it', linearMove, [linearChange[0].toArray(), linearChange[1].toArray()], angularMove, [angularChange[0].toArray(), angularChange[1].toArray()], this.normal.toArray(), this);
      }
    }();
    this.finalizeMovement = function(duration, linearMomentum, angularMomentum) {
    }
  });

  elation.extend('physics.contact_dynamic', function(contactargs) {
    elation.physics.contact.call(this, contactargs);
    this.penetrationTime = contactargs.penetrationTime || 0;
    this.collisionVelocities = [];

    if (this.bodies[0]) this.collisionVelocities[0] = this.bodies[0].velocity.clone();
    if (this.bodies[1]) this.collisionVelocities[1] = this.bodies[1].velocity.clone();

    /**
     * Resolve a collision using default physical simulation
     */
    this.resolve = function(t, a, b) {
      this.restitution = this.bodies[0].restitution * this.bodies[1].restitution;

      // Calculate friction from body materials (use geometric mean, or max if one is zero)
      var friction0 = this.bodies[0].material ? this.bodies[0].material.dynamicfriction : 0;
      var friction1 = this.bodies[1].material ? this.bodies[1].material.dynamicfriction : 0;
      if (friction0 > 0 && friction1 > 0) {
        this.friction = Math.sqrt(friction0 * friction1);
      } else {
        this.friction = Math.max(friction0, friction1);
      }

      if (!this.initialized) {
        this.calculateInternals(t);
        this.initialized = true;
      }
      // Move the object to its exact collision point
      this.applyPositionChange(t, a, b);

      // Fire events for both objects, and combine them into one array
      var events = elation.events.fire({type: 'physics_collide', element: this.bodies[0], data: this});
      events.push.apply(events, elation.events.fire({type: 'physics_collide', element: this.bodies[1], data: this}));

      if (!elation.events.wasDefaultPrevented(events)) {
        // Apply velocity impulse if there's actual penetration
        // (penetration < 0 means objects are overlapping)
        if (this.penetration < 0) {
          this.applyVelocityChange(t, a, b);
          this.finalizeMovement(t, a, b);
        }
        events.push.apply(events, elation.events.fire({type: 'physics_collision_resolved', element: this.bodies[0], data: this}));
        events.push.apply(events, elation.events.fire({type: 'physics_collision_resolved', element: this.bodies[1], data: this}));
      }
    }
    this.calculateInternals = (function() {
      let scaledVelocity = new THREE.Vector3();
      return function(duration) {
        this.calculateContactMatrix();

        // For dynamic collisions, we time-shift forwards using the specified penetrationTime to figure out the exact point we collided

        // Calculate relative position and inertial moment for the first body
        this.relativePositions[0] = this.bodies[0].localToWorldPos().add(scaledVelocity.copy(this.bodies[0].velocity).multiplyScalar(duration * this.penetrationTime)).sub(this.point); // allocate vector
        var mworld0 = this._tmpmat.makeRotationFromQuaternion(this.bodies[0].orientationWorld);
        this.inertialMoments[0] = this.bodies[0].collider.momentInverse.clone().multiply(mworld0); // allocate matrix

        // If we have a second body, figure out its position and inertial moment
        if (this.bodies[1]) {
          this.relativePositions[1] = this.bodies[1].localToWorldPos().add(scaledVelocity.copy(this.bodies[1].velocity).multiplyScalar(duration * this.penetrationTime)).sub(this.point); // allocate vector
          var mworld1 = this._tmpmat.makeRotationFromQuaternion(this.bodies[1].orientationWorld);
          this.inertialMoments[1] = this.bodies[1].collider.momentInverse.clone().multiply(mworld1); // allocate matrix
        }

        // calculate velocities
        this.velocity = this.calculateLocalVelocity(0, duration);
        if (this.bodies[1]) {
          this.velocity.sub(this.calculateLocalVelocity(1, duration));
        }
        this.calculateDesiredDeltaVelocity(duration);
      }
    })();
    this.applyPositionChange = (function() {
      // closure scratch variables
      let angularInertiaWorld = new THREE.Vector3();
      return function(duration, linearChange, angularChange) {
        var linearInertia = [],
            linearMove = [],
            angularInertia = [],
            angularMove = [],
            totalInertia = 0,
            totalMass = 0;
        if (this.penetrationTime !== null) {
          for (let i = 0; i < this.bodies.length; i++) {
            var body = this.bodies[i];
            if (body && body.mass > 0) {
              angularInertiaWorld.crossVectors(this.relativePositions[i], this.normal);
              angularInertiaWorld.applyMatrix4(this.inertialMoments[i]);
              angularInertiaWorld.cross(this.relativePositions[i]);
              angularInertia[i] = angularInertiaWorld.dot(this.normal);

              //body.position.copy(this.relativePositions[i]).add(this.point).add(body.velocity.clone().multiplyScalar(duration * this.penetrationTime));
              if (this.collisionVelocities[i]) {
                body.position.add(this.collisionVelocities[i].clone().multiplyScalar(duration * this.penetrationTime));
              }

              linearInertia[i] = (body.mass == 0 ? 0 : 1 / body.mass);
              totalInertia += linearInertia[i] + angularInertia[i];
              totalMass += body.mass;
            }
          }
        }
      }
    })();
    this.finalizeMovement = (function() {
      let scaledVelocity = new THREE.Vector3();
      return function(duration, linearMomentum, angularMomentum) {
        // We've timeshifted our object to the collision point and calculated its new velocity - use the remaining
        // frame time to move the object along our new velocity vector to its reflected position

        // TODO - instead of just blindly moving, we should put this rigidbody back through the collision system again
        // and resolve any further collisions which might happen within our timestep.  This would prevent us from tunneling
        // through other objects after our first bounce.

        for (let i = 0; i < this.bodies.length; i++) {
          var body = this.bodies[i];
          if (body && body.mass > 0) {
            body.position.copy(this.relativePositions[i]).add(this.point).add(scaledVelocity.copy(body.velocity).multiplyScalar(duration * (1 - this.penetrationTime)));
          }
        }
      }
    })();
  }, elation.physics.contact);

});
