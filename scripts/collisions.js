'use strict';
/**
 * colliders 
 */
elation.require([], function() {

  elation.extend("physics.colliders.helperfuncs", new function() {
    this.sphere_sphere = function() {
      // closure scratch variables
      var thispos = new THREE.Vector3(),
          otherpos = new THREE.Vector3(),
          midline = new THREE.Vector3(),
          scaledVelocity = new THREE.Vector3(),
          intersectionPoint = new THREE.Vector3();

      return function(obj1, obj2, contacts, dt) {
        if (!contacts) contacts = [];

        // Work in world space
        obj1.body.localToWorldPos(thispos.set(0,0,0));
        obj2.body.localToWorldPos(otherpos.set(0,0,0));

        let dynamic = true; // TODO - this should either be a flag on rigid bodies, or a configurable threshold based on velocity
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
          let v = scaledVelocity.copy(obj1.body.velocity).sub(obj2.body.velocity).multiplyScalar(dt);

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
      var center = new THREE.Vector3(),      // center of sphere, box-space coordinates
          centerWorld = new THREE.Vector3(), // center of sphere, world-space coordinates
          diff = new THREE.Vector3(),
          closest = new THREE.Vector3(),
          scale = new THREE.Vector3(),
          scaledmin = new THREE.Vector3(),
          scaledmax = new THREE.Vector3();

      return function(box, sphere, contacts, dt) {
        if (!contacts) contacts = [];

        // Get sphere position in world and in the box's coordinate space
        if (sphere.offset) {
          sphere.body.localToWorldPos(centerWorld.copy(sphere.offset));
        } else {
          sphere.body.localToWorldPos(centerWorld.set(0,0,0));
        }
        box.body.worldToLocalPos(center.copy(centerWorld));
        box.body.worldToLocalScale(scale.set(1,1,1));

        scaledmin.copy(scale).multiply(box.min);
        scaledmax.copy(scale).multiply(box.max);

        // Early out if any of the axes are separating
        if ((center.x + sphere.radius < scaledmin.x || center.x - sphere.radius > scaledmax.x) ||
            (center.y + sphere.radius < scaledmin.y || center.y - sphere.radius > scaledmax.y) ||
            (center.z + sphere.radius < scaledmin.z || center.z - sphere.radius > scaledmax.z)) {
          return false;
        }

        // Find closest point on box
        closest.x = elation.utils.math.clamp(center.x, scaledmin.x, scaledmax.x);
        closest.y = elation.utils.math.clamp(center.y, scaledmin.y, scaledmax.y);
        closest.z = elation.utils.math.clamp(center.z, scaledmin.z, scaledmax.z);

        // See if we're in contact
        diff.subVectors(closest, center);
        var dist = diff.lengthSq();
        if (dist > sphere.radius * sphere.radius) {
          return 0;
        }
  //console.log('BOING', closest.toArray(), center.toArray(), diff.toArray(), dist, sphere.radius);

        // Transform back to world space
        box.body.localToWorldPos(closest);

        var contact = new elation.physics.contact({
          point: closest.clone(), // allocate point
          normal: centerWorld.clone().sub(closest).normalize(), // allocate normal
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

    this.box_box = function() {
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
    this.cylinder_box = function(cylinder, box, contacts, dt) {
      //return this.cylinder_sphere(cylinder, sphere, contacts);
    }
    this.cylinder_cylinder = function(cylinder, box, contacts, dt) {
      //return this.cylinder_sphere(cylinder, sphere, contacts);
    }
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
      var spherepos = new THREE.Vector3();
      var start = new THREE.Vector3();
      var end = new THREE.Vector3();
      var point = new THREE.Vector3();
      var normal = new THREE.Vector3();

      return function(capsule, sphere, contacts, dt) {
        var radius = capsule.radius + sphere.radius;
        capsule.body.localToWorldPos(start.set(0,-capsule.length/2,0));
        capsule.body.localToWorldPos(end.set(0,capsule.length/2,0));
        sphere.body.localToWorldPos(point.set(0,0,0));

        var closest = elation.physics.colliders.helperfuncs.closest_point_on_line(start, end, point);
        normal.subVectors(closest, point);
        var distance = normal.length();
  //console.log(distance, radius, capsule.radius, sphere.radius);
        if (distance <= radius) {
          normal.divideScalar(distance);
          var contact = new elation.physics.contact({
            normal: normal.clone(),
            point: closest.add(normal.multiplyScalar(capsule.radius)),
            penetration: radius - distance,
            bodies: [capsule.body, sphere.body]
          });
          contacts.push(contact);
        }
        return contacts;
      }
    }();

    this.capsule_box = function() {
      // closure scratch variables
      var boxpos = new THREE.Vector3();
      var start = new THREE.Vector3();
      var end = new THREE.Vector3();
      var point = new THREE.Vector3();
      var normal = new THREE.Vector3();
      var rigid = new elation.physics.rigidbody();

      return function(capsule, box, contacts, dt) {

        start.set(0,-capsule.length/2,0);
        end.set(0,capsule.length/2,0);
        if (capsule.offset) {
          start.add(capsule.offset);
          end.add(capsule.offset);
        }
        capsule.body.localToWorldPos(start);
        capsule.body.localToWorldPos(end);
        //box.body.localToWorldPos(point.set(0,0,0));

        // FIXME - ugly hack using two spheres
        // TODO - use proper sphere-swept line calculations
        rigid.velocity = capsule.body.velocity;
        rigid.orientation = capsule.body.orientation;
        rigid.position = start;

        var sphere = new elation.physics.colliders.sphere(rigid, {radius: capsule.radius});
        var head = elation.physics.colliders.helperfuncs.box_sphere(box, sphere);

        rigid.position = end;
        var sphere2 = new elation.physics.colliders.sphere(rigid, {radius: capsule.radius});
        var tail = elation.physics.colliders.helperfuncs.box_sphere(box, sphere2);

        if (head && tail) {
          head[0].bodies[1] = capsule.body;
          tail[0].bodies[1] = capsule.body;
          //contacts.push(head[0].penetration > tail[0].penetration ? head[0] : tail[0]);
          contacts.push(head[0]);
        } else if (head) {
          head[0].bodies[1] = capsule.body;
          contacts.push(head[0]);
        } else if (tail) {
          tail[0].point.y -= capsule.length;
          tail[0].bodies[1] = capsule.body;
          contacts.push(tail[0]);
        }

        return contacts;
      }
    }();
    this.capsule_cylinder = function() {
      // closure scratch variables
      var cylpos = new THREE.Vector3();
      var start = new THREE.Vector3();
      var end = new THREE.Vector3();
      var point = new THREE.Vector3();
      var normal = new THREE.Vector3();

      return function(capsule, cylinder, contacts, dt) {

        capsule.body.localToWorldPos(start.set(0,-capsule.length/2,0));
        capsule.body.localToWorldPos(end.set(0,capsule.length/2,0));
        //box.body.localToWorldPos(point.set(0,0,0));

        var sphere = new elation.physics.colliders.sphere(capsule.body, {radius: capsule.radius});
        sphere.offset = new THREE.Vector3(0,-capsule.length/2,0);
        if (capsule.offset) sphere.offset.add(capsule.offset);
        var head = elation.physics.colliders.helperfuncs.box_sphere(box, sphere);
        sphere.offset = new THREE.Vector3(0,capsule.length/2,0);
        if (capsule.offset) sphere.offset.add(capsule.offset);
        var tail = elation.physics.colliders.helperfuncs.box_sphere(box, sphere);
        if (head && tail) {
          head[0].bodies[1] = capsule.body;
          tail[0].bodies[1] = capsule.body;
          contacts.push(head[0].penetration > tail[0].penetration ? head[0] : tail[0]);
        } else if (head) {
          head[0].bodies[1] = capsule.body;
          contacts.push(head[0]);
        } else if (tail) {
          tail[0].bodies[1] = capsule.body;
          contacts.push(tail[0]);
        }

        return contacts;
      }
    }();
    this.triangle_sphere = function() {
      // closure scratch variables
      var spherepos = new THREE.Vector3(),
          endpos = new THREE.Vector3(),
          scale = new THREE.Vector3(),
          scalednormal = new THREE.Vector3(),
          scaledVelocity = new THREE.Vector3(),
          scaledaccel = new THREE.Vector3(),
          intersectionPoint = new THREE.Vector3(),
          triangleClosestPoint = new THREE.Vector3(),
          normal = new THREE.Vector3();

      // Reference: http://www.peroxide.dk/papers/collision/collision.pdf

      return function(triangle, sphere, contacts, dt) {
        triangle.body.worldToLocalScale(scale.set(1,1,1));

        // Sphere position should already be in the same coordinate system as the triangle
        spherepos.copy(triangle.normal).multiplyScalar(-sphere.radius).multiply(scale).add(sphere.body.position);
        if (sphere.offset) {
          spherepos.add(sphere.offset);
        }

        scaledVelocity.copy(sphere.body.velocity)//.multiply(scale);
        scaledVelocity.multiplyScalar(dt);
        endpos.copy(spherepos).add(scaledVelocity);

        // Find intersection between the ray our sphere is travelling and the plane upon which our triangle rests
        let intersectionPlane = elation.physics.colliders.helperfuncs.line_plane(spherepos, endpos, triangle.p1, triangle.p2, triangle.p3, intersectionPoint);
        if (intersectionPlane) {
          if (triangle.containsPoint(intersectionPlane.point)) {
            // If the intersection point is inside of our triangle, we've collided with the triangle's face
            var contact = new elation.physics.contact_dynamic({
              normal: triangle.normal.clone(), // allocate normal
              point: intersectionPlane.point.clone(), // allocate point
              penetrationTime: intersectionPlane.t,
              bodies: [triangle.body, sphere.body],
              triangle: triangle
            });
            contacts.push(contact);
            return contacts;
          } else if (false) {
            // Check if the sphere intersects with the edge of the triangle
            elation.physics.colliders.helperfuncs.closest_point_on_triangle(intersectionPlane.point, triangle.p1, triangle.p2, triangle.p3, triangleClosestPoint);
            let intersectionEdge = elation.physics.colliders.helperfuncs.line_sphere(triangleClosestPoint, scaledVelocity.copy(sphere.body.velocity).multiplyScalar(-dt).add(triangleClosestPoint), sphere.body.position, sphere.radius, intersectionPoint, true);
            normal.copy(triangleClosestPoint).sub(sphere.body.position).normalize();
            if (intersectionEdge && intersectionEdge.point.distanceTo(triangleClosestPoint) <= sphere.radius - scaledVelocity.dot(normal)) {
              //console.log('got an intersection on the edge', intersectionEdge, triangleClosestPoint);
              var contact = new elation.physics.contact_dynamic({
                normal: normal.clone(), // allocate point
                point: triangleClosestPoint.clone(), // allocate point
                penetrationTime: intersectionEdge.t,
                bodies: [triangle.body, sphere.body],
                triangle: triangle
              });

              contacts.push(contact);
              return contacts;
            }
          }
        }
      }
    }();
    this.mesh_sphere = function() {
      // closure scratch variables
      var localsphere,
          meshsphere; // A sphere collider representing the collider in the mesh's local space
      let meshworldpos = new THREE.Vector3();

      return function(mesh, sphere, contacts, dt) {
        var localcontacts = [];

        // The sphere and mesh we're passed in are both potentially in different coordinate spaces.
        // Here, we transform the sphere into the mesh's coordinate space, which makes calculating
        // intersections more efficient.  Once we've determined local collisions, we then need to
        // transform the collision position and normal back into the sphere's original coordinate space.

// TODO - instead of being in the mesh's coordinate space, we should transform both the mesh and the sphere into world coordinate space.  This is a bit less efficient but means we automatically take into account scale and parent offsets

        if (!localsphere) {
          // Allocate the static closure variable for first-time use
          localsphere = new elation.physics.rigidbody();
          localsphere.setCollider('sphere', {radius: 1});
          meshsphere = new elation.physics.rigidbody();
          meshsphere.setCollider('sphere', {radius: 1});
        }

        // Transform sphere and mesh positions, velocity, and acceleration to world coordinates
        mesh.body.localToWorldPos(meshworldpos.set(0, 0, 0))
        sphere.body.localToWorldPos(localsphere.position.set(0, 0, 0))

        // Velocity and acceleration shouldn't be affected by the object's scale, so we divide the sphere's velocity and accel by its scale to undo what the localToWorld applies
        sphere.body.localToWorldPos(localsphere.velocity.copy(sphere.body.velocity).divide(sphere.body.scale)).sub(localsphere.position);
        sphere.body.localToWorldPos(localsphere.acceleration.copy(sphere.body.acceleration).divide(sphere.body.scale)).sub(localsphere.position);

        // Transform position, velocity, and acceleration into mesh-local coordinate system
        mesh.body.worldToLocalPos(localsphere.velocity.add(meshworldpos));
        mesh.body.worldToLocalPos(localsphere.acceleration.add(meshworldpos));

        //let localvel = mesh.body.worldToLocalDir(sphere.body.localToWorldDir(sphere.body.velocity.clone()));

        // Transform sphere into local coordinates
        mesh.body.worldToLocalPos(localsphere.position);
        if (sphere.offset) {
          // Transform sphere offset into world coordinate space, then into mesh-relative coordinate space
          localsphere.collider.offset = mesh.body.worldToLocalDir(sphere.body.localToWorldDir(sphere.offset.clone())); 
        }
        //mesh.body.worldToLocal(sphere.localToWorld(localsphere.dir.set(0, 0, 0)))

        localsphere.collider.radius = sphere.radius;
        //mesh.body.worldToLocalScale(sphere.body.localToWorldScale(localsphere.collider.scale.set(1,1,1)));
        localsphere.collider.offset = sphere.offset;

        meshsphere.collider.radius = mesh.radius;

        // early-out if the mesh's bounding sphere wouldn't intersect with the potentially-colliding sphere
        //let spherecollisions = elation.physics.colliders.helperfuncs.sphere_sphere(meshsphere.collider, localsphere.collider);
        //if (spherecollisions.length > 0) {
        if (localsphere.position.length() <= mesh.radius + localsphere.velocity.length()) {
          //console.log('potentially colliding', mesh, sphere, localsphere);
          for (var i = 0; i < mesh.triangles.length; i++) {
            elation.physics.colliders.helperfuncs.triangle_sphere(mesh.triangles[i], localsphere.collider, localcontacts, dt);
          }
        }
/*
        if (!worldsphere) {
          // Allocate the static closure variable for first-time use
          worldsphere = new elation.physics.rigidbody();
          worldsphere.setCollider('sphere', {radius: 1});
          meshsphere = new elation.physics.rigidbody();
          meshsphere.setCollider('sphere', {radius: 1});
          worldtriangle = new elation.physics.rigidbody();
          worldtriangle.setCollider('triangle', {});
        }
*/

        if (localcontacts.length > 0) {
          var closest = localcontacts[0];
          //console.log('got some local contacts!', localcontacts[0].point, localcontacts[0].normal, localsphere.position, localsphere.velocity, localcontacts[0]);
          for (var i = 0; i < localcontacts.length; i++) {
            if (closest.penetrationTime < localcontacts[i].penetrationTime) {
              closest = localcontacts[i];
            }
          }

          if (closest.bodies[0] === localsphere) {
            closest.bodies[0] = sphere.body;
          } else if (closest.bodies[1] === localsphere) {
            closest.bodies[1] = sphere.body;
          }
          mesh.body.localToWorldPos(closest.point);
          mesh.body.localToWorldDir(closest.normal);
          contacts.push(closest);
/*
            var contact = localcontacts[i];
            if (contact.bodies[0] === localsphere) {
              contact.bodies[0] = sphere.body;
            } else if (contact.bodies[1] === localsphere) {
              contact.bodies[1] = sphere.body;
            }

            mesh.body.localToWorldPos(contact.point);
            mesh.body.localToWorldDir(contact.normal);
            contacts.push(contact);
          }
*/
        }

        return contacts;
      }
    }();


    this.closest_point_on_sphere = function(point, center, radius) {
      var closest = point.clone().sub(center);
      closest.normalize().multiplyScalar(radius);
      return closest;
    }
    this.closest_point_on_line = function() {
      var line = new THREE.Vector3(),
          proj = new THREE.Vector3();

      return function(start, end, point) {
        line.copy(end).sub(start);
        var lengthSq = line.lengthSq();
        if (lengthSq < 1e-6) { // zero-length line
          return start.clone();
        }

        // Project point on to line
        proj.subVectors(point, start);
        var t = proj.dot(line) / lengthSq;
        if (t < 0) { // beyond the start
          return start.clone();
        } else if (t > 1) { // beyond the end
          return end.clone();
        }

        // Find point perpendicular to line segment
        proj.copy(start).add(line.multiplyScalar(t));
        return proj.clone();
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
    this.distance_to_line = function(start, end, point) {
      return this.closest_point_on_line(start, end, point).distanceTo(point);
    }

    this.ray_sphere = (function() {
      // Closure scratch variables
      let m = new THREE.Vector3();

      return function(raypos, dir, spherepos, radius, point) {
        m.copy(raypos).sub(spherepos);
        let b = m.dot(dir),
            c = m.dot(m) - radius * radius;
        // Exit if ray's origin is outside of sphere and ray is pointing away from sphere
        if (c > 0 && b > 0) return;
        let discr = b * b - c;
        // A negative discriminant corresponds to ray missing sphere
        if (discr < 0) return;
        // Ray now found to be intersecting sphere, compute smallest t value of intersection
        let t = -b - Math.sqrt(discr);
        if (t < 0) t = 0;
        if (!point) point = new THREE.Vector3(); // Allocate return vector if reference isn't passed in
        point.copy(dir).multiplyScalar(t).add(raypos); // FIXME - allocation
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
          point.copy(start).add(ab.multiplyScalar(t));
          return {point, t};
        }
      }
    })();
  });


  /*
   * =========
   * colliders
   * =========
   */
 
  elation.extend("physics.colliders.sphere", function(body, args) {
    this.type = 'sphere';
    this.body = body;
    this.radius = args.radius || args;
    this.scale = args.scale || new THREE.Vector3(1,1,1),
    this.offset = args.offset || false;

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
      var c = 5 / (2 * this.body.mass * this.radius * this.radius);
      this.momentInverse = new THREE.Matrix4();
      this.momentInverse.set(
        c, 0, 0, 0, 
        0, c, 0, 0, 
        0, 0, c, 0, 
        0, 0, 0, 1);
      return this.momentInverse;
    }
  });
  elation.extend("physics.colliders.plane", function(body, args) {
    this.type = 'plane';
    if (!args) args = {};
    this.body = body;
    this.normal = args.normal || new THREE.Vector3(0,1,0);
    this.offset = args.offset || 0;
    this.radius = Infinity;

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
      this.momentInverse = new THREE.Matrix4().identity();
      return this.momentInverse;
    }
  });
  elation.extend("physics.colliders.box", function(body, args) {
    this.type = 'box';
    if (!args) args = {};
    this.body = body;
    this.min = args.min || new THREE.Vector3(0,0,0);
    this.max = args.max || new THREE.Vector3(0,0,0);
    this.halfsize = this.max.clone().sub(this.min).divideScalar(2);
    this.offset = this.max.clone().add(this.min).divideScalar(2);

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
      } else {
        //console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      var diff = this.max.clone().sub(this.min);
      var xsq = diff.x*diff.x,
          ysq = diff.y*diff.y,
          zsq = diff.z*diff.z,
          m = 1/12 * this.body.mass;
      this.momentInverse = new THREE.Matrix4();
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
  });
  elation.extend("physics.colliders.cylinder", function(body, args) {
    this.type = 'cylinder';
    if (!args) args = {};
    this.body = body;
    this.radius = args.radius;
    this.height = args.height;
    this.offset = args.offset || new THREE.Vector3();

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
      } else {
        console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      var rsq = this.radius * this.radius,
          hsq = this.height * this.height,
          m = this.body.mass,
          i1 = (m * hsq / 12) + (m * rsq / 4),
          i2 = m * rsq / 2;
      this.momentInverse.set(
        i1, 0, 0, 0, 
        0, i2, 0, 0, 
        0, 0, i1, 0, 
        0, 0, 0, 1);
      return this.momentInverse;
    }
  });
  elation.extend("physics.colliders.capsule", function(body, args) {
    this.type = 'capsule';
    if (!args) args = {};
    this.body = body;
    this.radius = args.radius;
    this.length = args.length;
    this.offset = args.offset;

    this.getContacts = function(other, contacts, dt) {
      if (!contacts) contacts = [];
      if (other instanceof elation.physics.colliders.plane) {
        contacts = elation.physics.colliders.helperfuncs.capsule_plane(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.sphere) {
        contacts = elation.physics.colliders.helperfuncs.capsule_sphere(this, other, contacts, dt);
      } else if (other instanceof elation.physics.colliders.box) {
        contacts = elation.physics.colliders.helperfuncs.capsule_box(this, other, contacts, dt);
  /*
      } else if (other instanceof elation.physics.colliders.cylinder) {
        contacts = elation.physics.colliders.helperfuncs.capsule_cylinder(this, other, contacts);
      } else {
        console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
  */
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      var rsq = this.radius * this.radius,
          hsq = this.length * this.length,
          m = this.body.mass,
          i1 = (m * hsq / 12) + (m * rsq / 4),
          i2 = m * rsq / 2;
      this.momentInverse.set(
        i1, 0, 0, 0, 
        0, i2, 0, 0, 
        0, 0, i1, 0, 
        0, 0, 0, 1);
      return this.momentInverse;
    }
  });
  elation.extend("physics.colliders.mesh", function(body, args) {
    this.type = 'mesh';
    if (!args) args = {};
    this.body = body;

    this.mesh = args.mesh;
    this.radius = 0;

    this.extractTriangles = function(mesh) {
      let triangles = [];
      let radiusSq = 0;

      if (this.mesh.geometry) {
        if (this.mesh.geometry instanceof THREE.BufferGeometry) {
          let positions = this.mesh.geometry.attributes.position,
              posarr = positions.array;
          if (this.mesh.geometry.index) {
            let idx = this.mesh.geometry.index,
                idxarr = idx.array;
            for (var i = 0; i < idx.count / 3; i++) {
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
*/                  triangle = new elation.physics.colliders.triangle(this.body, [p1, p2, p3]);

              triangles.push(triangle);

              let l1 = p1.lengthSq(),
                  l2 = p2.lengthSq(),
                  l3 = p3.lengthSq();
              if (l1 > radiusSq) radiusSq = l1;
              if (l2 > radiusSq) radiusSq = l2;
              if (l3 > radiusSq) radiusSq = l3;
            }
          } else {
            for (var i = 0; i < positions.count / 3; i++) {
              let offset = i * 3 * 3,
                  p1 = new THREE.Vector3(posarr[offset    ], posarr[offset + 1], posarr[offset + 2]),
                  p2 = new THREE.Vector3(posarr[offset + 3], posarr[offset + 4], posarr[offset + 5]),
                  p3 = new THREE.Vector3(posarr[offset + 6], posarr[offset + 7], posarr[offset + 8]),
                  triangle = new elation.physics.colliders.triangle(this.body, [p1, p2, p3]);

              triangles.push(triangle);

              let l1 = p1.lengthSq(),
                  l2 = p2.lengthSq(),
                  l3 = p3.lengthSq();
              if (l1 > radiusSq) radiusSq = l1;
              if (l2 > radiusSq) radiusSq = l2;
              if (l3 > radiusSq) radiusSq = l3;
            }
          }
        }
      }
      this.radius = Math.sqrt(radiusSq);
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
            bodies[obj.uuid].position.copy(obj.position);
            bodies[obj.uuid].scale.copy(obj.scale);
            bodies[obj.uuid].orientation.copy(obj.quaternion);
            bodies[obj.uuid].object = this.body.object;
            if (obj instanceof THREE.Mesh) {
              bodies[obj.uuid].setCollider('mesh', {mesh: obj });
              //elation.events.add(bodies[obj.uuid], 'physics_collide', (ev) => elation.events.fire({type: 'physics_collide', element: this.body, event: ev}));
            }
            parent.add(bodies[obj.uuid]);
          }
          parent = bodies[obj.uuid];
        }
      }
    }

    this.extractObjects(this.mesh);
    this.triangles = this.extractTriangles(this.mesh);

    this.getContacts = function(other, contacts, dt) {
      if (!contacts) contacts = [];
      if (other instanceof elation.physics.colliders.sphere) {
        contacts = elation.physics.colliders.helperfuncs.mesh_sphere(this, other, contacts, dt);
  /*
      } else if (other instanceof elation.physics.colliders.box) {
        contacts = elation.physics.colliders.helperfuncs.mesh_box(this, other, contacts);
      } else if (other instanceof elation.physics.colliders.cylinder) {
        contacts = elation.physics.colliders.helperfuncs.mesh_cylinder(this, other, contacts);
      } else {
        console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
  */
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      var rsq = this.radius * this.radius,
          hsq = this.length * this.length,
          m = this.body.mass,
          i1 = (m * hsq / 12) + (m * rsq / 4),
          i2 = m * rsq / 2;
      // FIXME - this is not the inertial moment for a triangle
      this.momentInverse.set(
        i1, 0, 0, 0,
        0, i2, 0, 0,
        0, 0, i1, 0,
        0, 0, 0, 1);
      return this.momentInverse;
    }

    this.distanceTo = function(point) {
      return this.normal.dot(point) + this.offset;
    }
  });
  elation.extend("physics.colliders.triangle", function(body, args) {
    this.type = 'triangle';
    if (!args) args = {};
    this.body = body;

    this.v0 = new THREE.Vector3();
    this.v1 = new THREE.Vector3();

    this.normal = new THREE.Vector3(0,1,0);

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
  /*
      } else if (other instanceof elation.physics.colliders.box) {
        contacts = elation.physics.colliders.helperfuncs.capsule_box(this, other, contacts);
      } else if (other instanceof elation.physics.colliders.cylinder) {
        contacts = elation.physics.colliders.helperfuncs.capsule_cylinder(this, other, contacts);
      } else {
        console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
  */
      }
      return contacts;
    }
    this.getInertialMoment = function() {
      this.momentInverse = new THREE.Matrix4();
      var rsq = this.radius * this.radius,
          hsq = this.length * this.length,
          m = this.body.mass,
          i1 = (m * hsq / 12) + (m * rsq / 4),
          i2 = m * rsq / 2;
      // FIXME - this is not the inertial moment for a triangle
      this.momentInverse.set(
        i1, 0, 0, 0,
        0, i2, 0, 0,
        0, 0, i1, 0,
        0, 0, 0, 1);
      return this.momentInverse;
    }

    this.containsPoint = (function() {
      var p1 = new THREE.Vector3();
      var v2 = new THREE.Vector3();
      var scale = new THREE.Vector3();
      return function(point) {
        this.body.localToWorldScale(scale.set(1,1,1));

        p1.copy(this.p1).multiply(scale);

/*
        this.v0.x = (this.p2.x * scale.x) - p1.x;
        this.v0.y = (this.p2.y * scale.y) - p1.y;
        this.v0.z = (this.p2.z * scale.z) - p1.z;

        this.v1.x = (this.p3.x * scale.x) - p1.x;
        this.v1.y = (this.p3.y * scale.y) - p1.y;
        this.v1.z = (this.p3.z * scale.z) - p1.z;
*/
        this.v0.copy(this.p2).sub(p1);
        this.v1.copy(this.p3).sub(p1);

        v2.copy(point).sub(p1);

        let dot00 = this.v0.dot(this.v0),
            dot01 = this.v0.dot(this.v1),
            dot02 = this.v0.dot(v2),
            dot11 = this.v1.dot(this.v1),
            dot12 = this.v1.dot(v2);

        let invDenom = 1 / (dot00 * dot11 - dot01 * dot01),
            u = (dot11 * dot02 - dot01 * dot12) * invDenom,
            v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        return (u >= 0) && (v >= 0) && (u + v < 1);
      }
    })();
    this.distanceTo = (function() {
      let triangleClosestPoint = new THREE.Vector3();
      return function(point) {
        //return this.normal.dot(point) + this.offset;
        elation.physics.colliders.helperfuncs.closest_point_on_triangle(point, this.p1, this.p2, this.p3, triangleClosestPoint);
        return triangleClosestPoint.distanceTo(point);
      }
    })();
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
        this.applyVelocityChange(t, a, b);
        this.finalizeMovement(t, a, b);
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
        this.worldToContact = new THREE.Matrix4().getInverse(this.contactToWorld);
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
        velocityFromAccel -= .5 * lastaccel.copy(this.bodies[0].lastacceleration).multiplyScalar(duration).dot(this.normal);
      }
      if (this.bodies[1] && !this.bodies[1].state.sleeping) {
        velocityFromAccel += .5 * lastaccel.copy(this.bodies[1].lastacceleration).multiplyScalar(duration).dot(this.normal);
      }

      var restitution = this.restitution;
/*
      if (Math.abs(this.velocity.y) < 0.25) { // FIXME - velocity threshold should be configurable
        restitution = 0;
      }
*/
      this.desiredDeltaVelocity = -this.velocity.y - restitution * (this.velocity.y - velocityFromAccel);
      //if (this.desiredDeltaVelocity > 0) this.desiredDeltaVelocity *= -1;
      //console.log('desiredDeltaV: ' + this.desiredDeltaVelocity);
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
          this.relativePositions[1] = this.point.clone().sub(this.bodies[1].localToWorldPos(this.bodies[0].collider.offset.clone())); // allocate vector
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
        }

        if (this.bodies[1] && this.bodies[1].mass > 0) {
          rotationChange[1] = impulsiveTorque.crossVectors(impulse, this.relativePositions[1]).applyMatrix4(this.inertialMoments[1]);
          velocityChange[1] = impulsiveForce.copy(impulse).multiplyScalar(-1 / this.bodies[1].mass);
          this.impulses[1] = impulsiveForce.clone();

          this.bodies[1].addVelocity(impulsiveForce);
          this.bodies[1].addAngularVelocity(impulsiveTorque);
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
      return function() {
        var impulse = new THREE.Vector3();
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
        // If no event handlers handled this event, use our default collision response
        this.applyVelocityChange(t, a, b);
        this.finalizeMovement(t, a, b);
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
            //body.position.copy(this.relativePositions[i]).add(this.point).add(scaledVelocity.copy(body.velocity).multiplyScalar(duration * (1 - this.penetrationTime)));
          }
        }
      }
    })();
  }, elation.physics.contact);

});
