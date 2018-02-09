elation.require([], function() {
  elation.extend("physics.processor.base", function(parent) {
    this.parent = parent;
    this._tmpvec = new THREE.Vector3();
    this._tmpquat = new THREE.Quaternion();
    this._tmpmat = new THREE.Matrix4();

    this.update = function(objects, t, active) {
      if (typeof active == 'undefined') active = [];
      for (var i = 0; i < objects.length; i++) {
        var obj = objects[i];
        obj.updateState();
        if (!obj.state.sleeping) {
          active.push(obj);
        }
        if (obj.children.length > 0) {
          this.update(obj.children, t, active);
        }
      }
      return active;
    }
    this.iterate = function(objects, t) {
      console.log('iterate in base', this.parent);
    }
    this.collide = function(t) {
      if (t == 0) return; // paused, do nothing
      var collisions = [];
      var potentials = [];
      // FIXME - Brute force for now.  We should use octrees or BVH here
      var objects = this.parent.getObjects();
      for (var i = 0; i < objects.length-1; i++) {
        var obj1 = objects[i];
        if (obj1.collider) { // && obj1.collider.radius) {
          for (var j = i+1; j < objects.length; j++) {
            var obj2 = objects[j];
            //if (obj2.collider && obj2.collider.radius && !(obj1.state.sleeping && obj2.state.sleeping) && obj1.isPotentiallyColliding(obj2)) {
            if (obj2.collider && !(obj1.state.sleeping && obj2.state.sleeping)) {
              potentials.push([obj1, obj2]);
            }
          }
        }
      }
      if (potentials.length > 0) {
        for (var i = 0; i < potentials.length; i++) {
          var obj1 = potentials[i][0], obj2 = potentials[i][1];
          // Get list of all contact points between the two objects
          var contacts = obj1.getContacts(obj2, collisions);
          /*
          if (contacts && contacts.length > 0) {
            // Resolve the deepest contact first
            var deepest = this.getDeepestContact(contacts);
            collisions.push(deepest);
          }
          */
        }
        //console.log(potentials.length + ' potential crashes:', potentials, collisions);
      }
      return collisions;
    }
    this.getDeepestContact = function(contacts) {
      var deepest = 0;
      for (var i = 0; i < contacts.length; i++) {
        if (contacts[i].penetration < contacts[deepest].penetration) {
          deepest = i;
        }
      }
      return contacts[deepest];
    }
    this.resolve = function(t, contacts) {
      if (contacts.length == 0) {
        return;
      }
      if (true) {
        var linearChange = [
          new THREE.Vector3(),
          new THREE.Vector3()
        ];
        var angularChange = [
          new THREE.Vector3(),
          new THREE.Vector3()
        ];
        for (var i = 0; i < contacts.length; i++) {
          contacts[i].resolve(t, linearChange, angularChange);
        }
      } else {
        this.prepareContacts(contacts, t);
        this.adjustPositions(contacts, t);
        this.adjustVelocities(contacts, t);
      }
    }
    this.prepareContacts = function(contacts, t) {
      for (var i = 0; i < contacts.length; i++) {
        var contact = contacts[i];
        contact.calculateInternals(t);

        // Fire physics_collide event for each body involved
        // FIXME - should only fire once per object-pair
        var events = elation.events.fire({type: 'physics_collide', element: contact.bodies[0], data: contact});
        events.concat(elation.events.fire({type: 'physics_collide', element: contact.bodies[1], data: contact}));
      }
    }
    this.adjustPositions = function(contacts, t) {
      // TODO - class-wide settings
      var positionIterations = 100;
      var positionEpsilon = 1e-2;

      // closure scratch variables
      var deltaPosition = new THREE.Vector3();
      var linearChange = [
        new THREE.Vector3(),
        new THREE.Vector3()
      ];
      var angularChange = [
        new THREE.Vector3(),
        new THREE.Vector3()
      ];

      var iteration = 0,
          numcontacts = contacts.length,
          max;
      while (iteration < positionIterations) {
        max = positionEpsilon;
        var idx = numcontacts;
        for (var i = 0; i < numcontacts; i++) {
          if (contacts[i].penetration > max) {
            max = contacts[i].penetration;
            idx = i;
          }
        }
        if (idx == numcontacts) break;

        var lastcontact = contacts[idx];
        lastcontact.applyPositionChange(linearChange, angularChange, max);
  console.log('apply change', lastcontact);

        // Updating position might have changed penetration of other bodies, so update them
        for (var i = 0; i < numcontacts; i++) {
          var contact = contacts[i];
          // check each body in contact
          for (var b = 0; b < 2; b++) {
            if (!contact.bodies[b]) continue;
            // match against each body in the newly resolved contact
            for (var d = 0; d < 2; d++) {
              if (contact.bodies[b] == lastcontact.bodies[d]) {
                deltaPosition.crossVectors(angularChange[d], contact.relativePositions[b]).add(linearChange[d]);
                contact.penetration += deltaPosition.dot(contact.normal) * (b ? 1 : -1);
  console.log('update penetration', contact);
              }
            }
          }
        }
        iteration++;
      }
    }
    this.adjustVelocities = function(contacts, t) {
  /*
      // FIXME - split this out correctly!
      for (var i = 0; i < contacts.length; i++) {
        var contact = contacts[i];
      
        // Check to see if preventDefault was called in any event handlers
        var process = true;
        for (var j = 0; j < events.length; j++) {
          process = process && events[j].returnValue;
        }
        // If not already handled, fall back on default physically-simulated bounce handling
        if (process) {
          contact.applyVelocityChange(t);
          elation.events.fire({type: 'physics_collision_resolved', element: contact.bodies[0], data: contact});
          elation.events.fire({type: 'physics_collision_resolved', element: contact.bodies[1], data: contact});
        }
      }
  */
      // TODO - class-wide settings
      var velocityIterations = 100;
      var velocityEpsilon = 1e-2;

      // closure scratch variables
      var deltaVel = new THREE.Vector3();
      var velocityChange = [
        new THREE.Vector3(),
        new THREE.Vector3()
      ];
      var rotationChange = [
        new THREE.Vector3(),
        new THREE.Vector3()
      ];

      var iteration = 0, 
          numcontacts = contacts.length,
          max;
      while (iteration < velocityIterations) {
        max = velocityEpsilon;
        var idx = numcontacts;
        for (var i = 0; i < numcontacts; i++) {
          // Find the fastest-moving contact
          if (contacts[i].desiredDeltaVelocity > max) {
            max = contacts[i].desiredDeltaVelocity;
            idx = i;
          }
        }
  console.log('iter', max, idx);
        if (idx == numcontacts) break;

        // Resolve velocity for this contact
        contacts[idx].applyVelocityChange(velocityChange, rotationChange);

        // Recompute closing velocities for other contacts which may have changed
        for (var i = 0; i < numcontacts; i++) {
          for (var b = 0; b < 2; b++) {
            if (!contacts[i].bodies[b]) continue;

            for (var d = 0; d < 2; d++) {
              if (contacts[i].bodies[b] == contacts[idx].bodies[d]) {
                deltaVel.crossVectors(rotationChange[d], contacts[i].relativePositions[b]).add(velocityChange[d]).applyMatrix4(contacts[i].contactToWorld);
  console.log('update velocity', contacts[i], deltaVel.toArray());
                if (b == 1) deltaVel.multiplyScalar(-1);
                contacts[i].velocity.add(deltaVel);
                contacts[i].calculateDesiredDeltaVelocity(t);
              }
            }
          }
        }
        iteration++;
      } 
    }
  });
  elation.extend("physics.processor.cpu", function(parent) {
    elation.physics.processor.base.call(this, parent);
    this.iterate = function(objects, t) {
      if (t == 0) return; // paused, do nothing
      var framedata = {};
      for (var i = 0; i < objects.length; i++) {
        objects[i].updateAcceleration(framedata);
        var scaledtime = objects[i].getTimescale() * t;
        if (objects[i].state.accelerating || objects[i].state.moving) {
          this.iterateAxis(objects[i], 'x', scaledtime);
          this.iterateAxis(objects[i], 'y', scaledtime);
          this.iterateAxis(objects[i], 'z', scaledtime);
        }
        if (objects[i].state.rotating) {
          this.iterateRotation(objects[i], scaledtime);
        }
        objects[i].updateState(framedata);
        if (!objects[i].state.sleeping) {
          elation.events.fire({type: "physics_update", element: objects[i], data: t});
        }
      }
    }
    this.iterateAxis = function(obj, axis, t) {
      var pos = obj.position[axis],
          vel = obj.velocity[axis],
          accel = obj.acceleration[axis];

      vel += accel * t;
      pos += vel * t;

      obj.position[axis] = pos;
      obj.velocity[axis] = vel * Math.pow(obj.linearDamping, t);
    }
    this.iterateRotation = function(obj, t) {
      this._tmpvec.copy(obj.angularacceleration).multiplyScalar(t);
      obj.angular.add(this._tmpvec).multiplyScalar(Math.pow(obj.angularDamping, t));

      this._tmpvec.copy(obj.angular);
      var theta = this._tmpvec.length();
      this._tmpvec.divideScalar(theta);
      this._tmpquat.setFromAxisAngle(this._tmpvec, theta*t);
      obj.orientation.multiply(this._tmpquat);
    }
  }, false, elation.physics.processor.base);

  elation.extend("physics.processor.worker", function(parent) {
    elation.physics.processor.base.call(this, parent);

    this.worker = new WebWorker('cyclone-worker.js');

    this.iterate = function(objects, t) {
      if (t == 0) return; // paused, do nothing
      for (var i = 0; i < objects.length; i++) {
        objects[i].updateAcceleration();
        if (objects[i].state.accelerating || objects[i].state.moving) {
          this.iterateAxis(objects[i], 'x', t);
          this.iterateAxis(objects[i], 'y', t);
          this.iterateAxis(objects[i], 'z', t);
        }
        if (objects[i].state.rotating) {
          this.iterateRotation(objects[i], t);
        }
        objects[i].updateState();
        if (!objects[i].state.sleeping) {
          elation.events.fire({type: "physics_update", element: objects[i], data: t});
        }
      }
    }
    this.iterateAxis = function(obj, axis, t) {
      this._tmpvec.set(obj.position[axis], obj.velocity[axis], obj.acceleration[axis]);
      this._tmpvec.applyMatrix4(this.parent.physicsmatrix);
      obj.position[axis] = this._tmpvec.x;
      obj.velocity[axis] = this._tmpvec.y * Math.pow(obj.linearDamping, t);
    }
    this.iterateRotation = function(obj, t) {
      this._tmpvec.copy(obj.angularacceleration).multiplyScalar(t);
      obj.angular.add(this._tmpvec).multiplyScalar(Math.pow(obj.angularDamping, t));

      this._tmpvec.copy(obj.angular);
      var theta = this._tmpvec.length();
      this._tmpvec.divideScalar(theta);
      this._tmpquat.setFromAxisAngle(this._tmpvec, theta*t);
      obj.orientation.multiply(this._tmpquat);
    }
  }, false, elation.physics.processor.base);

  elation.extend("physics.processor.gpu", function(parent) {
    elation.physics.processor.base.call(this, parent);

    // TODO:
    // - pack linear+angular pos/vel/accel into texture (OES_texture_float?)
    // - glsl shader multiplies each point by the update matrix, results returned as image
    // - glReadPixels to read back result, map updates back to objects

    this.iterate = function(objects, t) {
      console.log('iterate in gpu not implemented yet', this.parent, objects);
    }
  }, false, elation.physics.processor.base);
});
