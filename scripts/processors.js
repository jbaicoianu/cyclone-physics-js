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
  this.collide = function() {
    var collisions = [];
    var potentials = [];
    // FIXME - Brute force for now.  We should use octrees or BVH here
    var objects = this.parent.getObjects();
    for (var i = 0; i < objects.length-1; i++) {
      var obj1 = objects[i];
      if (obj1.collider && obj1.collider.radius) {
        for (var j = i+1; j < objects.length; j++) {
          var obj2 = objects[j];
          if (obj2.collider && obj2.collider.radius && !(obj1.state.sleeping && obj2.state.sleeping) && obj1.isPotentiallyColliding(obj2)) {
            potentials.push([obj1, obj2, obj1.object.name, obj2.object.name]);
          }
        }
      }
    }
    if (potentials.length > 0) {
      for (var i = 0; i < potentials.length; i++) {
        var obj1 = potentials[i][0], obj2 = potentials[i][1];
        var contact = obj1.getContact(obj2, collisions);
      }
      //console.log(potentials.length + ' potential crashes:', potentials, collisions, this.parent.objects);
    }
    return collisions;
  }
  this.resolve = function(t, collisions) {
    for (var i = 0; i < collisions.length; i++) {
      var collision = collisions[i];
      collision.calculateInternals(t);
      collision.applyPositionChange();
      collision.applyVelocityChange();

      elation.events.fire({type: 'physics_collide', element: collision.bodies[0], data: collision});
      elation.events.fire({type: 'physics_collide', element: collision.bodies[1], data: collision});
    }
  }
});
elation.extend("physics.processor.cpu", function(parent) {
  elation.physics.processor.base.call(this, parent);
  this.iterate = function(objects, t) {
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
      elation.events.fire({type: "physics_update", element: objects[i], data: t});
    }
  }
  this.iterateAxis = function(obj, axis, t) {
    this._tmpvec.set(obj.position[axis], obj.velocity[axis], obj.acceleration[axis]);
    this._tmpvec.applyMatrix4(this.parent.physicsmatrix);
    obj.position[axis] = this._tmpvec.x;
    obj.velocity[axis] = this._tmpvec.y * Math.pow(obj.linearDamping, t);
  }
  this.iterateRotation = function(obj, t) {
    this._tmpvec.copy(obj.angularaccel).multiplyScalar(t);
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

