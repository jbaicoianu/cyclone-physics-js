elation.extend("physics.processor.base", function(parent) {
  this.parent = parent;
  this._tmpvec = new THREE.Vector3();
  this._tmpquat = new THREE.Quaternion();
  this._tmpmat = new THREE.Matrix4();

  this.update = function(t) {
    active = [];
    for (var i = 0; i < this.parent.objects.length; i++) {
      var obj = this.parent.objects[i];
      if (!obj.state.sleeping) {
        active.push(obj);
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
    for (var i = 0; i < this.parent.objects.length; i++) {
      var obj1 = this.parent.objects[i];
      if (obj1.radius) {
        for (var j = i+1; j < this.parent.objects.length; j++) {
          var obj2 = this.parent.objects[j];
          if (obj2.radius && !(obj1.state.sleeping && obj2.state.sleeping) && obj1.isPotentiallyColliding(obj2)) {
            potentials.push([obj1, obj2, obj1.object.name, obj2.object.name]);
          }
        }
      }
    }
    if (potentials.length > 0) {
      //console.log(potentials.length + ' potential crashes:', potentials);
      for (var i = 0; i < potentials.length; i++) {
        var obj1 = potentials[i][0], obj2 = potentials[i][1];
        var contact = obj1.getContact(obj2, collisions);
/*
        if (contact) {
          if (elation.utils.isArray(contact)) {
            collisions = collisions.concat(contact);
          } else {
            collisions.push(contact);
          }
        }
*/
      }
    }
    return collisions;
  }
  this.resolve = function(t, collisions) {
    for (var i = 0; i < collisions.length; i++) {
      var collision = collisions[i];
      collision.calculateInternals(t);
      collision.applyPositionChange();
      collision.applyVelocityChange();

      elation.events.fire({type: 'collide', target: collision.bodies[0], element: collision.bodies[0], data: collision});
    }
  }
});
elation.extend("physics.processor.cpu", function(parent) {
  elation.physics.processor.base.call(this, parent);
  this.iterate = function(objects, t) {
    for (var i = 0; i < objects.length; i++) {
      objects[i].updateAcceleration();
      if (objects[i].state.accelerating || objects[i].state.moving) {
        this.iterateAxis(objects[i], 'x');
        this.iterateAxis(objects[i], 'y');
        this.iterateAxis(objects[i], 'z');
      }
      if (objects[i].state.rotating) {
        this.iterateRotation(objects[i], t);
      }
      objects[i].updateState();
      elation.events.fire({type: "dynamicsupdate", element: objects[i], data: t});
    }
  }
  this.iterateAxis = function(obj, axis) {
    this._tmpvec.set(obj.position[axis], obj.velocity[axis], obj.acceleration[axis]);
    this._tmpvec.applyMatrix4(this.parent.physicsmatrix);
    obj.position[axis] = this._tmpvec.x;
    obj.velocity[axis] = this._tmpvec.y;
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

