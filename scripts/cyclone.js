elation.extend("physics.system", new function() {
  this.physicsmatrix = new THREE.Matrix4(1, 1, .5, 0, 0, 1, 1, 0, 0, 0, 1, 0);
  this.active = false;
  this.objects = [];
  this.processor = false;

  this.init = function() {
  }
  this.start = function() {
    this.active = true;
    if (!this.processor) {
      this.processor = new elation.physics.processor.cpu(this);
    }
  }
  this.step = function(t) {
    // If there are no objects we have nothing to do
    if (!this.active || this.objects.length == 0) return;
    
    // update matrix with new time values
    this.physicsmatrix.n12 = this.physicsmatrix.n23 = t;
    this.physicsmatrix.n13 = .5 * t * t;

    // step 1: update forces for each object, gather array of active objects
    var objects = this.processor.update(t);
    if (objects.length > 0) {
      // step 2: run physics simulation on all active objects
      this.processor.iterate(objects, t);

      // step 3: detect contacts
      var collisions = this.processor.collide(t)
      if (collisions.length > 0) {
        // step 4: resolve collisions
        this.processor.resolve(t, collisions);
      }
    }
  }
  this.add = function(obj) {
    this.objects.push(obj);
  }
  this.remove = function(obj) {
    var i = this.objects.indexOf(obj);
    if (i != -1) {
      this.objects.splice(i, 1);
    }
  }
});
elation.extend("physics.rigidbody", function(args) {
  this.position = new THREE.Vector3();
  this.orientation = new THREE.Quaternion();
  this.velocity = new THREE.Vector3();
  this.angular = new THREE.Vector3();
  this.forces = [];
  this.mass = 0;
  this.state = {sleeping: true, accelerating: false, moving: false, rotating: false};
  this.momentInverse = new THREE.Matrix4().identity();
  this.linearDamping = 1;
  this.angularDamping = 1;
  this.restitution = 1;
  this.paused = false;

  // Accumulation buffers for linear and rotational acceleration
  this.acceleration = new THREE.Vector3();
  this.angularaccel = new THREE.Vector3();
  this.force_accumulator = new THREE.Vector3();
  this.torque_accumulator = new THREE.Vector3();
  this._tmpvec = new THREE.Vector3();
  this._tmpvec2 = new THREE.Vector3();
  this.lastacceleration = new THREE.Vector3();

  this.init = function() {
    for (var k in args) {
      if (!elation.utils.isNull(args[k])) {
        this[k] = args[k];
      }
    }
    this.updateState();
  }
  this.updateState = function() {
    var lambda = 0.00001;
    this.state.forces = (this.forces.length > 0);
    this.state.accelerating = (this.acceleration && this.acceleration.lengthSq() > lambda);
    this.state.moving = (this.velocity && this.velocity.lengthSq() > lambda);
    this.state.rotating = ((this.angular && this.angular.lengthSq() > lambda) || (this.angularaccel && this.angularaccel.lengthSq() > lambda));

    this.state.sleeping = this.paused || !(this.state.forces || this.state.accelerating || this.state.moving || this.state.rotating);
    return this.state.sleeping;
  }

  this.clearAccumulators = function() {
    this.force_accumulator.set(0,0,0);
    this.torque_accumulator.set(0,0,0);
  }
  this.updateAcceleration = function() {
    this.lastacceleration.copy(this.acceleration);
    this.clearAccumulators();
    for (var k in this.forces) {
      this.forces[k].apply();
    }
    this.acceleration.copy(this.force_accumulator.divideScalar(this.mass));
    this.angularaccel.copy(this.torque_accumulator.applyMatrix4(this.momentInverse));
    this.updateState();
    //console.log([this.acceleration.x, this.acceleration.y, this.acceleration.z], [this.angularaccel.x, this.angularaccel.y, this.angularaccel.z]);
  }
  this.applyForce = function(force, relative) {
    this._tmpvec.copy(force);
    if (relative) {
      this._tmpvec2.getPositionFromMatrix(this.object.objects['3d'].matrixWorld);
      this._tmpvec.applyMatrix4(this.object.objects['3d'].matrixWorld).sub(this._tmpvec2);
    }
    this.force_accumulator.add(this._tmpvec);
  }
  this.applyForceAtPoint = function(force, point, relative) {
    this.applyForce(force, relative);
    this.applyTorque(point.clone().cross(force));
  }
  this.applyTorque = function(torque) {
    this.torque_accumulator.add(torque);
  }
  this.setVelocity = function(vel) {
    this.velocity.copy(vel);
    this.updateState();
  }
  this.addVelocity = function(vel) {
    this.velocity.add(vel);
    this.updateState();
  }
  this.setAngularVelocity = function(vel) {
    this.angular.copy(vel);
    this.updateState();
  }
  this.addAngularVelocity = function(vel) {
    this.angular.add(vel);
    this.updateState();
  }
  this.addForce = function(type, args) {
    var force = false;
    if (typeof elation.physics.forces[type] == 'function') {
      force = new elation.physics.forces[type](this, args);
      this.forces.push(force);
      this.updateAcceleration();
      this.updateState();
      console.log('added new force', force);
    } else {
      console.log('Unknown force type: ' + type);
    }
    return force;
  }
  this.removeForce = function(force) {
    var removes = [];
    if (typeof force == 'string') {
      for (var i = 0; i < this.forces.length; i++) {
        if (this.forces[i] instanceof elation.physics.forces[force]) {
          removes.unshift(i);
        }
      }
    } else {
      var idx = this.forces.indexOf(force);
      if (idx > -1) {
        removes.push(idx);
      }
    }
    if (removes.length > 0) {
      removes.sort();
      for (var i = removes.length; i > 0; --i) {
        this.forces.splice(i, 1);
      }
    }
  }
  this.updateForce = function(name, args) {
/*
    if (this.forces[name]) {
      this.forces[name].update(args);
    }
*/
  }
  this.updateMoment = function(shape, shapeargs) {
    switch (shape) {
      case 'box':
        var diff = shapeargs.max.clone().sub(shapeargs.min);
        var xsq = diff.x*diff.x,
            ysq = diff.y*diff.y,
            zsq = diff.z*diff.z,
            m = 1/12 * this.mass;
        this.momentInverse.set(
          1 / (m * (ysq + zsq)), 0, 0, 0, 
          0, 1 / (m * (xsq + zsq)), 0, 0, 
          0, 0, 1 / (m * (xsq + ysq)), 0, 
          0, 0, 0, 1);
        break;
      case 'sphere':
        var c = 5 / (2 * this.mass * this.radius * this.radius);
        this.momentInverse.set(
          c, 0, 0, 0, 
          0, c, 0, 0, 
          0, 0, c, 0, 
          0, 0, 0, 1);
        break;
      case 'cylinder':
        // FIXME - axes are probably wrong
        var rsq = shapeargs.x * shapeargs.x,
            hsq = shapeargs.y * shapeargs.y,
            xy = 1/(1/12 * m * hsq + 1/4 * m * rsq),
            z = 1/(1/2 * m * rsq);
        this.momentInverse.set(
          xy, 0, 0, 0, 
          0, xy, 0, 0, 
          0, 0, z, 0, 
          0, 0, 0, 1);
        break;
      default:
        console.log('Unimplemented inertia moment tensor: ' + shape);
    }
   
  }
  this.getWorldPosition = function(point) {
/*
    var worldpoint = this.position.clone();
    this.object.matrixWorld.multiplyVector3(worldpoint);
    this.object.matrixWorld.multiplyVector3(worldpoint);
    return worldpoint;
*/
    return this.object.localToWorld(point);
  }
  this.getDirectionWorld = function(dir) {
    var worlddir = this.getWorldPosition(dir);
    worlddir.sub(this._tmpvec2.getPositionFromMatrix(this.object.objects['3d'].matrixWorld));
    return worlddir;
  }
  this.getDirectionLocal = function(dir) {
    var worlddir = dir.clone();
    worlddir.add(this._tmpvec2.getPositionFromMatrix(this.object.objects['3d'].matrixWorld))
    var inv = new THREE.Matrix4();
    inv.getInverse(this.object.matrixWorld);
    worlddir.applyMatrix4(inv);
    return worlddir;
  }
  this.isPotentiallyColliding = function(other) {
    this._tmpvec.copy(this.object.matrixWorld.getPosition()).sub(other.object.matrixWorld.getPosition());
    return (
      //other.object != this.object.parent &&
      //this.object != other.object.parent &&
      this._tmpvec.lengthSq() <= Math.pow(this.radius + other.radius, 2)
     );
  }
  this.getContact = function(other, collisions) {
    if (this.collider && other.collider) {
      return this.collider.getContact(other.collider, collisions);
    }
    return false;
  }
  this.setCollider = function(type, colliderargs) {
    if (typeof elation.physics.colliders[type] == 'function') {
      this.collider = new elation.physics.colliders[type](this, colliderargs);
    } else {
      console.log('Unknown collider type ' + type);
    }
    this.updateMoment(type, colliderargs);
  }
  this.init();
});
function VECDUMP(v) {
  return '[' + v.x.toFixed(4) + ', ' + v.y.toFixed(4) + ', ' + v.z.toFixed(4) + ']';
}
function MATDUMP(m) {
  var ret = '';
  for (var i = 0; i < 3; i++) {
    ret += VECDUMP({x: m[i*3], y: m[i*3+1], z: m[i*3+2]}) + '\n';
  }
  return ret;
}
