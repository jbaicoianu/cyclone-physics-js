elation.extend("physics.rigidbody", function(args) {
  this.position = new THREE.Vector3();
  this.positionWorld = new THREE.Vector3();
  this.orientation = new THREE.Quaternion();
  this.orientationWorld = new THREE.Quaternion();
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

  this.parent = false;
  this.children = [];

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
    if (this.parent) {
      this.orientationWorld.multiplyQuaternions(this.parent.orientationWorld, this.orientation);
console.log(this.parent.orientation.toArray());
      this.positionWorld.copy(this.position).applyQuaternion(this.parent.orientation.clone().inverse()).add(this.parent.positionWorld);
    } else {
      this.orientationWorld.copy(this.orientation).inverse();
      this.positionWorld.copy(this.position);
    }

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
    if (this.forces.length > 0) {
      this.clearAccumulators();
      for (var k in this.forces) {
        this.forces[k].apply();
      }
      this.acceleration.copy(this.force_accumulator.divideScalar(this.mass));
      this.angularaccel.copy(this.torque_accumulator.applyMatrix4(this.momentInverse));
    }
    this.updateState();
    //console.log([this.acceleration.x, this.acceleration.y, this.acceleration.z], [this.angularaccel.x, this.angularaccel.y, this.angularaccel.z]);
  }
  this.applyForce = function(force, relative) {
    this._tmpvec.copy(force);
    if (relative) {
      this.localToWorldDir(this._tmpvec);
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
        var c = 5 / (2 * this.mass * shapeargs.radius * shapeargs.radius);
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
  this.localToWorldPos = function() {
    var tmpquat = new THREE.Quaternion();
    return function(point) {
      if (!point) point = new THREE.Vector3();
      point.add(this.position).applyQuaternion(tmpquat.copy(this.parent.orientation).inverse()).add(this.parent.positionWorld);
      return point;
    }
  }()
  this.worldToLocalPos = function(point) {
    if (!point) point = new THREE.Vector3();
    point.sub(this.parent.positionWorld).applyQuaternion(this.parent.orientation).sub(this.position);
    return point;
  }
  this.localToParentPos = function(pos) {
    if (this.parent) {
      
    }
  }
  this.localToWorldDir = function() {
    var tmpquat = new THREE.Quaternion();
    return function(dir) {
      tmpquat.copy(this.orientationWorld).inverse();
      return dir.applyQuaternion(tmpquat);
    }
  }();
  this.worldToLocalDir = function(dir) {
    return dir.applyQuaternion(this.orientationWorld);
  }
  this.isPotentiallyColliding = function(other) {
    this.localToWorldPos(this._tmpvec.set(0,0,0)).sub(other.localToWorldPos());
    return (
      //other.object != this.object.parent &&
      //this.object != other.object.parent &&
      this._tmpvec.lengthSq() <= Math.pow(this.collider.radius + other.collider.radius, 2)
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
  this.setDamping = function(linear, angular) {
    if (typeof angular == 'undefined') angular = linear;
    this.setLinearDamping(linear);
    this.setAngularDamping(angular);
  }
  this.setLinearDamping = function(linear) {
    this.linearDamping = linear;
  }
  this.setAngularDamping = function(angular) {
    this.angularDamping = angular;
  }
  this.add = function(body) {
    var idx = this.children.indexOf(body);
    if (idx == -1) this.children.push(body);
    body.parent = this;
  }
  this.remove = function(body) {
    var idx = this.children.indexOf(body);
    if (idx != -1) {
      this.children.splice(idx,1);
      body.parent = undefined;
    }
  }
  this.init();
});

