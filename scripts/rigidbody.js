elation.require(['physics.common'], function() {
  elation.extend("physics.rigidbody", function(args) {
    this.id = elation.physics.uniqueid++;
    this.position = new elation.physics.vector3();
    this.positionWorld = new elation.physics.vector3();
    this.orientation = new elation.physics.quaternion();
    this.orientationWorld = new elation.physics.quaternion();
    this.scale = new THREE.Vector3(1, 1, 1);
    this.velocity = new elation.physics.vector3();
    this.acceleration = new elation.physics.vector3();
    this.angular = new elation.physics.vector3();
    this.angularacceleration = new elation.physics.vector3();
    this.forces = [];
    this.constraints = [];
    this.mass = 0;
    this.gravity = false;
    this.state = {sleeping: true, accelerating: false, moving: false, rotating: false, colliding: false, changed: false};
    this.momentInverse = new THREE.Matrix4().identity();
    this.linearDamping = 1;
    this.angularDamping = 1;
    this.restitution = 1;
    this.timescale = 1;
    this.paused = false;
    this.material = {
      dynamicfriction: 0,
      staticfriction: 0,
      bounciness: 0,
    };

    this.parent = false;
    this.children = [];

    // Accumulation buffers for linear and rotational acceleration
    this.force_accumulator = new THREE.Vector3();
    this.torque_accumulator = new THREE.Vector3();
    this._tmpvec = new THREE.Vector3();
    this._tmpvec2 = new THREE.Vector3();
    this._tmpquat = new THREE.Quaternion();
    this.lastacceleration = new THREE.Vector3();

    this.init = function() {
      for (var k in args) {
        if (!elation.utils.isNull(args[k])) {
          this[k] = args[k];
        }
      }
      if (!this.id && this.object) this.id = this.object.objects['3d'].uuid;
      this.updateState();
    }
    this.updateState = function() {
      var lambda = 1e-20;
      this.processConstraints();

      if (this.parent) {
        this.orientationWorld.multiplyQuaternions(this.parent.orientationWorld, this.orientation);
        this.positionWorld.copy(this.position).applyQuaternion(this._tmpquat.copy(this.parent.orientation).invert()).add(this.parent.positionWorld);
      } else {
        this.orientationWorld.copy(this.orientation).invert();
        this.positionWorld.copy(this.position);
      }

      this.state.forces = false;
      for (var i = 0, l = this.forces.length; i < l; i++) {
        this.state.forces = this.state.forces || (typeof this.forces[i].sleepstate == 'function' ? !this.forces[i].sleepstate() : true);
      }
      this.state.accelerating = (this.acceleration && this.acceleration.lengthSq() > lambda);
      this.state.moving = (this.velocity && this.velocity.lengthSq() > lambda);
      this.state.rotating = ((this.angular && this.angular.lengthSq() > lambda) || (this.angularacceleration && this.angularacceleration.lengthSq() > lambda));

      this.state.changed = this.hasChanged();

      this.state.sleeping = this.paused || !(this.state.forces || this.state.accelerating || this.state.moving || this.state.rotating);
      return this.state.sleeping;
    }

    this.clearAccumulators = function() {
      this.force_accumulator.set(0,0,0);
      this.torque_accumulator.set(0,0,0);
    }
    this.updateAcceleration = function(framedata) {
      this.lastacceleration.copy(this.acceleration);
      if (this.forces.length > 0) {
        this.clearAccumulators();
        for (var k in this.forces) {
          this.forces[k].apply(framedata);
        }
        this.acceleration.copy(this.force_accumulator.divideScalar(this.mass));
        if (this.collider && this.collider.momentInverse) {
          this.angularacceleration.copy(this.torque_accumulator.applyMatrix4(this.collider.momentInverse));
        }
      }
      this.updateState();
      //console.log([this.acceleration.x, this.acceleration.y, this.acceleration.z], [this.angularacceleration.x, this.angularacceleration.y, this.angularacceleration.z]);
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
        this.updateAcceleration({});
        this.updateState();
        //console.log('added new force', force);
        elation.events.fire({type: 'force_add', element: this, data: force});
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
          let removedforces = this.forces.splice(i, 1);
          elation.events.fire({type: 'force_remove', element: this, data: removedforces[0]});
        }
      }
    }
    this.getForces = function(type) {
      var forces = [];
      for (var i = 0; i < this.forces.length; i++) {
        if (this.forces[i] instanceof elation.physics.forces[type]) {
          forces.push(this.forces[i]);
        }
      }
      return (forces.length > 0 ? forces : false);
    }
    this.updateForce = function(name, args) {
      /*
      if (this.forces[name]) {
        this.forces[name].update(args);
      }
      */
    }
    this.addConstraint = function(type, args) {
      var constraint = false;
      if (typeof elation.physics.constraints[type] == 'function') {
        constraint = new elation.physics.constraints[type](this, args);
        this.constraints.push(constraint);
        //this.updateConstraints();
        this.updateState();
        //console.log('added new constraint', constraint);
      } else {
        console.log('Unknown constraint type: ' + type);
      }
      return constraint;
    }
    this.removeConstraint = function(constraint) {
      var removes = [];
      if (typeof constraint == 'string') {
        for (var i = 0; i < this.constraints.length; i++) {
          if (this.constraints[i] instanceof elation.physics.constraints[constraint]) {
            removes.unshift(i);
          }
        }
      } else {
        var idx = this.constraints.indexOf(constraint);
        if (idx > -1) {
          removes.push(idx);
        }
      }
      if (removes.length > 0) {
        removes.sort();
        for (var i = removes.length; i > 0; --i) {
          this.constraints.splice(i, 1);
        }
      }
    }
    /*
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
        case 'matrix':
          this.momentInverse.copy(shapeargs);
          break;
        default:
          console.log('Unimplemented inertia moment tensor: ' + shape);
      }
    }
    */

    // Coordinate space transforms

    // world space to local space
    this.worldToLocalPos = function() {
      // closure for scratch variables
      var tmpquat = new THREE.Quaternion();
      return function(point) {
        if (!point) point = new elation.physics.vector3();
        if (this.parent) {
          point = this.parent.worldToLocalPos(point);
        }
        return this.parentToLocalPos(point).divide(this.scale);
      }
    }();
    // local space to world space
    this.localToWorldPos = function(point) {
      point = this.localToParentPos(point);
      if (this.parent) {
        point = this.parent.localToWorldPos(point);
      }
      return point;
    }
    // local space to parent space
    this.localToParentPos = function(point) {
      if (!point) point = new elation.physics.vector3();
      point.multiply(this.scale);
      return point.applyQuaternion(this.orientation).add(this.position);
    }
    // parent space to local space
    this.parentToLocalPos = function() {
      // closure for scratch variables
      var tmpquat = new THREE.Quaternion();
      return function(point) {
        if (!point) point = new THREE.Vector3();
        return point.sub(this.position).applyQuaternion(tmpquat.copy(this.orientation).invert());
      }
    }();
    // world direction to local direction
    this.worldToLocalDir = function() {
      // temp variable closure
      var tmpquat = new THREE.Quaternion();
      return function(dir) {
        return dir.applyQuaternion(tmpquat.copy(this.orientationWorld).invert());
      }
    }();
    // local direction to world direction
    this.localToWorldDir = function(dir) {
      return dir.applyQuaternion(this.orientationWorld);
    }
    // local direction to parent direction
    this.localToParentDir = function(dir) {
      return dir.applyQuaternion(this.orientation);
    }
    this.localToWorldScale = function(scale) {
      scale = this.localToParentScale(scale);
      if (this.parent) {
        scale = this.parent.localToWorldScale(scale);
      }
      return scale;
    }
    this.worldToLocalScale = function(scale) {
      if (this.parent) {
        scale = this.parent.worldToLocalScale(scale);
      }
      scale = this.parentToLocalScale(scale);
      return scale;
    }
    this.localToParentScale = function(scale) {
      if (!scale) scale = new THREE.Vector3(1, 1, 1);
      return scale.multiply(this.scale);
    }
    this.parentToLocalScale = function(scale) {
      if (!scale) scale = new THREE.Vector3(1, 1, 1);
      return scale.divide(this.scale);
    }

    this.isPotentiallyColliding = function() {
      // closure scratch vars
      var thispos = new THREE.Vector3(),
          otherpos = new THREE.Vector3(),
          diff = new THREE.Vector3();

      return function(other) {
        other.localToWorldPos(otherpos.set(0,0,0));
        this.localToWorldPos(thispos.set(0,0,0));
        diff.subVectors(otherpos, thispos);
        var radius = this.collider.radius + other.collider.radius;
        return (
          //other.object != this.object.parent &&
          //this.object != other.object.parent &&
          diff.lengthSq() <= radius * radius
         );
      }
    }();
    this.getContacts = function(other, collisions, dt) {
      var hasContacts = false;
      if (this.collider && other.collider) {
        hasContacts = this.collider.getContacts(other.collider, collisions, dt);
      }
      return hasContacts;
    }
    this.setCollider = function(type, colliderargs) {
      if (typeof type == 'object') {
        this.collider = type;
        this.collider.body = this;
      } else {
        if (typeof elation.physics.colliders[type] == 'function') {
          this.collider = new elation.physics.colliders[type](this, colliderargs);
        } else {
          console.log('Unknown collider type ' + type);
        }
      }
      this.collider.getInertialMoment();
      elation.events.fire({type: 'collider_change', element: this, data: this.collider});
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
      if (body.parent && body.parent !== this) {
        body.parent.remove(body);
      }
      var idx = this.children.indexOf(body);
      if (idx == -1) this.children.push(body);
      body.parent = this;
      this.position.changed = true; // Force hasChanged to be true
      elation.events.fire({type: 'add', element: this, data: body});
    }
    this.remove = function(body) {
      var idx = this.children.indexOf(body);
      if (idx != -1) {
        this.children.splice(idx,1);
        elation.events.fire({type: 'remove', element: this, data: body});
        body.parent = undefined;
      }
    }
    this.processConstraints = function(contactlist) {
      var wasConstrained = false;
      for (var i = 0; i < this.constraints.length; i++) {
        wasConstrained = wasConstrained || this.constraints[i].apply(contactlist);
      }
      return wasConstrained;
    }
    this.getTimescale = function() {
      var scale = this.timescale,
          p = this.parent;
      while (p) {
        scale *= p.timescale;
        p = p.parent;
      }
      return scale;
    }
    this.hasChanged = function() {
      return this.position.changed || this.orientation.changed || this.velocity.changed ||
             this.acceleration.changed ||this.angular.changed || this.angularacceleration.changed;
    }
    this.resetChangedFlag = function() {
      this.position.reset();
      this.positionWorld.reset();
      this.orientation.reset();
      //this.orientationWorld.reset();
      if (this.velocity.reset) this.velocity.reset();
      this.acceleration.reset();
      if (this.scale.reset) this.scale.reset();
      this.angular.reset();
      this.angularacceleration.reset();
    }
    this.init();
  });
});
