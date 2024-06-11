import { Vector3, Quaternion, Matrix4 } from 'three';
import { PhysicsSystem } from './system.js';
import { CycloneVector3, CycloneQuaternion } from './common.js';
import { forces } from './forces.js';
import { colliders } from './collisions.js';
import * as elation from 'elation';

export class RigidBody extends EventTarget {
  constructor(args) {
    super();
    this.id = PhysicsSystem.uniqueid++;
    this.position = new CycloneVector3();
    this.positionWorld = new CycloneVector3();
    this.orientation = new CycloneQuaternion();
    this.orientationWorld = new CycloneQuaternion();
    this.scale = new Vector3(1, 1, 1);
    this.scaleWorld = new Vector3(1, 1, 1);
    this.velocity = new CycloneVector3();
    this.acceleration = new CycloneVector3();
    this.angular = new CycloneVector3();
    this.angularacceleration = new CycloneVector3();
    this.forces = [];
    this.constraints = [];
    this.mass = 0;
    this.gravity = false;
    this.state = {sleeping: true, accelerating: false, moving: false, rotating: false, colliding: false, changed: false};
    this.momentInverse = new Matrix4().identity();
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
    this.force_accumulator = new Vector3();
    this.torque_accumulator = new Vector3();
    this._tmpvec = new Vector3();
    this._tmpvec2 = new Vector3();
    this._tmpquat = new Quaternion();
    this.lastacceleration = new Vector3();

    for (var k in args) {
      if (!elation.isNull(args[k])) {
        this[k] = args[k];
      }
    }
    if (!this.id && this.object) this.id = this.object.objects['3d'].uuid;
    this.updateState();
  }
  updateState() {
    var epsilon = 1e-20;
    this.processConstraints();

    if (this.parent) {
      this.orientationWorld.multiplyQuaternions(this.parent.orientationWorld, this.orientation);
      this.positionWorld.copy(this.position).applyQuaternion(this._tmpquat.copy(this.parent.orientation).invert()).add(this.parent.positionWorld);
      this.scaleWorld.copy(this.scale).multiply(this.parent.scaleWorld);
    } else {
      this.orientationWorld.copy(this.orientation).invert();
      this.positionWorld.copy(this.position);
    }

    this.state.forces = false;
    for (var i = 0, l = this.forces.length; i < l; i++) {
      this.state.forces = this.state.forces || (typeof this.forces[i].sleepstate == 'function' ? !this.forces[i].sleepstate() : true);
    }
    this.state.accelerating = (this.acceleration && this.acceleration.lengthSq() > epsilon);
    this.state.moving = (this.velocity && this.velocity.lengthSq() > epsilon);
    this.state.rotating = ((this.angular && this.angular.lengthSq() > epsilon) || (this.angularacceleration && this.angularacceleration.lengthSq() > epsilon));

    this.state.changed = this.hasChanged();

    this.state.sleeping = this.paused || !(this.state.forces || this.state.accelerating || this.state.moving || this.state.rotating);
    return this.state.sleeping;
  }

  clearAccumulators() {
    this.force_accumulator.set(0,0,0);
    this.torque_accumulator.set(0,0,0);
  }
  updateAcceleration(framedata) {
    this.lastacceleration.copy(this.acceleration);
    if (this.forces.length > 0) {
      this.clearAccumulators();
      for (var k in this.forces) {
        this.forces[k].apply(framedata); // FIXME - electrostatic force is the only one which uses this, as a way to cache results across objects each frame. Should evaluate alternate ways of doing that
      }
      this.acceleration.copy(this.force_accumulator).divideScalar(this.mass);
      if (this.collider && this.collider.momentInverse) {
        this.angularacceleration.copy(this.torque_accumulator.applyMatrix4(this.collider.momentInverse));
      }
    }
    this.updateState();
    //console.log([this.acceleration.x, this.acceleration.y, this.acceleration.z], [this.angularacceleration.x, this.angularacceleration.y, this.angularacceleration.z]);
  }
  applyForce(force, relative) {
    this._tmpvec.copy(force);
    if (relative) {
      this.localToWorldDir(this._tmpvec);
    }
    this.force_accumulator.add(this._tmpvec);
  }
  applyForceAtPoint(force, point, relative) {
    this.applyForce(force, relative);
    this.applyTorque(point.clone().cross(force));
  }
  applyTorque(torque) {
    this.torque_accumulator.add(torque);
  }
  setVelocity(vel) {
    this.velocity.copy(vel);
    this.updateState();
  }
  addVelocity(vel) {
    this.velocity.add(vel);
    this.updateState();
  }
  setAngularVelocity(vel) {
    this.angular.copy(vel);
    this.updateState();
  }
  addAngularVelocity(vel) {
    this.angular.add(vel);
    this.updateState();
  }
  addForce(type, args) {
    var force = false;
    if (typeof forces[type] == 'function') {
      force = new forces[type](this, args);
      this.forces.push(force);
      this.updateAcceleration({});
      this.updateState();
      //console.log('added new force', force);
      this.dispatchEvent(new CustomEvent('force_add', {detail: force}));
    } else {
      console.log('Unknown force type: ' + type);
    }
    return force;
  }
  removeForce(force) {
    var removes = [];
    if (typeof force == 'string') {
      for (var i = 0; i < this.forces.length; i++) {
        if (this.forces[i] instanceof forces[force]) {
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
        this.dispatchEvent(new CustomEvent('force_remove', {detail: removedForces[0]}));
      }
    }
  }
  getForces(type) {
    var forces = [];
    for (var i = 0; i < this.forces.length; i++) {
      if (this.forces[i] instanceof forces[type]) {
        forces.push(this.forces[i]);
      }
    }
    return (forces.length > 0 ? forces : false);
  }
  updateForce(name, args) {
    /*
    if (this.forces[name]) {
      this.forces[name].update(args);
    }
    */
  }
  addConstraint(type, args) {
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
  removeConstraint(constraint) {
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
  updateMoment(shape, shapeargs) {
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
  worldToLocalPos = (function() {
    // closure for scratch variables
    var tmpquat = new Quaternion();
    return function(point) {
      if (!point) point = new CycloneVector3();
      if (this.parent) {
        point = this.parent.worldToLocalPos(point);
      }
      return this.parentToLocalPos(point).divide(this.scale);
    }
  })()
  // local space to world space
  localToWorldPos(point) {
    point = this.localToParentPos(point);
    if (this.parent) {
      point = this.parent.localToWorldPos(point);
    }
    return point;
  }
  // local space to parent space
  localToParentPos(point) {
    if (!point) point = new CycloneVector3();
    point.multiply(this.scale);
    return point.applyQuaternion(this.orientation).add(this.position);
  }
  // parent space to local space
  parentToLocalPos = (function() {
    // closure for scratch variables
    var tmpquat = new Quaternion();
    return function(point) {
      if (!point) point = new Vector3();
      return point.sub(this.position).applyQuaternion(tmpquat.copy(this.orientation).invert());
    }
  })()
  // world direction to local direction
  worldToLocalDir = (function() {
    // temp variable closure
    var tmpquat = new Quaternion();
    return function(dir) {
      return dir.applyQuaternion(tmpquat.copy(this.orientationWorld).invert());
    }
  })()
  // local direction to world direction
  localToWorldDir(dir) {
    return dir.applyQuaternion(this.orientationWorld);
  }
  // local direction to parent direction
  localToParentDir(dir) {
    return dir.applyQuaternion(this.orientation);
  }
  localToWorldScale(scale) {
    scale = this.localToParentScale(scale);
    if (this.parent) {
      scale = this.parent.localToWorldScale(scale);
    }
    return scale;
  }
  worldToLocalScale(scale) {
    if (this.parent) {
      scale = this.parent.worldToLocalScale(scale);
    }
    scale = this.parentToLocalScale(scale);
    return scale;
  }
  localToParentScale(scale) {
    if (!scale) scale = new Vector3(1, 1, 1);
    return scale.multiply(this.scale);
  }
  parentToLocalScale(scale) {
    if (!scale) scale = new Vector3(1, 1, 1);
    return scale.divide(this.scale);
  }

  isPotentiallyColliding = (function() {
    // closure scratch vars
    var thispos = new Vector3(),
        otherpos = new Vector3(),
        diff = new Vector3();

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
  })
  getContacts(other, collisions, dt) {
    var hasContacts = false;
    if (this.collider && other.collider) {
      hasContacts = this.collider.getContacts(other.collider, collisions, dt);
    }
    return hasContacts;
  }
  setCollider(type, colliderargs) {
    if (typeof type == 'object') {
      this.collider = type;
      this.collider.body = this;
    } else {
      if (typeof colliders[type] == 'function') {
        this.collider = new colliders[type](this, colliderargs);
      } else {
        console.log('Unknown collider type ' + type);
      }
    }
    this.collider.getInertialMoment();
    this.dispatchEvent(new CustomEvent('collider_change', {detail: this.collider}));
  }
  setDamping(linear, angular) {
    if (typeof angular == 'undefined') angular = linear;
    this.setLinearDamping(linear);
    this.setAngularDamping(angular);
  }
  setLinearDamping(linear) {
    this.linearDamping = linear;
  }
  setAngularDamping(angular) {
    this.angularDamping = angular;
  }
  add(body) {
    if (body.parent && body.parent !== this) {
      body.parent.remove(body);
    }
    var idx = this.children.indexOf(body);
    if (idx == -1) this.children.push(body);
    body.parent = this;
    this.position.changed = true; // Force hasChanged to be true
    this.dispatchEvent(new CustomEvent('add', {detail: body}));
  }
  remove(body) {
    var idx = this.children.indexOf(body);
    if (idx != -1) {
      this.children.splice(idx,1);
      this.dispatchEvent(new CustomEvent('remove', {detail: body}));
      body.parent = undefined;
    }
  }
  processConstraints(contactlist) {
    var wasConstrained = false;
    for (var i = 0; i < this.constraints.length; i++) {
      wasConstrained = wasConstrained || this.constraints[i].apply(contactlist);
    }
    return wasConstrained;
  }
  getTimescale() {
    var scale = this.timescale,
        p = this.parent;
    while (p) {
      scale *= p.timescale;
      p = p.parent;
    }
    return scale;
  }
  hasChanged() {
    return this.position.changed || this.orientation.changed || this.velocity.changed ||
           this.acceleration.changed ||this.angular.changed || this.angularacceleration.changed;
  }
  resetChangedFlag() {
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
  clone(other) {
    return new RigidBody(other);
  }
}
