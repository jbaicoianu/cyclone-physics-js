import { Vector3, Quaternion, Matrix4 } from 'three';
import { PhysicsSystem } from './system.js';
import { CycloneVector3, CycloneQuaternion } from './common.js';
import { forces } from './forces.js';
import { constraints } from './constraints.js';
import { colliders } from './collisions.js';
import * as elation from 'elation';

/**
 * Represents a physically simulated object
 * @extends EventTarget
 */

export class RigidBody extends EventTarget {
  /**
   * Create a new RigidBody
   * @param {object} args
   * @param {vector3} args.position - initial position
   * @param {quaternion} args.orientation - initial orientation
   * @param {vector3} args.scale - scaling factor
   * @param {vector3} args.velocity - initial velocity
   * @param {vector3} args.acceleration - initial acceleration
   * @param {vector3} args.angular - rotation speed, in radians
   * @param {vector3} args.angularacceleration - rotational acceleration, in radians
   * @param {float} args.mass - object mass
   * @param {float} args.linearDamping - damping factor for linear motion
   * @param {float} args.angularDamping - damping factor for rotational motion
   * @param {float} args.restitution - bounciness, or how much energy is preserved after each collision
   * @param {float} args.timescale - time scaling factor for simulation
   */
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

    for (let k in args) {
      if (!elation.isNull(args[k])) {
        this[k] = args[k];
      }
    }
    if (!this.id && this.object) this.id = this.object.objects['3d'].uuid;
    this.updateState();
  }

  /**
   * Called after any changes to update the object's internal state
   */
  updateState() {
    const epsilon = 1e-20;
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
    for (let i = 0, l = this.forces.length; i < l; i++) {
      this.state.forces = this.state.forces || (typeof this.forces[i].sleepstate == 'function' ? !this.forces[i].sleepstate() : true);
    }
    this.state.accelerating = (this.acceleration && this.acceleration.lengthSq() > epsilon);
    this.state.moving = (this.velocity && this.velocity.lengthSq() > epsilon);
    this.state.rotating = ((this.angular && this.angular.lengthSq() > epsilon) || (this.angularacceleration && this.angularacceleration.lengthSq() > epsilon));

    this.state.changed = this.hasChanged();

    this.state.sleeping = this.paused || !(this.state.forces || this.state.accelerating || this.state.moving || this.state.rotating);
    return this.state.sleeping;
  }

  /**
   * Reset force and torque accumulators
   */
  clearAccumulators() {
    this.force_accumulator.set(0,0,0);
    this.torque_accumulator.set(0,0,0);
  }

  /**
   * Update the acceleration from forces acting on this object
   */
  updateAcceleration(framedata) {
    this.lastacceleration.copy(this.acceleration);
    if (this.forces.length > 0) {
      this.clearAccumulators();
      for (let k in this.forces) {
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
  /**
   * Apply an instantaneous force to the center of this object
   * @param {vector3} force - force vector, in N
   * @param {boolean} relative - whether force is being applied relative to the object (defaults to world space)
   */
  applyForce(force, relative) {
    this._tmpvec.copy(force);
    if (relative) {
      this.localToWorldDir(this._tmpvec);
    }
    this.force_accumulator.add(this._tmpvec);
  }
  /**
   * Apply an instantaneous force to an offset point on this object
   * @param {vector3} force - force vector, in N
   * @param {vector3} point - force application point
   * @param {boolean} relative - whether force is being applied relative to the object (defaults to world space)
   */
  applyForceAtPoint(force, point, relative) {
    this.applyForce(force, relative);
    this.applyTorque(point.clone().cross(force));
  }
  /**
   * Apply a rotational force to this object
   * @param {vector3} torque - torque vector, in Nm
   */
  applyTorque(torque) {
    this.torque_accumulator.add(torque);
  }
  /**
   * Change the velocity of this object instantaneously
   * @param {vector3} vel - velocity, in meters per second
   */
  setVelocity(vel) {
    this.velocity.copy(vel);
    this.updateState();
  }
  /*
   * Add velocity to this object instantaneously
   * @param {vector3} vel - velocity, in meters per second
   */
  addVelocity(vel) {
    this.velocity.add(vel);
    this.updateState();
  }
  /**
   * Change the rotational speed of this object instantaneously
   * @param {vector3} vel - angular velocity, in radians per second
   */
  setAngularVelocity(vel) {
    this.angular.copy(vel);
    this.updateState();
  }
  /**
   * Add rotational velocity to this object instantaneously
   * @param {vector3} vel - angular velocity, in radians per second
   */
  addAngularVelocity(vel) {
    this.angular.add(vel);
    this.updateState();
  }
  /**
   * Create a new force generator acting on this object
   * @param {string} - force type
   * @param {object} - force arguments
   */
  addForce(type, args) {
    let force = false;
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
  /**
   * Remove the specified force from this object
   * @param {force|string} force - force object to remove, or string representing type of force to remove
   */
  removeForce(force) {
    let removes = [];
    if (typeof force == 'string') {
      for (let i = 0; i < this.forces.length; i++) {
        if (this.forces[i] instanceof forces[force]) {
          removes.unshift(i);
        }
      }
    } else {
      let idx = this.forces.indexOf(force);
      if (idx > -1) {
        removes.push(idx);
      }
    }
    if (removes.length > 0) {
      removes.sort();
      for (let i = removes.length; i > 0; --i) {
        let removedforces = this.forces.splice(i, 1);
        this.dispatchEvent(new CustomEvent('force_remove', {detail: removedForces[0]}));
      }
    }
  }
  /**
   * List all forces of the specified type that act on this object
   * @param {string} type
   */
  getForces(type) {
    let forces = [];
    for (let i = 0; i < this.forces.length; i++) {
      if (this.forces[i] instanceof forces[type]) {
        forces.push(this.forces[i]);
      }
    }
    return (forces.length > 0 ? forces : false);
  }
  /**
   * Add a new constraint to this object
   * @param {string} type - type of constraint to add
   * @param {object} args - constraint arguments
   */
  addConstraint(type, args) {
    let constraint = false;
    if (typeof constraints[type] == 'function') {
      constraint = new constraints[type](this, args);
      this.constraints.push(constraint);
      //this.updateConstraints();
      this.updateState();
      //console.log('added new constraint', constraint);
    } else {
      console.log('Unknown constraint type: ' + type);
    }
    return constraint;
  }
  /**
   * Remove the specified constraing from this object
   * @param {string|constraint} constraint - a constraint object, or a string with the type of constraint to remove
   */
  removeConstraint(constraint) {
    let removes = [];
    if (typeof constraint == 'string') {
      for (let i = 0; i < this.constraints.length; i++) {
        if (this.constraints[i] instanceof constraints[constraint]) {
          removes.unshift(i);
        }
      }
    } else {
      let idx = this.constraints.indexOf(constraint);
      if (idx > -1) {
        removes.push(idx);
      }
    }
    if (removes.length > 0) {
      removes.sort();
      for (let i = removes.length; i > 0; --i) {
        this.constraints.splice(i, 1);
      }
    }
  }
  /*
  updateMoment(shape, shapeargs) {
    switch (shape) {
      case 'box':
        let diff = shapeargs.max.clone().sub(shapeargs.min);
        let xsq = diff.x*diff.x,
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
        let c = 5 / (2 * this.mass * shapeargs.radius * shapeargs.radius);
        this.momentInverse.set(
          c, 0, 0, 0, 
          0, c, 0, 0, 
          0, 0, c, 0, 
          0, 0, 0, 1);
        break;
      case 'cylinder':
        // FIXME - axes are probably wrong
        let rsq = shapeargs.x * shapeargs.x,
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

  /**
   * Convert position vector from world space to local space (NOTE - vector is modified in place)
   * @param {vector3} point - world point to transform
   * @return {vector3} transformed point
   */
  worldToLocalPos = (function() {
    // closure for scratch variables
    const tmpquat = new Quaternion();
    return function(point) {
      if (!point) point = new CycloneVector3();
      if (this.parent) {
        point = this.parent.worldToLocalPos(point);
      }
      return this.parentToLocalPos(point).divide(this.scale);
    }
  })()
  /**
   * Convert position vector from local space to world space (NOTE - vector is modified in place)
   * @param {vector3} point - local point to transform
   * @return {vector3} transformed point
   */
  localToWorldPos(point) {
    point = this.localToParentPos(point);
    if (this.parent) {
      point = this.parent.localToWorldPos(point);
    }
    return point;
  }
  /**
   * Convert position vector from local space to parent space (NOTE - vector is modified in place)
   * @param {vector3} point - local point to transform
   * @return {vector3} transformed point
   */
  localToParentPos(point) {
    if (!point) point = new CycloneVector3();
    point.multiply(this.scale);
    return point.applyQuaternion(this.orientation).add(this.position);
  }
  /**
   * Convert position vector from parent space to local space (NOTE - vector is modified in place)
   * @param {vector3} point - parent point to transform
   * @return {vector3} transformed point
   */
  parentToLocalPos = (function() {
    // closure for scratch variables
    const tmpquat = new Quaternion();
    return function(point) {
      if (!point) point = new Vector3();
      return point.sub(this.position).applyQuaternion(tmpquat.copy(this.orientation).invert());
    }
  })()
  /**
   * Convert direction unit vector from world space to local space (NOTE - vector is modified in place)
   * @param {vector3} dir - world direction to transform
   * @return {vector3} transformed direction vector
   */
  worldToLocalDir = (function() {
    // temp variable closure
    const tmpquat = new Quaternion();
    return function(dir) {
      return dir.applyQuaternion(tmpquat.copy(this.orientationWorld).invert());
    }
  })()
  /**
   * Convert direction unit vector from local space to world space (NOTE - vector is modified in place)
   * @param {vector3} dir - local direction to transform
   * @return {vector3} transformed direction vector
   */
  localToWorldDir(dir) {
    return dir.applyQuaternion(this.orientationWorld);
  }
  /**
   * Convert direction unit vector from local space to parent space (NOTE - vector is modified in place)
   * @param {vector3} dir - local direction to transform
   * @return {vector3} transformed direction vector
   */
  localToParentDir(dir) {
    return dir.applyQuaternion(this.orientation);
  }
  /**
   * Convert scale vector from local space to world space (NOTE - vector is modified in place)
   * @param {vector3} scale - local scale to transform
   * @return {vector3} transformed scale vector
   */
   localToWorldScale(scale) {
    scale = this.localToParentScale(scale);
    if (this.parent) {
      scale = this.parent.localToWorldScale(scale);
    }
    return scale;
  }
  /**
   * Convert scale vector from world space to local space (NOTE - vector is modified in place)
   * @param {vector3} scale - world scale to transform
   * @return {vector3} transformed scale vector
   */
  worldToLocalScale(scale) {
    if (this.parent) {
      scale = this.parent.worldToLocalScale(scale);
    }
    scale = this.parentToLocalScale(scale);
    return scale;
  }
  /**
   * Convert scale vector from local space to parent space (NOTE - vector is modified in place)
   * @param {vector3} scale - local scale to transform
   * @return {vector3} transformed scale vector
   */
  localToParentScale(scale) {
    if (!scale) scale = new Vector3(1, 1, 1);
    return scale.multiply(this.scale);
  }
  /**
   * Convert scale vector from parent space to local space (NOTE - vector is modified in place)
   * @param {vector3} scale - parent scale to transform
   * @return {vector3} transformed scale vector
   */
  parentToLocalScale(scale) {
    if (!scale) scale = new Vector3(1, 1, 1);
    return scale.divide(this.scale);
  }

  /**
   * Determine whether this object has a chance of colliding with another object (rough sphere test)
   * @param {RigidBody} other - the body to check collisions with
   * @returns {boolean}
   */
  isPotentiallyColliding = (function() {
    // closure scratch vars
    const thispos = new Vector3(),
          otherpos = new Vector3(),
          diff = new Vector3();

    return function(other) {
      other.localToWorldPos(otherpos.set(0,0,0));
      this.localToWorldPos(thispos.set(0,0,0));
      diff.subVectors(otherpos, thispos);
      let radius = this.collider.radius + other.collider.radius;
      // FIXME - should account for velocity of both objects
      return (
        //other.object != this.object.parent &&
        //this.object != other.object.parent &&
        diff.lengthSq() <= radius * radius
       );
    }
  })
  /**
   * Determine exact points of contact between this object and another
   * @param {RigidBody} other - the body to check collisions with
   * @returns {boolean|array} - list of contacts, or false if none
   */
  getContacts(other, collisions, dt) {
    let hasContacts = false;
    if (this.collider && other.collider) {
      hasContacts = this.collider.getContacts(other.collider, collisions, dt);
    }
    return hasContacts;
  }
  /**
   * Specify a collider for this object
   * @param {string|object} type - string representing the type of this collider, or a collider object
   * @param {object} colliderargs - array of arguments for the specified collider
   */
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
  /**
   * Set the linear and angular damping factors for this object
   * @param {float} linear - linear damping factor
   * @param {float} angular - angular damping factor
   */
  setDamping(linear, angular) {
    if (typeof angular == 'undefined') angular = linear;
    this.setLinearDamping(linear);
    this.setAngularDamping(angular);
  }
  /**
   * Set the linear damping factor for this object
   * @param {float} linear - linear damping factor
   */
  setLinearDamping(linear) {
    this.linearDamping = linear;
  }
  /**
   * Set the angular damping factor for this object
   * @param {float} angular - angular damping factor
   */
  setAngularDamping(angular) {
    this.angularDamping = angular;
  }
  /**
   * Add an object as a child of this object
   * @param {RigidBody} body - object to add
   */
  add(body) {
    if (body.parent && body.parent !== this) {
      body.parent.remove(body);
    }
    let idx = this.children.indexOf(body);
    if (idx == -1) this.children.push(body);
    body.parent = this;
    this.position.changed = true; // Force hasChanged to be true
    this.dispatchEvent(new CustomEvent('add', {detail: body}));
  }
  /**
   * Remove a child object from this object
   * @param {RigidBody} body - object to remove
   */
  remove(body) {
    let idx = this.children.indexOf(body);
    if (idx != -1) {
      this.children.splice(idx,1);
      this.dispatchEvent(new CustomEvent('remove', {detail: body}));
      body.parent = undefined;
    }
  }
  /**
   * Process the constraints that are attached to this object
   * @param {array} contactlist - a list of contacts this object is involved in
   * @returns {boolean} - whether any constraints were applied
   */
  processConstraints(contactlist) {
    let wasConstrained = false;
    for (let i = 0; i < this.constraints.length; i++) {
      wasConstrained = wasConstrained || this.constraints[i].apply(contactlist);
    }
    return wasConstrained;
  }
  /**
   * Get the timescale for this object
   * @returns {float} time scale for physical simulation of this object
   */
  getTimescale() {
    let scale = this.timescale,
        p = this.parent;
    while (p) {
      scale *= p.timescale;
      p = p.parent;
    }
    return scale;
  }
  /**
   * Has this object's position, orientation, velocity, acceleration, angular, or angularacceleration changed?
   * @returns {boolean}
   */
  hasChanged() {
    return this.position.changed || this.orientation.changed || this.velocity.changed ||
           this.acceleration.changed ||this.angular.changed || this.angularacceleration.changed;
  }
  /**
   * Reset changed flags on all changeable properties
   */
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
  /**
   * Return a clone of the specified RigidBody (static version)
   * @param {RigidBody} other - object to clone, if called non-statically will return a clone of this object
   * @returns {RigidBody}
   */
  static clone(other) {
    return new RigidBody(other);
  }
  /**
   * Return a clone of this object
   * @returns {RigidBody}
   */
  static clone() {
    return new RigidBody(this);
  }
}
