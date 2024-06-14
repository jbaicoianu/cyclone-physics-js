import { Vector3, Quaternion, Matrix4, Euler } from 'three';

/**
 * A stationary intersection between two bodies
 */
export class StaticContact {
  constructor(contactargs) {
    this.bodies = [];
    this.friction = contactargs.friction || 0;
    this.restitution = contactargs.restitution || 1;
    this.penetration = contactargs.penetration || 0;
    this.point = contactargs.point || new Vector3();
    this.normal = contactargs.normal || new Vector3(0,1,0);
    this.relativePositions = [];
    this.inertialMoments = [];
    this.impulses = [];
    this.contactToWorld = new Matrix4();
    this.worldToContact = new Matrix4();
    this.initialized = false;

    this._tmpmat = new Matrix4();

    if (contactargs.bodies) this.bodies = contactargs.bodies;
    if (contactargs.object1) this.bodies.push(contactargs.object1);
    if (contactargs.object2) this.bodies.push(contactargs.object2);
  }
  /**
   * Resolve a collision using default physical simulation
   */
  resolve(t, a, b) {
    this.restitution = this.bodies[0].restitution * this.bodies[1].restitution;
    if (!this.initialized) {
      this.calculateInternals(t);
      this.initialized = true;
    }
    // Fire events for both objects, and combine them into one array
    let ev1 = new CustomEvent('physics_collide', { detail: this, cancelable: true }),
        ev2 = new CustomEvent('physics_collide', { detail: this, cancelable: true });

    this.bodies[0].dispatchEvent(ev1),
    this.bodies[1].dispatchEvent(ev2);

    if (!(ev1.defaultPrevented || ev2.defaultPrevented)) {
      // If no event handlers handled this event, use our default collision response
      this.applyPositionChange(t, a, b);
      this.applyVelocityChange(t, a, b);
      this.finalizeMovement(t, a, b);
      let resolvedev1 = new CustomEvent('physics_collision_resolved', { detail: this }),
          resolvedev2 = new CustomEvent('physics_collision_resolved', { detail: this });
      this.bodies[0].dispatchEvent(resolvedev1);
      this.bodies[1].dispatchEvent(resolvedev2);
    }
  }
  /**
   * Generate a transform matrix which represents the collision's local coordinate space
   */
  calculateContactMatrix = (function() {
    // Closure for scratch variables
    var c1 = new Vector3(),
        c2 = new Vector3(),
        binormal = new Vector3();
  
    return function() {
      // Check whether the Z-axis is nearer to the X or Y axis
      this.normal.normalize();
      //var c1 = this.normal.clone().cross(new Vector3(0,0,1));
      //var c2 = this.normal.clone().cross(new Vector3(0,1,0));
      var normal = this.normal;
      c1.set(0,0,1).cross(this.normal);
      c2.set(0,1,0).cross(this.normal);
      var tangent = (c1.lengthSq() > c2.lengthSq() ? c1 : c2);
      tangent.normalize().negate();
      binormal.copy(tangent).cross(this.normal);

      this.contactToWorld = new Matrix4().identity();
      if (tangent.lengthSq() > 0) {
        this.contactToWorld.set(
          tangent.x, this.normal.x, binormal.x, 0,
          tangent.y, this.normal.y, binormal.y, 0,
          tangent.z, this.normal.z, binormal.z, 0,
          0, 0, 0, 1
        );
      }
      this.worldToContact = new Matrix4().copy(this.contactToWorld).invert();
    }
  })();
  /**
   * Calculate velocity relative to contact point, taking into account angular velocity
   */
  calculateLocalVelocity(index, duration) {
    // TODO - optimize local scratch variables
    var velocity = new Vector3();
    var accvel= new Vector3();
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
  calculateDesiredDeltaVelocity(duration) {
    // TODO - optimize local scratch variables
    var velocityFromAccel = 0;
    var lastaccel = new Vector3();

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
  calculateInternals(duration) {
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
  applyVelocityChange = (function() {
    // closure scratch variables
    var impulsiveForce = new Vector3();
    var impulsiveTorque = new Vector3();

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
  })()

  calculateFrictionlessImpulse = (function() {
    // closure scratch variables
    var deltaVelWorld = new Vector3();

    return function() {
      var impulse = new Vector3();
      
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
  })();

  calculateFrictionImpulse = (function() {
    return function() {
      var impulse = new Vector3();
    }
  })();

  applyPositionChange = (function() {
    // closure scratch variables
    var angularInertiaWorld = new Vector3();
    var projection = new Vector3();
    var angularLimit = 0.2;
    var euler = new Euler();
    var quat = new Quaternion();

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
  })();
  finalizeMovement(duration, linearMomentum, angularMomentum) {
  }
}

/**
 * A moving collision between two bodies
 */
export class DynamicContact extends StaticContact {
  constructor(contactargs) {
    super(contactargs);
    this.penetrationTime = contactargs.penetrationTime || 0;
    this.collisionVelocities = [];

    if (this.bodies[0]) this.collisionVelocities[0] = this.bodies[0].velocity.clone();
    if (this.bodies[1]) this.collisionVelocities[1] = this.bodies[1].velocity.clone();
  }
  /**
   * Resolve a collision using default physical simulation
   */
  resolve(t, a, b) {
    this.restitution = this.bodies[0].restitution * this.bodies[1].restitution;
    if (!this.initialized) {
      this.calculateInternals(t);
      this.initialized = true;
    }
    // Move the object to its exact collision point
    this.applyPositionChange(t, a, b);

    // Fire events for both objects
    let ev1 = new CustomEvent('physics_collide', { detail: this, cancelable: true }),
        ev2 = new CustomEvent('physics_collide', { detail: this, cancelable: true });

    this.bodies[0].dispatchEvent(ev1);
    this.bodies[1].dispatchEvent(ev2);

    if (!(ev1.defaultPrevented || ev2.defaultPrevented)) {
      // If no event handlers handled this event, use our default collision response
      this.applyVelocityChange(t, a, b);
      this.finalizeMovement(t, a, b);
      let resolvedev1 = new CustomEvent('physics_collision_resolved', { detail: this }),
          resolvedev2 = new CustomEvent('physics_collision_resolved', { detail: this });
      this.bodies[0].dispatchEvent(resolvedev1);
      this.bodies[1].dispatchEvent(resolvedev2);
    }
  }
  calculateInternals = (function() {
    let scaledVelocity = new Vector3();
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
  applyPositionChange = (function() {
    // closure scratch variables
    let angularInertiaWorld = new Vector3();
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
  finalizeMovement = (function() {
    let scaledVelocity = new Vector3();
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
}

