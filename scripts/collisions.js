/**
 * colliders 
 */

elation.extend("physics.colliders.helperfuncs", new function() {
  this._tmpvec = new THREE.Vector3();
  var _tmpvec2 = new THREE.Vector3();
  var _tmpvec3 = new THREE.Vector3();

  this.sphere_sphere = function(obj1, obj2, collisions) {
    var contact = false;
    var thispos = obj1.body.localToWorldPos(_tmpvec2.set(0,0,0)),
        otherpos = obj2.body.localToWorldPos(_tmpvec3.set(0,0,0)),
        midline = this._tmpvec.subVectors(otherpos, thispos),
        size = midline.length();
    midline.divideScalar(size);
    if (!(size <= 0 || size >= obj1.radius + obj2.radius)) {
      contact = new elation.physics.contact({
        normal: midline.clone(),
        point: thispos.clone().add(midline.multiplyScalar(obj1.radius)),
        penetration: obj1.radius + obj2.radius - size,
        object1: obj1.body,
        object2: obj2.body,
      });
      collisions.push(contact);
    }
    return contact;
  }
  this.sphere_plane = function(sphere, plane, collisions) {
    var contact = false;
    var pos = sphere.body.localToWorldPos(this._tmpvec.set(0,0,0));
    var norm = plane.body.localToWorldDir(plane.normal.clone());
    var distance = norm.dot(pos) - sphere.radius - plane.offset;
    if (distance < 0) {
      var sepspeed = sphere.body.velocity.dot(plane.normal);
      if (sepspeed <= 0) {
        //console.log('crash a sphere-plane!', sphere, plane, distance);
        collisions.push(new elation.physics.contact({
          normal: norm,
          point: pos.clone().sub(norm.clone().multiplyScalar(distance + sphere.radius)), 
          penetration: -distance,
          object1: sphere.body,
          object2: plane.body,
        }));
      }
    }
    return contact;
  }
  this.box_plane = function(box, plane, collisions) {
    var contacts = [];
    var contact = false;
    
    var min = box.min, max = box.max;
    var vertices = [
      [min.x, min.y, min.z],
      [min.x, min.y, max.z],
      [min.x, max.y, min.z],
      [min.x, max.y, max.z],

      [max.x, min.y, min.z],
      [max.x, min.y, max.z],
      [max.x, max.y, min.z],
      [max.x, max.y, max.z],
    ];
    var max = 0;
    for (var i = 0; i < vertices.length; i++) {
      var worldpos = box.body.localToWorldPos(this._tmpvec.set(vertices[i][0], vertices[i][1], vertices[i][2]));
      var vcollide = this.vertex_plane(worldpos, plane);
      if (vcollide) {
        vcollide.object1 = box.body;
        contacts.push(vcollide);
        if (vcollide.penetration > max) {
          max = vcollide.penetration;
          contact = vcollide;
        }
      }
    }
    if (contact) {
      collisions.push(new elation.physics.contact(contact));
    }

    return contact;
    //return (contacts.length > 0 ? contacts : false);
  }
  this.vertex_plane = function(vertex, plane) {
    var contact = false;
    var distance = vertex.dot(plane.normal);
    if (distance <= plane.offset) {
      contact = {
        normal: plane.normal.clone(),
        point: plane.normal.clone().multiplyScalar((plane.offset - distance) / 2).add(vertex),
        penetration: plane.offset - distance
      };
    }
    return contact;
  }
});
elation.extend("physics.colliders.sphere", function(body, args) {
  this.type = 'sphere';
  this.body = body;
  this.radius = args.radius || args;
  this._tmpvec = new THREE.Vector3();

  this.getContact = function(other, collisions) {
    var contact = false;
    switch(other.type) {
      case 'sphere':
        contact = elation.physics.colliders.helperfuncs.sphere_sphere(this, other, collisions);
        break;
      case 'plane':
        contact = elation.physics.colliders.helperfuncs.sphere_plane(this, other, collisions);
        break;
      default:
        console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
    }
    return contact;
  }
});
elation.extend("physics.colliders.plane", function(body, args) {
  this.type = 'plane';
  if (!args) args = {};
  this.body = body;
  this.normal = args.normal || new THREE.Vector3(0,1,0);
  this.offset = args.offset || 0;
  this.radius = Infinity;

  this.getContact = function(other, collisions) {
    var contact = false;
    if (other instanceof elation.physics.colliders.sphere) {
      contact = elation.physics.colliders.helperfuncs.sphere_plane(other, this, collisions);
    } else if (other instanceof elation.physics.colliders.box) {
      contact = elation.physics.colliders.helperfuncs.box_plane(other, this, collisions);
    } else {
      console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
    }
    return contact;
  }
  
});
elation.extend("physics.colliders.box", function(body, args) {
  this.type = 'box';
  if (!args) args = {};
  this.body = body;
  this.min = args.min || new THREE.Vector3(0,0,0);
  this.max = args.max || new THREE.Vector3(0,0,0);
  this.radius = this.max.clone().sub(this.min).length() / 2;

  this.getContact = function(other, collisions) {
    var contact = false;
    if (other instanceof elation.physics.colliders.plane) {
      contact = elation.physics.colliders.helperfuncs.box_plane(this, other, collisions);
    } else {
      console.log("Error: can't handle " + this.type + "-" + other.type + " collisions yet!");
    }
    return contact;
  }
  
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

  this._tmpmat = new THREE.Matrix4();

  if (contactargs.object1) this.bodies.push(contactargs.object1);
  if (contactargs.object2) this.bodies.push(contactargs.object2);

  this.resolve = function(t) {
    this.calculateInternals(t);
    this.applyPositionChange();
    this.applyVelocityChange();
  }
  this.calculateContactMatrix = function() {
    // TODO - make this more memory efficient

    // Check whether the Z-axis is nearer to the X or Y axis
    this.normal.normalize();
    var c1 = this.normal.clone().cross(new THREE.Vector3(0,0,1));
    var c2 = this.normal.clone().cross(new THREE.Vector3(0,1,0));
    var tangent = (c1.lengthSq() > c2.lengthSq() ? c1 : c2);
    tangent.normalize();
    var binormal = this.normal.clone().cross(tangent).normalize();

    this.contactToWorld = new THREE.Matrix4(
      tangent.x, this.normal.x, -binormal.x, 0,
      tangent.y, this.normal.y, -binormal.y, 0,
      tangent.z, this.normal.z, -binormal.z, 0,
      0, 0, 0, 1
    );
    this.worldToContact = new THREE.Matrix4().getInverse(this.contactToWorld);

    //console.log(MATDUMP(this.contactToWorld.m), this.restitution);
  }
  this.calculateLocalVelocity = function(index, duration) {
    var body = this.bodies[index];
    var velocity = body.angular.clone().cross(this.relativePositions[index]).clone().add(body.velocity);
    var foop = VECDUMP(velocity);
    //this.worldToContact.multiplyVector3(velocity);
    velocity.applyMatrix4(this.worldToContact);

    var accvel = body.lastacceleration.clone().multiplyScalar(duration);
    //this.worldToContact.multiplyVector3(accvel);
    accvel.applyMatrix4(this.worldToContact);
    //accvel.y = 0;
    velocity.add(accvel);
    
    //console.log('localvel was ' + foop + ", now " + VECDUMP(velocity));
    return velocity;
  }
  this.calculateDesiredDeltaVelocity = function(duration) {
    // FIXME - potential coordinate space confusion
    var velocityFromAccel = 0;

    velocityFromAccel += this.bodies[0].lastacceleration.clone().multiplyScalar(duration).dot(this.normal);

    this.desiredDeltaVelocity = -this.velocity.y - this.restitution * (this.velocity.y - velocityFromAccel);
  }
  this.calculateInternals = function(duration) {
    this.calculateContactMatrix();

    // Calculate relative position and velocity for the first body
    this.relativePositions[0] = this.point.clone().sub(this.bodies[0].localToWorldPos());
    this.velocity = this.calculateLocalVelocity(0, duration);
    var mworld0 = this._tmpmat.makeRotationFromQuaternion(this.bodies[0].orientationWorld);
    this.inertialMoments[0] = this.bodies[0].momentInverse.clone().multiply(mworld0);

    // If we have a second body, figure out its position and subtract its velocity
    if (this.bodies[1]) {
      this.relativePositions[1] = this.point.clone().sub(this.bodies[1].localToWorldPos());
      this.velocity.sub(this.calculateLocalVelocity(1, duration));
      var mworld1 = this._tmpmat.makeRotationFromQuaternion(this.bodies[1].orientationWorld);
      this.inertialMoments[1] = this.bodies[1].momentInverse.clone().multiply(mworld1);
    }
    this.calculateDesiredDeltaVelocity(duration);
  }
  this.applyVelocityChange = function() {
    // determine new velocities

    var impulse = this.calculateFrictionlessImpulse();
//console.log('apply impulse', impulse.toArray(), this);
    //this.contactToWorld.multiplyVector3(impulse);
    impulse.applyMatrix4(this.contactToWorld);
    this.impulse = impulse;

/*
    for (var i = 0; i < this.bodies.length; i++) {
      var body = this.bodies[i];
      if (body.mass > 0) {
        // FIXME - cyclone swaps impulse and relativeposition for the second object.  which is correct?
        //var impulsiveTorque = this.relativePositions[i].clone().cross(impulse);
  //body.velocity.set(0,0,0);
        this.impulses[i] = impulse.clone().multiplyScalar(1/body.mass * (i == 0 ? 1 : -1));
        body.addVelocity(this.impulses[i]);
        //body.addAngularVelocity(this.inertialMoments[i].multiplyVector3(impulsiveTorque));
        //body.addAngularVelocity(impulsiveTorque.applyMatrix4(this.inertialMoments[i]));
      }
    }
*/

    if (this.bodies[0] && this.bodies[0].mass > 0) {
      //var impulsiveTorque = this.relativePositions[0].clone().cross(impulse);
      this.impulses[0] = impulse.clone().multiplyScalar(1 / this.bodies[0].mass);
      this.bodies[0].addVelocity(this.impulses[0]);
    }

    if (this.bodies[1] && this.bodies[1].mass > 0) {
      //var impulsiveTorque = impulse.clone().cross(this.relativePositions[1]);
      this.impulses[1] = impulse.clone().multiplyScalar(-1 / this.bodies[1].mass);
      this.bodies[1].addVelocity(this.impulses[1]);
    }

    //console.log(VECDUMP(impulse), VECDUMP(impulsiveTorque), VECDUMP(this.bodies[0].velocity));
    //this.bodies[0].velocity.set(0,0,0);
    //this.bodies[0].angular.set(0,0,0);
    //this.bodies[0].removeForce('gravity');
  }
  this.calculateFrictionlessImpulse = function(tensor) {
    var impulse = new THREE.Vector3();
    
    var deltaVelocity = 0;
    for (var i = 0; i < this.bodies.length; i++) {
      if (this.bodies[i].mass > 0) {
        var deltaVelWorld = this.relativePositions[i].clone().cross(this.normal);
        //this.inertialMoments[i].multiplyVector3(deltaVelWorld);
        deltaVelWorld.applyMatrix4(this.inertialMoments[i]);
        deltaVelWorld.cross(this.relativePositions[i]);
        deltaVelocity += deltaVelWorld.dot(this.normal) + 1 / this.bodies[i].mass;
      }
    }
    
    // FIXME - potential coordinate space confusion
    impulse.y = this.desiredDeltaVelocity / deltaVelocity;
//console.log(VECDUMP(impulse), this.desiredDeltaVelocity, deltaVelocity);
    return impulse;
  }
  this.applyPositionChange = function() {
    // resolve penetration

    var linearInertia = [],
        linearMove = [],
        angularInertia = [],
        angularMove = [],
        totalInertia = 0;

    var linearChange = [],
        angularChange = [];
    
    var totalMass = 0;
    for (var i = 0; i < this.bodies.length; i++) {
      var body = this.bodies[i];
      if (body) {
        var angularInertiaWorld = this.relativePositions[i].clone().cross(this.normal);
        angularInertiaWorld.applyMatrix4(this.inertialMoments[i]);
        angularInertiaWorld.cross(this.relativePositions[i]);
        angularInertia[i] = 0;//angularInertiaWorld.dot(this.normal);

        linearInertia[i] = (body.mass == 0 ? 0 : 1 / body.mass);
        totalInertia += linearInertia[i] + angularInertia[i];
        totalMass += body.mass;
      }
    }

    if (totalMass > 0) { // Unstoppable force, immovable object
      for (var i = 0; i < this.bodies.length; i++) {
        var sign = (i == 0 ? 1 : -1);
        angularMove[i] = sign * this.penetration * (angularInertia[i] / totalInertia);
        linearMove[i] = sign * this.penetration * (linearInertia[i] / totalInertia);

        // TODO - fancy stuff with maxMagnitude to prevent uncontrolled spinning

/*
        if (angularMove[i] != 0) {
          var targetAngularDirection = this.relativePositions[i].clone().multiply(this.normal);
          targetAngularDirection.applyMatrix4(this.inertialMoments[i]);
          angularChange[i] = targetAngularDirection.multiplyScalar(angularMove[i] / angularInertia[i]);
        }
*/
        linearChange[i] = this.normal.clone().multiplyScalar(linearMove[i]);
        this.impulses[i] = linearChange[i];
        this.bodies[i].position.add(linearChange[i]);

        //var q = new THREE.Quaternion();
        //var theta = angularChange[i].length();
        // FIXME - potential coordinate space confusion
        //q.setFromAxisAngle(angularChange[i].clone().divideScalar(theta), theta);
        //this.bodies[i].orientation.copy(q);
      }
    }
    //console.log('move it', linearChange, linearMove, linearInertia, angularInertia, totalInertia, this);
  }
});


