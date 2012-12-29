/**
 * colliders 
 */

elation.extend("physics.colliders.helperfuncs", new function() {
  this._tmpvec = new THREE.Vector3();

  this.sphere_sphere = function(obj1, obj2, collisions) {
    var contact = false;
    var thispos = obj1.body.object.matrixWorld.getPosition().clone(),
        otherpos = obj2.body.object.matrixWorld.getPosition().clone(),
        midline = this._tmpvec.sub(thispos, otherpos),
        size = midline.length();
    if (!(size <= 0 || size > obj1.body.radius + obj2.body.radius)) {
      collisions.push(new elation.physics.contact({
        normal: midline.clone().divideScalar(size),
        point: thispos.clone().addSelf(midline).multiplyScalar(.5),
        penetration: obj1.body.radius + obj2.body.radius - size,
        object1: obj1.body,
        object2: obj2.body
      }));
    }
    return contact;
  }
  this.sphere_plane = function(sphere, plane, collisions) {
    var contact = false;
    var pos = this._tmpvec.copy(sphere.body.object.matrixWorld.getPosition());
    var distance = plane.normal.dot(pos) - sphere.body.radius - plane.offset;
    //console.log('crash a sphere-plane!', sphere, plane, distance);
    if (distance <= 0) {
      var sepspeed = sphere.body.velocity.dot(plane.normal);
      if (sepspeed <= 0) {
        collisions.push(new elation.physics.contact({
          normal: plane.normal,
          point: pos.clone().subSelf(plane.normal.clone().multiplyScalar(distance + sphere.body.radius)), 
          penetration: -distance,
          object1: sphere.body,
          //object2: plane.body
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
      this._tmpvec.set(vertices[i][0], vertices[i][1], vertices[i][2]);
      box.body.object.matrixWorld.multiplyVector3(this._tmpvec);
      var vcollide = this.vertex_plane(this._tmpvec, plane);
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
        point: plane.normal.clone().multiplyScalar((plane.offset - distance) / 2).addSelf(vertex),
        penetration: plane.offset - distance
      };
    }
    return contact;
  }
});
elation.extend("physics.colliders.sphere", function(body, args) {
  this.type = 'sphere';
  this.body = body;
  this.body.radius = args;
  this._tmpvec = new THREE.Vector3();

  this.getContact = function(other, collisions) {
    var contact = false;
    switch(other.type) {
      case 'sphere':
        contact = elation.physics.colliders.helperfuncs.sphere_sphere(this, other, collisions);
        break;
      case 'plane':
        contact = elation.physics.colliders.helperfuncs.sphere_plane(this, othe, collisionsr);
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

  if (contactargs.object1) this.bodies.push(contactargs.object1);
  if (contactargs.object2) this.bodies.push(contactargs.object2);

  this.calculateContactMatrix = function() {
    // FIXME - potential coordinate space confusion
    var tangents = [];
    /*
    if (Math.abs(this.normal.x) > Math.abs(this.normal.y)) {
      var s = 1 / Math.sqrt(this.normal.z * this.normal.z + this.normal.x * this.normal.x);
      tangents[0] = { x: this.normal.z * s,
                      y: 0,
                      z: -this.normal.x * s
                    };
      tangents[1] = { x: this.normal.y * tangents[0].x,
                      y: this.normal.z * tangents[0].x - this.normal.x * tangents[0].z, 
                      z: -this.normal.y * tangents[0].x
                    };
    } else {
      var s = 1 / Math.sqrt(this.normal.z * this.normal.z + this.normal.y + this.normal.y);
      tangents[0] = { x: 0,
                      y: -this.normal.z * s,
                      z: this.normal.y * s
                    };
      tangents[1] = { x: this.normal.y * tangents[0].z - this.normal.z * tangents[0].y,
                      y: -this.normal.x * tangents[0].z, 
                      z: this.normal.x * tangents[0].y
                    };
    }
    */
    tangents[0] = {x: 0, y: 0, z: 1};
    tangents[1] = {x: 1, y: 0, z: 0};
    this.contactToWorld = new THREE.Matrix3();
    this.contactToWorld.m = [
      tangents[0].x, tangents[0].y, tangents[0].z,
      this.normal.x, this.normal.y, this.normal.z,
      tangents[1].x, tangents[1].y, tangents[1].z];

    this.worldToContact = new THREE.Matrix3();
    this.worldToContact.m = [
      tangents[0].x, this.normal.x, tangents[1].x,
      tangents[0].y, this.normal.y, tangents[1].y,
      tangents[0].z, this.normal.z, tangents[1].z
    ];
    //console.log(MATDUMP(this.contactToWorld.m), this.restitution);
  }
  this.calculateLocalVelocity = function(index, duration) {
    var body = this.bodies[index];
    var velocity = body.angular.clone().crossSelf(this.relativePositions[index]).addSelf(body.velocity);
    var foop = VECDUMP(velocity);
    this.worldToContact.multiplyVector3(velocity);

    var accvel = body.lastacceleration.clone().multiplyScalar(duration);
    this.worldToContact.multiplyVector3(accvel);
    //accvel.y = 0;
    velocity.addSelf(accvel);
    
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
    this.relativePositions[0] = this.point.clone().subSelf(this.bodies[0].getWorldPosition());
    this.velocity = this.calculateLocalVelocity(0, duration);
    this.inertialMoments[0] = this.bodies[0].momentInverse.clone().multiplySelf(this.bodies[0].object.matrixWorld);

    // If we have a second body, figure out its position and subtract its velocity
    if (this.bodies[1]) {
      this.relativePositions[1] = this.point.clone().subSelf(this.bodies[1].getWorldPosition());
      this.velocity.subSelf(this.calculateLocalVelocity(1, duration));
      this.inertialMoments[1] = this.bodies[1].momentInverse.clone().multiplySelf(this.bodies[1].object.matrixWorld);
    }
    this.calculateDesiredDeltaVelocity(duration);
  }
  this.applyVelocityChange = function() {
    // determine new velocities

    var impulse = this.calculateFrictionlessImpulse();
    this.contactToWorld.multiplyVector3(impulse);

    for (var i = 0; i < this.bodies.length; i++) {
      var body = this.bodies[i];
      // FIXME - cyclone swaps impulse and relativeposition for the second object.  which is correct?
      var impulsiveTorque = this.relativePositions[i].clone().crossSelf(impulse);
      body.addVelocity(impulse.multiplyScalar(1/body.mass));
      //body.addAngularVelocity(this.inertialMoments[i].multiplyVector3(impulsiveTorque));
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
      var deltaVelWorld = this.relativePositions[i].clone().crossSelf(this.normal);
      this.inertialMoments[i].multiplyVector3(deltaVelWorld);
      deltaVelWorld.crossSelf(this.relativePositions[i]);
      deltaVelocity += deltaVelWorld.dot(this.normal) + 1 / this.bodies[i].mass;
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
    
    for (var i = 0; i < this.bodies.length; i++) {
      var body = this.bodies[i];
      var angularinertiaworld = this.relativePositions[i].clone().crossSelf(this.normal);
      this.inertialMoments[i].multiplyVector3(angularinertiaworld);
      angularinertiaworld.crossSelf(this.relativePositions[i]);

      angularInertia[i] = angularinertiaworld.dot(this.normal);
      linearInertia[i] = 1 / body.mass;
      totalInertia += linearInertia[i] + angularInertia[i];
    }

    for (var i = 0; i < this.bodies.length; i++) {
      var sign = (i == 0 ? 1 : -1);
      angularMove[i] = sign * this.penetration * (angularInertia[i] / totalInertia);
      linearMove[i] = sign * this.penetration;// * (linearInertia[i] / totalInertia);

      // TODO - fancy stuff with maxMagnitude

      if (angularMove[i] != 0) {
        var targetAngularDirection = this.relativePositions[i].clone().multiplySelf(this.normal);
        this.inertialMoments[i].multiplyVector3(targetAngularDirection);
        angularChange[i] = targetAngularDirection.multiplyScalar(angularMove[i] / angularInertia[i]);
      }
      linearChange[i] = this.normal.clone().multiplyScalar(linearMove[i]);
      this.bodies[i].position.addSelf(linearChange[i]);

      var q = new THREE.Quaternion();
      var theta = angularChange[i].length();
      // FIXME - potential coordinate space confusion
      q.setFromAxisAngle(angularChange[i].clone().divideScalar(theta), theta);
      //this.bodies[i].orientation.copy(q);
    }
  }
});


