/**
 * forces
 */
elation.require([], function() {

  elation.extend("physics.forces.gravity", function(body, args) {
    this.others = [];
    this.accel = new THREE.Vector3();
    this.gravsum = new THREE.Vector3();
    this._tmpvec = new THREE.Vector3();

    this.apply = function() {
      if (this.others.length > 0) {
        this.gravsum.set(0,0,0);
        for (var i = 0; i < this.others.length; i++) {
          if (this.others[i]) {
            this._tmpvec.sub(this.others[i].position, body.position);
            var rsq = this._tmpvec.lengthSq();
            var r = Math.sqrt(rsq);
            var a = 6.67384e-11 * this.others[i].mass / rsq;
            this.gravsum.x += a * this._tmpvec.x / r;
            this.gravsum.y += a * this._tmpvec.y / r;
            this.gravsum.z += a * this._tmpvec.z / r;
          }
        }
      } else {
        this.gravsum.copy(this.accel).multiplyScalar(body.mass);
      }
      //console.log("Gravity force: " + [this.gravsum.x, this.gravsum.y, this.gravsum.z] + " m/s^2");
      //return [this.gravsum, false];
      body.applyForce(this.gravsum);
      elation.events.fire({type: 'physics_force_apply', element: this});
    }
    this.update = function(updateargs) {
      if (updateargs instanceof THREE.Vector3) {
        //this.gravsum.copy(updateargs).multiplyScalar(body.mass);
        this.accel.copy(updateargs);
      } else {
        this.others = updateargs;
      }
      elation.events.fire({type: 'physics_force_update', element: this});
    }
    this.getOrbitalVelocity = function(point) {
      this._tmpvec.copy(point).normalize().crossSelf(new THREE.Vector3(0,1,0)).normalize();
  //foo.multiplyScalar(2 * Math.PI * this.position.length());
      var m = this.others[0].mass;
      this._tmpvec.multiplyScalar(Math.sqrt((m * m * 6.67384e-11) / ((m + body.mass) * point.length())));
      return this._tmpvec;
    }
    this.sleepstate = function() {
      return (this.gravsum.lengthSq() <= 1e-6);
    }
    this.update(args);
  });
  elation.extend("physics.forces.static", function(body, args) {
    this.force = args.force || (args instanceof THREE.Vector3 ? args : new THREE.Vector3());
    this.point = args.point || false;
    this.absolute = args.absolute || false

    this._tmpvec = new THREE.Vector3();
    this._tmpvec2 = new THREE.Vector3();

    this.apply = function() {
      this._tmpvec.copy(this.force);
      this._tmpvec2.copy(this.point);
      if (this.point) {
        //var torque = this.point.clone().crossSelf(this.accel);
        //body.momentInverse.multiplyVector3(torque);
        //return [this._tmpvec.clone(), torque];
        body.applyForceAtPoint(this._tmpvec.clone(), this._tmpvec2.clone(), !this.absolute);
      } else {
        body.applyForce(this._tmpvec, !this.absolute);
      }
      elation.events.fire({type: 'physics_force_apply', element: this});
    }
    this.update = function(updateargs) {
      if (updateargs instanceof THREE.Vector3) {
        this.force.copy(updateargs);
      } else {
        this.force.copy(updateargs.force);
        if (updateargs.point instanceof THREE.Vector3) {
          this.point = updateargs.point;
        }
        this.relative = updateargs.relative;
      }
      elation.events.fire({type: 'physics_force_update', element: this});
    }
    this.sleepstate = function() {
      return (this.force.lengthSq() <= 1e-6);
    }
  });
  elation.extend("physics.forces.friction", function(body, args) {
    this.friction = args;
    this.force = new THREE.Vector3();

    this.apply = function() {
      this.force.set(0,0,0);
      if (this.friction > 0) {
        var vsq = body.velocity.lengthSq();
        this.force.copy(body.velocity).multiplyScalar(-1 * this.friction * body.mass);
      }
      body.applyForce(this.force);
      elation.events.fire({type: 'physics_force_apply', element: this});
    }
    this.update = function(updateargs) {
      this.friction = updateargs;
      elation.events.fire({type: 'physics_force_update', element: this});
    }
    this.computeVelocityForDistance = function(dist, friction, mass) {
      var a = friction;
      //var v0 = (dist - .5*a*time^2) / time;


      var v0 = Math.sqrt((a * dist) / (.5 * mass));

      return v0;
    }
    this.sleepstate = function() {
      return !(this.friction > 1e-6 && body.velocity.lengthSq() > 1e-6);
    }
  });
  elation.extend("physics.forces.drag", function(body, args) {
    this.drag = 0;
    this.force = new THREE.Vector3();
    this.sleeping = false;

    this.apply = function() {
      this.force.set(0,0,0);
      if (this.drag > 0) {
        var v = body.velocity.length();
        this.force.copy(body.velocity).multiplyScalar(-.5*this.drag*v*body.mass);
      }
      //return [this.force, false];
      body.applyForce(this.force);
      elation.events.fire({type: 'physics_force_apply', element: this});
    }
    this.update = function(updateargs) {
      this.drag = updateargs;
      console.log('set drag to ', this.drag);
    }
    this.update(args);
  });
  elation.extend("physics.forces.aero", function(body, args) {
    if (!args) args = {};
    this.body = body;
    this.tensor = args.tensor || new THREE.Matrix4();
    this.position = args.position || new THREE.Vector3();
    this.sleeping = false;

    this._tmpvec = new THREE.Vector3();

    this.apply = function() {
      var force = this.getForceFromTensor(this.getTensor());
      this.body.applyForceAtPoint(force, this.position, true);
      elation.events.fire({type: 'physics_force_apply', element: this});
    }
    this.getForceFromTensor = function(tensor) {
      this.body.worldToLocalDir(this._tmpvec.copy(this.body.velocity)).negate();
      //var before = this._tmpvec.clone();
      this.getTensor().multiplyVector3(this._tmpvec).multiplyScalar(.5);
      //console.log('parent: ' + VECDUMP(this.body.velocity) + 'before: ' + VECDUMP(before) + ', after: ' + VECDUMP(this._tmpvec));
      return this._tmpvec;
    }
    this.getTensor = function() {
      return this.tensor;
    }
    this.updateTensor = function(tensor) {
      this.tensor.copy(tensor);
    }
    this.update = function(updateargs) {
      if (updateargs.tensor) {
        this.updateTensor(updateargs.tensor);
      }
      if (args.position) {
        if (args.position instanceof THREE.Vector3) {
          this.position.copy(args.position);
        } else {
          this.position.set(args.position[0], args.position[1], args.position[2]);
        }
      }
      elation.events.fire({type: 'physics_force_update', element: this});
    }
  });

  elation.extend("physics.forces.aerocontrol", function(body, args) {
    this.body = body;
    this.tensor = args.tensor || new THREE.Matrix4();
    this.tensor_min = args.tensor_min || new THREE.Matrix4();
    this.tensor_max = args.tensor_max || new THREE.Matrix4();
    this.position = args.position || new THREE.Vector3();
    this.control = 0;
    this.sleeping = false;

    this._tmpmat = new THREE.Matrix4();

    this.getTensor = function() {
      if (this.control <= -1) {
        return this.tensor_min;
      } else if (this.control >= 1) {
        return this.tensor_max;
      } else if (this.control < 0) {
        return this.interpolate(this.tensor_min, this.tensor, this.control+1);
      } else if (this.control > 0) {
        return this.interpolate(this.tensor, this.tensor_max, this.control);
      }
      return this.tensor;
    }
    this.interpolate = function(a, b, prop) {
      this._tmpmat.set(
        a.n11 * (1 - prop) + b.n11 * prop, a.n12 * (1 - prop) + b.n12 * prop, a.n13 * (1 - prop) + b.n13 * prop, 0,
        a.n21 * (1 - prop) + b.n21 * prop, a.n22 * (1 - prop) + b.n22 * prop, a.n23 * (1 - prop) + b.n23 * prop, 0,
        a.n31 * (1 - prop) + b.n31 * prop, a.n32 * (1 - prop) + b.n32 * prop, a.n33 * (1 - prop) + b.n33 * prop, 0,
        0, 0, 0, 1
      );
      return this._tmpmat;
    }
    this.setControl = function(c) {
      this.control = c;
    }
  });
  elation.physics.forces.aerocontrol.prototype = new elation.physics.forces.aero();

  elation.extend("physics.forces.buoyancy", function(body, args) {
    this.body = body;
    this.density = args.density || 1000; // water = 1000 kg/m^3
    this.volume = args.volume || 1;
    this.maxdepth = args.maxdepth || 10;
    this.waterheight = args.waterheight || 0;
    this.position = args.position || new THREE.Vector3(0,0,0);
    this.submerged = 0;
    this.force = new THREE.Vector3();
    this.positionworld = new THREE.Vector3();
    this.sleeping = false;

    this.apply = function() {
      var point = this.body.localToWorldPos(this.position.clone());
      var depth = point.y - this.maxdepth / 2;
      if (depth >= this.waterheight) {
        this.submerged = 0;
        this.force.y = 0;
      } else {
        if (depth <= this.waterheight - this.maxdepth) {
          this.force.y = this.density * this.volume;
          this.submerged = 1;
        } else {
          //ratio = depth / (this.waterheight - this.maxdepth);
          //this.submerged = depth / (this.waterheight - this.maxdepth) / 2 + this.maxdepth;
          //this.submerged = -(depth - this.maxdepth - this.waterheight) / 2 * this.maxdepth;
          this.submerged = -(depth / (this.maxdepth));
          //force.y = this.density * this.volume * -1 / ((depth - this.maxdepth - this.waterheight) / 2 * this.maxdepth);
          this.force.y = this.density * this.volume * this.submerged;
        }
        //this.force.y *= 0.0098;
        //force.y = this.density * this.volume * ratio * .0098;
        //this.body.applyForceAtPoint(this.force, this.position, false);
        this.positionworld.copy(this.position);
        this.body.applyForceAtPoint(this.force, this.body.localToWorldDir(this.positionworld), false);
      }
      elation.events.fire({type: 'physics_force_apply', element: this});
    }
    this.update = function() {
      elation.events.fire({type: 'physics_force_update', element: this});
    }
  });

  elation.extend("physics.forces.spring", function(body, args) {
    this.body = body;
    this.connectionpoint = args.connectionpoint || new THREE.Vector3(0,0,0);
    this.otherconnectionpoint = args.otherconnectionpoint || new THREE.Vector3(0,0,0);
    this.other = args.other || false;
    this.anchor = args.anchor || false;
    this.strength = elation.utils.any(args.strength, 1);
    this.midpoint = args.midpoint || false;
    this.restlength = elation.utils.any(args.restlength, 0);
    this.bungee = args.bungee || false;
    this.force = new THREE.Vector3();

    var _tmpvec1 = new THREE.Vector3();
    var _tmpvec2 = new THREE.Vector3();

    this.apply = function() {
      var lws = this.body.localToWorldPos(_tmpvec1.copy(this.connectionpoint));
      var ows = (this.other ? this.other.localToWorldPos(_tmpvec2.copy(this.otherconnectionpoint)) : this.anchor);

      this.force.subVectors(lws, ows);
      if (this.midpoint) {
        this.force.divideScalar(2);
      }
      //var magnitude = Math.abs(this.force.length() - this.restlength) * this.strength;
      var magnitude = this.force.length() + 1e-5;
      if (this.bungee && magnitude <= this.restlength) {
        this.force.set(0,0,0);
      } else {
        this.force.divideScalar(magnitude);
        magnitude = this.strength * (magnitude - this.restlength);
        this.force.multiplyScalar(-magnitude);
        this.force = this.body.worldToLocalDir(this.force);
        
        this.body.applyForceAtPoint(this.force, this.connectionpoint, true);
        if (this.other && this.other.mass) {
          this.other.applyForceAtPoint(this.force.multiplyScalar(-1), this.otherconnectionpoint, true);
        }
      }
      elation.events.fire({type: 'physics_force_apply', element: this});
    }
    this.update = function(updateargs) {
      for (var k in updateargs) {
        this[k] = updateargs[k];
      }
      elation.events.fire({type: 'physics_force_update', element: this});
    }
    this.sleepstate = function() {
      var lws = this.body.localToWorldPos(_tmpvec1.copy(this.connectionpoint));
      var ows = (this.other ? this.other.localToWorldPos(_tmpvec2.copy(this.otherconnectionpoint)) : this.anchor);
      return (lws.distanceToSquared(ows) <= 1e6);
    }
  });
  elation.extend("physics.forces.magnet", function(body, args) {
    this.force = new THREE.Vector3();
    this.anchor = args.anchor;
    this.strength = args.strength || 1;
    this.sleeping = false;

    this.apply = (function() {
      var tmpvec = new THREE.Vector3();
      return function() {
        var mu = 1.256636e-6;
        var qm1 = 1,
            qm2 = this.strength;
        this.force.subVectors(body.position, this.anchor);
        var rsq = this.force.lengthSq();

        var F = (mu * qm1 * qm2) / (4 * Math.PI * rsq);
        this.force.normalize().multiplyScalar(F);
  //console.log(this.force.toArray().toString());

        body.applyForce(this.force);
        elation.events.fire({type: 'physics_force_apply', element: this});
      }
    })();
    this.update = function(updateargs) {
      if (updateargs.anchor) {
        this.anchor = updateargs.anchor;
      }
      if (updateargs.strength) {
        this.strength = updateargs.strength;
      }
      elation.events.fire({type: 'physics_force_update', element: this});
    }
  });
  elation.extend("physics.forces.repel", function(body, args) {
    this.force = new THREE.Vector3();
    this.anchor = args.anchor;
    this.strength = args.strength || 1;
    this.sleeping = false;

    this.apply = (function() {
      var tmpvec = new THREE.Vector3();
      return function() {
        this.force.subVectors(body.position, this.anchor);
        var rsq = this.force.lengthSq();

        this.force.normalize().multiplyScalar(this.strength);
  //console.log(this.force.toArray().toString());

        body.applyForce(this.force);
        elation.events.fire({type: 'physics_force_apply', element: this});
      }
    })();
    this.update = function(updateargs) {
      if (updateargs.anchor) {
        this.anchor = updateargs.anchor;
      }
      if (updateargs.strength) {
        this.strength = updateargs.strength;
      }
      elation.events.fire({type: 'physics_force_update', element: this});
    }
  });

  elation.extend("physics.forces.electrostatic", function(body, args) {
    this.force = new THREE.Vector3();
    this.charge = elation.utils.any(args.charge, 1);
    this.maxdist = elation.utils.any(args.maxdist, Infinity);
    this.sleeping = false;

    this.apply = (function() {
      var tmpvec = new THREE.Vector3();
      // TODO - the constant Ke is actually dependent on the electric permittivity of the material the charges are immersed in, but this is good enough for most cases
      var Ke = 8.9875517873681764e9; 
      return function(framedata) {
        var nearby = body.parent.children;
        if (!nearby) return;

        if (typeof framedata['electrostaticID'] == 'undefined') {
          framedata['electrostaticID'] = 0;
          framedata['electrostaticMatrix'] = {};
          framedata['electrostaticSeen'] = {};
        }
        var thisID = framedata['electrostaticID'];
        var matrix = framedata['electrostaticMatrix'];
        var seen = framedata['electrostaticSeen'];

        this.force.set(0,0,0);
        var mycharge = Ke * this.charge;
        for (var i = 0; i < nearby.length; i++) {
          var other = nearby[i];
          if (other === body) continue;
          if (other.position.distanceToSquared(body.position) > this.maxdist * this.maxdist) {
            continue;
          }
          var pairID = Math.min(thisID, i) + '_' + Math.max(thisID, i);
          if (matrix[pairID]) {
            console.log('found an existing solution', pairID, matrix[pairID]);
              //matrix[pairID] = tmpvec.toArray();
            this.force.add(tmpvec.fromArray(matrix[pairID]).multiplyScalar(1));
          } else {
            var forces = other.getForces('electrostatic');
            if (forces) {
              tmpvec.subVectors(body.position, other.position);
              var r = tmpvec.length();
              var rsq = Math.pow(r + 1e6, 2);
              //var rsq = tmpvec.lengthSq() + 1e6;
  //console.log(rsq, body.position.toArray(), other.position.toArray());

              //this.force.add(tmpvec.normalize().multiplyScalar(mycharge * forces[0].charge / rsq));
              //this.force.add(tmpvec.normalize().multiplyScalar(Ke * this.charge * forces[0].charge / rsq));
              this.force.add(tmpvec.multiplyScalar((mycharge * forces[0].charge) / (rsq * r)));

              //matrix[pairID] = tmpvec.toArray();
            }
          }
  //console.log(this.force.toArray().toString());
        }
        framedata['electrostaticID']++;
        body.applyForce(this.force);
        elation.events.fire({type: 'physics_force_apply', element: this});
      }
    })();
    this.update = function(updateargs) {
      for (var k in updateargs) {
        this[k] = updateargs[k];
      }
      elation.events.fire({type: 'physics_force_update', element: this});
    }
  });
});
