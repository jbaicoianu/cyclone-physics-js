/**
 * forces
 */

elation.extend("physics.forces.gravity", function(body, args) {
  this.others = [];
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
    }
    //console.log("Gravity force: " + [this.gravsum.x, this.gravsum.y, this.gravsum.z] + " m/s^2");
    //return [this.gravsum, false];
    body.applyForce(this.gravsum);
  }
  this.update = function(updateargs) {
    if (updateargs instanceof THREE.Vector3) {
      this.gravsum.copy(updateargs).multiplyScalar(body.mass);
    } else {
      this.others = updateargs;
    }
  }
  this.getOrbitalVelocity = function(point) {
    this._tmpvec.copy(point).normalize().crossSelf(new THREE.Vector3(0,1,0)).normalize();
//foo.multiplyScalar(2 * Math.PI * this.position.length());
    var m = this.others[0].mass;
    this._tmpvec.multiplyScalar(Math.sqrt((m * m * 6.67384e-11) / ((m + body.mass) * point.length())));
    return this._tmpvec;
  }
  this.update(args);
});
elation.extend("physics.forces.static", function(body, args) {
  this.force = args.force || new THREE.Vector3();
  this.point = args.point || false;
  this.absolute = args.absolute || false

  this._tmpvec = new THREE.Vector3();
  this._tmpvec2 = new THREE.Vector3();

  this.apply = function() {
    this._tmpvec.copy(this.force);
    this._tmpvec2.copy(this.point);
    //body.object.matrixWorld.multiplyVector3(this._tmpvec).subSelf(body.object.matrixWorld.getPosition());
    if (this.point) {
      //var torque = this.point.clone().crossSelf(this.accel);
      //body.momentInverse.multiplyVector3(torque);
      //return [this._tmpvec.clone(), torque];
      body.applyForceAtPoint(this._tmpvec.clone(), this._tmpvec2.clone(), !this.absolute);
    } else {
      body.applyForce(this._tmpvec, !this.absolute);
    }
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
  }
  this.update = function(updateargs) {
    this.friction = updateargs;
  }
});
elation.extend("physics.forces.drag", function(body, args) {
  this.drag = 0;
  this.force = new THREE.Vector3();
  this.apply = function() {
    this.force.set(0,0,0);
    if (this.drag > 0) {
      var v = body.velocity.length();
      this.force.copy(body.velocity).multiplyScalar(-.5*this.drag*v*body.mass);
    }
    //return [this.force, false];
    body.applyForce(this.force);
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
  this._tmpvec = new THREE.Vector3();

  this.apply = function() {
    var force = this.getForceFromTensor(this.getTensor());
    this.body.applyForceAtPoint(force, this.position, true);
  }
  this.getForceFromTensor = function(tensor) {
    this._tmpvec.copy(this.body.getDirectionLocal(this.body.velocity)).negate();
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
  }
});

elation.extend("physics.forces.aerocontrol", function(body, args) {
  this.body = body;
  this.tensor = args.tensor || new THREE.Matrix4();
  this.tensor_min = args.tensor_min || new THREE.Matrix4();
  this.tensor_max = args.tensor_max || new THREE.Matrix4();
  this.position = args.position || new THREE.Vector3();
  this.control = 0;

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
  this.apply = function() {
  }
});

