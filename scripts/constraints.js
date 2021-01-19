elation.require(['physics.common'], function() {
  /**
   * constraints
   */

  // pivot rotates a single body around its origin, with angular constraints
  elation.extend("physics.constraints.pivot", function(body, args) {
    this.apply = function(contactlist) {
      
    }
  });

  // hinge mounts to one body, and pivots another
  elation.extend("physics.constraints.hinge", function(body, args) {
    this.apply = function(contactlist) {
    }
  });

  // rod links two objects to each other with a fixed length
  elation.extend("physics.constraints.rod", function(body, args) {
    this.apply = function(contactlist) {
    }
  });

  // cable links two objects to each other with a flexible length of string
  elation.extend("physics.constraints.cable", function(body, args) {
    this.apply = function(contactlist) {
    }
  });

  // axis restricts an object's rotation to a single axis, optionally with a min and max angle
  elation.extend("physics.constraints.axis", function(body, args) {
    this.axis = args.axis;
    this.min = args.min || false;
    this.max = args.max || false;
    this.enabled = true;

    var ortho = new THREE.Vector3(),
        trans = new THREE.Vector3(),
        flat = new THREE.Vector3(),
        cross = new THREE.Vector3(),
        scaledAxis = new THREE.Vector3(),
        neworient = new THREE.Quaternion();

    this.apply = function(contactlist) {
      if (!this.enabled) return false;

      //ortho.set(0,0,-1); // FIXME - figure out orthogonal vector based on this.axis
      ortho.set(0,0,1); // FIXME - figure out orthogonal vector based on this.axis
      trans.copy(ortho).applyQuaternion(body.orientation);
      flat.subVectors(trans, scaledAxis.copy(this.axis).multiplyScalar(trans.dot(this.axis))).normalize();
      cross.crossVectors(ortho, flat);
      // FIXME - wow, what a hack
      var sign = (cross.x ? cross.x / Math.abs(cross.x) : 1) * (cross.y ? cross.y / Math.abs(cross.y) : 1) * (cross.z ? cross.z / Math.abs(cross.z) : 1);
      var angle = Math.acos(ortho.dot(flat)) * sign;
      //console.log(angle, sign, this.axis.toArray(), ortho.toArray(), cross.toArray());
      if (this.min && this.max) {
        angle = THREE.Math.clamp(angle, this.min, this.max);
      }
      if (angle == 0.0) angle = 0.0001;
      //angle = THREE.Math.clamp(angle, min, max);
      //console.log(angle, sign, this.min, this.max);
      
      neworient.setFromAxisAngle(this.axis, angle);
      if (!neworient.equals(body.orientation)) {
        body.orientation.copy(neworient);
      }
      return false;
    }
  });
  // speed constraint restricts the object to a maximum speed
  elation.extend("physics.constraints.speed", function(body, args) {
    this.maxspeed = (typeof args != 'undefined' ? args : Infinity);
    this.enabled = true;
    this.apply = function(contactlist) {
      if (!this.enabled) return false;
      var speedSq = body.velocity.lengthSq(),
          maxSpeedSq = this.maxspeed * this.maxspeed;
      if (speedSq > maxSpeedSq) {
        body.velocity.normalize().multiplyScalar(this.maxspeed);
      }
    }
  });
});
