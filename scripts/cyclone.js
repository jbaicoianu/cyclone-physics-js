elation.require(["physics.processors", "physics.rigidbody", "physics.forces", "physics.constraints", "physics.collisions"], function() {
  elation.extend("physics.system", function(args) {
    this.physicsmatrix = new THREE.Matrix4().set(1, 1, .5, 0, 0, 1, 1, 0, 0, 0, 1, 0);
    this.active = false;
    this.children = [];
    this.processor = false;
    this.args = args || {};
    this.position = this.positionWorld = new THREE.Vector3();
    this.orientation = this.orientationWorld = new THREE.Quaternion();
    this.substep = elation.utils.any(this.args.substep, true);
    this.substepMaxDelta = elation.utils.any(this.args.substepMaxDelta, 1/10);
    this.substepMaxSteps = elation.utils.any(this.args.substepMaxSteps, 6);
    this.timescale = 1;
    

    this.init = function() {
      if (this.args.autostart !== false) {
        this.start();
      }
    }
    this.start = function() {
      this.active = true;
      if (!this.processor) {
        this.processor = new elation.physics.processor.cpu(this);
      }
    }
    this.stop = function() {
      this.active = false;
    }
    this.step = function(t) {
      // If there are no objects we have nothing to do
      if (!this.active || this.children.length == 0) return;

      var steps = 1;
      if (this.substep && t > this.substepMaxDelta) {
        steps = Math.min(Math.round(t / this.substepMaxDelta), this.substepMaxSteps);
      }
      
      var step = 0;
      while (t > 0) {
        var steptime = (step < steps ? Math.min(t, this.substepMaxDelta) : t);
        // update matrix with new time values
        this.physicsmatrix.elements[4] = this.physicsmatrix.elements[9] = steptime;
        this.physicsmatrix.elements[8] = .5 * steptime * steptime;
        // step 1: update forces for each object, gather array of active objects
        var objects = this.processor.update(this.children, steptime);
        if (objects.length > 0) {
          // step 2: run physics simulation on all active objects
          this.processor.iterate(objects, steptime);

          // step 3: detect contacts
          var collisions = this.processor.collide(steptime);
          if (collisions && collisions.length > 0) {
            // step 4: resolve collisions
            this.processor.resolve(steptime, collisions);
          }
        }
        t -= steptime;
        step++;
      }
    }
    this.add = function(obj) {
      obj.parent = this;
      this.children.push(obj);
    }
    this.remove = function(obj) {
      if (obj.parent && obj.parent != this) {
        obj.parent.remove(obj);
        obj.parent = false;
      } else {
        var i = this.children.indexOf(obj);
        if (i != -1) {
          this.children.splice(i, 1);
        }
      }
    }
    this.getObjects = function(objects, all) {
      if (typeof objects == 'undefined') objects = this.children;
      if (typeof all == 'undefined') all = [];

      for (var i = 0; i < objects.length; i++) {
        all.push(objects[i]);
        if (objects[i].children.length > 0) {
          this.getObjects(objects[i].children, all);
        }
      }
      return all;
    }
    this.worldToLocalPos = function(point) {
      return point;
    }
    this.localToWorldPos = function(point) {
      return point;
    }
    this.localToParentPos = function(point) {
      return point;
    }
    this.parentToLocalPos = function(point) {
      return point;
    }
    this.worldToLocalDir = function(dir) {
      return dir;
    }
    this.localToWorldDir = function(dir) {
      return dir;
    }
    this.init();
  });
});
