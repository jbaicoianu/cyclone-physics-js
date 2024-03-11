elation.require(["physics.common", "physics.processors", "physics.processors.worker", "physics.processors.cpu", "physics.processors.gpu", "physics.processors.physx", "physics.rigidbody", "physics.forces", "physics.constraints", "physics.collisions"], function() {
  elation.extend("physics.system", function(args) {
    this.active = false;
    this.children = [];
    this.processor = false;
    this.args = args || {};
    this.position = this.positionWorld = new THREE.Vector3();
    this.orientation = this.orientationWorld = new THREE.Quaternion();
    this.scale = this.scaleWorld = new THREE.Vector3(1, 1, 1);
    this.substep = elation.utils.any(this.args.substep, true);
    this.substepMaxDelta = elation.utils.any(this.args.substepMaxDelta, 20/1000);
    this.substepMaxSteps = elation.utils.any(this.args.substepMaxSteps, 4);
    this.processortype = elation.utils.any(this.args.processortype, 'cpu');
    this.timescale = 1;
    
    elation.physics.uniqueid = 1;

    this.init = function() {
      if (this.args.autostart !== false) {
        this.start();
      }
    }
    this.start = function(args) {
      this.active = true;
      if (!this.processor) {
        let processortype = (this.processortype in elation.physics.processor ? this.processortype : 'cpu');
        this.processor = new elation.physics.processor[processortype](this, args);
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

        // step 1: update forces for each object, gather array of active objects
        var objects = this.processor.update(this.children, steptime);
        if (objects.length > 0) {

          // step 2: update velocities for all active objects
          this.processor.iterateVelocities(objects, steptime);

          // step 3: detect contacts
          var collisions = this.processor.collide(steptime);

          // step 4: resolve collisions
          if (collisions && collisions.length > 0) {
            this.processor.resolve(steptime, collisions);
          }

          // step 5: update positions for all active objects
          this.processor.iteratePositions(objects, steptime);
        }
        t -= steptime;
        step++;
      }
    }
    this.add = function(obj) {
      obj.parent = this;
      this.children.push(obj);
      elation.events.fire({type: 'add', element: this, data: obj});
    }
    this.remove = function(obj) {
      if (obj.parent && obj.parent != this) {
        obj.parent.remove(obj);
        elation.events.fire({type: 'remove', element: obj.parent, data: obj});
        obj.parent = false;
      } else {
        var i = this.children.indexOf(obj);
        if (i != -1) {
          this.children.splice(i, 1);
          elation.events.fire({type: 'remove', element: this, data: obj});
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
    this.localToWorldScale = function(scale) {
      return scale;
    }
    this.worldToLocalScale = function(scale) {
      return scale;
    }
    this.init();
  });
});
