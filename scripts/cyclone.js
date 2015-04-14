elation.require(["physics.processors", "physics.rigidbody", "physics.forces", "physics.collisions"], function() {
  elation.extend("physics.system", function(args) {
    this.physicsmatrix = new THREE.Matrix4().set(1, 1, .5, 0, 0, 1, 1, 0, 0, 0, 1, 0);
    this.active = false;
    this.objects = [];
    this.processor = false;
    this.args = args || {};

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
      if (!this.active || this.objects.length == 0) return;
      
      // update matrix with new time values
      this.physicsmatrix.elements[4] = this.physicsmatrix.elements[9] = t;
      this.physicsmatrix.elements[8] = .5 * t * t;
      // step 1: update forces for each object, gather array of active objects
      var objects = this.processor.update(this.objects, t);
      if (objects.length > 0) {
        // step 2: run physics simulation on all active objects
        this.processor.iterate(objects, t);

        // step 3: detect contacts
        var collisions = this.processor.collide(t)
        if (collisions && collisions.length > 0) {
          // step 4: resolve collisions
          this.processor.resolve(t, collisions);
        }
      }
    }
    this.add = function(obj) {
      this.objects.push(obj);
    }
    this.remove = function(obj) {
      if (obj.parent) {
        obj.parent.remove(obj);
      } else {
        var i = this.objects.indexOf(obj);
        if (i != -1) {
          this.objects.splice(i, 1);
        }
      }
    }
    this.getObjects = function(objects, all) {
      if (typeof objects == 'undefined') objects = this.objects;
      if (typeof all == 'undefined') all = [];

      for (var i = 0; i < objects.length; i++) {
        all.push(objects[i]);
        if (objects[i].children.length > 0) {
          this.getObjects(objects[i].children, all);
        }
      }
      return all;
    }
    this.init();
  });
});
