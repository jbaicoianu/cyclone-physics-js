elation.require(["physics.common"], function() {
  elation.extend("physics.processor.base", function(parent) {
    this.parent = parent;
    this._tmpvec = new THREE.Vector3();
    this._tmpquat = new THREE.Quaternion();
    this._tmpmat = new THREE.Matrix4();

    this.update = function(objects, t, active) {
      if (typeof active == 'undefined') active = [];
      for (var i = 0; i < objects.length; i++) {
        var obj = objects[i];
        obj.updateState();
        if (!obj.state.sleeping) {
          active.push(obj);
        }
        if (obj.children.length > 0) {
          this.update(obj.children, t, active);
        }
      }
      return active;
    }
    this.iterate = function(objects, t) {
      console.log('iterate in base', this.parent);
    }
    this.iteratePositions = function(objects, t) {
    }
    this.iterateVelocities = function(objects, t) {
    }
    this.collide = function(t) {
      if (t == 0) return; // paused, do nothing
      var collisions = [];

      // Lazy-init octree
      if (!this._octree) this._octree = new elation.physics.octree();

      // Collect bodies with colliders
      var objects = this.parent.getObjects();
      var collidable = [];
      for (var i = 0; i < objects.length; i++) {
        if (objects[i].collider) {
          objects[i].state.colliding = false;
          collidable.push(objects[i]);
        }
      }

      // Build octree (computes AABBs internally, passing dt for velocity expansion)
      this._octree.build(collidable, t);
      var potentials = this._octree.getPotentialPairs();

      // Narrow phase
      for (var i = 0; i < potentials.length; i++) {
        var obj1 = potentials[i][0], obj2 = potentials[i][1];
        // Skip if both sleeping
        if (obj1.state.sleeping && obj2.state.sleeping) continue;
        // Get list of all contact points between the two objects
        var contacts = obj1.getContacts(obj2, [], t);
        if (contacts && contacts.length > 0) {
          for (var j = 0; j < contacts.length; j++) {
            collisions.push(contacts[j]);
          }
          obj1.state.colliding = true;
          obj2.state.colliding = true;
        }
      }

      return collisions;
    }
    this.getDeepestContact = function(contacts) {
      var deepestStatic = -1,
          firstDynamic = -1;
      for (var i = 0; i < contacts.length; i++) {
        if (typeof contacts[i].penetrationTime != 'undefined') {
          if (firstDynamic == -1 || contacts[i].penetrationTime < contacts[firstDynamic].penetrationTime) {
            firstDynamic = i;
          }
        } else {
          if (deepestStatic == -1 || contacts[i].penetration < contacts[deepestStatic].penetration) {
            deepestStatic = i;
          }
        }
      }
      // Prioritize dynamic collisions over static
      if (firstDynamic != -1) {
        return contacts[firstDynamic];
      } else if (deepestStatic != -1) {
        return contacts[deepestStatic];
      }
      return contacts[0];
    }
    this.resolve = function(t, contacts) {
      if (contacts.length == 0) {
        return;
      }
      var linearChange = [
        new THREE.Vector3(),
        new THREE.Vector3()
      ];
      var angularChange = [
        new THREE.Vector3(),
        new THREE.Vector3()
      ];
      while (contacts.length > 0) {
        let contact = contacts.shift();
        contact.resolve(t, linearChange, angularChange, contacts);
        // console.log('blah', contacts.length, linearChange[1].toArray().map(x => +x.toFixed(4)), contact.bodies)
        //break;
      }
    }
  });
});
