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
      var potentials = [];
      // FIXME - Brute force for now.  We should use octrees or BVH here
      var objects = this.parent.getObjects();
      for (var i = 0; i < objects.length-1; i++) {
        var obj1 = objects[i];
        if (obj1.collider) { // && obj1.collider.radius) {
          for (var j = i+1; j < objects.length; j++) {
            var obj2 = objects[j];
            //if (obj2.collider && obj2.collider.radius && !(obj1.state.sleeping && obj2.state.sleeping) && obj1.isPotentiallyColliding(obj2)) {
            if (obj2.collider && !(obj1.state.sleeping && obj2.state.sleeping)) {
              obj1.state.colliding = false;
              obj2.state.colliding = false;
              potentials.push([obj1, obj2]);
            }
          }
        }
      }
      if (potentials.length > 0) {
        //for (var i = 0; i < potentials.length; i++) {
        while (potentials.length > 0) {
          let potentialpair = potentials.shift();
          var obj1 = potentialpair[0], obj2 = potentialpair[1];
          // Get list of all contact points between the two objects
          var contacts = obj1.getContacts(obj2, [], t);
          if (contacts && contacts.length > 0) {
            // Resolve the deepest contact first
            var deepest = this.getDeepestContact(contacts);
            collisions.push(deepest);
            obj1.state.colliding = true;
            obj2.state.colliding = true;
          }
        }
        //console.log(potentials.length + ' potential crashes:', potentials, collisions);
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
      }
    }
  });
});
