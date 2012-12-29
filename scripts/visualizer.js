elation.extend("physics.visualizer", function() {
  this.markers = [];
  this.unused = [];
  this.growrate = 5;
  this.growmax = 10;
  this.markermaterial = new THREE.MeshBasicMaterial({
    color: 0xff0000,
    opacity: 0.2,
    transparent: true,
    blending: THREE.AdditiveBlending
  });
  elation.events.add(elation.space.fly(0), 'renderframe_start', this);
  this.hud = elation.ui.hud.addWidget("physvis");

  this.showCollision = function(data) {
    this.hud.showCollision(data);
    //this.pauseObject(data.bodies[0]);
    this.getCollisionMarker(data);
  }
  this.getCollisionMarker = function(collision) {
    if (this.unused.length > 0) {
      var marker = this.unused.pop();
    } else {
      var marker = new THREE.Mesh(new THREE.SphereGeometry(1, 36, 18), this.markermaterial);
    }
    marker.position.copy(collision.point);
    marker.scale.set(1,1,1);
    elation.space.fly(0).scene.add(marker);
    this.markers.push(marker);
var foo = new elation.space.meshes.arrow({material: new THREE.MeshBasicMaterial({color: 0xff00ff, depthTest: false, depthWrite: false, transparent: true})});
foo.useQuaternion = true;
var axis = collision.velocity.clone();
var angle = axis.length();
axis.divideScalar(angle);
foo.quaternion.setFromAxisAngle(axis, angle);
marker.add(foo);
    return marker;
  }
  this.pauseObject = function(body) {
    body.paused = true;
    body.updateState();
  }
  this.handleEvent = function(ev) {
    if (typeof this[ev.type] == 'function') {
      this[ev.type](ev);
    }
  }
  this.renderframe_start = function(ev) {
    var remove = [];
    for (var i = 0; i < this.markers.length; i++) {
      var marker = this.markers[i];
      //marker.scale.multiplyScalar(1 + this.growrate * ev.data.lastupdatedelta);
      if (marker.scale.x >= this.growmax) {
        remove.push(i);
      }
    }
//console.log(this.markers, remove);
    for (var j = remove.length-1; j >= 0; j--) {
      var marker = this.markers[remove[j]];
      marker.parent.remove(marker);
      this.markers.splice(remove[j], 1);
      this.unused.push(marker);
    }
  }
});
elation.extend("ui.widgets.physvis", function(hud, args) {
  this.hud = hud;
  this.init = function() {
    this.container = elation.html.create({
      tag: 'div',
      classname: 'physvis',
      append: document.body
    });
    this.list = elation.html.create({
      tag: 'ul',
      append: this.container
    });
  }
  this.showCollision = function(collision) {
    var li = elation.html.create({
      tag: 'li',
      content: "Collision: " + collision.bodies[0].object.name + " " + VECDUMP(collision.point)
    });
    this.list.insertBefore(li, this.list.firstChild);
  }
  this.init();
});
