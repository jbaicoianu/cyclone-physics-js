elation.require(["physics.processors"], function() {
  elation.extend("physics.processor.gpu", function(parent) {
    elation.physics.processor.base.call(this, parent);

    // TODO:
    // - pack linear+angular pos/vel/accel into texture (OES_texture_float?)
    // - glsl shader multiplies each point by the update matrix, results returned as image
    // - glReadPixels to read back result, map updates back to objects

    this.iterate = function(objects, t) {
      console.log('iterate in gpu not implemented yet', this.parent, objects);
    }
  }, false, elation.physics.processor.base);
});
