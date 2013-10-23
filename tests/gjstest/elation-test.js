function ElationTest() {
  this.numsteps = 1000;
  this.stepsize = .1;
}
registerTestSuite(ElationTest);
if (typeof console == 'undefined') {
  console = { log: log }
}

isNearVector = function(v, absoluteError) {
  if (typeof absoluteError == 'undefined') absoluteError = 1e-6;
  return new gjstest.Matcher(
    'is a vector having elements within ' + absoluteError + ' of [' + (v instanceof Array ? v : v.toArray()) + ']',
    'is not a vector having elements within ' + absoluteError + ' of [' + (v instanceof Array ? v : v.toArray()) + ']',
    function(actual) {

      if (!(actual instanceof THREE.Vector3)) {
        return "which is not a vector";
      }
      var expect = (v instanceof THREE.Vector3 ? v.toArray() : v);
      return (Math.abs(expect[0] - actual.x) <= absoluteError * Math.max(1.0, Math.abs(expect[0]), Math.abs(actual.x))) &&
             (Math.abs(expect[1] - actual.y) <= absoluteError * Math.max(1.0, Math.abs(expect[0]), Math.abs(actual.y))) &&
             (Math.abs(expect[2] - actual.z) <= absoluteError * Math.max(1.0, Math.abs(expect[0]), Math.abs(actual.z)));
    }
  );
}

ElationTest.prototype.isLoaded = function() {
  expectThat(elation, not(isUndefined));
  expectThat(elation.events, not(isUndefined));
  expectThat(elation.physics, not(isUndefined));
  expectThat(elation.physics.system, not(isUndefined));
}

