Cyclone Javascript Physics Engine

A manual port of the Cyclone Physics Engine (http://procyclone.com/) from C++ to JavaScript

Usage:

        // initial setup
        elation.physics.system.start();

        var smallball = new elation.physics.rigidbody({ mass: 1, position: new THREE.Vector3(0, 0, 1) }) );
        smallball.setCollider("sphere", 2);
        elation.physics.system.add(smallball);

        var bigball = new elation.physics.rigidbody({ mass: 10, position: new THREE.Vector3(1, 0, 0) }) );
        bigball.setCollider("sphere", 5);
        elation.physics.system.add(bigball);

        // render loop
        var lasttime = Date.now();
        requestAnimationFrame(stepfunc);

        function stepfunc(t) {
          elation.physics.system.iterate(t - lasttime);
          renderer.render();
        });

