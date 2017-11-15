/* Lemmings - robot and GUI script.
 *
 * Copyright 2016 Harmen de Weerd
 * Copyright 2017 Johannes Keyser, James Cooke, George Kachergis
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */




// Simulation settings; please change anything that you think makes sense.
simInfo = {
  maxSteps: 50000,  // maximal number of simulation steps to run
  airDrag: 0.1,  // "air" friction of enviroment; 0 is vacuum, 0.9 is molasses
  boxFric: 0.005, // friction between boxes during collisions
  boxMass: 0.01,  // mass of boxes
  boxSize: 20,  // size of the boxes, in pixels
  robotSize: 13,  // approximate robot radius, in pixels (note the SVG gets scaled down)
  robotMass: 0.4, // robot mass (a.u)
  gravity: 0,  // constant acceleration in Y-direction
  bayRobot: null,  // currently selected robot
  baySensor: null,  // currently selected sensor
  bayScale: 3,  // scale within 2nd, inset canvas showing robot in it's "bay"
  doContinue: true,  // whether to continue simulation, set in HTML
  debugSensors: true,  // plot sensor rays and mark detected objects
  debugMouse: true,  // allow dragging any object with the mouse
  engine: null,  // MatterJS 2D physics engine
  world: null,  // world object (composite of all objects in MatterJS engine)
  runner: null,  // object for running MatterJS engine
  height: null,  // set in HTML file; height of arena (world canvas), in pixels
  width: null,  // set in HTML file; width of arena (world canvas), in pixels
  curSteps: 0,  // increased by simStep()

  tresHoldInCorner: 3 * 20,
  percentageBlueBoxesInWall: 0
};


// Description of robot(s), and attached sensor(s) used by InstantiateRobot()
RobotInfo = [
    {body: null,  // for MatterJS body, added by InstantiateRobot()
        color: [255, 255, 255],  // color of the robot shape
        init: {x: 50, y: 50, angle: 0},  // initial position and orientation
        sensors: [  // define an array of sensors on the robot
            // define one sensor
            {sense: senseColor,  // function handle, determines type of sensorString     !!!!!  DONE !!!!
                attachRadius: 2*simInfo.robotSize,
                minVal: 0,  // minimum detectable distance, in pixels
                maxVal: 20,  // maximum detectable distance, in pixels
                attachAngle: -Math.PI/10,  // where the sensor is mounted on robot bodySVGpoints         
                lookAngle: 0,  // direction the sensor is looking (relative to center-out)		
                id: 'leftCol',  // a unique, arbitrary ID of the sensor, for printing/debugging
                color: [0, 250, 0],  // sensor color [in RGB], to distinguish them
                parent: null,  // robot object the sensor is attached to, added by InstantiateRobot
                value: null  // sensor value, i.e. distance in pixels; updated by sense() function
            },
            {sense: senseColor,  // function handle, determines type of sensor
                attachRadius: 0.5*simInfo.robotSize,
                minVal: 0,  // minimum detectable distance, in pixels
                maxVal: 2,  // maximum detectable distance, in pixels								NOTE: This does not scale properly.
                attachAngle: -Math.PI/7.5,  // where the sensor is mounted on robot body
                lookAngle: Math.PI/2,  // direction the sensor is looking (relative to center-out)
                id: 'midCol',  // a unique, arbitrary ID of the sensor, for printing/debugging
                color: [0, 250, 0],  // sensor color [in RGB], to distinguish them
                parent: null,  // robot object the sensor is attached to, added by InstantiateRobot
                value: null  // sensor value, i.e. distance in pixels; updated by sense() function
            },
            {sense: senseColor,  // function handle, determines type of sensor
                attachRadius: 2*simInfo.robotSize,
                minVal: 0,  // minimum detectable distance, in pixels
                maxVal: 20,  // maximum detectable distance, in pixels
                attachAngle: Math.PI/1.7,  // where the sensor is mounted on robot body
                lookAngle: -Math.PI/6,  // direction the sensor is looking (relative to center-out)
                id: 'rightCol',  // a unique, arbitrary ID of the sensor, for printing/debugging
                color: [0, 250, 0],  // sensor color [in RGB], to distinguish them
                parent: null,  // robot object the sensor is attached to, added by InstantiateRobot
                value: null  // sensor value, i.e. distance in pixels; updated by sense() function
            },
            {sense: senseDistance,  // function handle, determines type of sensor
                attachRadius: -0.8*simInfo.robotSize,
                minVal: 20,  // minimum detectable distance, in pixels
                maxVal: 100,  // maximum detectable distance, in pixels
                attachAngle: -Math.PI/5,  // where the sensor is mounted on robot body
                lookAngle: 8*Math.PI/20,  // direction the sensor is looking (relative to center-out)
                id: 'dist',  // a unique, arbitrary ID of the sensor, for printing/debugging
                color: [175, 0, 175],  // sensor color [in RGB], to distinguish them
                parent: null,  // robot object the sensor is attached to, added by InstantiateRobot
                value: null  // sensor value, i.e. distance in pixels; updated by sense() function
            }
            // define another sensor
            /*{sense: senseDistance, attachRadius: 3*simInfo.robotSize, minVal: 0, maxVal: 50, attachAngle: -Math.PI/4,
                lookAngle: 0, id: 'distL', color: [0, 150, 0], parent: null, value: null
            },

            {sense: senseColor,  // function handle, determines type of sensor
                attachRadius: 3*simInfo.robotSize,
                minVal: 0,  // minimum detectable distance, in pixels
                maxVal: 50,  // maximum detectable distance, in pixels
                attachAngle: Math.PI/4,  // where the sensor is mounted on robot body
                lookAngle: 0,  // direction the sensor is looking (relative to center-out)
                id: 'colR',  // a unique, arbitrary ID of the sensor, for printing/debugging
                color: [150, 0, 0],  // sensor color [in RGB], to distinguish them
                parent: null,  // robot object the sensor is attached to, added by InstantiateRobot
                value: null  // sensor value, i.e. distance in pixels; updated by sense() function
            },
            // define another sensor
            {sense: senseColor, attachRadius: 3*simInfo.robotSize, minVal: 0, maxVal: 50, attachAngle: -Math.PI/4,
                lookAngle: 0, id: 'colL', color: [0, 150, 0], parent: null, value: null
            }*/
        ]
    }
];


robots = new Array();
sensors = new Array();

function init() {  // called once when loading HTML file
  const robotBay = document.getElementById("bayLemming"),
        arena = document.getElementById("arenaLemming"),
        height = arena.height,
        width = arena.width;
  simInfo.height = height;
  simInfo.width = width;

  /* Create a MatterJS engine and world. */
  simInfo.engine = Matter.Engine.create();
  simInfo.world = simInfo.engine.world;
  simInfo.world.gravity.y = simInfo.gravity;
  simInfo.engine.timing.timeScale = 1;

  /* Create walls and boxes, and add them to the world. */
  // note that "roles" are custom properties for rendering (not from MatterJS)
  function getWall(x, y, width, height) {
    return Matter.Bodies.rectangle(x, y, width, height,
                                   {isStatic: true, role: 'wall',
                                    color:[150, 150, 150]});
  };
  const wall_lo = getWall(width/2, height-5, width-5, 5),
        wall_hi = getWall(width/2, 5, width-5, 5),
        wall_le = getWall(5, height/2, 5, height-15),
        wall_ri = getWall(width-5, height/2, 5, height-15);
  Matter.World.add(simInfo.world, [wall_lo, wall_hi, wall_le, wall_ri]);

  /* Add a bunch of boxes in a neat grid. */
  function getBox(x, y) {
    // flip coin for red vs blue and add rgb
    colFlag = Math.round(Math.random());  // random 0,1 variable for box color
    if (colFlag == 1 ){
      color = [0, 0, 200];
    }
    else {
      color = [200, 0, 0];
    }
    box = Matter.Bodies.rectangle(x, y, simInfo.boxSize, simInfo.boxSize,
                                  {frictionAir: simInfo.airDrag,
                                   friction: simInfo.boxFric,
                                   mass: simInfo.boxMass,
                                   role: 'box',
                                   color: color});
    return box;
  };

  const startX = 100, startY = 100,
        nBoxX = 5, nBoxY = 5,
        gapX = 40, gapY = 30,
        stack = Matter.Composites.stack(startX, startY,
                                        nBoxX, nBoxY,
                                        gapX, gapY, getBox);
  Matter.World.add(simInfo.world, stack);

  /* Add debugging mouse control for dragging objects. */
  if (simInfo.debugMouse){
    const mouseConstraint = Matter.MouseConstraint.create(simInfo.engine,
                              {mouse: Matter.Mouse.create(arena),
                               // spring stiffness mouse ~ object
                               constraint: {stiffness: 0.5}});
    Matter.World.add(simInfo.world, mouseConstraint);
  }
  // Add the tracker functions from mouse.js
  addMouseTracker(arena);
  addMouseTracker(robotBay);

  /* Running the MatterJS physics engine (without rendering). */
  simInfo.runner = Matter.Runner.create({fps: 60, isFixed: false});
  Matter.Runner.start(simInfo.runner, simInfo.engine);
  // register function simStep() as callback to MatterJS's engine events
  Matter.Events.on(simInfo.engine, 'tick', simStep);

  /* Create robot(s). */
  setRobotNumber(1);  // requires defined simInfo.world
  loadBay(robots[0]);

};

function convrgb(values) {
  return 'rgb(' + values.join(', ') + ')';
};


function rotate(robot, torque=0) {
  /* Apply a torque to the robot to rotate it.
   *
   * Parameters
   *   torque - rotational force to apply to the body.
   *            Try values around +/- 0.005.
   */
  robot.body.torque = torque;
 };

function drive(robot, force=0) {
  /* Apply a force to the robot to move it.
   *
   * Parameters
   *   force - force to apply to the body.
   *           Try values around +/- 0.0005.
   */
  const orientation = robot.body.angle,
        force_vec = Matter.Vector.create(force, 0),
        move_vec = Matter.Vector.rotate(force_vec, orientation);
  Matter.Body.applyForce(robot.body, robot.body.position , move_vec);
};

/*
Lower 2 functions written by Max & Felicity for generating noise for the colour sensors
 */
function gaussColorNoise(sigma=5) {
    const x0 = 1.0 - Math.random();
    const x1 = 1.0 - Math.random();
    return sigma * Math.sqrt(-2 * Math.log(x0)) * Math.cos(2 * Math.PI * x1);
};

function colorNoise(color) {
    var noise = [Math.round(gaussColorNoise()),
        Math.round(gaussColorNoise()),
        Math.round(gaussColorNoise())];

    var newColor = [0,0,0];
    for(i=0; i<newColor.length;i++) {
        newColor[i] = color[i] + noise[i];
        // In case the new color values become negative, they should be minimally set to 0
        if (newColor[i] < 0) {
            newColor[i] = 0;
        }
    }
    return newColor;
}



function senseColor() {
    /* WRITTEN AFTERWARDS by Matthijs.
     *
     * Color sensor simulation. Called from sensor object, returns nothing, updates a new reading into this.value.
     *
     * Idea: Cast a ray with a certain length from the sensor, and check
     *       via collision detection if objects intersect with the ray.
     *       Update value with object color. Writes [0,0,0] if no color is found!
     * Note: Sensor ray needs to ignore robot (parts), or start outside of it.
     *       The latter is easy with the current circular shape of the robots.
     * Note: Order of tests are optimized by starting with max ray length, and
     *       then only testing the maximal number of initially resulting objects.
     * Note: The sensor's "ray" could have any other (convex) shape;
     *       currently it's just a very thin rectangle.
     */

    const context = document.getElementById('arenaLemming').getContext('2d');
    var bodies = Matter.Composite.allBodies(simInfo.engine.world);

    const robotAngle = this.parent.body.angle,
        attachAngle = this.attachAngle,
        rayAngle = robotAngle + attachAngle + this.lookAngle;

    const rPos = this.parent.body.position,
        rSize = this.attachRadius,
        startPoint = {x: rPos.x + (rSize+1) * Math.cos(robotAngle + attachAngle),
            y: rPos.y + (rSize+1) * Math.sin(robotAngle + attachAngle)};

    function getEndpoint(rayLength) {
        return {x: rPos.x + (rSize + rayLength) * Math.cos(rayAngle),
            y: rPos.y + (rSize + rayLength) * Math.sin(rayAngle)};
    };

    function sensorRay(bodies, rayLength) {
        bodies = bodies.filter(body => body.role != 'robot');
        // Cast ray of supplied length and return the bodies that collide with it.
        const rayWidth = 1e-100,
            endPoint = getEndpoint(rayLength);
        rayX = (endPoint.x + startPoint.x) / 2,
            rayY = (endPoint.y + startPoint.y) / 2,
            rayRect = Matter.Bodies.rectangle(rayX, rayY, rayLength, rayWidth,
                {isSensor: true, isStatic: true,
                    angle: rayAngle, role: 'sensor'});

        var collidedBodies = [];
        for (var bb = 0; bb < bodies.length; bb++) {
            var body = bodies[bb];
            // coarse check on body boundaries, to increase performance:
            if (Matter.Bounds.overlaps(body.bounds, rayRect.bounds)) {
                for (var pp = body.parts.length === 1 ? 0 : 1; pp < body.parts.length; pp++) {
                    var part = body.parts[pp];
                    // finer, more costly check on actual geometry:
                    if (Matter.Bounds.overlaps(part.bounds, rayRect.bounds)) {
                        const collision = Matter.SAT.collides(part, rayRect);
                        if (collision.collided) {
                            collidedBodies.push(body);
                            break;
                        }
                    }
                }
            }
        }
        return collidedBodies;
    };

    // call 1x with full length, and check all bodies in the world;
    // in subsequent calls only check the bodies resulting here
    var rayLength = this.maxVal;
    bodies = sensorRay(bodies, rayLength);
    // if some collided, search for maximal ray length without collisions
    if (bodies.length > 0) {
        var lo = 0,
            hi = rayLength;
        while (lo < rayLength) {
            if (sensorRay(bodies, rayLength).length > 0) {
                hi = rayLength;
            }
            else {
                lo = rayLength;
            }
            rayLength = Math.floor(lo + (hi-lo)/2);
        }
    }
    // increase length to (barely) touch closest body (if any)
    rayLength += 1;
    bodies = sensorRay(bodies, rayLength);
    if(bodies.length == 1)
      color = bodies[0].color;
    else if (bodies.length == 0)
      color = [0, 0, 0];


    if (simInfo.debugSensors) {  // if invisible, check order of object drawing
        // draw the resulting ray
        endPoint = getEndpoint(rayLength);
        context.beginPath();
        context.moveTo(startPoint.x, startPoint.y);
        context.lineTo(endPoint.x, endPoint.y);
        //context.color
        context.strokeStyle = convrgb(this.color);
        context.lineWidth = 2.0;
        context.stroke();
        // mark all objects's lines intersecting with the ray
        for (var bb = 0; bb < bodies.length; bb++) {
            var vertices = bodies[bb].vertices;
            context.moveTo(vertices[0].x, vertices[0].y);
            for (var vv = 1; vv < vertices.length; vv += 1) {
                context.lineTo(vertices[vv].x, vertices[vv].y);
            }
            context.closePath();
        }
        context.stroke();
    }
	
    this.value = colorNoise(color);
};

function senseDistance() {
  /* Distance sensor simulation based on ray casting. Called from sensor
   * object, returns nothing, updates a new reading into this.value.
   *
   * Idea: Cast a ray with a certain length from the sensor, and check
   *       via collision detection if objects intersect with the ray.
   *       To determine distance, run a Binary search on ray length.
   * Note: Sensor ray needs to ignore robot (parts), or start outside of it.
   *       The latter is easy with the current circular shape of the robots.
   * Note: Order of tests are optimized by starting with max ray length, and
   *       then only testing the maximal number of initially resulting objects.
   * Note: The sensor's "ray" could have any other (convex) shape;
   *       currently it's just a very thin rectangle.
   */

  const context = document.getElementById('arenaLemming').getContext('2d');
  var bodies = Matter.Composite.allBodies(simInfo.engine.world);

  const robotAngle = this.parent.body.angle,
        attachAngle = this.attachAngle,
        rayAngle = robotAngle + attachAngle + this.lookAngle;

  const rPos = this.parent.body.position,
        rSize = this.attachRadius,
        startPoint = {x: rPos.x + (rSize+1) * Math.cos(robotAngle + attachAngle),
                      y: rPos.y + (rSize+1) * Math.sin(robotAngle + attachAngle)};

  function getEndpoint(rayLength) {
    return {x: rPos.x + (rSize + rayLength) * Math.cos(rayAngle),
            y: rPos.y + (rSize + rayLength) * Math.sin(rayAngle)};
  };

  function sensorRay(bodies, rayLength) {
      bodies = bodies.filter(body => body.role != 'robot');
    // Cast ray of supplied length and return the bodies that collide with it.
    const rayWidth = 1e-100,
          endPoint = getEndpoint(rayLength);
    rayX = (endPoint.x + startPoint.x) / 2,
    rayY = (endPoint.y + startPoint.y) / 2,
    rayRect = Matter.Bodies.rectangle(rayX, rayY, rayLength, rayWidth,
                                      {isSensor: true, isStatic: true,
                                       angle: rayAngle, role: 'sensor'});

    var collidedBodies = [];
    for (var bb = 0; bb < bodies.length; bb++) {
      var body = bodies[bb];
      // coarse check on body boundaries, to increase performance:
      if (Matter.Bounds.overlaps(body.bounds, rayRect.bounds)) {
        for (var pp = body.parts.length === 1 ? 0 : 1; pp < body.parts.length; pp++) {
          var part = body.parts[pp];
          // finer, more costly check on actual geometry:
          if (Matter.Bounds.overlaps(part.bounds, rayRect.bounds)) {
            const collision = Matter.SAT.collides(part, rayRect);
            if (collision.collided) {
              collidedBodies.push(body);
              break;
            }
          }
        }
      }
    }
    return collidedBodies;
  };

  // call 1x with full length, and check all bodies in the world;
  // in subsequent calls, only check the bodies resulting here
  var rayLength = this.maxVal;
  bodies = sensorRay(bodies, rayLength);
  // if some collided, search for maximal ray length without collisions
  if (bodies.length > 0) {
    var lo = 0,
        hi = rayLength;
    while (lo < rayLength) {
      if (sensorRay(bodies, rayLength).length > 0) {
        hi = rayLength;
      }
      else {
        lo = rayLength;
      }
      rayLength = Math.floor(lo + (hi-lo)/2);
    }
  }
  // increase length to (barely) touch closest body (if any)
  rayLength += 1;
  bodies = sensorRay(bodies, rayLength);

  if (simInfo.debugSensors) {  // if invisible, check order of object drawing
    // draw the resulting ray
    endPoint = getEndpoint(rayLength);
    context.beginPath();
    context.moveTo(startPoint.x, startPoint.y);
    context.lineTo(endPoint.x, endPoint.y);
    context.strokeStyle = convrgb(this.color);
    context.lineWidth = 0.5;
    context.stroke();
    // mark all objects's lines intersecting with the ray
    for (var bb = 0; bb < bodies.length; bb++) {
      var vertices = bodies[bb].vertices;
      context.moveTo(vertices[0].x, vertices[0].y);
      for (var vv = 1; vv < vertices.length; vv += 1) {
        context.lineTo(vertices[vv].x, vertices[vv].y);
      }
      context.closePath();
    }
    context.stroke();
  }

  // indicate if the sensor exceeded its maximum length by returning infinity
  if (rayLength > this.maxVal) {
    rayLength = Infinity;
  }
  else {
    // apply mild noise on the sensor reading, and clamp between valid values
    function gaussNoise(sigma=1) {
      const x0 = 1.0 - Math.random();
      const x1 = 1.0 - Math.random();
      return sigma * Math.sqrt(-2 * Math.log(x0)) * Math.cos(2 * Math.PI * x1);
    };
    rayLength = Math.floor(rayLength + gaussNoise(3));
    rayLength = Matter.Common.clamp(rayLength, this.minVal, this.maxVal);
  }

  this.value = rayLength;
};

function dragSensor(sensor, event) {
  const robotBay = document.getElementById('bayLemming'),
        bCenter = {x: robotBay.width/2,
                   y: robotBay.height/2},
        rSize = sensor.attachRadius,
        bScale = simInfo.bayScale,
        sSize = sensor.getWidth(),
        mAngle = Math.atan2(  event.mouse.x - bCenter.x,
                            -(event.mouse.y - bCenter.y));
  sensor.info.attachAngle = mAngle;
  sensor.x = bCenter.x - sSize - bScale * rSize * Math.sin(-mAngle);
  sensor.y = bCenter.y - sSize - bScale * rSize * Math.cos( mAngle);
  repaintBay();
}

function loadSensor(sensor, event) {
  loadSensorInfo(sensor.sensor);
}

function loadSensorInfo(sensorInfo) {
  simInfo.baySensor = sensorInfo;
}

function loadBay(robot) {
  simInfo.bayRobot = robot;
  sensors = new Array();
  const robotBay = document.getElementById("bayLemming");
  const bCenter = {x: robotBay.width/2,
                   y: robotBay.height/2},
        bScale = simInfo.bayScale;

  for (var ss = 0; ss < robot.info.sensors.length; ++ss) {
    const curSensor = robot.sensors[ss],
          attachAngle = curSensor.attachAngle,
          rSize = curSensor.attachRadius;
    // put current sensor into global variable, make mouse-interactive
    sensors[ss] = makeInteractiveElement(new SensorGraphics(curSensor),
                                         document.getElementById("bayLemming"));
    const sSize = sensors[ss].getWidth();
    sensors[ss].x = bCenter.x - sSize - bScale * rSize * Math.sin(-attachAngle);
    sensors[ss].y = bCenter.y - sSize - bScale * rSize * Math.cos( attachAngle);
    sensors[ss].onDragging = dragSensor;
    sensors[ss].onDrag = loadSensor;
  }
  repaintBay();
}

function SensorGraphics(sensorInfo) {
  this.info = sensorInfo;
  this.plotSensor = plotSensor;
  // add functions getWidth/getHeight for graphics.js & mouse.js,
  // to enable dragging the sensor in the robot bay
  this.getWidth = function() { return 6; };
  this.getHeight = function() { return 6; };
}

function loadFromSVG() {
  var vertexSets = [];
  const svg = document.getElementById('robotbodySVG'),
        data = svg.contentDocument;


  jQuery(data).find('path').each(function(_, path) {
    var points = Matter.Svg.pathToVertices(path, 30);
     vertexSets.push(Matter.Vertices.scale(points, 0.2, 0.2))
  });

  return vertexSets;
};

function InstantiateRobot(robotInfo) {
  // load robot's body shape from SVG file
  const bodySVGpoints = loadFromSVG();
  this.body = Matter.Bodies.fromVertices(robotInfo.init.x,
                                         robotInfo.init.y,
                                        bodySVGpoints,
                                         {frictionAir: simInfo.airDrag,
                                          mass: simInfo.robotMass,
                                          color: [255, 255, 255],
                                          role: 'robot'}, true);


  Matter.World.add(simInfo.world, this.body);
  Matter.Body.setAngle(this.body, robotInfo.init.angle);

  // instantiate its sensors
  this.sensors = robotInfo.sensors;
  for (var ss = 0; ss < this.sensors.length; ++ss) {
    this.sensors[ss].parent = this;
  }

  // attach its helper functions
  this.rotate = rotate;
  this.drive = drive;
  this.info = robotInfo;
  this.plotRobot = plotRobot;

  // add functions getWidth/getHeight for graphics.js & mouse.js,
  // to enable selection by clicking the robot in the arena
  this.getWidth = function() { return 2 * simInfo.robotSize; };
  this.getHeight = function() { return 2 * simInfo.robotSize; };
}

function robotUpdateSensors(robot) {
  // update all sensors of robot; puts new values into sensor.value
  for (var ss = 0; ss < robot.sensors.length; ss++) {
    robot.sensors[ss].sense();
  }
};

function getSensorValById(robot, id) {
  for (var ss = 0; ss < robot.sensors.length; ss++) {
    if (robot.sensors[ss].id == id) {
      return robot.sensors[ss].value;
    }
  }
  return undefined;  // if not returned yet, id doesn't exist
};

function robotMove(robot) {
  // TODO: Define Lemming program here.
  const dist = getSensorValById(robot, 'dist');

	//gives string of object
  const objLeft = classifyRGB(getSensorValById(robot, 'leftCol')),
        objRight = classifyRGB(getSensorValById(robot, 'rightCol')),
        objMid = classifyRGB(getSensorValById(robot, 'midCol'));
 // Initial version of the Lemmings program - Johan
 // The following rules are implemented:
	// By default the Lemming should wander around in a slight curve
	// If it senses a block in its subsequent behavior depends on wherter a block is in the gripper and the block color
	// - If it carries no block it should drive straight towards it, and thus get the block in the gripper
	// - If it carries a blue block, it keeps wandering and ignores the detected block
	// - If it carries a red block it turns left to leave the block
	// If it senses a wall its behavior should depend on whether a block is in the gripper and the block color
	// - If it carries no block it should turn either left of right
	// - If it carreis a blue block it should turn left to leave the block
	// - If it carries a red block it should turn right to keep the block
	
	driveConstant = 0.00025;
	turnConstant = 0.0025;
	

	sensesBlock = false;	
	sensesWall = false;

	if(objMid == "Red" || objMid == "Blue"){
		sensesBlock = true;
	}

	if(objLeft == "Wall" || objRight == "Wall"){
		sensesWall = true;
	}


	
	// if(sensesBlock){
	// 	switch(objMid){
	// 		case "Empty":
	// 		case "Robot":
	// 		case "Wall":
	// 			driveVector(robot, force=driveConstant, angle=0);
	// 			break;
	// 		case "Blue":
	// 			driveVector(robot, force=driveConstant, angle=turnConstant);
	// 			break;
	// 		case "Red":
	// 			driveVector(robot, force=0, angle=turnConstant);
	// 			break;
	// 		default:
	// 			console.log("Error")
	// 	}
	// }
	if(sensesWall){
		switch(objMid){
			case "Empty":
        driveVector(robot, force=0, angle=-6*turnConstant);
        break;
			case "Robot":
        driveVector(robot, force=0, angle=-2*turnConstant);
        break;
			case "Blue":
				driveVector(robot, force=0, angle=-turnConstant);
				break;
			case "Red":
				driveVector(robot, force=0, angle=turnConstant);
				break;
			default:
				console.log("Error")
		}
	} else if (dist != Infinity) {
      switch(objMid){
        case "Empty":
          driveVector(robot, force=driveConstant, angle=0);
          break;
        case "Robot":
          driveVector(robot, force=0, angle=-2*turnConstant);
          break;
        case "Blue":
          driveVector(robot, force=driveConstant, angle=turnConstant);
          break;
        case "Red":
          if (objLeft == "Red" || objRight == "Red") {
            driveVector(robot, force=0, angle=-2*turnConstant); //Leave red block at red blocks
          } else {
            driveVector(robot, force=driveConstant, angle=turnConstant);
          }
          break;
        default:
          console.log("Error")
      }
    
  } else {
    driveVector(robot, force=driveConstant, angle=turnConstant);
  }

	//TODO check which direction is left and right
	//TODO work out turning left to leave blue block if wall sensed
	//TODO work out grasping and leaving blocks
	//TODO test if the function driveVector works
	// if(!sensesWall && !sensesBlock){
	// 	driveVector(robot, force=driveConstant, angle=turnConstant);
	// }
	
	//Helper function
		//use the concept of a vector to determine movement of a robot
	function driveVector(robot, force, angle){
		robot.drive(robot, force)
		robot.rotate(robot, angle)
}
};



function plotSensor(context, x = this.x, y = this.y) {
  context.beginPath();
  context.arc(x + this.getWidth()/2,
              y + this.getHeight()/2,
              this.getWidth()/2, 0, 2*Math.PI);
  context.closePath();
  context.fillStyle = 'black';
  context.strokeStyle = 'black';
  context.fill();
  context.stroke();
}

function plotRobot(context,
                   xTopLeft = this.body.position.x,
                   yTopLeft = this.body.position.y) {
  var x, y, scale, angle, i, half, full,
      rSize = simInfo.robotSize;
  const showInternalEdges = true;


  if (context.canvas.id == "bayLemming") {
    scale = simInfo.bayScale;
    half = Math.floor(rSize/2*scale);
    full = half* 2;
    x = xTopLeft + full;
    y = yTopLeft + full;
    angle = -Math.PI / 2;
  } else {
    scale = 1;
    half = Math.floor(rSize/2*scale);
    full = half * 2;
    x = xTopLeft;
    y = yTopLeft;
    angle = this.body.angle;
  }
  context.save();
  context.translate(x, y);
  context.rotate(angle);

  if (context.canvas.id == "arenaLemming") {
    // draw into world canvas without transformations,
    // because MatterJS thinks in world coords...
    context.restore();

    const body = this.body;
    // handle compound parts


    context.beginPath();
    for (k = body.parts.length > 1 ? 1 : 0; k < body.parts.length; k++) {
      part = body.parts[k];
      context.moveTo(part.vertices[0].x,
                     part.vertices[0].y);
      for (j = 1; j < part.vertices.length; j++) {
        if (!part.vertices[j - 1].isInternal || showInternalEdges) {
          context.lineTo(part.vertices[j].x,
                         part.vertices[j].y);
        } else {
          context.moveTo(part.vertices[j].x,
                         part.vertices[j].y);
        }

        if (part.vertices[j].isInternal && !showInternalEdges) {
          context.moveTo(part.vertices[(j + 1) % part.vertices.length].x,
                         part.vertices[(j + 1) % part.vertices.length].y);
        }
      }
      context.lineTo(part.vertices[0].x,
                     part.vertices[0].y);
    }

    context.strokeStyle = convrgb(body.color);
    context.lineWidth = 1.5;
    context.stroke();

    // to draw the rest, rotate & translate again
    context.save();
    context.translate(x, y);
    context.rotate(angle);
  }

  // Plot sensor positions into world canvas.
  if (context.canvas.id == "arenaLemming") {
    for (ss = 0; ss < this.info.sensors.length; ++ss) {
      context.beginPath();
      full = 2*Math.floor(this.info.sensors[ss].attachRadius/2*scale)
      context.arc(full * Math.cos(this.info.sensors[ss].attachAngle),
                  full * Math.sin(this.info.sensors[ss].attachAngle),
                  scale, 0, 2*Math.PI);
      context.closePath();
      context.fillStyle = 'black';
      context.strokeStyle = 'black';
      context.fill();
      context.stroke();
    }
  }
  context.restore();
}

/*
Added by Matthijs. Classifies rgb values into percepts.
 */
function classifyRGB(rgb) {
    r = rgb[0];g = rgb[1];b = rgb[2];

    if (r<25&&g<25&&b<25){
        return("Empty");
    } else if (r>150&&g<25&&b<25){
        return("Red");
    } else if (r<25&&g<25&&b>150){
        return("Blue");
    } else if (r>200&&g>200&&b>200) {
        return ("Robot");
    } else if (r>100&&r<200&&g>100&&g<200&&b>100&&b<200) {
        return ("Wall");
    }
}

function simStep() {
  // advance simulation by one step (except MatterJS engine's physics)
  if (simInfo.curSteps < simInfo.maxSteps && simInfo.percentageBlueBoxesInWall < 0.85) {
    repaintBay();
      drawBoard();
    for (var rr = 0; rr < robots.length; ++rr) {
      robotUpdateSensors(robots[rr]);
      robotMove(robots[rr]);
      // To enable selection by clicking (via mouse.js/graphics.js),
      // the position on the canvas needs to be defined in (x, y):
      const rSize = simInfo.robotSize;
      robots[rr].x = robots[rr].body.position.x - rSize;
      robots[rr].y = robots[rr].body.position.y - rSize;
    }
    // count and display number of steps
    simInfo.curSteps += 1;
    document.getElementById("SimStepLabel").innerHTML =
      padnumber(simInfo.curSteps, 5) +
      ' of ' +
      padnumber(simInfo.maxSteps, 5);

    if (simInfo.curSteps%30==0)
      updateStatistics();
  }
  else {
    toggleSimulation();
  }
}

/*
Updates the statistics in simInfo.
 */
function updateStatistics() {
  bodies = Matter.Composite.allBodies(simInfo.world)
  boxes = bodies.filter(body => body.role == "box");
  blueBoxes = boxes.filter(box => box.color[2] > box.color[0]);
  redBoxes = boxes.filter(box => box.color[0] > box.color[2]);
  blueBoxesPos = blueBoxes.map(box => box.position);
  redBoxesPos = redBoxes.map(box => box.position);

  function calcPercentageTresholdFromCorner(boxesPos,treshold){
        height = simInfo.height;
        width = simInfo.width;
        wallThickness = 5;


      boxesDisFromCorner = boxesPos.map(boxPos => Math.sqrt(
           Math.pow(Math.min(boxPos.x - wallThickness,(width - wallThickness) - boxPos.x),2) +
           Math.pow(Math.min(boxPos.y - wallThickness,(height - wallThickness) - boxPos.y),2)));

      boxesWithinTreshFromCorner = boxesDisFromCorner.filter(boxDist => boxDist < treshold)

      return(boxesWithinTreshFromCorner.length / blueBoxes.length);
    }

    simInfo.percentageBlueBoxesInWall = calcPercentageTresholdFromCorner(blueBoxesPos,simInfo.tresHoldInCorner)
  //AGAIN UNUSED, BUT MAYBE NESSECARY LATER
  //Calculating the mean distance from the center
  /*function calcMeanDistFromCenter(boxesPos){
      height = simInfo.height;
      width = simInfo.width;
      boxesPosFromCenter = boxesPos.map(boxPos => Math.sqrt(Math.pow(boxPos.x-width/2,2) + Math.pow(boxPos.y-height/2,2)));
      return(mean(boxesPosFromCenter));
  }
  simInfo.meanBlueDisToCenter.push(calcMeanDistFromCenter(blueBoxesPos))
  simInfo.meanRedDisToCenter.push(calcMeanDistFromCenter(redBoxesPos))


    //Calculating the mean distance from the closest wall
    function calcMeanDistFromWall(boxesPos){
        height = simInfo.height;
        width = simInfo.width;
        wallThickness = 5;

        boxesPosFromWall = boxesPos.map(boxPos => Math.min(
            boxPos.x - wallThickness,
            boxPos.y - wallThickness,
            (height - wallThickness) - boxPos.y,
            (width - wallThickness) - boxPos.x));
        return(mean(boxesPosFromWall));
    }
    simInfo.meanBlueDisToClosestWall.push(calcMeanDistFromWall(blueBoxesPos))
    simInfo.meanRedDisToClosestWall.push(calcMeanDistFromWall(redBoxesPos))*/
}
function drawBoard() {
  var context = document.getElementById('arenaLemming').getContext('2d');
  context.fillStyle = "#444444";
  context.fillRect(0, 0, simInfo.width, simInfo.height);

  // draw objects within world
  const Composite = Matter.Composite,
        bodies = Composite.allBodies(simInfo.world);

  for (var bb = 0; bb < bodies.length; bb += 1) {
    var vertices = bodies[bb].vertices,
        vv;

    // draw all non-robot bodies here (walls and boxes)
    // don't draw robot's bodies here; they're drawn in plotRobot()
    if (bodies[bb].role != 'robot') {
      context.beginPath();
      context.moveTo(vertices[0].x, vertices[0].y);
      for (vv = 1; vv < vertices.length; vv += 1) {
        context.lineTo(vertices[vv].x, vertices[vv].y);
      }
      if (bodies[bb].color) {
        context.strokeStyle = convrgb(bodies[bb].color);
        context.closePath();
        context.stroke();
      }
    }
  }
  context.lineWidth = 1;

  // draw all robots
  for (var rr = 0; rr < robots.length; ++rr) {
    robots[rr].plotRobot(context);
  }
}

function repaintBay() {
  // update inset canvas showing information about selected robot
  const robotBay = document.getElementById('bayLemming'),
        context = robotBay.getContext('2d');
  context.clearRect(0, 0, robotBay.width, robotBay.height);
  simInfo.bayRobot.plotRobot(context, 10, 10);
  for (var ss = 0; ss < sensors.length; ss++) {
    sensors[ss].plotSensor(context);
  }

  // print sensor values of selected robot next to canvas
  if (!(simInfo.curSteps % 5)) {  // update slow enough to read
    var sensorString = '';
    const rsensors = simInfo.bayRobot.sensors;
    for (ss = 0; ss < rsensors.length; ss++) {
      sensorString += '<br> id \'' + rsensors[ss].id + '\': ' +
        padnumber(rsensors[ss].value, 2);
    }
    document.getElementById('SensorLabel').innerHTML = sensorString;
  }
}

function setRobotNumber(newValue) {
  var n;
  while (robots.length > newValue) {
    n = robots.length - 1;
    Matter.World.remove(simInfo.world, robots[n].body);
    robots[n] = null;
    robots.length = n;
  }

  while (robots.length < newValue) {
    if (newValue > RobotInfo.length) {
      console.warn('You request '+newValue+' robots, but only ' + RobotInfo.length +
                   ' are defined in RobotInfo!');
      toggleSimulation();
      return;
    }
    n = robots.length;
    robots[n] = makeInteractiveElement(new InstantiateRobot(RobotInfo[n]),
                                       document.getElementById("arenaLemming"));

    robots[n].onDrop = function(robot, event) {
      robot.isDragged = false;
    };

    robots[n].onDrag = function(robot, event) {
      	robot.isDragged = true;
        loadBay(robot);
        return true;
    };
  }
}


function padnumber(number, size) {
  if (number == Infinity) {
    return 'inf';
  }
  const s = "000000" + number;
  return s.substr(s.length - size);
}

function format(number) {
  // prevent HTML elements to jump around at sign flips etc
  return (number >= 0 ? '+' : '−') + Math.abs(number).toFixed(1);
}

function mean(numbers){
  sum = 0;
  for (i in numbers){
    sum = sum + numbers[i]
  }
  return sum/numbers.length
}

function toggleSimulation() {
  simInfo.doContinue = !simInfo.doContinue;
  if (simInfo.doContinue) {
    Matter.Runner.start(simInfo.runner, simInfo.engine);
  }
  else {
    Matter.Runner.stop(simInfo.runner);
  }
}


/*
CURRENTLY UNUSED, BUT MAYBE NEEDED LATER
 This is the code used to save files to the system. Comes from Github.
 Source: http://purl.eligrey.com/github/FileSaver.js/blob/master/FileSaver.js
 */
var saveAs=saveAs||function(e){"use strict";if(typeof e==="undefined"||typeof navigator!=="undefined"&&/MSIE [1-9]\./.test(navigator.userAgent)){return}var t=e.document,n=function(){return e.URL||e.webkitURL||e},r=t.createElementNS("http://www.w3.org/1999/xhtml","a"),o="download"in r,a=function(e){var t=new MouseEvent("click");e.dispatchEvent(t)},i=/constructor/i.test(e.HTMLElement)||e.safari,f=/CriOS\/[\d]+/.test(navigator.userAgent),u=function(t){(e.setImmediate||e.setTimeout)(function(){throw t},0)},s="application/octet-stream",d=1e3*40,c=function(e){var t=function(){if(typeof e==="string"){n().revokeObjectURL(e)}else{e.remove()}};setTimeout(t,d)},l=function(e,t,n){t=[].concat(t);var r=t.length;while(r--){var o=e["on"+t[r]];if(typeof o==="function"){try{o.call(e,n||e)}catch(a){u(a)}}}},p=function(e){if(/^\s*(?:text\/\S*|application\/xml|\S*\/\S*\+xml)\s*;.*charset\s*=\s*utf-8/i.test(e.type)){return new Blob([String.fromCharCode(65279),e],{type:e.type})}return e},v=function(t,u,d){if(!d){t=p(t)}var v=this,w=t.type,m=w===s,y,h=function(){l(v,"writestart progress write writeend".split(" "))},S=function(){if((f||m&&i)&&e.FileReader){var r=new FileReader;r.onloadend=function(){var t=f?r.result:r.result.replace(/^data:[^;]*;/,"data:attachment/file;");var n=e.open(t,"_blank");if(!n)e.location.href=t;t=undefined;v.readyState=v.DONE;h()};r.readAsDataURL(t);v.readyState=v.INIT;return}if(!y){y=n().createObjectURL(t)}if(m){e.location.href=y}else{var o=e.open(y,"_blank");if(!o){e.location.href=y}}v.readyState=v.DONE;h();c(y)};v.readyState=v.INIT;if(o){y=n().createObjectURL(t);setTimeout(function(){r.href=y;r.download=u;a(r);h();c(y);v.readyState=v.DONE});return}S()},w=v.prototype,m=function(e,t,n){return new v(e,t||e.name||"download",n)};if(typeof navigator!=="undefined"&&navigator.msSaveOrOpenBlob){return function(e,t,n){t=t||e.name||"download";if(!n){e=p(e)}return navigator.msSaveOrOpenBlob(e,t)}}w.abort=function(){};w.readyState=w.INIT=0;w.WRITING=1;w.DONE=2;w.error=w.onwritestart=w.onprogress=w.onwrite=w.onabort=w.onerror=w.onwriteend=null;return m}(typeof self!=="undefined"&&self||typeof window!=="undefined"&&window||this.content);if(typeof module!=="undefined"&&module.exports){module.exports.saveAs=saveAs}else if(typeof define!=="undefined"&&define!==null&&define.amd!==null){define("FileSaver.js",function(){return saveAs})}

function exportExcel()
{
    var data = [simInfo.meanRedDisToCenter, simInfo.meanBlueDisToCenter, simInfo.meanRedDisToClosestWall, simInfo.meanBlueDisToClosestWall];
    var keys = ['MeanRedDisToCenter', 'MeanBlueDisToCenter', 'MeanRedDisToClosestWall', 'MeanBlueDisToClosestWall'];

    var convertToCSV = function(data, keys) {
        var orderedData = [];
        for (var i = 0, iLen = data.length; i < iLen; i++) {
            temp = data[i];
            for (var j = 0, jLen = temp.length; j < jLen; j++) {

                quotes = ['"'+temp[j]+'"'];
                if (!orderedData[j]) {
                    orderedData.push([quotes]);
                } else {
                    orderedData[j].push(quotes);
                }
            }
        }
        return keys.join(',') + '\r\n' + orderedData.join('\r\n');
    }


    var str = convertToCSV(data, keys);
    var blob = new Blob([str], {type: "text/plain;charset=utf-8"});
    var d = new Date;
    var filename = 'Lemmings run ' + d.getMinutes() + "min" + d.getHours() + "h" + d.getDate()+ "d" + d.getMonth() + "m";
    saveAs(blob, [filename+'.csv']);
}
