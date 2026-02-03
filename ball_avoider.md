### the likely impossible concept of making the robot avoid ramp balls

#### How would this need to be done?

* first: object detection detects balls - tyler has already done a fair bit of work on this
	* the challenge, of course, would be mapping these to 3d space, though our AprilTag software might have cooked up a part of this
* Second: figure out robot weights, at what point will the robot properly fall onto the slope? - moment of inertia might be part of this?
* third, by figuring out where the robot's edges would end up, creating a direct path toward the first working area where the robot can fall down without touching the ball - basically, find start-point, endpoint, and then create a curve between them so that the edge won't go over a ball
	* Again, the challenge is mainly doing this all in 3D space rather than 2D space
* from there, we just command the swerve module to run the motors according to our path (we might have to calculate the path manually, since we're in 3D space
