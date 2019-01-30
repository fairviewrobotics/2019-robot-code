package frc.team2036.robot.vision.linesensing;


/* VisionRunner: run the line sensing algorithm in a different thread, and calculate needed x, y, and theta movement
    theta target assumed to be zero*/

class VisionRunner(val camera_index: Int, public var x_target: Int, public var y_target: Int, public var x_cof: Double, public var y_cof: Double, public var theta_cof: Double, public var x_min: Double, public var y_min: Double, public var theta_min: Double,  public var x_dead: Int, public var y_dead: Int, public var theta_dead: Int, public var x_max: Double, public var y_max: Double, public var theta_max: Double): Thread() {

    public var line_sensing: LineSense;

    init {
        line_sensing = LineSense()
        line_sensing.openCamera(camera_index)
    }

    public override fun run(){
        while(true){
            this.line_sensing.runAlgorithm();
        }
    }

    //get dx, dy, and dtheta

    public fun getDX(): Double {
        synchronized(this.line_sensing){

            var dx: Double;

            dx = (this.x_target - this.line_sensing.algorithm.centroid_x)

            if(Math.abs(dx) > this.x_dead){
                var speed: Double =
                    Math.signum(dx) * ( this.x_min +
                    (this.x_cof * Math.abs(dx-this.x_dead)))

                return minOf(maxOf(speed, -this.x_max), this.x_max)

            }

            return 0.0;

        }
    }

    public fun getDY(): Double {
        synchronized(this.line_sensing){
            /* var dy: Double;

            dy = (this.y_target - this.line_sensing.algorithm.centroid_y)

            if(Math.abs(dy) > this.y_dead){
                var speed: Double =
                    Math.signum(dy) * ( this.y_min +
                    (this.y_cof * Math.abs(dy-this.y_dead)))

                return minOf(maxOf(speed, -this.y_max), this.y_max)
            }

            return 0.0;*/
            return this.y_min;
        }
    }

    public fun getDTheta(): Double {
        synchronized(this.line_sensing){
            var real_angle: Double = this.line_sensing.algorithm.angle;
            if(real_angle > 90.0){
                real_angle -= 180.0;
            }

            println(real_angle)

            if(Math.abs(real_angle) > this.theta_dead){
                return Math.signum(real_angle) * this.theta_min;
            }

            return 0.0
        }
    }

}