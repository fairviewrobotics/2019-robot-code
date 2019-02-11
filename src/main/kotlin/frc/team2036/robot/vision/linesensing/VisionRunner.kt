package frc.team2036.robot.vision.linesensing;
import frc.team2036.robot.knightarmor.*;


/* VisionRunner: run the line sensing algorithm in a different thread, and calculate needed x, y, and theta movement
    theta target assumed to be zero*/

class VisionRunner(val camera_index: Int, public var x_target: Int, public var y_target: Int, public var x_cof: Double, public var y_cof: Double, public var theta_cof: Double, public var x_min: Double, public var y_min: Double, public var theta_min: Double,  public var x_dead: Int, public var y_dead: Int, public var theta_dead: Int, public var x_max: Double, public var y_max: Double, public var theta_max: Double, public var theta_x_adjust: Double): Thread() {

    public var dx: Double = 0.0
    public var dy: Double = 0.0
    public var dt: Double = 0.0

    public var line_sensing: LineSense

    public var camera_open: Boolean = false

    init {
        line_sensing = LineSense()
        line_sensing.openCamera(camera_index)
        camera_open = line_sensing.cam.isOpened()
        if(camera_open){
            KnightScribe.log("Line Tracking Camera Opened\n", KnightScribeLogLevel.INFO)
        } else {
            KnightScribe.log("Camera Failed to Open. Line Tracking is Disabled.", KnightScribeLogLevel.WARNING)
        }
        
    }

    public override fun run(){
        while(camera_open){
            this.line_sensing.runAlgorithm();
        }
    }

    //calculate dx, dy, and dtheta
    public fun calculate_dvars(){
        if(!this.camera_open){
            this.dx = 0.0
            this.dy = 0.0
            this.dt = 0.0
            return;
        }
        synchronized(this.line_sensing){

            var diffTheta = this.getDiffTheta()

            /* apply angle */
            if(Math.abs(diffTheta) > this.theta_dead){
                this.dt = Math.signum(diffTheta) * ( this.theta_min +
                    (this.theta_cof * Math.abs(diffTheta)))

                //adjust x for theta
                this.dx = this.dt * this.theta_x_adjust
                this.dy = 0.0

            }
            else {
                /* apply dx and dy */
                this.dt = 0.0

                this.dx = this.getDXElem()
                this.dy = this.getDYElem()
            }

        }
    }

    //get dx, dy, and dtheta

    fun getDXElem(): Double {
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

    fun getDYElem(): Double {
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

    fun getDiffTheta(): Double {
        synchronized(this.line_sensing){
            var real_angle: Double = this.line_sensing.algorithm.angle;
            if(real_angle > 90.0){
                real_angle -= 180.0;
            }

            return real_angle
        }
    }

}