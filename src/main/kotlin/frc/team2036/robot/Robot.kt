package frc.team2036.robot

import edu.wpi.first.wpilibj.Joystick
// import edu.wpi.first.wpilibj.TalonSRX
import edu.wpi.first.wpilibj.Spark
import frc.team2036.robot.knightarmor.KnightBot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.Preferences
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.CounterBase.EncodingType

import edu.wpi.first.wpilibj.Talon

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;

// OPENCV
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;


import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;

class Robot : KnightBot() {
    //left joystick
    lateinit var controller0: Joystick
    //right joystick
    lateinit var controller1: Joystick
    lateinit var drivetrain: MecanumDrive
    lateinit var elevatorMotor: WPI_TalonSRX
    //lateinit var elevator: Elevator
    lateinit var rearElevatorMotor: Spark
    lateinit var grabMotor1: WPI_VictorSPX
    lateinit var grabMotor2: Talon
    lateinit var hatchIntake: Talon

    //lateinit var line_runner: VisionRunner

    lateinit var streaming: Streaming

    lateinit var frontCamera: VideoCapture

    lateinit var outputStream: CvSource

    lateinit var frontCameraStream: CvSource
    lateinit var frontImageMat: Mat

    lateinit var elevatorPos: DoubleArray
    val numElevatorPos: Int = 6

    val timer: Timer
    var elevatorTime: Double = 0.0
    var pElevatorButton: Boolean = false
    var autoStartEnabled: Boolean = false
    lateinit var encoder: Encoder

    init {
        timer = Timer()
    }


    fun init_system(){

        this.controller0 = Joystick(0)
        this.controller1 = Joystick(1)
        this.drivetrain = MecanumDrive(WPI_TalonSRX(1), WPI_TalonSRX(2), WPI_TalonSRX(3), WPI_TalonSRX(4))
        this.elevatorMotor = WPI_TalonSRX(20)
        //elevator = Elevator(encoder, 20)
        this.rearElevatorMotor = Spark(8)
        this.grabMotor1 = WPI_VictorSPX(11)
        this.grabMotor2 = Talon(7)
        this.hatchIntake = Talon(9)

        this.elevatorPos = DoubleArray(numElevatorPos)
        this.encoder = Encoder(0, 1, false, EncodingType.k4X)

        //line_runner = VisionRunner(0, 120, 150, 0.007, 0.007, 0.005, 0.2, 0.2, 0.1, 45, 45, 8, 0.3, 0.3, 0.3, 0.3)
        //line_runner.line_sensing.algorithm.setDownscaleSize(240, 180)
        //line_runner.start()

        /* SmartDashboard.putNumber("vision-x-cof", 0.007)
        SmartDashboard.putNumber("vision-y-cof", 0.007)
        SmartDashboard.putNumber("vision-theta-cof", 0.005)

        SmartDashboard.putNumber("vision-x-max", 0.3)
        SmartDashboard.putNumber("vision-y-max", 0.3)
        SmartDashboard.putNumber("vision-theta-max", 0.3)

        SmartDashboard.putNumber("vision-x-dead", 45.0)
        SmartDashboard.putNumber("vision-y-dead", 45.0)
        SmartDashboard.putNumber("vision-theta-dead", 8.0)

        SmartDashboard.putNumber("vision-x-min", 0.2)
        SmartDashboard.putNumber("vision-y-min", 0.2)
        SmartDashboard.putNumber("vision-theta-min", 0.2)
        SmartDashboard.putNumber("vision-theta-x-adjust", 0.3)*/

        streaming = Streaming()
        streaming.start()

        readElevatorVals()
    }

    /* Read in the elevator encoder values from the smartDashboard prefrences */
    fun readElevatorVals(){
        val prefs = Preferences.getInstance()
        for(ind in 0..(numElevatorPos -1)){
            elevatorPos[ind] = prefs.getDouble("pos$ind", 1.0)
        }

    }

    override fun robotInit() {
        this.init_system()
    }

    fun mainLoop() {
        println(encoder.get())
        //this.frontImageMat.clear()

        //vision test
        /* synchronized(this.line_runner) {
            line_runner.x_cof = SmartDashboard.getNumber("vision-x-cof", 0.007)
            line_runner.y_cof = SmartDashboard.getNumber("vision-y-cof", 0.007)
            line_runner.theta_cof = SmartDashboard.getNumber("vision-theta-cof", 0.005)

            line_runner.x_max = SmartDashboard.getNumber("vision-x-max", 0.2)
            line_runner.y_max = SmartDashboard.getNumber("vision-y-max", 0.2)
            line_runner.theta_max = SmartDashboard.getNumber("vision-theta-max", 0.1)

            line_runner.x_min = SmartDashboard.getNumber("vision-x-min", 0.2)
            line_runner.y_min = SmartDashboard.getNumber("vision-y-min", 0.2)
            line_runner.theta_min = SmartDashboard.getNumber("vision-theta-min", 0.1)

            line_runner.x_dead = SmartDashboard.getNumber("vision-x-dead", 45.0).toInt()
            line_runner.y_dead = SmartDashboard.getNumber("vision-y-dead", 45.0).toInt()
            line_runner.theta_dead = SmartDashboard.getNumber("vision-theta-dead", 8.0).toInt()

            line_runner.theta_x_adjust = SmartDashboard.getNumber("vision-theta-x-adjust", 0.3)

            if(line_runner.camera_open){
                outputStream.putFrame(this.line_runner.line_sensing.algorithm.blured)
            }
    }*/

        /*if (this.frontCamera.isOpened()) {
            this.frontCamera.read(frontImageMat)
            Imgproc.resize(frontImageMat, frontImageMat, Size(320.0, 200.0))
            Imgproc.cvtColor(frontImageMat, frontImageMat, Imgproc.COLOR_BGR2GRAY);
            frontCameraStream.putFrame(frontImageMat)
        } else {
            //println("Not open")
        }*/


        //run drive train
        /* if(this.controller.getBButton()){
            var dx: Double = 0.0
            var dy: Double = 0.0
            var dt: Double = 0.0

            synchronized(this.line_runner) {
                if(this.controller.getXButton()){
                    dx = -this.line_runner.dx
                }
                if(this.controller.getYButton()){
                    dy = this.line_runner.dy
                }
                if(this.controller.getAButton()){
                    dt = this.line_runner.dt
                }

                //run alignment
                this.drivetrain.driveCartesian(dx, dy, dt);
                    //this.drivetrain.driveCartesian(0.0, 0.0, this.line_runner.getDTheta());
                }
        } else {*/
            //run teleop
            val speed: Double = (this.controller1.getZ() - 1.0) * -0.5
            this.drivetrain.driveCartesian(-this.controller0.getX()*speed, -this.controller0.getY()*speed, this.controller1.getX()*speed)
        //}

        //run intake

        when {
            this.controller1.getRawButton(5) -> {
                this.hatchIntake.set(1.0)
            }
            this.controller1.getRawButton(4) -> {
                this.hatchIntake.set(-1.0)
            }
            else -> {
                this.hatchIntake.set(0.0)
            }
        }

        /* run elevator */

        when {
            controller0.getRawButton(10) -> {
                val diff = -100 - encoder.get()

                if (diff < -5) {
                    elevatorMotor.set(-1.0)
                } else if (diff > 5) {
                    elevatorMotor.set(1.0)
                } else {
                    elevatorMotor.set(0.0)
                }

            }
            this.controller1.getRawButton(3) -> {
                this.elevatorMotor.set(-1.0)
            }
            this.controller1.getRawButton(2) -> {
                this.elevatorMotor.set(1.0)
            }
            else -> {
                this.elevatorMotor.set(0.0)
            }
        }

        /* run rear elevator */

        when {
            this.controller0.getRawButton(3) -> {
                this.rearElevatorMotor.set(1.0)
            }
            this.controller0.getRawButton(2) -> {
                this.rearElevatorMotor.set(-1.0)
            }
            else -> {
                this.rearElevatorMotor.set(0.0)
            }
        }

        /* run ball intake */

        when {
            this.controller0.getRawButton(1) -> {
                this.grabMotor1.set(-1.0)
                this.grabMotor2.set(1.0)
            }
            this.controller1.getRawButton(1) -> {
                this.grabMotor1.set(0.5)
                this.grabMotor2.set(-0.5)
            }
            else -> {
                this.grabMotor1.set(0.0)
                this.grabMotor2.set(0.0)
            }
        }

        if(this.controller0.getRawButton(6)){
            streaming.stop()
            streaming = Streaming()
            streaming.start()
            readElevatorVals()
        }

        if(this.controller0.getRawButton(7) && !pElevatorButton){
            timer.reset()
            timer.start()
            autoStartEnabled = true
        }

        if(autoStartEnabled) {
            elevatorTime = timer.get()
            if (elevatorTime<2.0) {
                this.hatchIntake.set(-1.0)
                this.rearElevatorMotor.set(1.0)
                this.elevatorMotor.set(1.0)
            } else {
                this.hatchIntake.set(0.0)
                this.rearElevatorMotor.set(0.0)
                this.elevatorMotor.set(0.0)
                autoStartEnabled = false
            }
        }
        pElevatorButton = this.controller0.getRawButton(7)

    }

    override fun teleopPeriodic(){
        this.mainLoop()
    }

     override fun autonomousPeriodic(){
        this.mainLoop()
    }


}
