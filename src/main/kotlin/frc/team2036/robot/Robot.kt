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

    val timer: Timer
    var elevatorTime: Double = 0.0
    var pElevatorButton: Boolean = false
    var autoStartEnabled: Boolean = false
    lateinit var encoder: Encoder

    lateinit var elevator: Elevator

    init {
        timer = Timer()
    }


    fun init_system(){

        this.controller0 = Joystick(0)
        this.controller1 = Joystick(1)
        this.drivetrain = MecanumDrive(WPI_TalonSRX(1), WPI_TalonSRX(2), WPI_TalonSRX(3), WPI_TalonSRX(4))
        this.rearElevatorMotor = Spark(8)
        this.grabMotor1 = WPI_VictorSPX(11)
        this.grabMotor2 = Talon(7)
        this.hatchIntake = Talon(9)

        this.elevatorMotor = WPI_TalonSRX(20)
        this.encoder = Encoder(0, 1, false, EncodingType.k4X)
        encoder.reset()

        this.elevator = Elevator(elevatorMotor, encoder)
    

        streaming = Streaming()
        streaming.start()

    }

    override fun robotInit() {
        this.init_system()
        elevator.reset()
    }

    override fun autonomousInit() {
        elevator.reset()
    }

    fun mainLoop() {
        //println(encoder.get() - elevator.offset)
       
            val speed: Double = (this.controller1.getZ() - 1.0) * -0.5
            this.drivetrain.driveCartesian(-this.controller0.getX()*speed, -this.controller0.getY()*speed, this.controller1.getX()*speed)


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
            controller1.getRawButton(10) -> {
                this.elevator.setEncoder(-194)
            }
            controller1.getRawButton(11) -> {
                this.elevator.setEncoder(2)
            }
            controller1.getRawButton(7) -> {
                this.elevator.setEncoder(-132)
            }
            controller1.getRawButton(8) -> {
                this.elevator.setEncoder(62)
            }
            controller1.getRawButton(9) -> {
                this.elevator.setEncoder(188)
            }
            this.controller1.getRawButton(3) -> {
                this.elevator.setSpeed(-1.0)
            }
            this.controller1.getRawButton(2) -> {
                this.elevator.setSpeed(1.0)
            }
            else -> {
                this.elevator.setNone()
            }
        }

        elevator.run()

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
        }

        if(this.controller0.getRawButton(10) && !pElevatorButton){
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
        pElevatorButton = this.controller0.getRawButton(10)

    }

    override fun teleopPeriodic(){
        this.mainLoop()
    }

     override fun autonomousPeriodic(){
        this.mainLoop()
    }


}
