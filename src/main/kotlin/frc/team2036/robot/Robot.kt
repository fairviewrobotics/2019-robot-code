package frc.team2036.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.PWMTalonSRX
import frc.team2036.robot.knightarmor.KnightBot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

import frc.team2036.robot.vision.linesensing.*;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;

class Robot : KnightBot() {

    lateinit var controller: XboxController
    lateinit var drivetrain: MecanumDrive
    lateinit var elevatorMotor: PWMTalonSRX
    lateinit var grabMotor1: PWMTalonSRX
    lateinit var grabMotor2: PWMTalonSRX

    lateinit var line_runner: VisionRunner

    lateinit var outputStream: CvSource



    override fun robotInit() {
        this.controller = XboxController(0)
        this.drivetrain = MecanumDrive(PWMTalonSRX(0), PWMTalonSRX(1), PWMTalonSRX(2), PWMTalonSRX(3))
        this.elevatorMotor = PWMTalonSRX(4)
        this.grabMotor1 = PWMTalonSRX(5)
        this.grabMotor2 = PWMTalonSRX(6)

        line_runner = VisionRunner(0, 120, 150, 0.007, 0.007, 0.005, 0.2, 0.2, 0.1, 45, 45, 8, 0.3, 0.3, 0.3, 0.3)
        line_runner.line_sensing.algorithm.setDownscaleSize(240, 180)
        line_runner.start()

        SmartDashboard.putNumber("vision-x-cof", 0.007)
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
        SmartDashboard.putNumber("vision-theta-x-adjust", 0.3)

        outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

    }


    override fun teleopPeriodic() {
        //vision test
        synchronized(this.line_runner) {
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

            outputStream.putFrame(this.line_runner.line_sensing.algorithm.blured);
        }


        //run drive train
        if(this.controller.getBButton()){
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
        } else {
            //run teleop
            this.drivetrain.driveCartesian(this.controller.getX(GenericHID.Hand.kLeft), -this.controller.getY(GenericHID.Hand.kLeft), this.controller.getX(GenericHID.Hand.kRight))
        }
        //run elevator
        this.elevatorMotor.speed = this.controller.getY(GenericHID.Hand.kRight) * 1.0;
        //this.drivetrain.driveCartesian(1.0, 0.0, 0.0);

        //run intake
        when {
            this.controller.getBumper(GenericHID.Hand.kLeft) -> {
                this.grabMotor1.speed = -1.0
                this.grabMotor2.speed = 1.0
            }
            this.controller.getBumper(GenericHID.Hand.kRight) -> {
                this.grabMotor1.speed = 1.0
                this.grabMotor2.speed = -1.0
            }
            else -> {
                this.grabMotor1.speed = 0.0
                this.grabMotor2.speed = 0.0
            }
    }


    }

}
