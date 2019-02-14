package frc.team2036.robot

import edu.wpi.first.wpilibj.Joystick
// import edu.wpi.first.wpilibj.TalonSRX
import edu.wpi.first.wpilibj.Spark
import frc.team2036.robot.knightarmor.KnightBot
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX

import frc.team2036.robot.vision.linesensing.*;

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

class Streaming: Thread() {


    lateinit var frontCamera: VideoCapture

    lateinit var frontCameraStream: CvSource
    lateinit var frontImageMat: Mat

    

    init {
        this.frontCameraStream = CameraServer.getInstance().putVideo("Camera", 320, 200);

        this.frontCamera = VideoCapture(0)

        if(!this.frontCamera.isOpened()){
            println("Failed to Open Camera")
        }
        else {
            println("Opened Camera\n");
        }

        this.frontImageMat = Mat()
    }


    fun stream(){
        if (this.frontCamera.isOpened()) {
            this.frontCamera.read(frontImageMat)
            Imgproc.resize(frontImageMat, frontImageMat, Size(320.0, 200.0))
            Imgproc.cvtColor(frontImageMat, frontImageMat, Imgproc.COLOR_BGR2GRAY);
            frontCameraStream.putFrame(frontImageMat)
        } else {
            //println("Not open")
        }
    }

    override fun run(){
        while(true){
            this.stream()
        }
    }

}
