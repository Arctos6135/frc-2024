package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.VisionInputs;

public class VisionIOUSB extends VisionIO {
    CvSink cvSink = new CvSink("opencv_USB Camera 0");

    Mat kernel = new Mat(3, 3, Imgproc.THRESH_BINARY);
    Mat hierarchy = new Mat();
    MatOfPoint2f point = new MatOfPoint2f();

    //UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    //MjpegServer mjpegServer = new MjpegServer("serve_USB Camera 0", 1181);


    List<MatOfPoint> matPoints = new ArrayList<>();

    double yaw;
    double pitch;
    boolean hasTarget = false;

    public VisionIOUSB() {
        Thread visionThread = new Thread(this::visionCommand);
        visionThread.setDaemon(true);
        visionThread.start();
    }
    
    public void visionCommand() {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        kernel.create(3, 3, Imgproc.THRESH_BINARY);
        kernel.setTo(new Scalar(new double[]{1.0, 1.0, 1.0}));
        // Set the resolution
        camera.setResolution(640, 480);

        // Get a CvSink. This will capture Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);
        // Mats are very memory expensive. Lets reuse these.
        var mat = new Mat();
        
        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) {
            System.out.println("Running thread");
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
            }

            System.out.println("Found frame");
            // hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV);

            mat.convertTo(mat, Imgproc.COLOR_BGR2HSV);

            Core.inRange(mat, new Scalar(new double[]{0.0, 0, 0}), new Scalar(new double[]{180, 255, 255}), mat);
            outputStream.putFrame(mat);
            try {
                mat.wait(1000, 0);
            } catch (Exception e) {
                // TODO: handle exception
            }


            Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_OPEN, kernel);
            matPoints.clear();
            Imgproc.findContours(mat, matPoints, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            System.out.println("Did countour processing");

            if (matPoints.isEmpty()) {
                hasTarget = false;
                continue;
            } else {
                hasTarget = true;
            }

            System.out.println("Found target");

            MatOfPoint largest = matPoints.get(0);
            for (MatOfPoint matOfPoint : matPoints) {
                if (Imgproc.contourArea(matOfPoint) > Imgproc.contourArea(largest)) {
                    largest = matOfPoint;
                }
            }

            largest.convertTo(point, CvType.CV_32FC2);
            RotatedRect rect = Imgproc.minAreaRect(point);
            
            // this needs to be changed
            double fieldLength = 100;

            // these might need to be negative
            // TODO: locking
            yaw = Math.atan(rect.center.x / fieldLength);
            pitch = Math.atan(rect.center.y / fieldLength);

            // Give the output stream a new image to display
            outputStream.putFrame(mat);
        }
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.hasTarget = hasTarget;
        inputs.isConnected = true;
        inputs.noteAngle = yaw;
        inputs.noteDistance = PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.cameraHeight,
            VisionConstants.targetHeight,
            VisionConstants.cameraPitch,
            Units.degreesToRadians(pitch)
        );
        inputs.latencyMillis = 0;
    }
}