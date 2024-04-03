package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOUSB {

    
    Mat kernel = new Mat();

    UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    MjpegServer mjpegServer = new MjpegServer("serve_USB Camera 0", 1181);

    CvSink cvSink = new CvSink("opencv_USB Camera 0");

    List<MatOfPoint> matPoints = new ArrayList<>();

    double yaw;
    double pitch;
    Object mutex;
    
    public VisionIOUSB() {

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
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
            }
            // hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV);

            mat.convertTo(mat, Imgproc.COLOR_BGR2HSV);
            Core.inRange(mat, new Scalar(new double[]{0.0, 0.0, 0.0}), new Scalar(new double[]{0.0, 0.0, 0.0}), mat);
            Imgproc.morphologyEx(mat, mat, Imgproc.MORPH_OPEN, kernel);
            Imgproc.findContours(mat, matPoints, null, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint largest = matPoints.get(0);
            for (MatOfPoint matOfPoint : matPoints) {
                if (Imgproc.contourArea(matOfPoint) > Imgproc.contourArea(largest)) {
                    largest = matOfPoint;
                }
            }

            MatOfPoint2f point = new MatOfPoint2f();
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
}
