package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.VisionUtils.NewSkyStonePipeline;
import org.firstinspires.ftc.teamcode.VisionUtils.StonePipeline;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.nio.ByteBuffer;
import java.util.ArrayList;

@TeleOp(name="Skystone Recognition-O")
public class SkystoneRecognitionO extends LinearOpMode {

    OpenCvCamera webcam;
    NewSkyStonePipeline newSkyStonePipeline;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        newSkyStonePipeline = new NewSkyStonePipeline();
        newSkyStonePipeline.setView_source(1);
        webcam.setPipeline(newSkyStonePipeline);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        waitForStart();

        newSkyStonePipeline.setView_source(2);
        while (opModeIsActive())
        {
            try {
                ArrayList<MatOfPoint> contours = newSkyStonePipeline.convexHullsOutput();
                MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
                Rect[] boundRect = new Rect[contours.size()];
                try {
                    for (int i = 0; i < contours.size(); i++) {
                        contoursPoly[i] = new MatOfPoint2f();
                        Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                        boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                    }
                } catch(IndexOutOfBoundsException e) {
                    e.printStackTrace();
                }

                try {
                    telemetry.addData("Skystones found:", boundRect.length);
                    for (int i = 0; i < boundRect.length; i++) {
                        telemetry.addData("Left X, " + i, boundRect[i].x);
                    }
                    telemetry.update();
                    sleep(100);
                } catch(NullPointerException e) {
                    e.printStackTrace();
                }
            } catch(ArrayIndexOutOfBoundsException e) {
                e.printStackTrace();
            }
        }

    }

}
