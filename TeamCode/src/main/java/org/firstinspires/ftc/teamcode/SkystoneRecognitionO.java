package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionUtils.NewSkyStonePipeline;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name = "Skystone Recognition-O")
public class SkystoneRecognitionO extends LinearOpMode {

    OpenCvCamera webcam;
    NewSkyStonePipeline newSkyStonePipeline;
    int stone = -1;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        newSkyStonePipeline = new NewSkyStonePipeline();
        newSkyStonePipeline.setView_source(1);
        webcam.setPipeline(newSkyStonePipeline);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        newSkyStonePipeline.setView_source(3);
        boolean found = false;
        ElapsedTime et = new ElapsedTime();
        et.reset();
        while (!isStopRequested() && !found) {
            try {
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
                    } catch (IndexOutOfBoundsException e) {
                        e.printStackTrace();
                    }

                    if (boundRect.length == 0 && et.seconds() > 2.5) {
//                        telemetry.log().add("Skystone is in pos 3, 6.");
                        stone = 3;
                        found = true;
                    } else {
                        telemetry.log().add("Skystone x:" + boundRect[0].x);
                        if (boundRect[0].x > 217) {
//                            telemetry.log().add("Skystone is in pos 1, 4.");
                            stone = 1;
                            found = true;
                        } else {
//                            telemetry.log().add("Skystone is in pos 2, 5.");
                            stone = 2;
                            found = true;
                        }
                    }

                } catch (NullPointerException e) {
                    e.printStackTrace();
                }
            } catch (ArrayIndexOutOfBoundsException e) {
                e.printStackTrace();
            }
        }
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        waitForStart();
        telemetry.log().clear();

        while(opModeIsActive()) {
            telemetry.addData("Skystone",stone);
            telemetry.update();
        }

    }

}
