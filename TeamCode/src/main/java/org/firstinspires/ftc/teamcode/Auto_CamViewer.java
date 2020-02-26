package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name="Align")
public class Auto_CamViewer extends LinearOpMode {

    OpenCvCamera webcam;
    NewSkyStonePipeline newSkyStonePipeline;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        newSkyStonePipeline = new NewSkyStonePipeline();
        newSkyStonePipeline.setView_source(3);
        webcam.setPipeline(newSkyStonePipeline);
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        telemetry.log().add("ready - press play");

        while(!isStopRequested() && !isStarted()) {

            try {
                ArrayList<MatOfPoint> contours = newSkyStonePipeline.convexHullsOutput();
                MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
                Rect[] boundRect = new Rect[contours.size()];
                try {
                    try {
                        for (int i = 0; i < contours.size(); i++) {
                            contoursPoly[i] = new MatOfPoint2f();
                            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                        }
                    } catch(NullPointerException e) {
                        e.printStackTrace();
                    }
                } catch(IndexOutOfBoundsException e) {
                    e.printStackTrace();
                }

                try {
                    telemetry.addData("Skystones found:", boundRect.length);
                    if(boundRect.length == 0) {
                        telemetry.addLine("Skystone is in pos 3, 6.");
                    } else {
                        if(boundRect[0].x >= 236) {
                            telemetry.addLine("Skystone is in pos 1, 4.");
                        } else {
                            telemetry.addLine("Skystone is in pos 2, 5.");
                        }
                    }
                    if(boundRect[0] != null) {
                        telemetry.addData("Skystone x", boundRect[0].x);
                    } else {
                        telemetry.addData("Skystone x", "DNE");
                    }
                    telemetry.update();

                } catch(NullPointerException e) {
                    e.printStackTrace();
                }
            } catch(ArrayIndexOutOfBoundsException e) {
                e.printStackTrace();
            }
        }
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        telemetry.log().add("aight, imma head out.");
    }


}
