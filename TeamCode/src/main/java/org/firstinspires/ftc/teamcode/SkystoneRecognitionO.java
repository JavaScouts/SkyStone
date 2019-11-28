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
import org.firstinspires.ftc.teamcode.VisionUtils.StonePipeline;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;

@TeleOp(name="Skystone Recognition-O")
public class SkystoneRecognitionO extends LinearOpMode {

    private static StonePipeline p = new StonePipeline();
    private VuforiaLocalizer vuforia;
    private Thread visionThread;
    private final Object imgLock = new Object();
    private double area = 0.0;
    private double leftBound = 0.0;

    private static final String VUFORIA_KEY =
            "AdQfAyr/////AAABmUh6Z5KT20+RoULUpgxmoc9nIV2FKHL5EaGvj3PPgHtOujprWlIvVPgxtFaYImMYo175bgHUe+tHxxYynQmrgtrPcCBOIgpyptC6DCkr4lG4jZ59rDYEVPh+IUNKMWOgtphivaS+ZSclNCN2+uE40/oqQ0HuRLAGcxe/UviDbt6IafV2RkFFs412uP1E5XL/66hm46TahtlARJNQsKMTrxCNa8OFwvzC9ZW/ryimTGl46MdL9L6oI8JLHGm7GB7y7GS9GtqasKZvhgP4QCNgKHUDiC6urJ2BML9DO34qRY9zEELLG1fi92G4tB7P/0BsREjvNs28UNrXYrldXaJkAIK3pK2NJHNWFUuy1h7mgd+x";

    @Override
    public void runOpMode(){

        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);

        visionThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(opModeIsActive()) {

                    try {
                        Image i = vuforia.getFrameQueue().take().getImage(0);
                        if(i == null) {
                            continue;
                        }
                        ByteBuffer bb = i.getPixels();
                        if(bb == null) {
                            continue;
                        }
                        byte[] b = new byte[bb.capacity()];
                        bb.get(b);
                        telemetry.addData("Byte[] created with len",b.length);
                        Mat m = new Mat(i.getHeight(), i.getWidth(), CvType.CV_8UC3);
                        m.put(0,0,b);
                        telemetry.addLine("Mat created with byte info.");
                        p.process(m);
                        telemetry.addLine("Mat processed.");
                        for(MatOfPoint mop : p.convexHullsOutput()) {
                            Rect r = Imgproc.boundingRect(mop);
                            synchronized (imgLock) {
                                area = r.area();
                                leftBound = r.x;
                            }
                            telemetry.addData("Area of "+String.valueOf(p.convexHullsOutput().indexOf(mop)), area);
                            telemetry.addData("X of "+String.valueOf(p.convexHullsOutput().indexOf(mop)), leftBound);
                        }
                        telemetry.update();

                    } catch (InterruptedException e) {
                        e.printStackTrace();
                        requestOpModeStop();
                    }
                }

            }
        });
//        visionThread.start();

        waitForStart();

        while (opModeIsActive()) {
            try {
                Image i = vuforia.getFrameQueue().take().getImage(0);
                if(i == null) {
                    telemetry.addLine("Image missing");
                    telemetry.update();
                    continue;
                }
                ByteBuffer bb = i.getPixels();
                if(bb == null) {
                    telemetry.addLine("No pixels in image");
                    telemetry.update();
                    continue;
                }
                byte[] b = new byte[bb.capacity()];
                bb.get(b);
                telemetry.addData("Byte[] created with len",b.length);
                Mat m = new Mat(i.getHeight(), i.getWidth(), CvType.CV_8UC4);
                m.put(0,0,b);
                telemetry.addLine("Mat created with byte info.");
                telemetry.addData("First pixel data",m.get(0,0).toString());
                p.process(m);
                telemetry.addLine("Mat processed.");
                telemetry.addData("contours found",p.findContoursOutput().size());
                for(MatOfPoint mop : p.convexHullsOutput()) {
                    Rect r = Imgproc.boundingRect(mop);
                    area = r.area();
                    leftBound = r.x;
                    telemetry.addData("Area of "+String.valueOf(p.convexHullsOutput().indexOf(mop)), area);
                    telemetry.addData("X of "+String.valueOf(p.convexHullsOutput().indexOf(mop)), leftBound);
                }
                telemetry.update();

            } catch (InterruptedException e) {
                e.printStackTrace();
                requestOpModeStop();
            }
        }

    }

}
