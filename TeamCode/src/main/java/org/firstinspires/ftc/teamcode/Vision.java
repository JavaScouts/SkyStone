package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.VuMarkTarget;
import com.vuforia.VuMarkTargetResult;
import com.vuforia.VuMarkTemplate;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaPoseMatrix;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Deprecated
public class Vision {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY =
            "AdQfAyr/////AAABmUh6Z5KT20+RoULUpgxmoc9nIV2FKHL5EaGvj3PPgHtOujprWlIvVPgxtFaYImMYo175bgHUe+tHxxYynQmrgtrPcCBOIgpyptC6DCkr4lG4jZ59rDYEVPh+IUNKMWOgtphivaS+ZSclNCN2+uE40/oqQ0HuRLAGcxe/UviDbt6IafV2RkFFs412uP1E5XL/66hm46TahtlARJNQsKMTrxCNa8OFwvzC9ZW/ryimTGl46MdL9L6oI8JLHGm7GB7y7GS9GtqasKZvhgP4QCNgKHUDiC6urJ2BML9DO34qRY9zEELLG1fi92G4tB7P/0BsREjvNs28UNrXYrldXaJkAIK3pK2NJHNWFUuy1h7mgd+x";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private VuforiaTrackable trackable;

    private HardwareMap hardwareMap;
    private OpMode opMode;
    private Telemetry telemetry;

    Vision() {
    }

    void init(HardwareMap map, OpMode om) {
        hardwareMap = map;
        opMode = om;
        telemetry = om.telemetry;
    }


    void initVuforia() {

        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    void initTfod() {
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minimumConfidence = 0.8;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    void activateTfod() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    void shutdownTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    VuforiaTrackables getTargets() {
        return this.vuforia.loadTrackablesFromAsset("Skystone");
    }

    void loadTargetAsTemplate() {
        VuforiaTrackables trackables = getTargets();
        trackable = trackables.get(0);
        trackable.setName("Skystone Target"); // can help in debugging; otherwise not necessary
    }

    void activateTargets(VuforiaTrackables targets) {
        if(targets != null) {
            targets.activate();
        }
    }

    VuforiaTrackable getTrackable() {
        return trackable;
    }

    boolean getIsTargetSeen() {
        boolean vis = false;
        if (trackable != null) {
            vis = ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible();
        } else {
            telemetry.addLine("Trackable is null");
            telemetry.update();
        }
        return vis;
    }

    List<Recognition> getRecognitions() {
        if (tfod != null) {
            return tfod.getUpdatedRecognitions();
        }
        return null;
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    Recognition getFirstSkystoneSeen() {
        if (tfod != null) {
            List<Recognition> rec = getRecognitions();
            if(rec != null) {
                Collections.sort(rec, new Comparator<Recognition>() {
                    @Override
                    public int compare(Recognition r1, Recognition r2) {
                        return Integer.valueOf((int) r1.getLeft()).compareTo((int) r2.getLeft());
                    }
                });
                for (Recognition recognition : rec) {
                    if (recognition.getLabel().equals("Skystone")) {
                        return recognition;
                    }
                }
            }
        }
        return null;
    }

    String whatIsSeen() {
        String result = "Currently seen: ";
        if (tfod != null) {
            List<Recognition> rec = getRecognitions();
            if(rec != null) {
                Collections.sort(rec, new Comparator<Recognition>() {
                    @Override
                    public int compare(Recognition r1, Recognition r2) {
                        return Integer.valueOf((int) r1.getLeft()).compareTo((int) r2.getLeft());
                    }
                });
                for (Recognition recognition : rec) {

                    result+="a "+recognition.getLabel()+" at "+String.valueOf(recognition.getLeft())+" pixels ";

                }
            }
        }
        return null;
    }

    int numberOfRecognitions() {

        if(tfod != null) {
            List<Recognition> bunch = getRecognitions();
            if(bunch != null) {
                return bunch.size();
            }
        }
        return 0;
    }




}