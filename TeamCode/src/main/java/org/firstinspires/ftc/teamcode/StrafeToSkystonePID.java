package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name="Find Skystone via PID")
public class StrafeToSkystonePID extends LinearOpMode {

    private ModernRoboticsI2cGyro g;
    private ModernRoboticsI2cColorSensor c;
    private ModernRoboticsI2cRangeSensor rn;
    private DcMotor l, r, bl, br, c1, c2;
    private Servo hl;
    private static final double WHEEL_RADIUS = 2.98;
    private static final double CENTER_TO_WHEEL = 8.53;
    private static final double COUNTS_PER_MOTOR_REV = 537.6;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 2.95;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double PI = 3.1415;
    private double scale2 = 2.35;
    private double scale3 = 0.1;

    private Hardware h = new Hardware();
    private Vision v = new Vision();
    private ElapsedTime runtime = new ElapsedTime();

    private VuforiaLocalizer vuforia;
    private VuforiaTrackables trackables;

    private static final String VUFORIA_KEY =
            "AdQfAyr/////AAABmUh6Z5KT20+RoULUpgxmoc9nIV2FKHL5EaGvj3PPgHtOujprWlIvVPgxtFaYImMYo175bgHUe+tHxxYynQmrgtrPcCBOIgpyptC6DCkr4lG4jZ59rDYEVPh+IUNKMWOgtphivaS+ZSclNCN2+uE40/oqQ0HuRLAGcxe/UviDbt6IafV2RkFFs412uP1E5XL/66hm46TahtlARJNQsKMTrxCNa8OFwvzC9ZW/ryimTGl46MdL9L6oI8JLHGm7GB7y7GS9GtqasKZvhgP4QCNgKHUDiC6urJ2BML9DO34qRY9zEELLG1fi92G4tB7P/0BsREjvNs28UNrXYrldXaJkAIK3pK2NJHNWFUuy1h7mgd+x";
    private VuforiaTrackable trackable;

    @Override
    public void runOpMode() {

        h.init(hardwareMap, this);
        //setupVision();
        l = h.leftDrive;
        r = h.rightDrive;
        bl = h.backLDrive;
        br = h.backRDrive;
        c1 = h.Collec1;
        c2 = h.Collec2;
        hl = h.hookLeft;
        g = h.gyro;
        c = h.color;
        rn = h.range;
        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.log().add("Hardware initialized.");

        sleep(120);
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        g.calibrate();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (!isStopRequested() && g.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
        waitForStart();

        pidDrive(0.5, -100,0, 10, 0.05,0);
        sleep(500);
        pidDrive(0.5,100,0,10,0.05, 0);


    }

    void pidDrive(double powerLimit, double x, double rot, double timeoutS, double correction, int offset) {

        rot = 0;
        double y = 0;
        if (opModeIsActive()) {

            double reciprocal_radius = 1 / WHEEL_RADIUS;
            double lr = reciprocal_radius * (x - y - (rot * (2 * CENTER_TO_WHEEL)));
            double rr = reciprocal_radius * (x + y + (rot * (2 * CENTER_TO_WHEEL)));
            double blr = reciprocal_radius * (x + y - (rot * (2 * CENTER_TO_WHEEL)));
            double brr = reciprocal_radius * (x - y + (rot * (2 * CENTER_TO_WHEEL)));
            double l_count = l.getCurrentPosition() + (lr * scale2 * COUNTS_PER_INCH);
            double r_count = r.getCurrentPosition() + (rr * scale2 * COUNTS_PER_INCH);
            double bl_count = bl.getCurrentPosition() + (blr * scale2 * COUNTS_PER_INCH);
            double br_count = br.getCurrentPosition() + (brr * scale2 * COUNTS_PER_INCH);
            double l_power = map(lr * scale3, -l_count * scale3, l_count * scale3, powerLimit, powerLimit);
            double r_power = map(rr * scale3, -r_count * scale3, r_count * scale3, powerLimit, powerLimit);
            double bl_power = map(blr * scale3, -bl_count * scale3, bl_count * scale3, powerLimit, powerLimit);
            double br_power = map(brr * scale3, -br_count * scale3, br_count * scale3, powerLimit, powerLimit);
            l.setTargetPosition((int) l_count);
            r.setTargetPosition((int) r_count);
            bl.setTargetPosition((int) bl_count);
            br.setTargetPosition((int) br_count);

            h.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            l.setPower(l_power);
            r.setPower(r_power);
            bl.setPower(bl_power);
            br.setPower(br_power);

            while (opModeIsActive() &&
                    (l.isBusy() || r.isBusy() || bl.isBusy() || br.isBusy()) &&
                    (runtime.seconds() < timeoutS)) {

                // adjust angle from 0 > 360 to -180 to 180 so that 0 is straight ahead
                int rawZ = g.getHeading() +offset;
                int adjustedZ = 0;
                if(rawZ > 180 + offset) {
                    adjustedZ = rawZ - 360;
                } else {
                    adjustedZ = rawZ;
                }


                telemetry.addData("Powers", "Powers are %7f :%7f :%7f :%7f", l.getPower(), r.getPower(), bl.getPower(), br.getPower());
                telemetry.addData("Gyro-raw",rawZ);
                telemetry.addData("Gyro-adj",adjustedZ);

                if(adjustedZ < rot) {

                    l.setPower(l.getPower() + correction);
                    bl.setPower(bl.getPower() + correction);
                    r.setPower(r.getPower() - correction);
                    br.setPower(br.getPower() - correction);
                    telemetry.addLine("Correcting for leftwards drift.");

                }
                if(adjustedZ > rot) {

                    l.setPower(l.getPower() - correction);
                    bl.setPower(bl.getPower() - correction);
                    r.setPower(r.getPower() + correction);
                    br.setPower(br.getPower() + correction);
                    telemetry.addLine("Correcting for rightwards drift.");

                }
                telemetry.update();
                if (closeEnough((int) l_count, (int) r_count, (int) bl_count, (int) br_count)) {
                    break;
                }
                sleep(75);

            }
            cancelMovement();
        }
    }


    void pidStrafe(double powerLimit, double y, double rot, double timeoutS, double correction, int offset) {

        rot = 0;
        double x = 0;
        if (opModeIsActive()) {

            double reciprocal_radius = 1 / WHEEL_RADIUS;
            double lr = reciprocal_radius * (x - y - (rot * (2 * CENTER_TO_WHEEL)));
            double rr = reciprocal_radius * (x + y + (rot * (2 * CENTER_TO_WHEEL)));
            double blr = reciprocal_radius * (x + y - (rot * (2 * CENTER_TO_WHEEL)));
            double brr = reciprocal_radius * (x - y + (rot * (2 * CENTER_TO_WHEEL)));
            double l_count = l.getCurrentPosition() + (lr * scale2 * COUNTS_PER_INCH);
            double r_count = r.getCurrentPosition() + (rr * scale2 * COUNTS_PER_INCH);
            double bl_count = bl.getCurrentPosition() + (blr * scale2 * COUNTS_PER_INCH);
            double br_count = br.getCurrentPosition() + (brr * scale2 * COUNTS_PER_INCH);
            double l_power = map(lr * scale3, -l_count * scale3, l_count * scale3, powerLimit, powerLimit);
            double r_power = map(rr * scale3, -r_count * scale3, r_count * scale3, powerLimit, powerLimit);
            double bl_power = map(blr * scale3, -bl_count * scale3, bl_count * scale3, powerLimit, powerLimit);
            double br_power = map(brr * scale3, -br_count * scale3, br_count * scale3, powerLimit, powerLimit);
            l.setTargetPosition((int) l_count);
            r.setTargetPosition((int) r_count);
            bl.setTargetPosition((int) bl_count);
            br.setTargetPosition((int) br_count);

            h.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            l.setPower(l_power);
            r.setPower(r_power);
            bl.setPower(bl_power);
            br.setPower(br_power);

            while (opModeIsActive() &&
                    (l.isBusy() || r.isBusy() || bl.isBusy() || br.isBusy()) &&
                    (runtime.seconds() < timeoutS)) {

                // adjust angle from 0 > 360 to -180 to 180 so that 0 is straight ahead
                int rawZ = g.getHeading() + offset;
                int adjustedZ = 0;
                if(rawZ > 180 + offset) {
                    adjustedZ = rawZ - 360;
                } else {
                    adjustedZ = rawZ;
                }

                telemetry.addData("Powers", "Powers are %7f :%7f :%7f :%7f", l.getPower(), r.getPower(), bl.getPower(), br.getPower());
                telemetry.addData("Gyro-raw",rawZ);
                telemetry.addData("Gyro-adj",adjustedZ);

                if(adjustedZ < rot + offset) {

                    l.setPower(l.getPower() - correction);
                    bl.setPower(bl.getPower() + correction);
                    r.setPower(r.getPower() - correction);
                    br.setPower(br.getPower() + correction);
                    telemetry.addLine("Correcting for leftwards drift.");

                }
                if(adjustedZ > rot + offset) {

                    l.setPower(l.getPower() + correction);
                    bl.setPower(bl.getPower() - correction);
                    r.setPower(r.getPower() + correction);
                    br.setPower(br.getPower() - correction);
                    telemetry.addLine("Correcting for rightwards drift.");

                }
                telemetry.update();
                if (closeEnough((int) l_count, (int) r_count, (int) bl_count, (int) br_count)) {
                    break;
                }
                sleep(320);

            }
            cancelMovement();
        }
    }


    void cancelMovement() {

        telemetry.update();
        h.setPower(0);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    boolean closeEnough(int l_target, int r_target, int bl_target, int br_target) {

        return  (l_target-30<=l.getCurrentPosition() && l.getCurrentPosition()<=l_target+30) &&
                (r_target-30<=r.getCurrentPosition() && r.getCurrentPosition()<=r_target+30) &&
                (bl_target-30<=bl.getCurrentPosition() && bl.getCurrentPosition()<=bl_target+30) &&
                (br_target-30<=br.getCurrentPosition() && br.getCurrentPosition()<=br_target+30);

    }

    double map(double x, double min_a, double max_a, double min_b, double max_b) {
        return (x - min_a) / (max_a - min_a) * (max_b - min_b) + min_b;
    }


}
