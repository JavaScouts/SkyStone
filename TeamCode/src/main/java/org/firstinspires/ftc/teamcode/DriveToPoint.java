package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Deprecated
//@Autonomous(name="Drive To Point")
public class DriveToPoint extends LinearOpMode {

    private static final double WHEEL_RADIUS = 2.98;
    private static final double CENTER_TO_WHEEL = 8.53;
    private static final double COUNTS_PER_MOTOR_REV = 537.6;
    private static final double DRIVE_GEAR_REDUCTION  = 1.0 ;
    private static final double WHEEL_DIAMETER_INCHES = 2.95 ;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double PI = 3.1415;
    private double scale = 0.004;
    private double scale2 = 2.35;
    private double scale3 = 0.1;     // DO NOT CHANGE PLEASE AND THNAK
    private DcMotor l, r, bl, br;
    private Hardware h = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {

        h.init(hardwareMap, this);
        l = h.leftDrive;
        r = h.rightDrive;
        bl = h.backLDrive;
        br = h.backRDrive;
        //scale2 = 1 / scale;
        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path",  "Starting at %7d :%7d :%7d :%7d",
                l.getCurrentPosition(),
                r.getCurrentPosition(),
                bl.getCurrentPosition(),
                br.getCurrentPosition());
        telemetry.update();

        waitForStart();

        driveToPoint(0.5,24,0,0,10);
        sleep(100);
        driveToPoint(0.5,-24,0,0,10);
        sleep(100);
        driveToPoint(0.5,0,24,0,10);
        sleep(100);
        driveToPoint(0.5,0,-24,0,10);
        sleep(100);
        driveToPoint(0.5,24,-24,0,10);
        sleep(100);
        driveToPoint(0.5,-24,24,0,10);
        sleep(100);
        driveToPoint(0.5,24,-24,-PI/2,10);
        sleep(100);
        driveToPoint(0.5,-24,24,PI/2,10);

    }


    void driveToPoint(double powerLimit, double x, double y, double rot, double timeoutS) {

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
            double l_power = map(lr * scale3, -l_count*scale3,l_count*scale3,powerLimit, powerLimit);
            double r_power = map(rr * scale3, -r_count*scale3,r_count*scale3,powerLimit, powerLimit);
            double bl_power = map(blr * scale3, -bl_count*scale3,bl_count*scale3,powerLimit, powerLimit);
            double br_power = map(brr * scale3, -br_count*scale3,br_count*scale3,powerLimit, powerLimit);
            l.setTargetPosition((int)l_count);
            r.setTargetPosition((int)r_count);
            bl.setTargetPosition((int)bl_count);
            br.setTargetPosition((int)br_count);

            h.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            l.setPower(l_power);
            r.setPower(r_power);
            bl.setPower(bl_power);
            br.setPower(br_power);

            while (opModeIsActive() &&
                   l.isBusy() || r.isBusy() || bl.isBusy() || br.isBusy() &&
                   runtime.seconds() < timeoutS) {

                telemetry.addLine("WE GOIn");
                telemetry.addData("Powers", "Powers are %7f :%7f :%7f :%7f", l_power, r_power, bl_power, br_power);
                telemetry.addData("Path - Target(c)",  "Running to %7d :%7d :%7d :%7d", (int)l_count, (int)r_count, (int)bl_count, (int)br_count);
                telemetry.addData("Path - Current(c)",
                                   "Running at %7d :%7d :%7d :%7d",
                                           l.getCurrentPosition(), r.getCurrentPosition(), bl.getCurrentPosition(), br.getCurrentPosition());
                telemetry.update();

            }

            h.setPower(0);
            h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    double[] calculatePosition(int l, int r, int bl, int br){

        l *= scale;
        r *= scale;
        bl *= scale;
        br *= scale;

        double position_x = (l +
                r +
                bl +
                br) * (WHEEL_RADIUS / 4);

        double position_y = (-l +
                r +
                bl +
                -br) * (WHEEL_RADIUS / 4);

        double angular_position = (-l +
                r +
                -bl +
                br) * (WHEEL_RADIUS / (8 * CENTER_TO_WHEEL));

        return new double[]{position_x, position_y, angular_position};

    }

    double map(double x, double min_a, double max_a, double min_b, double max_b) {

        return (x-min_a)/(max_a-min_a) * (max_b-min_b) + min_b;

    }

}
