package org.firstinspires.ftc.teamcode;

import android.widget.ArrayAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware;

import java.util.ArrayList;

@Deprecated
@Disabled
@TeleOp(name = "Max Speed Test")
public class MaxSpeedTest extends LinearOpMode {

    private Hardware h;
    DcMotorEx l, r, bl, br;
    ArrayList<DcMotorEx> motors = new ArrayList<>(4);

    @Override
    public void runOpMode() {

        h = new Hardware();
        h.init(hardwareMap, this);
        l = (DcMotorEx) h.leftDrive;
        r = (DcMotorEx) h.rightDrive;
        bl = (DcMotorEx) h.backLDrive;
        br = (DcMotorEx) h.backRDrive;
        motors.add(l);
        motors.add(r);
        motors.add(br);
        motors.add(bl);

        waitForStart();

        double max_speed = 0;
        while(opModeIsActive()) {

            h.manualDrive(1.0);
            h.moveRobot();

            telemetry.addData("Current Fastest",max_speed);

            for (DcMotorEx motor : motors) {

                double v = motor.getVelocity(AngleUnit.RADIANS);
                telemetry.addData(motor.getDeviceName(), v);
                if(v > max_speed) {
                    max_speed = v;
                }

            }

            telemetry.update();

        }

    }




}