package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

@TeleOp(name = "Servo")
public class ServoTest extends LinearOpMode {

    private Hardware h;

    @Override
    public void runOpMode() {

        h = new Hardware();
        h.init(hardwareMap, this);

        waitForStart();

        double pleft = 0;
        double pright = 0;
        while(opModeIsActive()) {

            double left = Math.abs(gamepad1.left_stick_y);
            double right = Math.abs(gamepad1.right_stick_y);
            if(left != 0) {
                h.hookLeft.setPosition(left);
                pleft = left;
            } else if ( right != 0){
                h.hookRight.setPosition(right);
                pright = right;
            } else {
                h.hookLeft.setPosition(pleft);
                h.hookRight.setPosition(pright);
            }

            telemetry.addData("left", pleft);
            telemetry.addData("right", pright);
            telemetry.update();

        }

    }

}