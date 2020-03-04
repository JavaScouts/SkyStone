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

        double w;
        while(opModeIsActive()) {

            w = Math.abs(gamepad2.left_stick_y);
            if(gamepad2.a) {
                h.grabClaw.setPosition(0);
            } else if(gamepad2.y) {
                h.grabClaw.setPosition(0.6);
            } else {
                h.grabClaw.setPosition(w);
            }

            telemetry.addData("", w);
            telemetry.update();

        }

    }

}