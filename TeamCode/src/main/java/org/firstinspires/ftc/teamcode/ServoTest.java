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

        while(opModeIsActive()) {

            h.grabArm.setPosition(Math.abs(gamepad1.left_stick_y));
            h.grabClaw.setPosition(Math.abs(gamepad1.right_stick_y));

        }

    }




}