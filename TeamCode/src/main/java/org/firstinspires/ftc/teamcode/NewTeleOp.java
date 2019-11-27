package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Liam on 9/8/2018.
 */

@TeleOp(name = "Test Chassis")
public class NewTeleOp extends LinearOpMode {

    Hardware robot = new Hardware();
    DcMotor Collec1;
    DcMotor Collec2;
    DcMotor Rev;
    Servo big;
    Servo small;
    Servo found;

    double power = 0.4;
    double pos = 0;
    double multiplier = 0.92;
    boolean moved = false;
    boolean slid = false;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);

        Collec1 = hardwareMap.dcMotor.get("c1");
        Collec2 = hardwareMap.dcMotor.get("c2");
        small = hardwareMap.servo.get("small");
        big = hardwareMap.servo.get("big");
        found = hardwareMap.servo.get("found");

        Collec1.setDirection(DcMotorSimple.Direction.REVERSE);
        Collec2.setDirection(DcMotorSimple.Direction.FORWARD);
        Rev = hardwareMap.dcMotor.get("rev");

        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                multiplier = 0.92;
            } else if (gamepad1.right_bumper) {
                multiplier = 0.3;
            }

            if (gamepad1.right_stick_x == 0) {
                robot.smartPower = 0;
                telemetry.addLine("Smart Power resetting due to release of joystick.");
            }

            robot.smartManualDrive(multiplier);
            robot.moveRobot();


            if(gamepad1.dpad_left) {

                robot.backLDrive.setPower(power);
                robot.leftDrive.setPower(-power);
                robot.backRDrive.setPower(-power);
                robot.rightDrive.setPower(power);

            } else if(gamepad1.dpad_right) {

                robot.backLDrive.setPower(-power);
                robot.leftDrive.setPower(power);
                robot.backRDrive.setPower(power);
                robot.rightDrive.setPower(-power);

            } else if(gamepad1.dpad_up) {

                robot.backLDrive.setPower(power);
                robot.leftDrive.setPower(power);
                robot.backRDrive.setPower(power);
                robot.rightDrive.setPower(power);

            } else if(gamepad1.dpad_down) {

                robot.backLDrive.setPower(-power);
                robot.leftDrive.setPower(-power);
                robot.backRDrive.setPower(-power);
                robot.rightDrive.setPower(-power);

            }
            if (gamepad2.x){
                small.setPosition(0);
            } else if (gamepad2.b){
                small.setPosition(0.85);
            }
            if (gamepad2.y){
                big.setPosition(0);
            } else if (gamepad2.a){
                big.setPosition(1.0);
            }
            if (gamepad2.left_bumper){
                found.setPosition(0);
            } else if (gamepad2.right_bumper){
                found.setPosition(0.5);
            }

            Collec1.setPower(gamepad2.left_stick_y);
            Collec2.setPower(gamepad2.left_stick_y);
            Rev.setPower(gamepad2.right_stick_y);


            telemetry.addData("fl", robot.leftDrive.getCurrentPosition());
            telemetry.addData("fr", robot.rightDrive.getCurrentPosition());
            telemetry.addData("bl", robot.backLDrive.getCurrentPosition());
            telemetry.addData("br", robot.backRDrive.getCurrentPosition());

            telemetry.update();
        }

    }

}



