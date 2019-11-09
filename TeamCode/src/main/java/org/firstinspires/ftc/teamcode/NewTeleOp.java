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

        Collec2.setDirection(DcMotorSimple.Direction.REVERSE);
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


