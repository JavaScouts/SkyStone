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
    Servo found;

    double power = 0.4;
    double pos = 0;
    double multiplier = 0.92;
    boolean moved = false;
    boolean slid = false;
    boolean foundation = false;
    boolean block = false;
    boolean smallmove = false;
    boolean bigmove = false;


    @Override
    public void runOpMode() {

        boolean lastResetState = false;
        boolean curResetState  = false;
        boolean lastResetState2 = false;
        boolean curResetState2  = false;
        boolean lastResetState3 = false;
        boolean curResetState3  = false;
        boolean lastResetState4 = false;
        boolean curResetState4  = false;
        robot.init(hardwareMap, this);

        Collec1 = hardwareMap.dcMotor.get("c1");
        Collec2 = hardwareMap.dcMotor.get("c2");
        found = hardwareMap.servo.get("found");
        Rev = robot.Rev;

        Collec1.setDirection(DcMotorSimple.Direction.REVERSE);
        Collec2.setDirection(DcMotorSimple.Direction.FORWARD);

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
                robot.small.setPosition(0.1);
            } else if (gamepad2.b){
                robot.small.setPosition(0.7);
            }
            if (gamepad2.a){
                robot.big.setPosition(0.05);
            } else if (gamepad2.y){
                robot.big.setPosition(0.6);
            }

            telemetry.addData("s servo",robot.small.getPosition());
            telemetry.addData("B servo", robot.big.getPosition());
/*            if(gamepad2.a) {

                big.setPosition(0);
                sleep(50);
                small.setPosition(0.1);

            }

            if(gamepad2.x) {

                small.setPosition(0.75);
                sleep(425);
                moveUpOne();
                sleep(20);
                big.setPosition(0.6);

            }

            if(gamepad2.b) {

                small.setPosition(0.1);
                sleep(250);
                big.setPosition(0);
                small.setPosition(0.75);
                sleep(400);
                moveDownOne();
                sleep(20);
                small.setPosition(0.1);

            }

*/
            curResetState = (gamepad2.left_bumper);
            if (curResetState && !lastResetState) {
                foundation = !foundation;
            }
            lastResetState = curResetState;
            if(foundation) {
                robot.hookLeft.setPosition(0.7);
            } else {
                robot.hookLeft.setPosition(0);
            }

            curResetState2 = (gamepad2.y);
            if (curResetState2 && !lastResetState2) {
                smallmove = !smallmove;
            }
            lastResetState2 = curResetState2;
            if(smallmove) {
                robot.small.setPosition(0.1);
            } else {
                robot.small.setPosition(0.7);
            }

            curResetState3 = (gamepad2.right_bumper);
            if (curResetState3 && !lastResetState3) {
                block = !block;
            }
            lastResetState3 = curResetState3;
            if(block) {
                robot.push_block_further_in_to_placer.setPosition(0.4);
            } else {
                robot.push_block_further_in_to_placer.setPosition(0);
            }

            curResetState4 = (gamepad2.a);
            if (curResetState4 && !lastResetState4) {
                bigmove = !bigmove;
            }
            lastResetState4 = curResetState4;
            if(bigmove) {
                robot.big.setPosition(0.05);
            } else {
                robot.big.setPosition(0.6);
            }

            double lifterPower = gamepad2.left_stick_y * 0.6;
            if (gamepad2.left_trigger > 0.05) {
                lifterPower -= 0.001;
            }
            Rev.setPower(lifterPower);

            double collecPowe = gamepad2.right_stick_y;
            Collec1.setPower(collecPowe);
            Collec2.setPower(collecPowe);

            telemetry.addData("fl", robot.leftDrive.getCurrentPosition());
            telemetry.addData("fr", robot.rightDrive.getCurrentPosition());
            telemetry.addData("bl", robot.backLDrive.getCurrentPosition());
            telemetry.addData("br", robot.backRDrive.getCurrentPosition());
            telemetry.addData("lift", Rev.getCurrentPosition());
            telemetry.addData("z", robot.gyro.getHeading());

            telemetry.update();
        }

    }

}



