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

@Deprecated
//@TeleOp(name = "Final TeleOp")
public class FinalTeleOp extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    Servo hiccup;

    //VuforiaTracking tracking = new VuforiaTracking();

    boolean LastDetent;
    boolean Detent;
    double power = 0.4;
    double pos = 0;
    double multiplier = 0.92;
    boolean moved = false;
    boolean slid = false;
    String kys = "";


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);
        hiccup = hardwareMap.get(Servo.class, "m");


        //not sure what this mess does but i don't trust myself to fix it
        robot.cup.setDirection(DcMotorSimple.Direction.REVERSE);
        // robot.cup.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.screw.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.screw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        //robot.cup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.rightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.backRDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.backLDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        robot.backRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        robot.backLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        robot.gyro.calibrate();
        LastDetent = true;
        moved = false;
        slid = false;
        kys = "screw mode";


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

            //refer to control diagram for how to control our robot, and an explanation of all this stuff
            /*if (gamepad2.a) {
                robot.ball.setPosition(1.0);
            } else if (gamepad2.b) {
                robot.ball.setPosition(0);
            } else if (gamepad2.y){
                robot.ball.setPosition(0.9);
            }78

*/
            if (gamepad2.left_bumper) {
                hiccup.setPosition(0);
            }

            if (gamepad2.right_bumper) {
                hiccup.setPosition(1.0);
            }

            if (gamepad2.x) {
                pos = 0.4;
                moved = true;
            }

            if(gamepad2.a) {
                pos = 0.9;
                moved = true;

            }

            if(gamepad2.b) {
                pos = 0;
                moved = true;
            }
            if(moved) {
                robot.ball.setPosition(pos);
            }

            //use dpad to drive robot slowly
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

            //this system is effective for moving the arm and holding it in position.
           /* Detent = (Math.abs(gamepad2.left_stick_y) <= 0.1);
            // check for hold or move
            if (Detent) {
                // we are in hold so use RTP
                robot.cup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.cup.setPower(0.1);

                // Did we JUST let go of the stick?  if so, save location.
                if (!LastDetent)
                    robot.cup.setTargetPosition(robot.cup.getCurrentPosition());

            } else if (gamepad2.x) {
                robot.cup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.cup.setPower(0.1);
                robot.cup.setTargetPosition(-138);

            } else {

                // we are in move so use RWE
                robot.cup.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (gamepad2.left_stick_y > 0)
                    robot.cup.setPower(gamepad2.left_stick_y * 0.4);
                else
                    robot.cup.setPower(gamepad2.left_stick_y * 0.5);
            }

            // remember last detent state for next time around.
            LastDetent = Detent;*/

            if (gamepad2.left_trigger == 1) {
                slid = false;
            }
            if (gamepad2.right_trigger == 1) {
                slid = true;
            }

            double right = -gamepad2.right_stick_y;
            double left = -gamepad2.left_stick_y;
            //double re = -gamepad2.right_stick_x;

            robot.cup.setPower(left);
            robot.cup2.setPower(left);

            if (!slid) {
                robot.screw.setPower(right);
                robot.rev.setPower(0);
                kys = "screw mode";
            }
            if (slid) {
                robot.screw.setPower(0);
                robot.rev.setPower(-right);
                kys = "slider mode";
            }
            //rev.setPower(re);

            //add all data from sensors and encoders
            telemetry.addData("Current mode is", kys);
            telemetry.addData("Current speed multiplier is", multiplier);
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addData("cup", robot.cup.getCurrentPosition());
            telemetry.addData("screw", robot.screw.getCurrentPosition());
            telemetry.addData("fl", robot.leftDrive.getCurrentPosition());
            telemetry.addData("fr", robot.rightDrive.getCurrentPosition());
            telemetry.addData("bl", robot.backLDrive.getCurrentPosition());
            telemetry.addData("br", robot.backRDrive.getCurrentPosition());
            telemetry.addData("range", robot.range.getDistance(DistanceUnit.INCH));

            telemetry.update();
        }

    }

}



