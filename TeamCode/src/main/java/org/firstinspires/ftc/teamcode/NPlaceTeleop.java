package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name= "placement yuh yuh yuh")
public class NPlaceTeleop extends LinearOpMode {

    Hardware h = new Hardware();
    ElapsedTime e = new ElapsedTime();
    CRServo p;
    CRServo p1;

    @Override
    public void runOpMode() {

        h.init(hardwareMap, this);

        p = hardwareMap.crservo.get("p");
        p1 = hardwareMap.crservo.get("p1");
        h.Rev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        boolean isHeld = false;
        boolean curResetState;
        boolean lastResetState = false;
        boolean place = false;
        boolean curResetState2;
        boolean lastResetState2 = false;
        boolean firstTime = false;
        while(opModeIsActive()) {

            double collecPowe = gamepad2.right_stick_y;
            h.Collec1.setPower(collecPowe);
            h.Collec2.setPower(collecPowe);

            p.setPower(collecPowe);
            p1.setPower(-collecPowe);

            curResetState = (gamepad2.left_bumper);
            if (curResetState && !lastResetState) {
                place = true;
            }
            lastResetState = curResetState;

            curResetState2 = (gamepad2.a);
            if (curResetState2 && !lastResetState2) {
                firstTime = true;
            }
            lastResetState2 = curResetState2;

            if(place) {

                close_grabber();
                sleep(400);
                left_to_stone(firstTime);
                int count = up_to_stone(firstTime);
                placement_motions();
                down_some(count);

                place = false;
                firstTime = false;

            }

        }

    }

    void close_grabber() {
        h.big.setPosition(0.78);
    }
    void open_grabber() {
        h.big.setPosition(0.08);
    }
    void arm_out() {
        h.small.setPosition(0.35);
    }
    void arm_in() {
        h.small.setPosition(0);
    }

    void placement_motions() {
        arm_out();
        sleep(700);
        open_grabber();
        sleep(300);
        arm_in();
        sleep(700);
    }

    void left_to_stone(boolean firstTime) {
        double range = h.frontRange.getDistance(DistanceUnit.INCH);

        if(firstTime) {
            return;
        }
        while(range > 6 && opModeIsActive()) {
            h.moveRobot(0, 0.1, 0);
            range = h.frontRange.getDistance(DistanceUnit.INCH);
        }
        e.reset();
        while(e.seconds() < 0.5 && opModeIsActive()) {
            h.moveRobot(0, 0.1, 0);
            return;
        }
    }

    int up_to_stone(boolean firstTime) {
        double range = h.frontRange.getDistance(DistanceUnit.INCH);
        int returnpos = h.Rev.getCurrentPosition();
        if(firstTime) {
            e.reset();
            while (e.seconds() < 0.25 && opModeIsActive()) {
                h.Rev.setPower(-0.15);
                return returnpos;
            }
        }
        while(range > 6 && opModeIsActive()) {
            h.Rev.setPower(-0.1);
            range = h.frontRange.getDistance(DistanceUnit.INCH);
        }
        return returnpos;
    }

    void down_some(int count) {

        h.Rev.setTargetPosition(count);
        h.Rev.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        h.Rev.setPower(0.15);
        while(opModeIsActive() && h.Rev.isBusy()) {
            idle();
        }
        h.Rev.setPower(0);
    }


}
