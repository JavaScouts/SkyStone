package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous(name="(Blue1)  >1Stone>Deliver>Park", group="b_final")
public class Auto_B1 extends BaseAutonomous {

    @Override
    public void before_start() {
        before_start_blue();
    }

    @Override
    public void after_start() {

        h.small.setPosition(0.1);
        h.big.setPosition(0.05);
        h.push_block_further_in_to_placer.setPosition(0);
        h.hookRight.setPosition(0);
        h.hookLeft.setPosition(0);
        driveToPoint(0.51, 0, -44, 0, 10);
        gyroTurn(0.51,0);
        sleep(50);

        //if skystone is in first position
        if (driveToPoint(0.44, -500, 0, 0, 10, "detect-v3",0.1) < 0.3) {

            sleep(50);
            driveToPoint(0.6, 12, 0, 0, 10);
            sleep(10);
            driveToPoint(0.5, 0, 0, PI / 4, 10);
            sleep(10);
            driveToPoint(0.45, -17, 0, 0, 10, "collect");
            sleep(200);
            h.push_block_further_in_to_placer.setPosition(0.4);
            sleep(700);
            driveToPoint(0.45, 17, 0, 0, 10);
            sleep(50);
            h.small.setPosition(0.7);
            driveToPoint(0.5, 0, 0, -PI / 4, 10);
            h.Collec2.setPower(0);
            h.Collec1.setPower(0);
            sleep(50);
            driveToPoint(0.6, 0, 9, 0, 10);
            gyroTurn(0.3,0);
            sleep(50);

        //skystone is somewhere else
        } else {

            sleep(50);
            driveToPoint(0.6, 12, 0, 0, 10);
            sleep(10);
            driveToPoint(0.6,0,-16.4,0,10);
            sleep(5);
            driveToPoint(0.6,-8.7,0,0,10,"collect");
            sleep(200);
            h.push_block_further_in_to_placer.setPosition(0.4);
            sleep(700);
            driveToPoint(0.6,8.7,0,0,10);
            sleep(10);
            h.small.setPosition(0.7);
            driveToPoint(0.7,0,27,0,10);
            h.Collec2.setPower(0);
            h.Collec1.setPower(0);
            gyroTurn(0.3,0);
            sleep(50);

        }
        h.push_block_further_in_to_placer.setPosition(0);
        driveToPoint(0.7, 1000, 0, 0, 10, "range-1", 22);
        sleep(50);
        gyroTurn(0.55,-90);
        driveToPoint(0.7,13,0,0,10);

        place_block();

        driveToPoint(0.6,-13,0,0,10);
        sleep(50);
        gyroTurn(0.45,0);
        driveToPoint(0.6,-1000,0,0,10,"range-2",58);
        telemetry.log().add("aight, imma head out");

        /*
        driveToPoint(0.6, 12, 0, 0, 10);
        sleep(10);
        driveToPoint(0.6,0,-15,0,10);
        sleep(5);
        driveToPoint(0.6,-8.7,0,0,10,"collect");
        sleep(100);
        h.push_block_further_in_to_placer.setPosition(0.4);
        sleep(700);
        driveToPoint(0.6,8.7,0,0,10);
        sleep(10);
        h.small.setPosition(0.7);
        driveToPoint(0.7,0,28,0,10);
        h.Collec2.setPower(0);
        h.Collec1.setPower(0);
        gyroTurn(0.5,0);
        sleep(50);*/

    }

}