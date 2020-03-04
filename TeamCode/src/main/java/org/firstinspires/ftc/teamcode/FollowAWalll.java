package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Deprecated
@Disabled
@Autonomous(name = "FollowAWall")
public class FollowAWalll extends LinearOpMode {

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);

        waitForStart();

        while(opModeIsActive()) {
            driveAlongWall(5);
        }
    }

    //this method is adapted from the pushbot example class for encoder driving
    public void driveAlongWall(double distance)
    {
        double TOLERANCE = 0.5;
        double GAIN = 0.1;
        double leftFrontPower = 0;
        double leftBackPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0;
        double range = robot.sideRange.getDistance(DistanceUnit.INCH);
        telemetry.addData("backRange", range);
        telemetry.addData("error", range-distance);

        if(range < distance - TOLERANCE) {

            leftBackPower = GAIN;
            leftFrontPower = GAIN;
            rightBackPower = GAIN + Range.scale(range, -8, 0, 0, 1);
            rightFrontPower = GAIN + Range.scale(range, -8, 0, 0, 1);
            telemetry.addLine("THE ROBOT IS TOO CLOSE");

        } else if(range > distance + TOLERANCE) {

            leftBackPower = GAIN + Range.scale(range, 0, 8, 0., 1);
            leftFrontPower = GAIN + Range.scale(range, 0, 8, 0, 1);
            rightBackPower = GAIN;
            rightFrontPower = GAIN;
            telemetry.addLine("THE ROBOT IS WAY TOO FAR");

        } else {

            leftBackPower = 0.4;
            leftFrontPower = 0.4;
            rightBackPower = 0.4;
            rightFrontPower = 0.4 ;

        }


        /*if(error < TOLERANCE)  //The robot is too close to the wall
        {
            error = Math.abs(error);
            leftFrontPower = GAIN * error;
            leftBackPower = GAIN * error;
            rightFrontPower = problemGAIN * error;
            rightBackPower = problemGAIN * error;
            telemetry.addLine("THE ROBOT IS TOO CLOSE");
        } else if(error > TOLERANCE)  //The robot is too far away
        {
            leftFrontPower = problemGAIN * error;
            leftBackPower = problemGAIN * error;
            rightFrontPower = GAIN * error;
            rightBackPower = GAIN * error;
            telemetry.addLine("THE ROBOT IS WAY TOO FAR");
        } else {
            leftFrontPower = 0.5;
            leftBackPower = 0.5;
            rightFrontPower = 0.5;
            rightBackPower = 0.5;
            telemetry.addLine("JUST RIGHT");
        }*/
        robot.backLDrive.setPower(Range.clip(leftBackPower,-0.6,0.6));
        robot.leftDrive.setPower(Range.clip(leftFrontPower,-0.6,0.6));
        robot.backRDrive.setPower(Range.clip(rightBackPower,-0.6,0.6));
        robot.rightDrive.setPower(Range.clip(rightFrontPower,-0.6,0.6));

        telemetry.addData("back left power", leftBackPower);
        telemetry.addData("back right power", leftBackPower);
        telemetry.addData("front left power", leftBackPower);
        telemetry.addData("front right power", leftBackPower);
        telemetry.update();

    }

}


