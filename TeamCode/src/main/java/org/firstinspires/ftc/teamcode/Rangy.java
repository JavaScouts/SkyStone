package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "backRange")
public class Rangy extends LinearOpMode {

    Hardware h = new Hardware();
    @Override
    public void runOpMode() {

        h.init(hardwareMap, this);

        waitForStart();
        double dist = 0;
        while(opModeIsActive()) {

            h.manualDrive(0.8);
            h.moveRobot();

            dist = h.backRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("dist",dist);
            telemetry.update();

        }


    }


}
