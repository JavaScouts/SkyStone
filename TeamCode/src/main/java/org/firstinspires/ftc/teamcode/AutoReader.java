package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;

@Autonomous(name = "read")
public class AutoReader extends LinearOpMode {

    @Override
    public void runOpMode() {

        FTCFileReader f = new FTCFileReader("test.txt");
        waitForStart();

//        telemetry.log().add(f.getPath());

        telemetry.addData("STONE_1",f.getKey("STONE_1"));
        telemetry.addData("STONE_2",f.getKey("STONE_2"));
        telemetry.addData("STONE_3",f.getKey("STONE_3"));
        telemetry.addData("STONE_4",f.getKey("STONE_4"));
        telemetry.addData("STONE_5",f.getKey("STONE_5"));
        telemetry.addData("STONE_6",f.getKey("STONE_6"));
        telemetry.update();

        sleep(5000);

    }


}
