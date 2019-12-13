package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "(Blue3) >Wait>Park")
public class Auto_B_Park extends BaseAutonomous {


    static final long SECONDS_TO_WAIT = 5;


    @Override
    public void before_start() {
        before_start_blue();
    }

    @Override
    public void after_start() {

        sleep(SECONDS_TO_WAIT*1000);
        driveToPoint(0.6,70,0,0,10);

    }

}
