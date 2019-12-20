package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = ">Wait>Park", group="standard")
public class Auto_Park extends BaseAutonomous {




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
