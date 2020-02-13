package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto B1")
public class Auto_B1_New extends BaseTrajectory {

    @Override
    public void afterRun() {

        ready_arm();
        switch(stone) {

            case 1:
                moveRelative(4, 36.2,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 1);
                moveRelative(51, 0, 0, 3);
                break;

            case 2:
                moveRelative(-3, 36.2,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 0.5);
                moveRelative(58, 0, 0, 3);
                break;

            case 3:
                moveRelative(-11, 36.2,0, 2.01);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 0.5);
                moveRelative(66, 0, 0, 3.3);
                break;

            default:
                moveRelative(-11, 36.2,0, 3);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 0.5);
                moveRelative(56, 0, 0, 3);
                break;
        }

        moveRelative(0, 15, 0, 0.5);
        drop_stone();
        moveRelative(0,-15,0,0.5);

        switch(stone) {

            case 1:
                moveRelative(-60, 5, 0, 3);
                moveRelative(0, 10, 0, 1);
                grab_stone();
                raise_stone();
                moveRelative(54, 0, 0, 3);
                break;

            case 2:
                moveRelative(-66, 5, 0, 3);
                moveRelative(0, 10, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(60, 0, 0, 3);
                break;

            case 3:
                moveRelative(-72, 5, 0, 3);
                moveRelative(0, 10, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(66, 0, 0, 3.3);
                break;

            default:
                moveRelative(-60, 36.2,0, 3);
                moveRelative(0, -10, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(56, 0, 0, 3);
                break;
        }

        moveRelative(0, 15, 0, 0.5);
        drop_stone();

    }

}