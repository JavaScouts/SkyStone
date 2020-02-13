package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto B1")
public class Auto_B1_New extends BaseTrajectory {

    // TO EDIT SERVO POSITIONINGS AND TIMINGS, SEE LINES 214 - 236 in BaseTrajectory
    // To EDIT others its pretty self-explanatory
    // if the ROBOT does not correct its ANGLE fast ENOUGH, increase KP by 3 or 4 until it does (BaseTrajectory)
    // if results are not expected, call leon
    // or use band

    @Override
    public void afterRun() {

        ready_arm();
        switch(stone) {

            case 1:
                //move to 1, grab and raise, move to found
                moveRelative(4, 36.2,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 1);
                moveRelative(51, 0, 0, 3);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(-3, 36.2,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 0.5);
                moveRelative(58, 0, 0, 3);
                break;

            case 3:
                //move to 3, grab and raise, move to found
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

        //strafe into it and place and strafe out
        moveRelative(0, 15, 0, 0.5);
        drop_stone();
        moveRelative(0,-15,0,0.5);

        switch(stone) {

            case 1: //case 4 (same as case 1 but move back instead of moving out)
                moveRelative(-60, 5, 0, 3);
                moveRelative(0, 10, 0, 1);
                grab_stone();
                raise_stone();
                moveRelative(54, 0, 0, 3);
                break;

            case 2: //case 5
                moveRelative(-66, 5, 0, 3);
                moveRelative(0, 10, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(60, 0, 0, 3);
                break;

            case 3: //cae 6
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