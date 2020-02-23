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
                moveRelative(4, 32,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 1);
                moveRelative(51, 0, 0, 3);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(-3, 32,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 0.5);
                moveRelative(58, 0, 0, 3);
                break;

            case 3:
                //move to 3, grab and raise, move to found
                moveRelative(-11, 32,0, 2.01);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 0.5);
                moveRelative(66, 0, 0, 3.3);
                break;

            default:
                //move to 2, grab and raise, move to found
                moveRelative(-3, 32,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(0, -15, 0, 0.5);
                moveRelative(58, 0, 0, 3);
                break;
        }

    }

}