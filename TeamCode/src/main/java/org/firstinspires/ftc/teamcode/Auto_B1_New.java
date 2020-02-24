package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto B1")
public class Auto_B1_New extends BaseTrajectory {

    // TO EDIT SERVO POSITIONINGS AND TIMINGS, SEE LINES 214 - 236 in BaseTrajectory
    // To EDIT others its pretty self-explanatory
    // if the ROBOT does not correct its ANGLE fast ENOUGH, increase KP by 3 or 4 until it does (BaseTrajectory)
    // if results are not expected, call leon
    // or use band

    double STONE_1 = 10;
    double STONE_2 = 2;
    double STONE_3 = -6;
    double STONE_4 = -16;
    double STONE_5 = -26;
    double STONE_6 = -34;

    @Override
    public void afterRun() {

        ready_arm();
        switch(stone) {

            case 1:
                //move to 1, grab and raise, move to found
                moveRelative(STONE_1, 26,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_1 + 58, 0, 0, 3);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(STONE_2, 26,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_2 + 58, 0, 0, 3);
                break;

            case 3:
                //move to 3, grab and raise, move to found
                moveRelative(STONE_3, 26,0, 2.01);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_3 + 58, 0, 0, 3.3);
                break;

            default:
                //move to 2, grab and raise, move to found
                moveRelative(STONE_2, 26,0, 2);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_2 + 58, 0, 0, 3);
                break;
        }

        range_drive(25, 0.25);
        drop_stone();

        switch(stone) {

            case 1:
                //move to 1, grab and raise, move to found
                moveRelative(-(-STONE_1 + 58) + STONE_4, 0,0, 3);
                ready_arm();
                range_drive(22, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_4 + 58, 0, 0, 3);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(-(-STONE_2 + 58) + STONE_5, 0,0, 3);
                ready_arm();
                range_drive(22, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_5 + 58, 0, 0, 3);
                break;

            case 3:
                //move to 3, grab and raise, move to found
                moveRelative(-(-STONE_3 + 58) + STONE_6, 0,0, 3);
                ready_arm();
                range_drive(22, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_6 + 58, 0, 0, 3.3);
                break;

            default:
                //move to 2, grab and raise, move to found
                moveRelative(-58 + STONE_5, 0,0, 3);
                ready_arm();
                grab_stone();
                raise_stone();
                moveRelative(STONE_5 + 80, 0, 0, 3);
                break;
        }

        range_drive(25, 0.5);
        drop_stone();

    }

}