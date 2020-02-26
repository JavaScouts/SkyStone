package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto B1")
public class Auto_B1_New extends BaseTrajectory {

    // TO EDIT SERVO POSITIONINGS AND TIMINGS, SEE LINES 214 - 236 in BaseTrajectory
    // To EDIT others its pretty self-explanatory
    // if the ROBOT does not correct its ANGLE fast ENOUGH, increase KP by 3 or 4 until it does (BaseTrajectory)
    // if results are not expected, call leon
    // or use band

    private static final double STONE_1 = 10;
    private static final double STONE_2 = 4;
    private static final double STONE_3 = -6;
    private static final double STONE_4 = -9;
    private static final double STONE_5 = -17;
    private static final double STONE_6 = -23.5;

    private static final double WALL_TO_STONE = 26.4;
    private static final double STRAFE_CORRECTION = 4.7;

    private static final double DROPOFF_LOCATION_1 = 74;
    private static final double DROPOFF_LOCATION_2 = 67;
    private static final double DROPOFF_LOCATION_3 = 61;

    @Override
    public void afterRun() {

        ready_arm();
        switch(stone) {

            case 1:
                //move to 1, grab and raise, move to found
                moveRelative(STONE_1, WALL_TO_STONE,0, 1.6);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_1 + DROPOFF_LOCATION_1, 0, 0, 2.7);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(STONE_2, WALL_TO_STONE,0, 1.6);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_2 + DROPOFF_LOCATION_1, 0, 0, 2.7);
                break;

            case 3:
                //move to 3, grab and raise, move to found
                moveRelative(STONE_3, WALL_TO_STONE,0, 1.6);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_3 + DROPOFF_LOCATION_1, 0, 0, 2.9);
                break;

            default:
                //move to 2, grab and raise, move to found
                moveRelative(STONE_2, WALL_TO_STONE,0, 1.6);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_2 + DROPOFF_LOCATION_1, 0, 0, 2.7);
                break;
        }

        drop_stone();

        switch(stone) {

            case 1:
                //move to 1, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_1 + STONE_4, 0,0, 2.7, "18");
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_4 + DROPOFF_LOCATION_2, 0, 0, 2.7);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_1 + STONE_5, 0,0, 2.7, "9");
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_5 + DROPOFF_LOCATION_2, 0, 0, 2.7);
                break;

            case 3:
                //move to 3, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_1 + STONE_6, 0,0, 2.7);
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_6 + DROPOFF_LOCATION_2, 0, 0, 2.9);
                break;

            default:
                //move to 2, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_1 + STONE_5, 0,0, 2.7);
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_5 + DROPOFF_LOCATION_2, 0, 0, 2.7);
                break;
        }

        drop_stone();

        switch(stone) {

            case 1:
                //move to 1, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_2 + STONE_2, 0,0, 2.3, "34");
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_2 + DROPOFF_LOCATION_3, 0, 0, 2.3);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_2 + STONE_1, 0,0, 2.3, "41");
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_1 + DROPOFF_LOCATION_3, 0, 0, 2.3);
                break;

            case 3:
                //move to 3, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_2 + STONE_1, 0,0, 2.3, "41");
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_1 + DROPOFF_LOCATION_3, 0, 0, 2.3);
                break;

            default:
                //move to 2, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_2 + STONE_1, 0,0, 2.3);
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.5);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_1 + DROPOFF_LOCATION_3, 0, 0, 2.3);
                break;
        }

        moveRelative(0, STRAFE_CORRECTION, 0, 0.5);
        drop_stone();

        //TODO add another stone

        curve_it(85, -0.3, 0.3);
        moveRelative(8,0,0, 1, "HOLD CURRENT");
        sleep(100);
        //TODO grab foundation
        curve_it(5,0.3,-0.3);
        moveRelative(4,-16,0,2);

    }

}