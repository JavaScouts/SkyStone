package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto B1 no found")
public class Auto_B1_NOFOUN extends BaseTrajectory {

    // TO EDIT SERVO POSITIONINGS AND TIMINGS, SEE LINES 214 - 236 in BaseTrajectory
    // To EDIT others its pretty self-explanatory
    // if the ROBOT does not correct its ANGLE fast ENOUGH, increase KP by 3 or 4 until it does (BaseTrajectory)
    // if results are not expected, call leon
    // or use band

    private static final double STONE_1 = -11;
    private static final double STONE_2 = -4;
    private static final double STONE_3 = 6;
    private static final double STONE_4 = 9;
    private static final double STONE_5 = 17;
    private static final double STONE_6 = 24;

    private static final double WALL_TO_STONE = 27.3;
    private static final double STRAFE_CORRECTION = 9;
    private static final double DROP_CORRECTION = 5;
    private static final double RETURN_CORRECTION = 9;

    private static final double DROPOFF_LOCATION_1 = -76;
    private static final double DROPOFF_LOCATION_2 = -68;
    private static final double DROPOFF_LOCATION_3 = -63;

    @Override
    public void afterRun() {

        sleep(5000);
        ready_arm();
        foundation_up();
        switch(stone) {

            case 3:
                //move to 1, grab and raise, move to found
                moveRelative(STONE_1, WALL_TO_STONE,0, 1.35);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_1 + DROPOFF_LOCATION_1, 0, 0, 2.3);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(STONE_2, (WALL_TO_STONE-2),0, 1.35);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_2 + DROPOFF_LOCATION_1, 0, 0, 2.3);
                break;

            case 1:
                //move to 3, grab and raise, move to found
                moveRelative(STONE_3, WALL_TO_STONE,0, 1.35);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_3 + DROPOFF_LOCATION_1, 0, 0, 2.6);
                break;

            default:
                //move to 2, grab and raise, move to found
                moveRelative(STONE_2, WALL_TO_STONE,0, 1.35);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_2 + DROPOFF_LOCATION_1, 0, 0, 2.3);
                break;
        }

        moveRelative(0, DROP_CORRECTION, 0, 0.27);
        drop_stone();
        moveRelative(0, -RETURN_CORRECTION, 0, 0.27);


        switch(stone) {

            case 3:
                //move to 1, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_1 + STONE_4-1, 0,0, 2.3, "18R");
                ready_arm()
                ;
                moveRelative(0, STRAFE_CORRECTION, 0, 0.27);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_4 + DROPOFF_LOCATION_2, 0, 0, 2.3);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_1 + STONE_5-1, 0,0, 2.3, "9R");
                ready_arm();
                moveRelative(0, (STRAFE_CORRECTION + 2.), 0, 0.27);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_5 + DROPOFF_LOCATION_2, 0, 0, 2.3);
                break;

            case 1:
                //move to 3, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_1 + STONE_6-1.5, 0,0, 2.3);
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.27);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_6 + DROPOFF_LOCATION_2, 0, 0, 2.6);
                break;

            default:
                //move to 2, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_1 + STONE_5, 0,0, 2.3);
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.27);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_5 + DROPOFF_LOCATION_2, 0, 0, 2.3);
                break;
        }

        moveRelative(0, DROP_CORRECTION, 0, 0.27);
        drop_stone();
        moveRelative(0, -RETURN_CORRECTION-3, 0, 0.27);

        switch(stone) {

            case 3:
                //move to 1, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_2 + STONE_2, 0,0, 1.9, "34R");
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.27);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_2 + DROPOFF_LOCATION_3, 0, 0, 1.9);
                break;

            case 2:
                //move to 2, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_2 + STONE_1 + 2, 0,0, 1.9, "39.5R");
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.27);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_1 + DROPOFF_LOCATION_3, 0, 0, 1.9);
                break;

            case 1:
                //move to 3, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_2 + STONE_1 + 4, 0,0, 1.9, "39.5R");
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.27);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_1 + DROPOFF_LOCATION_3, 0, 0, 1.9);
                break;

            default:
                //move to 2, grab and raise, move to found
                moveRelative(-DROPOFF_LOCATION_2 + STONE_1, 0,0, 1.9);
                ready_arm();
                moveRelative(0, STRAFE_CORRECTION, 0, 0.27);
                grab_stone();
                raise_stone();
                moveRelative(-STONE_1 + DROPOFF_LOCATION_3, 0, 0, 1.9);
                break;
        }

        moveRelative(0, DROP_CORRECTION, 0, 0.27);
        drop_stone();
        //TODO add another stone

        moveRelative(25,0,0,2);

    }

}