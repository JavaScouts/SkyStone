package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "park")
public class park extends BaseTrajectory {

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

    private static final double WALL_TO_STONE = 25.2;
    private static final double STRAFE_CORRECTION = 7;
    private static final double DROP_CORRECTION = 5;
    private static final double RETURN_CORRECTION = 9;

    private static final double DROPOFF_LOCATION_1 = -76;
    private static final double DROPOFF_LOCATION_2 = -68;
    private static final double DROPOFF_LOCATION_3 = -63;

    @Override
    public void afterRun() {

        moveRelative(20,0,0,1.5);

    }

}