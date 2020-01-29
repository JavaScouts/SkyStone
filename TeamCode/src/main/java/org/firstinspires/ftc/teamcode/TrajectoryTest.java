package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RIP KOBE")
public class TrajectoryTest extends BaseTrajectory {

    @Override
    public boolean setDriveOnly() {
        return true;
    }

    @Override
    public Pose[] setPoses() {

        return new Pose[]{

                // start pos
                new Pose(0, 0, 0),
                // move 1
                new Pose(20, 0, PI/4).convertIntoM()
        };

    }

    @Override
    public double[] setTimings() {

        return new double[]{

                //move 1
                3
        };

    }

    @Override
    public Pose[] setStonePoses() {

        return new Pose[]{

                //pos 1
                new Pose(0, 35, 0) . convertIntoM(),

                //pos 2
                new Pose(-8, 35, 0) . convertIntoM(),

                //pos 3
                new Pose(-16, 35, 0) .convertIntoM()

        };

    }

}
