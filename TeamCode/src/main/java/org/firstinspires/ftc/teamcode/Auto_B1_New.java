package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "qwuorieqeuwpori")
public class Auto_B1_New extends BaseTrajectory {

    @Override
    public Pose[] setPoses() {

        return new Pose[]{

                // start pos
                new Pose(0, 0, 0),
                // skystone pos (placeholder) -- move 1
                new Pose(0, 0, 0),
                // move around the center -- move 2
                new Pose(30, 30,0)  .convertIntoM(),
                // move around the center -- move 3
                new Pose(50,30,0)   .convertIntoM(),
                // move foundation -- move4
                new Pose(60,6,-PI/4).convertIntoM(),
                // park -- move 5
                new Pose(30,30, 0)  .convertIntoM()

        };

    }

    @Override
    public double[] setTimings() {

        return new double[]{

                //move 1
                3,
                //move 2
                3,
                //move 3
                3,
                //move 4
                5,
                //move 5
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