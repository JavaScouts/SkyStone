package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.text.DecimalFormat;

@Autonomous(name = "aaaah")
public class Auto_Trajectory extends LinearOpMode {

    Pose[] p = new Pose[]{
            new Pose(0, 0,0),
            new Pose(-16, 30, 0).convertIntoM(),
            new Pose(30, 20,-Math.PI/8).convertIntoM(),
            new Pose(60,30,0).convertIntoM(),
            new Pose(60,6,Math.PI/4).convertIntoM(),
            new Pose(30,25, 0).convertIntoM()
    };
    double[] timings = new double[]{
            2,
            2,
            2.5,
            5,
            3
    };
    private Hardware h = new Hardware();
    private static DecimalFormat df = new DecimalFormat("0.000");
    private ElapsedTime e = new ElapsedTime();
    double dt;

    @Override
    public void runOpMode() {

        h.init(hardwareMap, this);
        TrajectoryFollower tf = new TrajectoryFollower(
                p,
                timings,
                new Wheels(h.leftDrive, h.rightDrive, h.backLDrive, h.backRDrive));
        tf.buildTrajectories();

        waitForStart();

        tf.run();

    }

    public double[] where(double[] prev) {

        // output format : [l, r, bl, br, time, x, y, t]
        double Vx, Vy, Vt;
        double PI = 3.1415;
        double ctr = PI / 145.6;
        double[] counts = new double[]{h.leftDrive.getCurrentPosition() * ctr, h.rightDrive.getCurrentPosition() * ctr, h.backLDrive.getCurrentPosition() * ctr, h.backRDrive.getCurrentPosition() * ctr};
        double[] change = new double[]{0, 0, 0, 0};
        double[] out = new double[]{0, 0, 0, 0, 0, 0, 0, 0};

        // add values to change
        for (int i = 0; i < counts.length; i++) {
            change[i] = counts[i] - prev[i];
        }

        // store current in output (return output)
        System.arraycopy(counts, 0, out, 0, counts.length);
        System.arraycopy(prev, 5, out, 5, 3);

        // calc and store dt
        double cur = e.nanoseconds();
        double prevt = prev[4];
        out[4] = cur;
        dt = (cur - prevt) / 1000000000;
        telemetry.addData("DT", dt);

        // calculate wheel velocities and store in change
        for (int i = 0; i < change.length; i++) {
            change[i] /= dt;
        }

        Vx = (change[0] + change[1] + change[2] + change[3]) * (0.0375 / 4);
        Vy = (-change[0] + change[1] + change[2] - change[3]) * (0.0375 / 4);
        Vt = (-change[0] + change[1] - change[2] + change[3]) * (0.0375 / (4 * 0.3429));

        telemetry.addData("Velocities", "X:[+" + df.format(Vx * 39.37) + "]  Y:[+" + df.format(Vy * 39.37) + "]  T:[+" + df.format(Vt * 39.37) + "]");
        //System.out.println("V: "+Vx+" | "+Vy+" | "+Vt+"");

        out[7] += dt * Vt;
        out[5] += dt * ((Vx * Math.cos(out[7])) - (Vy * Math.sin(out[7])));
        out[6] += dt * ((Vx * Math.sin(out[7])) + (Vy * Math.cos(out[7])));

        //System.out.println("P: "+out[5]+" | "+out[6]+" | "+out[7]+"");
        //System.out.println(out[0]+" "+out[1]+" "+out[2]+" "+out[3]+" "+out[4]+" "+out[5]+" "+out[6]+" "+out[7]+"");

        return out;
    }
    class TrajectoryFollower {

        Trajectory[] trajectories;
        Wheels wheels;
        private ElapsedTime e = new ElapsedTime();
        Pose[] poses;
        double[] timings;

        TrajectoryFollower(Pose[] poses_, double[] timings_, Wheels wheels_) {
            trajectories = new Trajectory[poses_.length-1];
            wheels = wheels_;
            poses = poses_;
            timings = timings_;
        }

        void buildTrajectories() {
            for(int i = 0; i < trajectories.length; i++) {

                trajectories[i] = new Trajectory(poses[i], poses[i+1], timings[i]);

            }
        }
        void run() {

            double[] a = new double[]{0, 0, 0, 0, 0, 0, 0, 0};

            for(Trajectory t : trajectories) {

                t.start = new Pose(a[5], a[6], a[7]);
                e.reset();
                double endtime = t.tf;
                double cur = e.seconds();
                Pose velocity;
                while(cur < endtime) {

                    a = where(a);
                    cur = e.seconds();
                    velocity = t.getVelocity(cur);
                    wheels.convertVelocityToWheelVelocities(velocity);
                    wheels.startMotors();
                    telemetry.addData("Current vel target", "x:[%7f] y:[%7f] t:[%7f]",velocity.x, velocity.y, velocity.t);
                    telemetry.addData("Current wheel target", "[%7f] [%7f] [%7f] [%7f]", wheels.l, wheels.r, wheels.bl, wheels.br);
                    telemetry.addData("Current pos estimate", "x:[%7f] y:[%7f] t:[%7f]",a[5], a[6], a[7]);
                    telemetry.update();

                }
                wheels.stopMotors();

            }

        }

    }

    class Wheels {

        public double l;
        public double r;
        public double bl;
        public double br;
        private DcMotorEx lm, rm, blm, brm;

        Wheels(DcMotor lm_, DcMotor rm_, DcMotor blm_, DcMotor brm_) {
            lm = (DcMotorEx) lm_;
            rm = (DcMotorEx) rm_;
            blm = (DcMotorEx) blm_;
            brm = (DcMotorEx) brm_;
        }

        void stopMotors() {

            lm.setVelocity(0);
            rm.setVelocity(0);
            blm.setVelocity(0);
            brm.setVelocity(0);

        }
        void startMotors() {

            lm.setVelocity(l, AngleUnit.RADIANS);
            rm.setVelocity(r, AngleUnit.RADIANS);
            blm.setVelocity(bl, AngleUnit.RADIANS);
            brm.setVelocity(br, AngleUnit.RADIANS);

        }

        void convertVelocityToWheelVelocities(Pose vel) {

            double Vx = vel.x;
            double Vy = vel.y;
            double W = vel.t;
            double OneOverR = 1 / 0.075/2;
            double LxPlusLy = 0.3429;

            // calculate motor angular velocity ( rad / sec )
            this.l = OneOverR * (Vx - Vy - (W * (LxPlusLy)));
            this.r = OneOverR * (Vx + Vy + (W * (LxPlusLy)));
            this.bl = OneOverR * (Vx + Vy - (W * (LxPlusLy)));
            this.br = OneOverR * (Vx - Vy + (W * (LxPlusLy)));

            //gear ratio
            this.l /= 2;
            this.r /= 2;
            this.bl /= 2;
            this.br /= 2;

        }
    }

    class Trajectory {

        public Pose start;
        public Pose end;
        public double tf;

        Trajectory(Pose start_, Pose end_, double time) {
            start = start_;
            end = end_;
            tf = time;
        }

        Pose getPosition(double t) {

            double a = start.x;
            double b = (3 * (end.x - start.x))/Math.pow(tf, 2);
            double c = (2 * (start.x - end.x))/Math.pow(tf, 3);
            double x = a + (b * Math.pow(t, 2)) + (c * Math.pow(t, 3));

            a = start.y;
            b = (3 * (end.y - start.y))/Math.pow(tf, 2);
            c = (2 * (start.y - end.y))/Math.pow(tf, 3);
            double y = a + (b * Math.pow(t, 2)) + (c * Math.pow(t, 3));

            a = start.t;
            b = (3 * (end.t - start.t))/Math.pow(tf, 2);
            c = (2 * (start.t - end.t))/Math.pow(tf, 3);
            double theta = a + (b * Math.pow(t, 2)) + (c * Math.pow(t, 3));

            return new Pose(x, y, theta);

        }

        Pose getVelocity(double t) {

            double a = (6 * (end.x - start.x))/Math.pow(tf, 2);
            double b = (6 * (start.x - end.x))/Math.pow(tf, 3);
            double x = (a * t) + (b * Math.pow(t, 2));

            a = (6 * (end.y - start.y))/Math.pow(tf, 2);
            b = (6 * (start.y - end.y))/Math.pow(tf, 3);
            double y = (a * t) + (b * Math.pow(t, 2));

            a = (6 * (end.t - start.t))/Math.pow(tf, 2);
            b = (6 * (start.t - end.t))/Math.pow(tf, 3);
            double theta = (a * t) + (b * Math.pow(t, 2));

            return new Pose(x, y, theta);

        }

    }


    class Pose {

        public double x;
        public double y;
        public double t;

        Pose(double x_, double y_, double t_) {
            x = x_;
            y = y_;
            t = t_;
        }

        Pose convertMtoIn() {
            return new Pose(x * 39.37, y * 39.37, t);
        }
        Pose convertIntoM() {
            return new Pose(x / 39.37, y / 39.37, t);
        }

    }



}
