package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.VisionUtils.NewSkyStonePipeline;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.text.DecimalFormat;
import java.util.ArrayList;

public abstract class BaseTrajectory extends LinearOpMode {

    static final double PI = Math.PI;
    Pose[] p = new Pose[]{};
    Pose[] sp = new Pose[]{};
    double[] timings = new double[]{};
    boolean drive_only = true;

    private Hardware h = new Hardware();
    private static DecimalFormat df = new DecimalFormat("0.000");
    private ElapsedTime e = new ElapsedTime();
    private double dt;
    private double KP = 1.4;
    private double KI = 0;
    private double KD = 0;
    OpenCvCamera webcam;
    NewSkyStonePipeline newSkyStonePipeline;
    int stone = -1;

    //customizable
    abstract Pose[] setPoses();
    abstract double[] setTimings();
    abstract Pose[] setStonePoses();
    abstract boolean setDriveOnly();

    @Override
    public void runOpMode() {

        drive_only = setDriveOnly();
        p = setPoses();
        timings = setTimings();
        sp = setStonePoses();

        h.init(hardwareMap, this);
        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (!drive_only) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.openCameraDevice();
            newSkyStonePipeline = new NewSkyStonePipeline();
            newSkyStonePipeline.setView_source(1);
            webcam.setPipeline(newSkyStonePipeline);
            webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            newSkyStonePipeline.setView_source(3);
            boolean found = false;
            ElapsedTime et = new ElapsedTime();
            et.reset();
            while (!isStopRequested() && !found) {
                try {
                    try {
                        ArrayList<MatOfPoint> contours = newSkyStonePipeline.convexHullsOutput();
                        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
                        Rect[] boundRect = new Rect[contours.size()];
                        try {
                            for (int i = 0; i < contours.size(); i++) {
                                contoursPoly[i] = new MatOfPoint2f();
                                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                            }
                        } catch (IndexOutOfBoundsException e) {
                            e.printStackTrace();
                        }

                        if (boundRect.length == 0 && et.seconds() > 2.5) {
//                        telemetry.log().add("Skystone is in pos 3, 6.");
                            stone = 3;
                            found = true;
                        } else {
                            telemetry.log().add("Skystone x:" + boundRect[0].x);
                            if (boundRect[0].x > 217) {
//                            telemetry.log().add("Skystone is in pos 1, 4.");
                                stone = 1;
                                found = true;
                            } else {
//                            telemetry.log().add("Skystone is in pos 2, 5.");
                                stone = 2;
                                found = true;
                            }
                        }

                    } catch (NullPointerException e) {
                        e.printStackTrace();
                    }
                } catch (ArrayIndexOutOfBoundsException e) {
                    e.printStackTrace();
                }
            }
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }

        waitForStart();
        telemetry.log().clear();

        if(!drive_only) {
            if (stone == 1) {
                p[1] = sp[0];
            } else if (stone == 2) {
                p[1] = sp[1];
            } else if (stone == 3) {
                p[1] = sp[2];
            } else {
                p[0] = new Pose(0, 0, 0);
            }
        }

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

        out[5] = Vx;
        out[6] = Vy;
        out[7] = Vt;

//        telemetry.addData("Velocities", "X:[+" + df.format(Vx * 39.37) + "]  Y:[+" + df.format(Vy * 39.37) + "]  T:[+" + df.format(Vt * 39.37) + "]");
        //System.out.println("V: "+Vx+" | "+Vy+" | "+Vt+"");
//        for positioning
//        out[7] += dt * Vt;
//        out[5] += dt * ((Vx * Math.cos(out[7])) - (Vy * Math.sin(out[7])));
//        out[6] += dt * ((Vx * Math.sin(out[7])) + (Vy * Math.cos(out[7])));

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

            for(int i = 0; i < trajectories.length; i++) {

                if(i != 0) {
                    double ang = trajectories[i-1].end.t;
                    trajectories[i].start.x = (trajectories[i-1].end.x * Math.cos(ang)) - (trajectories[i-1].end.y * Math.sin(ang));
                    trajectories[i].start.y = (trajectories[i-1].end.x * Math.sin(ang)) + (trajectories[i-1].end.y * Math.cos(ang));
                }
                telemetry.log().add("Starting at [%7f] [%7f] [%7f]", trajectories[i].start.x, trajectories[i].start.y, trajectories[i].start.t);
                e.reset();
                double endtime = trajectories[i].tf;
                double cur = e.seconds();
                Pose velocity;
                while(cur < endtime) {

                    a = where(a);
                    cur = e.seconds();
                    velocity = trajectories[i].getVelocity(cur);
                    wheels.convertVelocityToWheelVelocities(velocity);
                    wheels.startMotors();
                    telemetry.addData("Current vel target", "x:[%7f] y:[%7f] t:[%7f]",velocity.x, velocity.y, velocity.t);
                    telemetry.addData("Current vel estimate", "x:[%7f] y:[%7f] t:[%7f]",a[5], a[6], a[7]);
//                    telemetry.addData("Current wheel target", "[%7f] [%7f] [%7f] [%7f]", wheels.l, wheels.r, wheels.bl, wheels.br);
                    telemetry.update();

                }
                wheels.stopMotors();
            }

        }
        void runPidCorrected() {

            double[] a = new double[]{0, 0, 0, 0, 0, 0, 0, 0};

            for(Trajectory t : trajectories) {

                //standard variables
                e.reset();
                double endtime = t.tf;
                double cur = e.seconds();
                Pose velocity;

                //pid variables, maybe get rid of the use part?
                boolean use_x_pid = false;
                boolean use_y_pid = false;
                boolean use_t_pid = false;
                if (t.start.x - t.end.x != 0) {
                    use_x_pid = true;
                }
                if (t.start.y - t.end.y != 0) {
                    use_y_pid = true;
                }
                if (t.start.t - t.end.t != 0) {
                    use_t_pid = true;
                }

                double err;
                double output1 = 0;
                double output2 = 0;
                double output3 = 0;

                while(cur < endtime) {

                    //standard trajectory follow
                    a = where(a);
                    cur = e.seconds();
                    velocity = t.getVelocity(cur);
                    telemetry.addData("Current vel target", "x:[%7f] y:[%7f] t:[%7f]",velocity.x, velocity.y, velocity.t);

                    if(use_x_pid) {
                        err = velocity.x - a[5];
                        output1 = (KP * err);
                    }
                    if(use_y_pid) {
                        err = velocity.y - a[6];
                        output2 = (KP * err);
                    }
                    /*if(use_t_pid) {
                        //todo incorporate gyro sensor
                        err = velocity.t - a[7];
                        output3 = (KP * err);
                    }
*/
                    velocity.x += output1;
                    velocity.y += output2;
                    //velocity.t += output3;

                    wheels.convertVelocityToWheelVelocities(velocity);
                    wheels.startMotors();

                    telemetry.addData("Current vel estimate", "x:[%7f] y:[%7f] t:[%7f]",a[5], a[6], a[7]);
                    telemetry.addData("Current adjustment", "x:[%7f] y:[%7f] t:[%7f]", output1, output2, output3);
//                    telemetry.addData("Current wheel target", "[%7f] [%7f] [%7f] [%7f]", wheels.l, wheels.r, wheels.bl, wheels.br);
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

            double W = vel.t;
            double Vx = vel.x;
            double Vy = vel.y;
//            double Vx = (vel.x * Math.cos(W)) - (vel.y * Math.sin(W));
//            double Vy = (vel.x * Math.sin(W)) + (vel.y * Math.cos(W));
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
        public Pose start_v;
        public Pose end;
        public Pose end_v;
        public double tf;

        Trajectory(Pose start_, Pose end_, double time) {
            start = start_;
            end = end_;
            if(end.t != start.t) {

                double x0 = start.x;
                double x1 = end.x;
                double y0 = start.y;
                double y1 = end.y;
                double x = x1 - x0;
                double y = y1 - y0;
                double chord = Math.sqrt(x * x + y * y);
                double t = end.t - start.t;
                double outer = PI - (2 * t);
                double inner = PI - PI/2 - outer/2;
                double radius = (chord / 2) / Math.sin(inner / 2);
                double arclength = (inner * radius);
                telemetry.log().add("arc len"+arclength);
                double offset = arclength - chord;
                end.x += offset;
                end.y += offset;

            }
            tf = time;
        }

        Trajectory(Pose start_, Pose end_, Pose start_v_, Pose end_v_, double time) {
            start = start_;
            end = end_;
            tf = time;
            start_v = start_v_;
            end_v = end_v_;
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

            double a = (6 * (end.t - start.t))/Math.pow(tf, 2);
            double b = (6 * (start.t - end.t))/Math.pow(tf, 3);
            double theta = (a * t) + (b * t * t);

            a = (6 * (end.x - start.x))/Math.pow(tf, 2);
            b = (6 * (start.x - end.x))/Math.pow(tf, 3);
            double x = (a * t) + (b * t * t);

            a = (6 * (end.y - start.y))/Math.pow(tf, 2);
            b = (6 * (start.y - end.y))/Math.pow(tf, 3);
            double y = (a * t) + (b * t * t);
/*
            double derivCoeff = 1.0 / Math.sqrt(x * x + y * y);
            y *= theta * derivCoeff;
            x *= theta * derivCoeff;*/

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

        void changeFrame() {
            x = (x * Math.cos(t)) - (y * Math.sin(t));
            y = (x * Math.sin(t)) + (y * Math.cos(t));
        }

        Pose convertMtoIn() {
            return new Pose(x * 39.37, y * 39.37, t);
        }
        Pose convertIntoM() {
            return new Pose(x / 39.37, y / 39.37, t);
        }

    }



}
