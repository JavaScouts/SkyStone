package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.VisionUtils.NewSkyStonePipeline;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.text.DecimalFormat;
import java.util.ArrayList;

public abstract class BaseTrajectory extends LinearOpMode {

    static final double PI = Math.PI;
    Pose[] p = new Pose[]{};
    Pose[] sp = new Pose[]{};
    double[] timings = new double[]{};
    ArrayList<Integer> used = new ArrayList<>(6);
    boolean drive_only = false;

    private Hardware h = new Hardware();
    private Pose startPose = new Pose(0, 0, 0);
    private Wheels constantWheels;
    private static DecimalFormat df = new DecimalFormat("0.000");
    private ElapsedTime e = new ElapsedTime();
    private double dt;
    private ModernRoboticsI2cGyro g;
    private double KPT = 40;
    private double KPM = 0.5;
    private double KI = 0.0;
    private double KD = 0.5;

    LockObject lock;
    int intermediateposition;
    int stone = -1;
    //customizable
    abstract void afterRun();

    @Override
    public void runOpMode() {

        beforeRun();
        afterRun();

    }

    void beforeRun() {

        lock = new LockObject(-1);

        h.init(hardwareMap, this);
        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        g = h.gyro;

        g.calibrate();

        while (!isStopRequested() && g.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(e.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(6);
        }

        constantWheels = new Wheels(h.leftDrive,
                                    h.rightDrive,
                                    h.backLDrive,
                                    h.backRDrive);

        if (!drive_only) {
            vision_thread(lock);
        }

        while(!isStopRequested() && !isStarted()) {
            synchronized (lock) {
                intermediateposition = lock.result;
            }
            telemetry.addData("current result", intermediateposition);
            telemetry.update();
        }
        synchronized (lock) {
            lock.shouldEnd = true;
        }
        stone = intermediateposition;

        telemetry.log().clear();

    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 backRange  (
        robotError = targetAngle - g.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }


    void moveRelative(double x, double y, double rot, double time) {
        moveRelative(x, y, rot, time, "");
    }

    void moveRelative(double x, double y, double rot, double time, String command) {

        Trajectory toFollow = new Trajectory(startPose,
                                             new Pose(x, y, rot).convertIntoM(),
                                             time);

        double[] a = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
        telemetry.log().add("Starting at [%7f] [%7f] [%7f]", toFollow.start.x, toFollow.start.y, toFollow.start.t);
        e.reset();
        double endtime = toFollow.tf;
        double cur = e.seconds();
        Pose velocity;
        double err = 0;
        double prev_err = 0;
        double integral = 0;
        double derivative = 0;
        double output = 0;
        double output1 = 0;
        double output2 = 0;
        dt = 0;
        double prev = 0;
        boolean needsToEnd = false;
        if(command.equals("HOLD CURRENT")) {
            rot = g.getIntegratedZValue();
        }
        /*boolean shouldRange = false;
        double range = 0;
        double prevrange = 0;
        if(command != "HOLD CURRENT" && command != "") {
            shouldRange = true;
            command = command.substring(0, command.length()-1);
            range = Double.valueOf(command);
        }*/
        while(cur < endtime && !needsToEnd && opModeIsActive()) {
            /*if(shouldRange) {
                if(range > prevrange + 10 || range < prevrange - 10) {
                    range = prevrange;
                }
                if (h.backRange.getDistance(DistanceUnit.INCH) < range) {
                    needsToEnd = true;
                }
                prevrange = range;
                telemetry.addData("backRange",range);
            }*/
            a = where(a);
            cur = e.seconds();
            dt = cur - prev;
            velocity = toFollow.getVelocity(cur);
            constantWheels.convertVelocityToWheelVelocities(velocity);
            telemetry.addData("Current vel target", "x:[%7f] y:[%7f] t:[%7f]",velocity.x, velocity.y, velocity.t);
            telemetry.addData("Current pos estimate", "x:[%7f] y:[%7f] t:[%7f]",a[5], a[6], a[7]);

            err = getError(rot);
            err = map(err, -180, 180, -1, 1);
            integral = integral + (err * dt);
            derivative = (err - prev_err) / dt;
            output = (KPT * err) + (KI * integral) + (KD * derivative);
            telemetry.addData("output",output);
            //output = map(output, -max_error, max_error, -adj, adj);
            //telemetry.addData("adjust_output",output);
            prev_err = err;
            telemetry.addData("p", err);
            telemetry.addData("i", integral);
            telemetry.addData("d", derivative);
/*
            err = velocity.x - a[5];
            output1 = (KPM * err);
            constantWheels.forwardAdd(output1);
            err = velocity.y - a[6];
            output2 = (KPM * err);
            constantWheels.strafeAdd(output2);*/

            constantWheels.adjust(output);
            telemetry.addData("Current adjustments", "x:[%7f] y:[%7f] t:[%7f]",output1, output2, output);

            constantWheels.startMotors();

            telemetry.addData("Current wheel target", "[%7f] [%7f] [%7f] [%7f]", constantWheels.l, constantWheels.r, constantWheels.bl, constantWheels.br);

            telemetry.update();
            prev = cur;

        }

        constantWheels.stopMotors();

    }

    void new_thread_command(final String value) {

        new Thread(new Runnable() {
            @Override
            public void run() {
                switch(value) {
                    case "GRAB":
                        h.grabArm.setPosition(0.40);
                        h.grabClaw.setPosition(0);
                        sleep(650);
                        break;
                    case "RAISE":
                        h.grabArm.setPosition(0.8);
                        sleep(700);
                        break;
                    case "DROP":
                        h.grabArm.setPosition(0.8);
                        h.grabClaw.setPosition(0.6);
                        sleep(400);
                        break;
                    case "READY":
                        h.grabClaw.setPosition(0.6);
                        h.grabArm.setPosition(0.45);
                        sleep(500);
                        break;
                    default:
                        h.grabArm.setPosition(0.8);
                        sleep(700);
                        break;
                }
            }
        }).start();

    }

    void foundation_up() {
        h.hookRight.setPosition(0.5);
        h.hookLeft.setPosition(0.6);
    }
    void foundation_down() {
        h.hookRight.setPosition(0);
        h.hookLeft.setPosition(1);
    }
    void grab_stone() {
        h.grabArm.setPosition(0.40);
        h.grabClaw.setPosition(0);
        sleep(520);
    }

    void raise_stone() {
        h.grabArm.setPosition(0.8);
        sleep(660);
    }

    void drop_stone() {
        h.grabArm.setPosition(0.45);
        sleep(500);
        h.grabClaw.setPosition(0.6);
        sleep(300);
        h.grabArm.setPosition(0.8);
    }
    void ready_arm() {
        h.grabClaw.setPosition(0.6);
        h.grabArm.setPosition(0.45);
        sleep(450);
    }

    void range_drive(double distance, double power) {

        if(h.backRange.getDistance(DistanceUnit.INCH) < distance) {

            h.leftDrive.setPower(-power);
            h.backRDrive.setPower(-power);
            h.rightDrive.setPower(power);
            h.backLDrive.setPower(power);

        } else if(h.backRange.getDistance(DistanceUnit.INCH) > distance) {

            h.leftDrive.setPower(power);
            h.backRDrive.setPower(power);
            h.rightDrive.setPower(-power);
            h.backLDrive.setPower(-power);

        }

        while(!within(h.backRange.getDistance(DistanceUnit.INCH), distance, 1) && opModeIsActive()) {

            idle();

        }

        h.setPower(0);

    }

    boolean within(double val1, double val2, double thresh) {

        return (val1 > val2 - thresh) &&
                (val1 < val2 + thresh);

    }

    void curve_it(double desired_heading, double l_power, double r_power) {

        double e = getError(desired_heading);
        while(opModeIsActive() && e!=0 && e!=1 && e!=-1) {

            e = getError(desired_heading);
            h.leftDrive.setPower(l_power);
            h.rightDrive.setPower(r_power);
            h.backRDrive.setPower(r_power);
            h.backLDrive.setPower(l_power);
            telemetry.addData("g heading", g.getIntegratedZValue());
            telemetry.addData("err", e);
            telemetry.update();

        }
        h.setPower(0);

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
/*

        out[5] = Vx;
        out[6] = Vy;
        out[7] = Vt;
*/

//        telemetry.addData("Velocities", "X:[+" + df.format(Vx * 39.37) + "]  Y:[+" + df.format(Vy * 39.37) + "]  T:[+" + df.format(Vt * 39.37) + "]");
//        System.out.println("V: "+Vx+" | "+Vy+" | "+Vt+"");
//        for positioning
        out[7] += dt * Vt;
        out[5] += dt * ((Vx * Math.cos(out[7])) - (Vy * Math.sin(out[7])));
        out[6] += dt * ((Vx * Math.sin(out[7])) + (Vy * Math.cos(out[7])));

//        System.out.println("P: "+out[5]+" | "+out[6]+" | "+out[7]+"");
//        System.out.println(out[0]+" "+out[1]+" "+out[2]+" "+out[3]+" "+out[4]+" "+out[5]+" "+out[6]+" "+out[7]+"");

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
                telemetry.log().add("point:" + poses[i].x + ", " + poses[i].y);

            }
        }

        void run() {

            telemetry.log().clear();
            double[] a = new double[]{0, 0, 0, 0, 0, 0, 0, 0};

            for(int i = 0; i < trajectories.length; i++) {

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
                        output1 = (KPT * err);
                    }
                    if(use_y_pid) {
                        err = velocity.y - a[6];
                        output2 = (KPT * err);
                    }
                    /*if(use_t_pid) {
                        //todo incorporate gyro sensor
                        err = velocity.t - a[7];
                        output3 = (KPT * err);
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
            brm.setVelocity(br, AngleUnit.RADIANS);
            rm.setVelocity(r, AngleUnit.RADIANS);
            blm.setVelocity(bl, AngleUnit.RADIANS);

        }
        void adjust(double output) {
            l -= output;
            br += output;
            r += output;
            bl -= output;
        }
        void forwardAdd(double output){
            l += output;
            br += output;
            r += output;
            bl += output;
        }
        void strafeAdd(double output){
            l -= output;
            br -= output;
            r += output;
            bl += output;
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
        public String info = "";
        public int pos = 0;

        Pose(double x_, double y_, double t_) {
            x = x_;
            y = y_;
            t = t_;
        }

        Pose(String info_) {
            info = info_;
        }

        Pose(double x_, double y_, double t_, String info_) {
            info = info_;
            x = x_;
            y = y_;
            t = t_;
        }

        Pose(double x_, double y_, double t_, String info_, int pos_) {
            info = info_;
            pos = pos_;
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

    double map(double x, double min_a, double max_a, double min_b, double max_b) {
        return (x - min_a) / (max_a - min_a) * (max_b - min_b) + min_b;
    }

    void vision_thread(final LockObject o) {

        final OpenCvCamera webcam;
        final NewSkyStonePipeline newSkyStonePipeline;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        newSkyStonePipeline = new NewSkyStonePipeline();

        new Thread(new Runnable() {
            @Override
            public void run() {
                int stone = -1;
                boolean shouldEnd = false;
                webcam.openCameraDevice();
                newSkyStonePipeline.setView_source(1);
                webcam.setPipeline(newSkyStonePipeline);
                try {
                    webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                }catch (OpenCvCameraException e){
                    e.printStackTrace();
                    return;
                }
                newSkyStonePipeline.setView_source(3);
                boolean found = false;
                ElapsedTime et = new ElapsedTime();
                et.reset();
                while (!isStopRequested() && !isStarted() && !shouldEnd) {
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

                            if (boundRect.length == 0 && et.seconds() > 3) {
                                stone = 3;
                                found = true;
                            } else {
                                if (boundRect[0].x > 217) {
                                    stone = 1;
                                    found = true;
                                } else {
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
                    if(found) {
                        synchronized (o) {
                            o.result = stone;
                            shouldEnd = o.shouldEnd;
                        }
                        et.reset();
                        found = false;
                    }
                }
                webcam.stopStreaming();
                webcam.closeCameraDevice();
            }
        }).start();

    }

    class LockObject {
        LockObject(int result_) {
            result = result_;
        }
        public int result;
        public boolean shouldEnd = false;
    }

}
