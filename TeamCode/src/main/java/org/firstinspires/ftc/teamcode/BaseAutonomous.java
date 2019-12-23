package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public abstract class BaseAutonomous extends LinearOpMode {

    ModernRoboticsI2cGyro g;
    ModernRoboticsI2cColorSensor c;
    ModernRoboticsI2cColorSensor cr;
    Rev2mDistanceSensor rn;
    DcMotor l, r, bl, br, c1, c2, Rev;
    private static final double WHEEL_RADIUS = 2.98;
    private static final double CENTER_TO_WHEEL = 8.53;
    private static final double COUNTS_PER_MOTOR_REV = 537.6;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 2.95;
    private boolean RED;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double SLOWDOWN = 0.08;
    static final double PI = 3.1415;
    private double scale2 = 2.35;
    private double scale3 = 0.1;

    Hardware h = new Hardware();
    private Vision v = new Vision();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        before_start();
        awaiting_start();
        after_start();

    }

    public void before_start_blue() {

        h.init(hardwareMap, this);
        //setupVision();
        l = h.leftDrive;
        r = h.rightDrive;
        bl = h.backLDrive;
        br = h.backRDrive;
        c1 = h.Collec1;
        c2 = h.Collec2;
        Rev = h.Rev;
        g = h.gyro;
        c = h.color;
        rn = h.range;
        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.log().add("Hardware initialized.");

        sleep(120);
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        g.calibrate();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (!isStopRequested() && g.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(10);
        }

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        RED = false;

    }

    public void before_start_red() {

        h.init(hardwareMap, this);
        //setupVision();
        l = h.leftDrive;
        r = h.rightDrive;
        bl = h.backLDrive;
        br = h.backRDrive;
        c1 = h.Collec1;
        c2 = h.Collec2;
        Rev = h.Rev;
        g = h.gyro;
        cr = h.color2;
        rn = h.range;
        h.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.log().add("Hardware initialized.");

        sleep(120);
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        g.calibrate();

        // Wait until the gyro calibration is complete
        runtime.reset();
        while (!isStopRequested() && g.isCalibrating()) {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            sleep(10);
        }

        telemetry.log().clear();
        telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear();
        RED = true;

    }

    private void awaiting_start() {

        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", g.getIntegratedZValue());
            telemetry.update();
        }
        telemetry.log().clear();

        g.resetZAxisIntegrator();

    }

    public abstract void before_start();
    public abstract void after_start();

    void curve_it(double desired_heading, double l_power, double r_power) {

        double e = getError(desired_heading);
        while(opModeIsActive() && e!=0 && e!=1 && e!=-1) {

            e = getError(desired_heading);
            l.setPower(l_power);
            r.setPower(r_power);
            bl.setPower(l_power);
            br.setPower(r_power);
            telemetry.addData("g heading", g.getIntegratedZValue());
            telemetry.addData("err", e);
            telemetry.update();

        }
        cancelMovement();

    }

    void place_block() {
        up_one();
        h.big.setPosition(0.6);
        sleep(700);
        h.small.setPosition(0.1);
        sleep(300);
        h.small.setPosition(0.7);
        h.big.setPosition(0.05);
        sleep(700);
        down_the_rest();
    }
    void up_one() {

        ElapsedTime t = new ElapsedTime();
        t.reset();
        while(t.seconds() < 1.25 && opModeIsActive()) {
            Rev.setPower(-0.25);
        }
        Rev.setPower(-0.001);

    }

    void down_the_rest() {

        ElapsedTime t = new ElapsedTime();
        t.reset();
        while(t.seconds() < 1.25 && opModeIsActive()) {
            Rev.setPower(0.25);
        }
        Rev.setPower(-0.001);

    }

    double driveToPoint(double powerLimit, double x, double y, double rot, double timeoutS) {

        return driveToPoint(powerLimit, x, y, rot, timeoutS, "", 0);

    }

    double driveToPoint(double powerLimit, double x, double y, double rot, double timeoutS, String command) {

        return driveToPoint(powerLimit, x, y, rot, timeoutS, command, 0);

    }


    double driveToPoint(double powerLimit, double x, double y, double rot, double timeoutS, String command, double params) {

        double error, steer, max, leftSpeed, rightSpeed;

        if (opModeIsActive()) {

            powerLimit -= SLOWDOWN;

            double reciprocal_radius = 1 / WHEEL_RADIUS;
            double lr = reciprocal_radius * (x - y - (rot * (2 * CENTER_TO_WHEEL)));
            double rr = reciprocal_radius * (x + y + (rot * (2 * CENTER_TO_WHEEL)));
            double blr = reciprocal_radius * (x + y - (rot * (2 * CENTER_TO_WHEEL)));
            double brr = reciprocal_radius * (x - y + (rot * (2 * CENTER_TO_WHEEL)));
            double l_count = l.getCurrentPosition() + (lr * scale2 * COUNTS_PER_INCH);
            double r_count = r.getCurrentPosition() + (rr * scale2 * COUNTS_PER_INCH);
            double bl_count = bl.getCurrentPosition() + (blr * scale2 * COUNTS_PER_INCH);
            double br_count = br.getCurrentPosition() + (brr * scale2 * COUNTS_PER_INCH);
            double l_power = map(lr * scale3, -l_count * scale3, l_count * scale3, powerLimit, powerLimit);
            double r_power = map(rr * scale3, -r_count * scale3, r_count * scale3, powerLimit, powerLimit);
            double bl_power = map(blr * scale3, -bl_count * scale3, bl_count * scale3, powerLimit, powerLimit);
            double br_power = map(brr * scale3, -br_count * scale3, br_count * scale3, powerLimit, powerLimit);
            l.setTargetPosition((int) l_count);
            r.setTargetPosition((int) r_count);
            bl.setTargetPosition((int) bl_count);
            br.setTargetPosition((int) br_count);

            h.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            l.setPower(Math.abs(l_power));
            r.setPower(Math.abs(r_power));
            bl.setPower(Math.abs(bl_power));
            br.setPower(Math.abs(br_power));

            while (opModeIsActive() &&
                    (l.isBusy() || r.isBusy() || bl.isBusy() || br.isBusy()) &&
                    (runtime.seconds() < timeoutS)) {

                telemetry.addLine("WE GOIn");
                telemetry.addData("Powers", "Powers are %7f :%7f :%7f :%7f", l_power, r_power, bl_power, br_power);

                switch (command) {
                    case "detect-v1":
                        /*Recognition sky = v.getFirstSkystoneSeen();

                        if (sky != null) {
                            telemetry.addData("skystone found", sky.getLeft());
                            telemetry.addData("number of rec", v.numberOfRecognitions());
                            telemetry.addData("what is seen?", v.whatIsSeen());
                            if (sky.getLeft() < 123857829) {
                                cancelMovement();
                                return 0;
                            }
                        }*/
                        break;
                    case "detect-v2":
                        /*OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getFtcCameraFromTarget();
                        if (pose != null) {
                            telemetry.addData("Pose", v.format(pose));
                            Orientation angles = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                            if (angles.firstAngle > -50) {
                                cancelMovement();
                                return 0;
                            }
                        } else {
                            telemetry.addLine("No skystone found.");
                        }
                        */break;
                    case "detect-v3":
                        double delay = 1.1;
                        if (params != 0) {
                            delay = params;
                        }
                        if(!RED) {
                            if (c.red() == 0 && runtime.seconds() < delay) {
                                telemetry.addData("Facing", "Waiting");
                                break;
                            }
                            if (c.red() > 0 && runtime.seconds() > delay) {
                                telemetry.addData("Facing", "Stone");
                                break;
                            }
                            if (c.red() == 0 && runtime.seconds() > delay) {
                                telemetry.addData("Facing", "Skystone");
                                telemetry.log().add(runtime.seconds() + " seconds have elapsed.");
                                cancelMovement();
                                return runtime.seconds();
                            }
                        } else {
                            if (cr.red() == 0 && runtime.seconds() < delay) {
                                telemetry.addData("Facing", "Waiting");
                                break;
                            }
                            if (cr.red() > 0 && runtime.seconds() > delay) {
                                telemetry.addData("Facing", "Stone");
                                break;
                            }
                            if (cr.red() == 0 && runtime.seconds() > delay) {
                                telemetry.addData("Facing", "Skystone");
                                telemetry.log().add(runtime.seconds() + " seconds have elapsed.");
                                cancelMovement();
                                return runtime.seconds();
                            }
                        }
                        break;

                    case "collect":
                        c1.setPower(1.0);
                        c2.setPower(1.0);
                        break;

                    case "range-1":
                        double stop = 20;
                        if (params != 0) {
                            stop = params;
                        }
                        double dist = rn.getDistance(DistanceUnit.INCH);
                        if (dist < stop) {
                            telemetry.addLine("Stopping due to range");
                            cancelMovement();
                            return 0;
                        } else {
                            telemetry.addData("Range:", dist);
                        }
                        break;
                    case "range-2":
                        double stop2 = 20;
                        if (params != 0) {
                            stop2 = params;
                        }
                        double dist2 = rn.getDistance(DistanceUnit.INCH);
                        if (dist2 > stop2) {
                            telemetry.addLine("Stopping due to range");
                            cancelMovement();
                            return 0;
                        } else {
                            telemetry.addData("Range:", dist2);
                        }
                        break;
                    case "correct":
                        double angle = 0;
                        if (params != 0) {
                            angle = params;
                        }
                        // adjust relative speed based on heading error.
                        error = getError(angle);
                        steer = getSteer(error, 0.15);

                        // if driving in reverse, the motor correction also needs to be reversed
                        if (y < 0) {
                            steer *= -1.0;
                        }

                        leftSpeed = powerLimit - steer;
                        rightSpeed = powerLimit + steer;

                        // Normalize speeds if either one exceeds +/- 1.0;
                        max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                        if (max > 1.0) {
                            leftSpeed /= max;
                            rightSpeed /= max;
                        }

                        l.setPower(leftSpeed);
                        bl.setPower(leftSpeed);
                        r.setPower(rightSpeed);
                        br.setPower(rightSpeed);

                        // Display drive status for the driver.
                        telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                        break;

                    case "place":

                        break;

                    default:
                        telemetry.addLine("we goin folks");
                        break;
                }
                telemetry.update();
                if (closeEnough((int) l_count, (int) r_count, (int) bl_count, (int) br_count)) {
                    break;
                }
            }

            cancelMovement();

        }
        telemetry.addLine("FINISHED MOVE");
        telemetry.update();
        return 0;

    }

    void pidDrive(double powerLimit, double x, double y, double rot, double timeoutS, double correction) {

        rot = 0;
        if (opModeIsActive()) {

            double reciprocal_radius = 1 / WHEEL_RADIUS;
            double lr = reciprocal_radius * (x - y - (rot * (2 * CENTER_TO_WHEEL)));
            double rr = reciprocal_radius * (x + y + (rot * (2 * CENTER_TO_WHEEL)));
            double blr = reciprocal_radius * (x + y - (rot * (2 * CENTER_TO_WHEEL)));
            double brr = reciprocal_radius * (x - y + (rot * (2 * CENTER_TO_WHEEL)));
            double l_count = l.getCurrentPosition() + (lr * scale2 * COUNTS_PER_INCH);
            double r_count = r.getCurrentPosition() + (rr * scale2 * COUNTS_PER_INCH);
            double bl_count = bl.getCurrentPosition() + (blr * scale2 * COUNTS_PER_INCH);
            double br_count = br.getCurrentPosition() + (brr * scale2 * COUNTS_PER_INCH);
            double l_power = map(lr * scale3, -l_count * scale3, l_count * scale3, powerLimit, powerLimit);
            double r_power = map(rr * scale3, -r_count * scale3, r_count * scale3, powerLimit, powerLimit);
            double bl_power = map(blr * scale3, -bl_count * scale3, bl_count * scale3, powerLimit, powerLimit);
            double br_power = map(brr * scale3, -br_count * scale3, br_count * scale3, powerLimit, powerLimit);
            l.setTargetPosition((int) l_count);
            r.setTargetPosition((int) r_count);
            bl.setTargetPosition((int) bl_count);
            br.setTargetPosition((int) br_count);

            h.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            l.setPower(l_power);
            r.setPower(r_power);
            bl.setPower(bl_power);
            br.setPower(br_power);

            while (opModeIsActive() &&
                    (l.isBusy() || r.isBusy() || bl.isBusy() || br.isBusy()) &&
                    (runtime.seconds() < timeoutS)) {

                telemetry.addData("Powers", "Powers are %7f :%7f :%7f :%7f", l_power, r_power, bl_power, br_power);
                telemetry.addData("Gyro", g.getIntegratedZValue());

                if (g.getIntegratedZValue() < rot) {

                    l.setPower(l.getPower() + correction);
                    bl.setPower(bl.getPower() + correction);
                    r.setPower(r.getPower() - correction);
                    br.setPower(br.getPower() - correction);
                    telemetry.addLine("Correcting for leftwards drift.");

                }
                if (g.getIntegratedZValue() > rot) {

                    l.setPower(l.getPower() - correction);
                    bl.setPower(bl.getPower() - correction);
                    r.setPower(r.getPower() + correction);
                    br.setPower(br.getPower() + correction);
                    telemetry.addLine("Correcting for rightwards drift.");

                }
                telemetry.update();
                if (closeEnough((int) l_count, (int) r_count, (int) bl_count, (int) br_count)) {
                    break;
                }
            }
            cancelMovement();
        }
    }


    void cancelMovement() {

        telemetry.update();
        h.setPower(0);
        h.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    boolean closeEnough(int l_target, int r_target, int bl_target, int br_target) {

        return (l_target - 50 <= l.getCurrentPosition() && l.getCurrentPosition() <= l_target + 50) &&
                (r_target - 50 <= r.getCurrentPosition() && r.getCurrentPosition() <= r_target + 50) &&
                (bl_target - 50 <= bl.getCurrentPosition() && bl.getCurrentPosition() <= bl_target + 50) &&
                (br_target - 50 <= br.getCurrentPosition() && br.getCurrentPosition() <= br_target + 50);

    }

    double map(double x, double min_a, double max_a, double min_b, double max_b) {
        return (x - min_a) / (max_a - min_a) * (max_b - min_b) + min_b;
    }

    /*void setupVision() {

        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        trackables = this.vuforia.loadTrackablesFromAsset("Skystone");
        trackable = trackables.get(0);
        trackable.setName("Skystone Target"); // can help in debugging; otherwise not necessary

    }*/

    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        h.setPower(0);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        l.setPower(leftSpeed);
        r.setPower(rightSpeed);
        bl.setPower(leftSpeed);
        br.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - g.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
