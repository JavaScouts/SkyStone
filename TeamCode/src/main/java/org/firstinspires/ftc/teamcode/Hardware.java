package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Hardware {

    //initialize everything
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor backLDrive;
    DcMotor backRDrive;
    DcMotor Collec1;
    DcMotor Collec2;
    DcMotor Rev;

    Servo hookLeft, hookRight;
    Servo small;
    Servo big;
    Servo push_block_further_in_to_placer;

    ModernRoboticsI2cGyro gyro;
    Rev2mDistanceSensor range;
    ModernRoboticsI2cColorSensor color;
    ModernRoboticsI2cColorSensor color2;

    //driving coefficients
    private double driveAxial = 0;   // Positive is forward
    private double driveLateral = 0;   // Positive is right
    private double driveYaw = 0;   // Positive is CCW
    private static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    private static final double P_TURN_COEFF = 0.1;
    private double KP = 8;
    private double KI = 0;
    private double KD = 0.005;


    double smartPower = 0;

    private LinearOpMode myOpMode;
    private HardwareMap map;

    //constructor
    Hardware() {

    }

    void init(HardwareMap map1, LinearOpMode opMode) {

        map = map1;
        myOpMode = opMode;

        //get motors from map
        leftDrive = map.dcMotor.get("fl");
        rightDrive = map.dcMotor.get("fr");
        backLDrive = map.dcMotor.get("bl");
        backRDrive = map.dcMotor.get("br");
        Collec1 = map.dcMotor.get("c1");
        Collec2 = map.dcMotor.get("c2");
        Rev = map.dcMotor.get("rev");
        Rev.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hookLeft = map.servo.get("hl");
        hookRight = map.servo.get("found2");
        push_block_further_in_to_placer = map.servo.get("pb");
        small = map.servo.get("small");
        big = map.servo.get("big");
        range = map.get(Rev2mDistanceSensor.class, "r");
        gyro = map.get(ModernRoboticsI2cGyro.class, "g");
        color = map.get(ModernRoboticsI2cColorSensor.class, "c");
        color2 = map.get(ModernRoboticsI2cColorSensor.class,"cr");

        //reverse nessecary motors
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backLDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        Collec1.setDirection(DcMotorSimple.Direction.FORWARD);
        Collec2.setDirection(DcMotorSimple.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //STOP EVERYTHING
        moveRobot(0, 0, 0);

        myOpMode.telemetry.addData("Initialization:", "Complete!");
        myOpMode.telemetry.addData(" Left Encoder Count", leftDrive.getCurrentPosition());
        myOpMode.telemetry.addData(" Right Encoder Count", rightDrive.getCurrentPosition());
        myOpMode.telemetry.addData(" BLeft Encoder Count", backLDrive.getCurrentPosition());
        myOpMode.telemetry.addData(" BRight Encoder Count", backRDrive.getCurrentPosition());

        myOpMode.telemetry.update();

    }

    void setMode(DcMotor.RunMode mode) {

        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
        backLDrive.setMode(mode);
        backRDrive.setMode(mode);

    }

    void setPower(double power) {

        leftDrive.setPower(power);
        rightDrive.setPower(power);
        backLDrive.setPower(power);
        backRDrive.setPower(power);

    }

    void smartManualDrive(double multiplier) {

        setAxial(-myOpMode.gamepad1.left_stick_y * multiplier);
        setLateral(myOpMode.gamepad1.left_stick_x * multiplier);

        double MAX = myOpMode.gamepad1.right_stick_x * multiplier;
        double INCREMENT = Math.abs(MAX / 11);

        if (MAX > 0) {
            // Keep stepping up until we hit the max value.
            smartPower += INCREMENT;
            if (smartPower >= MAX) {
                smartPower = MAX;
            }
        } else if (MAX < 0) {
            // Keep stepping down until we hit the min value.
            smartPower -= INCREMENT;
            if (smartPower <= MAX) {
                smartPower = MAX;
            }
        }

        //myOpMode.telemetry.addData("Smart Power>", smartPower);
        setYaw(smartPower);

    }

    void manualDrive(double multiplier) {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        setAxial(-myOpMode.gamepad1.left_stick_y * multiplier);
        setLateral(myOpMode.gamepad1.left_stick_x * multiplier);
        setYaw(myOpMode.gamepad1.right_stick_x * multiplier);

    }

    private void moveRobot(double axial, double lateral, double yaw) {

        setAxial(axial);
        setYaw(yaw);
        setLateral(lateral);
        moveRobot();

    }

    void moveRobot() {
        // calculate required motor powers to achieve axis motions, based on the movement of mecanum wheels
        double backL = driveAxial - driveLateral + driveYaw;
        double backR = driveAxial + driveLateral - driveYaw;
        double left = driveAxial + driveLateral + driveYaw;
        double right = driveAxial - driveLateral - driveYaw;

        // normalize all motor speeds so no values exceeds 100%
        double max = Math.max(Math.abs(left), Math.abs(right));
        max = Math.max(max, Math.abs(backL));
        max = Math.max(max, Math.abs(backR));
        if (max > 1.0) {
            backR /= max;
            backL /= max;
            right /= max;
            left /= max;
        }

        // Set drive motor smartPower levels
        backLDrive.setPower(backL);
        backRDrive.setPower(backR);
        leftDrive.setPower(left);
        rightDrive.setPower(right);

        // Display Telemetry
        //myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], >>Y[%+5.2f]<<", driveAxial, driveLateral, driveYaw);
        //myOpMode.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f], BL[%+5.2f], BR[%+5.2f]", left, right, backL, backR);

    }

    double[] moveRobot2(double angle, double[] prev, boolean is_first_run) {
        // calculate required motor powers to achieve axis motions, based on the movement of mecanum wheels
        double backL = driveAxial - driveLateral + driveYaw;
        double backR = driveAxial + driveLateral - driveYaw;
        double left = driveAxial + driveLateral + driveYaw;
        double right = driveAxial - driveLateral - driveYaw;

        double err;
        double prev_err = prev[0];
        double integral = prev[1];
        double derivative;
        double output;
        double dt = 0.001;
        //PID
        if(is_first_run) {
            prev = new double[] {0,0};
            prev_err = 0;
            integral = 0;
        }

        err = getError(angle);
        err = BaseAutonomous.map(err, -180, 180, -1, 1);
        integral = integral + (err * dt);
        derivative = (err - prev_err) / dt;
        output = (KP * err) + (KI * integral) + (KD * derivative);
        myOpMode.telemetry.addData("pid output",output);
        prev[0] = err;
        prev[1] = integral;

        left -= output;
        right += output;
        backL -= output;
        backR += output;

        // normalize all motor speeds so no values exceeds 100%
        double max = Math.max(Math.abs(left), Math.abs(right));
        max = Math.max(max, Math.abs(backL));
        max = Math.max(max, Math.abs(backR));
        if (max > 1.0) {
            backR /= max;
            backL /= max;
            right /= max;
            left /= max;
        }

        // Set drive motor smartPower levels
        leftDrive.setPower(left);
        rightDrive.setPower(right);
        backRDrive.setPower(backR);
        backLDrive.setPower(backL);

        // Display Telemetry
        myOpMode.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f], BL[%+5.2f], BR[%+5.2f]", left, right, backL, backR);
        return prev;

    }

    private void setAxial(double axial) {
        driveAxial = Range.clip(axial, -1, 1);
    }

    private void setLateral(double lateral) {
        driveLateral = Range.clip(lateral, -1, 1);
    }

    private void setYaw(double yaw) {
        driveYaw = Range.clip(yaw, -1, 1);
    }

    //the following 4 methods are adapted from the pushbot gyro example class
    void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading. onHeading() moves the robot.
        while (myOpMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {

            myOpMode.telemetry.update();

        }
    }

    Boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn smartPower based on +/- error
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
        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
        backLDrive.setPower(leftSpeed);
        backRDrive.setPower(rightSpeed);

        myOpMode.telemetry.addData("Target", "%5.2f", angle);
        myOpMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        myOpMode.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
