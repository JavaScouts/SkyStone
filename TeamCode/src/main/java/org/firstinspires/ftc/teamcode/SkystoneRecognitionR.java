package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Skystone Recognition-R")
public class SkystoneRecognitionR extends LinearOpMode {

    private ModernRoboticsI2cColorSensor color;
    private int col, red, green, blue;

    @Override
    public void runOpMode() {

        color = hardwareMap.get(ModernRoboticsI2cColorSensor.class,"c");

        waitForStart();

        while(opModeIsActive()) {

            col = color.argb();
            telemetry.addData("argb",col);
            red = color.red();
            telemetry.addData("red",red);
            green = color.green();
            telemetry.addData("green",green);
            blue = color.blue();
            telemetry.addData("blue",blue);
            telemetry.update();

        }

    }
}
