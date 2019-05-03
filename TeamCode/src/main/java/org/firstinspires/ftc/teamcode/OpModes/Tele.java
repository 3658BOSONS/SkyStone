package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class Tele extends LinearOpMode
{

    AnalogInput odometry;

    @Override
    public void runOpMode()
    {

        odometry = hardwareMap.get(AnalogInput.class, "odometry");

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.clear();
            telemetry.addLine("Raw: " + odometry.getVoltage());
            telemetry.addLine("Max: " + odometry.getMaxVoltage());
            telemetry.addLine("Raw Position: " + (odometry.getVoltage() / odometry.getMaxVoltage()) * 360);
            telemetry.update();
        }

    }

}
