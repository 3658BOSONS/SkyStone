package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;

@TeleOp(name="TeleOp", group="TeleOp")
public class Tele extends LinearOpMode
{

    DriveTrain drive;

    @Override
    public void runOpMode()
    {

       drive = new DriveTrain(hardwareMap.get(DcMotor.class, "fl"), hardwareMap.get(DcMotor.class, "fr"),
               hardwareMap.get(DcMotor.class, "br"), hardwareMap.get(DcMotor.class, "bl"));

        waitForStart();
        while(opModeIsActive())
        {

            if(gamepad1.a){
                drive.move((Math.PI / 2), .8, .2);
            }
            else if(gamepad1.b){
                drive.move(Math.PI / 4, 1, 0);
            }
            else if(gamepad1.x){
                drive.move(Math.PI / 6, 1, 0);
            }
            else if(gamepad1.y){
                drive.move(Math.PI / 3, 1, 0);
            }
            else if(gamepad1.dpad_up){
                drive.move(Math.PI / 2, gamepad1.right_trigger, 0);
            }
            else if(gamepad1.dpad_down){
                drive.move(Math.PI * 1.5, gamepad1.right_trigger, 0);
            }
            else if(gamepad1.dpad_left){
                drive.move(Math.PI, gamepad1.right_trigger, 0);
            }
            else if(gamepad1.dpad_right){
                drive.move(0, gamepad1.right_trigger, 0);
            }else{
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y; //Y is upside down

                double theta = Math.atan2(y, x);
                double r = Math.sqrt((x * x) + (y * y));

                if(r > 1)
                    r = 1;

                drive.move(theta, r, gamepad1.right_stick_x);

                telemetry.clear();
                telemetry.addLine("x " + x);
                telemetry.addLine("y " + y);
                telemetry.addLine("theta " + Math.toDegrees(theta));
                telemetry.addLine("r " + r);
                telemetry.update();
            }
        }

    }

}
