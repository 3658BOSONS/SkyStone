package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Gyro;

@TeleOp(name="TeleOp", group="TeleOp")
public class Tele extends LinearOpMode
{

    private DriveTrain drive;
    private Gyro gyro;
    private double targetHeading = 0;
    private double headingLock;
    private boolean lastRT = false;
    final double kP = .04;
    final double kD = -1;

    final double strafeKP = .04;
    final double strafeKI = 0;
    final double strafeKD = -.5;
    double strafeI;
    double lastStrafeError;
    double lastStrafeTime;

    boolean fieldCentricEnabled = false;
    boolean lastA = false;

    final double tolerance = 0;

    @Override
    public void runOpMode()
    {
       drive = new DriveTrain(this);
       gyro = new Gyro(this);
        waitForStart();

        drive.getBL().startSpeedometer();
        drive.getBR().startSpeedometer();
        drive.getFL().startSpeedometer();
        drive.getFR().startSpeedometer();

        while(opModeIsActive())
        {
            boolean a = gamepad1.a;
            if(a && !lastA){
                if(fieldCentricEnabled){
                    fieldCentricEnabled = false;
                }else{
                    fieldCentricEnabled = true;
                }
            }
            lastA = a;

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; //Y is upside down
            double rx = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double r = Math.sqrt((x * x) + (y * y));

            if(fieldCentricEnabled)
                theta -= Math.toRadians(gyro.getAngle()) + Math.PI / 2;

            if(r > 1)
                r = 1;

            boolean rt = gamepad1.right_bumper;

            if(!lastRT && rt) {
                headingLock = gyro.getAngle();
                strafeI = 0;
            }
            if(rt)
            {
                double error = gyro.getAngle() - headingLock;
                long time = System.nanoTime();

                double P = strafeKP * error;
                strafeI += (time - lastStrafeTime) * error * strafeKI;
                double D = (error - lastStrafeError) / (time - lastStrafeTime) * strafeKD;

                double turn = P + strafeI + D;
                if(turn < -1)
                    turn = -1;
                else if(turn > 1)
                    turn = 1;

                rx = turn;

                lastStrafeError = error;
                lastStrafeTime = time;
            }

            lastRT = rt;

            drive.move(theta, r, rx);

            telemetry.clear();
            telemetry.addLine("Heading: " + gyro.getAngle());
            telemetry.addLine("FL RPM:  " + drive.getFL().getRPM());
            telemetry.addLine("BL RPM:  " + drive.getBL().getRPM());
            telemetry.addLine("FR RPM:  " + drive.getFR().getRPM());
            telemetry.addLine("BR RPM:  " + drive.getBR().getRPM());
            telemetry.update();

            if(gamepad1.right_trigger > .2)
            {
                targetHeading = gyro.getAbsoluteAngle();
            }

            //Auto Maneuvers
            if(gamepad1.right_stick_button) //Turn to heading
            {
                double target = targetHeading;
                if(Math.abs((gyro.getAbsoluteAngle() + 360) - gyro.getAbsoluteAngle()) < Math.abs(targetHeading - gyro.getAbsoluteAngle()))
                    target += 360;
                else if(Math.abs((gyro.getAbsoluteAngle() - 360) - gyro.getAbsoluteAngle()) < Math.abs(targetHeading - gyro.getAbsoluteAngle()))
                    target -= 360;

                double error = gyro.getAbsoluteAngle() - target;
                double lastError = error;
                long lastTime = System.nanoTime();
                while(Math.abs(error) > tolerance && !isStopRequested() && gamepad1.left_trigger < .2)
                {
                    error = gyro.getAbsoluteAngle() - target;
                    long time = System.nanoTime();

                    double P = error * kP;
                    double D = (error - lastError) / (time - lastTime) * kD;

                    drive.move(0, 0, P + D);

                    lastTime = time;
                    lastError = error;
                    telemetry.clear();
                    telemetry.addLine("Heading: " + gyro.getAngle());
                    telemetry.addLine("Error: " + error);
                    telemetry.update();
                }
                drive.move(0, 0, 0);
            }

            //Heading maintained circle
            if(gamepad1.y)
            {
                long start = System.currentTimeMillis();
                long time = System.currentTimeMillis();
                headingLock = gyro.getAngle();
                while(time - start < 5000 && !isStopRequested() && gamepad1.left_trigger < .2)
                {
                    time = System.currentTimeMillis();
                    double delta = (double) (time - start);
                    delta /= 5000;
                    delta *= 2 * Math.PI;
                    double x2 = Math.sin(delta);
                    double y2 = Math.cos(delta);

                    double error = gyro.getAngle() - headingLock;
                    long t = System.nanoTime();

                    double P = strafeKP * error;
                    strafeI += (t - lastStrafeTime) * error * strafeKI;
                    double D = (error - lastStrafeError) / (t - lastStrafeTime) * strafeKD;

                    double turn = P + strafeI + D;
                    if(turn < -1)
                        turn = -1;
                    else if(turn > 1)
                        turn = 1;

                    rx = turn;

                    lastStrafeError = error;
                    lastStrafeTime = time;

                    double theta2 = Math.atan2(y2, x2);
                    double r2 = Math.sqrt((x2 * x2) + (y2 * y2));

                    drive.move(theta2, r2, rx);
                }
            }

            //Differential turn
            if(gamepad1.x)
            {

                drive.getBL().setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.getFL().setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.getBR().setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.getFR().setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

                drive.getBL().setPower(.2);
                drive.getFL().setPower(.2);
                drive.getBR().setPower(1);
                drive.getFR().setPower(1);

                double lastAngle = gyro.getAngle();
                long lastTime = System.nanoTime();

                while(!isStopRequested() && gamepad1.left_trigger < .2)
                {
                    drive.getBL().setPower(.2);
                    drive.getFL().setPower(.2);
                    drive.getBR().setPower(1);
                    drive.getFR().setPower(1);

                    double angle = gyro.getAngle();
                    long time = System.nanoTime();
                    double deltaTime = time - lastTime;
                    double deltaAngle = angle - lastAngle;
                    telemetry.clear();
                    telemetry.addLine("Heading: " + gyro.getAngle() + " degrees");
                    telemetry.addLine("Angular Velocity: " + deltaAngle / deltaTime * 1000000000 + " dps");
                    telemetry.addLine("FL RPM:  " + drive.getFL().getRPM() + " rpm");
                    telemetry.addLine("BL RPM:  " + drive.getBL().getRPM() + " rpm");
                    telemetry.addLine("FR RPM:  " + drive.getFR().getRPM() + " rpm");
                    telemetry.addLine("BR RPM:  " + drive.getBR().getRPM() + " rpm");
                    telemetry.update();
                    lastAngle = angle;
                    lastTime = time;
                }

                drive.getBL().setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.getFL().setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.getBR().setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.getFR().setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }

            if(gamepad1.b) //Differential turn using heading control
            {
                double targetHeading;
                double initAngle = gyro.getAngle();

                int turnRadius = 12;
                int turnDegrees = 90;

                int leftTarget = (int) (2 * (turnRadius - drive.wheelWidth / 2) * Math.PI * drive.CPI * ((double) turnDegrees / 360));
                int rightTarget = (int) (2 * (turnRadius + drive.wheelWidth / 2) * Math.PI * drive.CPI * ((double) turnDegrees / 360));

                drive.getBL().setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.getFL().setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.getBR().setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.getFR().setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                drive.getBL().setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.getFL().setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.getBR().setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.getFR().setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                while(!isStopRequested() && gamepad1.left_trigger < .2 && Math.abs(drive.getBL().getCurrentPosition() - leftTarget) > 10) {
                    targetHeading = initAngle + (double) drive.getBL().getCurrentPosition() / (double) leftTarget * 360 * (360 / turnDegrees);
                    double error = gyro.getAngle() - targetHeading;
                    long time = System.nanoTime();

                    double P = strafeKP * error;
                    strafeI += (time - lastStrafeTime) * error * strafeKI;
                    double D = (error - lastStrafeError) / (time - lastStrafeTime) * strafeKD;

                    double turn = P + strafeI + D;
                    if(turn < -1)
                        turn = -1;
                    else if(turn > 1)
                        turn = 1;

                    rx = turn;

                    telemetry.clear();
                    telemetry.addLine("Error " + error);
                    telemetry.addLine("Left Target " + leftTarget);
                    telemetry.addLine("pos  " + drive.getBL().getCurrentPosition());
                    telemetry.update();

                    drive.move(Math.PI / 2, 1, rx);

                    lastStrafeError = error;
                    lastStrafeTime = time;
                }
            }
        }

    }

}
