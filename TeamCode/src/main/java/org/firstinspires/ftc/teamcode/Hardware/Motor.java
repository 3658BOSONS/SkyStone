package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Motor //DCMotor Wrapper Class with added functionality and usb communication spam avoidance and speedometer
{

    private DcMotor motor;
    private double lastPower;
    private int lastCount;
    private long lastTime;
    private double rpm;
    private LinearOpMode op;
    private boolean isSpeedometerEnabled;

    public Motor(String name, LinearOpMode op)
    {
        motor = op.hardwareMap.get(DcMotor.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.op = op;
        isSpeedometerEnabled = false;
        rpm = 0;
        lastTime = System.nanoTime();
        lastCount = 0;
    }

    public void setConstants(DcMotor.RunMode mode, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor.Direction direction)
    {
        motor.setMode(mode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setDirection(direction);
    }

    public void setRunMode(DcMotor.RunMode mode)
    {
        motor.setMode(mode);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior)
    {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setDirection(DcMotor.Direction direction)
    {
        motor.setDirection(direction);
    }
    public void setTargetPosition(int position)
    {
        motor.setTargetPosition(position);
    }

    public void setPower(double power)
    {
        //Update speedometer.  Best accuracy will come with ticking this twice in quick succession
        if(isSpeedometerEnabled)
        {
            int count = motor.getCurrentPosition();
            long time = System.nanoTime();
            double dCdT = (count - lastCount) / ((double) (time - lastTime) / 1000000000); //CurrentCPS
            double rps = dCdT / 28 / 40;
            rpm = rps * 60;
            lastCount = count;
            lastTime = time;
        }

        if(power == lastPower)
            return;
        if(power < -1)
            power = -1;
        else if(power > 1)
            power = 1;
        motor.setPower(power);
        lastPower = power;
    }

    public double getRPM()
    {
        return rpm;
    }
    public int getCurrentPosition(){
        return motor.getCurrentPosition();
    }

    public void startSpeedometer()
    {
        isSpeedometerEnabled = true;
    }

    public void stopSpeedometer()
    {
        isSpeedometerEnabled = true;
    }

}
