package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;

public class DrivingFunctions
{

    private DriveTrain dt;

    public DrivingFunctions(DriveTrain dt)
    {
        this.dt = dt;
    }

    public void differentialDrive(int radius, int time)
    {
        double outerRadius = radius + dt.wheelWidth / 2;
        double innerRadius = radius - dt.wheelWidth / 2;

        double speedRatio = innerRadius / outerRadius;

        int vMax = 1;
        int countsPerRotation = 1120;

        double rightPower = 1;
        double leftPower = speedRatio;

        long startTime = System.currentTimeMillis();

        setLeftPower(0);
        setRightPower(0);

        while(System.currentTimeMillis() - startTime < time)
        {
            setLeftPower(leftPower);
            setRightPower(rightPower);

            double rightSpeed = Math.abs(dt.getFR().getRPM());
            double leftSpeed = Math.abs(dt.getFL().getRPM());

            if(rightSpeed != 0) {
                double ratio = leftSpeed / rightSpeed;
                double error = speedRatio - ratio;
            }
        }

    }

    private void setLeftPower(double power)
    {
        dt.getFL().setPower(power);
        dt.getBL().setPower(power);
    }

    private void setRightPower(double power)
    {
        dt.getFR().setPower(power);
        dt.getBR().setPower(power);
    }

}
