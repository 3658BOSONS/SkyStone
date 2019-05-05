package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class OdometryWheel
{

    private AnalogInput encoder;

    //Angles are in radians
    private double lastAngle;
    private double thisAngle;
    private double wraps;
    private double zeroPoint;

    private Thread update;

    public OdometryWheel(AnalogInput encoder)
    {
        this.encoder = encoder;
        zeroPoint = 0;
    }

    public void reset()
    {
        lastAngle = 0;
        thisAngle = 0;
        wraps = 0;
        zeroPoint = thisAngle;
    }

    public double getGlobalAngle()
    {
        return (wraps + thisAngle - zeroPoint) * (2 * Math.PI);
    }

    private Runnable read = new Runnable(){
        @Override
        public void run()
        {
            thisAngle = Math.abs(encoder.getVoltage() / encoder.getMaxVoltage());
            if(lastAngle > .75 && thisAngle < .25)
                wraps++;
            else if(lastAngle < .25 && thisAngle > .75) {
                wraps --;
            }

            lastAngle = thisAngle;
        }
    };

}
