package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class OdometryWheel
{

    private AnalogInput encoder;

    private double lastAngle;
    private double globalAngle;

    private Thread update;

    public OdometryWheel(AnalogInput encoder)
    {
        this.encoder = encoder;
        lastAngle = 0;
        globalAngle = 0;


    }

    private Runnable read = new Runnable(){
        @Override
        public void run()
        {
            double thisAngle = Math.abs(encoder.getVoltage() / encoder.getMaxVoltage());



            lastAngle = thisAngle;
        }
    };

}
