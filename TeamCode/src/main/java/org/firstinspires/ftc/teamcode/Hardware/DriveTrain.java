package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain
{

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor bl;

    private int fieldOriented;

    public DriveTrain(DcMotor fl, DcMotor fr, DcMotor br, DcMotor bl)
    {
        this.fl = fl;
        this.fr = fr;
        this.br = br;
        this.bl = bl;

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //One thing to note is this works off a normal trig plane, it is not converted, so 0 is right, not forward.
    //Also power must be a value between 0 and 1.  Although technically it should be -1 to 1, it doesn't make much sense to input -1.
    //Turn is the amount of turn with the strafe.  -1 will be full power counterclockwise, 1 will be full power clockwise.
    public void move(double direction, double power, double turn)
    {
        //Convert from polar to cartesian
        double x = Math.cos(direction) * power;
        double y = Math.sin(direction) * power;

        //Calculate the wheel powers
        double fl_power = y - x*1.5 + turn;
        double fr_power = y + x*1.5 - turn;
        double br_power = y - x*1.5 - turn;
        double bl_power = y + x*1.5 + turn;

        //Turn put the values outside the range of -1 to 1, put them back

        //Find max
        double max = Math.max(Math.abs(fl_power), Math.abs(fr_power));
        max = Math.max(max, Math.abs(br_power));
        max = Math.max(max, Math.abs(bl_power));

        //Find scale
        double scale = 1.0;
        if(max > 1.0){
            scale = Math.abs(1.0 / max);
        }

        //Scale
        fl_power *= scale;
        fr_power *= scale;
        bl_power *= scale;
        br_power *= scale;

        //Set Power
        fl.setPower(fl_power);
        fr.setPower(fr_power);
        bl.setPower(bl_power);
        br.setPower(br_power);
    }

}
