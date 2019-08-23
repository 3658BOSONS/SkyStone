package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveTrain
{

    private Motor fl;
    private Motor fr;
    private Motor br;
    private Motor bl;

    private int fieldOriented;
    public static final double wheelWidth = 15.5;
    public static final int CPR = 28;
    public static final int wheelSize = 4;
    public static final int ratio = 40;
    public static final int CPI = CPR * ratio / wheelSize;

    public DriveTrain(LinearOpMode op)
    {
        fl = new Motor("fl", op);
        fr = new Motor("fr", op);
        br = new Motor("br", op);
        bl = new Motor("bl", op);

        fl.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.Direction.REVERSE);
        bl.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.Direction.REVERSE);
        fr.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.Direction.FORWARD);
        br.setConstants(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.Direction.FORWARD);
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

    public Motor getFL(){
        return fl;
    }
    public Motor getBR(){
        return br;
    }
    public Motor getBL(){
        return bl;
    }
    public Motor getFR(){
        return fr;
    }

}
