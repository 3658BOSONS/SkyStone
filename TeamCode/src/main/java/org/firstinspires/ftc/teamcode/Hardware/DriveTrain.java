package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain
{

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor br;
    private DcMotor bl;

    private Gyro gyro;
    private int fieldOriented;

    public DriveTrain(DcMotor fl, DcMotor fr, DcMotor br, DcMotor bl, Gyro gyro)
    {
        this.fl = fl;
        this.fr = fr;
        this.br = br;
        this.bl = bl;

        this.gyro = gyro;

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);

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

        direction += gyro.getAngle() * fieldOriented;

        //Convert from polar to cartesian
        double x = Math.cos(Math.toRadians(direction)) * power;
        double y = Math.sin(Math.toRadians(direction)) * power;

        //Calculate the wheel powers
        double fl_power = y + x - turn;
        double fr_power = y - x + turn;
        double br_power = y + x + turn;
        double bl_power = y - x - turn;

        //Turn put the values outside the range of -1 to 1, put them back

        //Find max
        double max = Math.max(fl_power, fr_power);
        max = Math.max(max, br_power);
        max = Math.max(max, bl_power);

        //Find scale
        double scale = 1.0 / max;

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

    public void setFieldOriented(boolean fieldOriented)
    {
        this.fieldOriented = fieldOriented ? 1 : 0;
    }

}
