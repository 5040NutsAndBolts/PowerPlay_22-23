package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Mechanism Test", group = "Teleop")
public class MechanismTest extends OpMode
{
    public DcMotorEx wheelA;

    @Override
    public void init() //initialization method
    {
        wheelA = hardwareMap.get(DcMotorEx.class, "wheel A");
        wheelA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() //teleop loop
    {
        if(gamepad1.left_trigger == 0)
            wheelA.setPower(-gamepad1.right_trigger * .5);
        else
            wheelA.setPower(gamepad1.left_trigger * .5);
    }
}
