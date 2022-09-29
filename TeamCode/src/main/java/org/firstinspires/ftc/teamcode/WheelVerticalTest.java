package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Wheel Vertical Test", group = "Teleop")

public class WheelVerticalTest extends OpMode
{
    public DcMotorEx wheelA;
    public DcMotorEx wheelB;

    @Override
    public void init()
    {
        wheelA = hardwareMap.get(DcMotorEx.class, "wheel A");
        wheelB = hardwareMap.get(DcMotorEx.class, "wheel B");
        wheelA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop()
    {
        wheelA.setPower(gamepad1.right_trigger);
        wheelB.setPower(-gamepad1.right_trigger);
        wheelA.setPower(-gamepad1.left_trigger);
        wheelB.setPower(gamepad1.left_trigger);
    }
}
