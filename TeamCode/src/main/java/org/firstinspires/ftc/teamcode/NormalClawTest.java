package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Normal Claw Test", group = "Teleop")
public class NormalClawTest extends OpMode
{
    public DcMotorEx motor1;

    @Override
    public void init()
    {
        motor1 = hardwareMap.get(DcMotorEx.class, "motor 1");
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop()
    {
        motor1.setPower(-gamepad1.right_trigger * .5);
        motor1.setPower(gamepad1.left_trigger * .5);
    }
}
