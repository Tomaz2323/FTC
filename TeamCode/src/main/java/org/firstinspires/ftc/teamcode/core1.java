package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "First_Teleop2")
public class core1 extends LinearOpMode {
    double vel = 5;

    DcMotorEx core;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        boolean isPressed = false;
        boolean isPressed2 = false;

        core = hardwareMap.get(DcMotorEx.class, "core");

        core.setDirection(DcMotor.Direction.FORWARD);

        core.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            core.setPower(vel);

        }
    }

}
