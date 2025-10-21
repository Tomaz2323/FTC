package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "First_Teleop2")
public class core1 extends LinearOpMode {
    DcMotorEx core;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization

        core = hardwareMap.get(DcMotorEx.class, "core2");

        core.setDirection(DcMotor.Direction.FORWARD);

        core.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a){
                core.setPower(1);
            }
            if(gamepad1.b){
                core.setPower(-1);
            }
            if(!gamepad1.a && !gamepad1.b){
                core.setPower(0);
            }
        }
    }

}
