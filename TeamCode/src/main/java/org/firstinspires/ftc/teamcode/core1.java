package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "First_Teleop2")
public class core1 extends LinearOpMode {
    double vel = 5;

    CRServo core;
    CRServo core2;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        boolean isPressed = false;
        boolean isPressed2 = false;

        core = hardwareMap.get(CRServo.class, "servin");
        core2 = hardwareMap.get(CRServo.class, "servin2");

        core.setDirection(DcMotor.Direction.FORWARD);
        core2.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                core.setPower(vel);
            }
            if(gamepad1.b){
                core.setPower(-vel);
            }

            if(gamepad1.y){
                core2.setPower(vel);
            }
            if(gamepad1.x){
                core2.setPower(-vel);
            }

            if(!gamepad1.a && !gamepad1.b){
                core.setPower(0);
            }
            if(!gamepad1.x && !gamepad1.y){
                core2.setPower(0);
            }


        }
    }

}
