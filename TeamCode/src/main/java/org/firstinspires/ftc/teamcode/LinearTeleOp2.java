package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "First_Teleop2")
public class LinearTeleOp2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        DcMotorEx motorOne;
        motorOne = hardwareMap.get(DcMotorEx.class, "motor_one");

        motorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int degrees = 105;
        double grau_motor = motorOne.getCurrentPosition();
        boolean isPressed = false;
        boolean isPressed2 = false;
        double vel = 0.2;

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.left_stick_y > -0.2 && gamepad1.left_stick_y < 0.2){
                motorOne.setPower(0);
                motorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            if(gamepad1.left_stick_y > 0.2){
                motorOne.setPower(vel);
            }

            if(gamepad1.left_stick_y < -0.2){
                motorOne.setPower(-vel);
            }

            if(gamepad1.right_bumper && !isPressed){
                vel += 0.2;
                isPressed = true;
            }else if(!gamepad1.right_bumper){
                isPressed = false;

            if(gamepad1.left_bumper && !isPressed2){
                vel -= 0.2;
                isPressed2 = true;
            }else if(!gamepad1.left_bumper) {
                isPressed2 = false;
            }

        }
    }
}
}
