package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "First_Teleop2")
public class core1 extends LinearOpMode {
    double vel = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        CRServo servo1;
        servo1 = hardwareMap.get(CRServo.class, "servo1");

        CRServo servo2;
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                servo1.setPower(1);
                servo2.setPower(-1);
            }
            if(gamepad1.b){
                servo1.setPower(-1);
                servo2.setPower(1);
            }

        }
    }

}
