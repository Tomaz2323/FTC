package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "First_Teleop2")
public class LinearTeleOp2 extends LinearOpMode {
    double vel = 0.2;
    private static final double roda_frente = Math.toRadians(0);
    private static final double roda_esquerda = Math.toRadians(120);
    private static final double roda_direita = Math.toRadians(240);

    DcMotorEx motorFrente;
    DcMotorEx motorEsquerda;
    DcMotorEx motorDireita;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        boolean isPressed = false;
        boolean isPressed2 = false;

        motorFrente = hardwareMap.get(DcMotorEx.class, "frente");
        motorEsquerda = hardwareMap.get(DcMotorEx.class, "esquerda");
        motorDireita = hardwareMap.get(DcMotorEx.class, "direta");

        motorFrente.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerda.setDirection(DcMotor.Direction.FORWARD);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);

        motorFrente.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorEsquerda.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            vel = Math.min(Math.max(vel, 0.2), 1.0);

            if(gamepad1.right_bumper && !isPressed){
                vel += 0.2;
                isPressed = true;
            }
            else if(!gamepad1.right_bumper){
                isPressed = false;

                if(gamepad1.left_bumper && !isPressed2){
                    vel -= 0.2;
                    isPressed2 = true;
                }else if(!gamepad1.left_bumper) {
                    isPressed2 = false;
                }

            }

            vel = Math.min(Math.max(vel, 0.2), 0.8);

            double frente = gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double giro = gamepad1.right_stick_x;

            OmniDrive(frente, lateral, giro);

            telemetry.addData("Velocidade do robô:", vel);
            telemetry.update();

        }
    }
    public void OmniDrive(double frente, double lateral, double giro){

        double power1 = (-Math.sin(roda_frente) * frente + Math.cos(roda_frente) * lateral + giro) * vel;
        double power2 = (-Math.sin(roda_esquerda) * frente + Math.cos(roda_esquerda) * lateral + giro) * vel;
        double power3 = (-Math.sin(roda_direita) * frente + Math.cos(roda_direita) * lateral + giro) * vel;

        double maxPower = Math.max(Math.abs(power1), Math.abs(power2));
        maxPower = Math.max(maxPower, Math.abs(power3));

        if (maxPower > 0.8) {
            power1 /= maxPower;
            power2 /= maxPower;
            power3 /= maxPower;
        }

        motorFrente.setPower(power1);
        motorEsquerda.setPower(power2);
        motorDireita.setPower(power3);

        telemetry.addData("Power frente:", power1);
        telemetry.addData("Power esquerda:", power2);
        telemetry.addData("Power direita:", power3);

    }
}
