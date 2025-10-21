package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "First_Autonomous")
public class FirstAuto extends LinearOpMode {

    double vel = 0.2;
    private static final double roda_frente = Math.toRadians(90);
    private static final double roda_esquerda = Math.toRadians(210);
    private static final double roda_direita = Math.toRadians(330);

    DcMotorEx motorFrente;
    DcMotorEx motorEsquerda;
    DcMotorEx motorDireita;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization code
        motorFrente = hardwareMap.get(DcMotorEx.class, "frente");
        motorEsquerda = hardwareMap.get(DcMotorEx.class, "esquerda");
        motorDireita = hardwareMap.get(DcMotorEx.class, "direta");
        waitForStart();

        while(opModeIsActive()){
            OmniDrive(30, 0, 0);

        }
    }

    public void OmniDrive(double frente, double lateral, double giro){
        double power1 = (frente * Math.sin(roda_frente) + lateral * Math.cos(roda_frente) + giro) * vel;
        double power2 = (frente * Math.sin(roda_esquerda) + lateral * Math.cos(roda_esquerda) + giro) * vel;
        double power3 = (frente * Math.sin(roda_direita) + lateral * Math.cos(roda_direita) + giro) * vel;

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