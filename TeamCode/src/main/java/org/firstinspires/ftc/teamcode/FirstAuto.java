package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.cam.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "First_Autonomous_CM")
public class FirstAuto extends LinearOpMode {

    // ===== CONSTANTES DE CONFIGURAÇÃO =====
    double vel = 0.6;
    private static final double roda_frente = Math.toRadians(90);
    private static final double roda_esquerda = Math.toRadians(210);
    private static final double roda_direita = Math.toRadians(330);

    // ===== CONSTANTES DE ENCODER =====
    // HD Hex Motor: 28 counts por revolução
    static final double HD_COUNTS_PER_REV = 28;
    // Redução de engrenagem (ajuste conforme seu kit)
    static final double DRIVE_GEAR_REDUCTION = 15;
    // Circunferência da roda em mm (90mm de diâmetro)
    static final double WHEEL_CIRCUMFERENCE_MM = 90 * Math.PI;
    // Counts por milímetro
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    // Counts por centímetro
    static final double DRIVE_COUNTS_PER_CM = DRIVE_COUNTS_PER_MM * 10;

    //câmera
    double fx = 475.144;
    double fy = 475.144;
    double cx = 302.239;
    double cy = 233.107;

    double tagsize = 0.14;

    final int TARGET_TAG_ID = 585;
    final double DESIRED_DISTANCE_METERS = 0.4; //4 cm

    final double KP_FRENTE = 0.7;
    final double KP_LATERAL = 1.0;
    final double KP_GIRO = 0.8;

    final double DISTANCE_TOLERANCE = 0.02; // 2 cm
    final double LATERAL_TOLERANCE = 0.02;  // 2 cm
    final double ANGLE_TOLERANCE = 0.05;

    DcMotorEx motorFrente;
    DcMotorEx motorEsquerda;
    DcMotorEx motorDireita;
    DcMotorEx shooter;

    CRServo servoIntake;
    CRServo servoIntake2;
    CRServo servoIntake3;
    Servo servoOutake;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Inicialização dos motores
        motorFrente = hardwareMap.get(DcMotorEx.class, "frente");
        motorEsquerda = hardwareMap.get(DcMotorEx.class, "esquerda");
        motorDireita = hardwareMap.get(DcMotorEx.class, "direta");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        servoIntake = hardwareMap.get(CRServo.class, "servoIntake");
        servoIntake2 = hardwareMap.get(CRServo.class, "servoIntake2");
        servoIntake3 = hardwareMap.get(CRServo.class, "servoIntake3");

        servoOutake = hardwareMap.get(Servo.class, "servoOutake");

        // Resetar encoders
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrente.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerda.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorDireita.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFrente.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorEsquerda.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorDireita.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Erro", "Câmera bagulhou");
                telemetry.update();
            }
        });

        // Caso queira ver a câmera no Drive
        //telemetry.setMsTransmissionInterval(50);
        //telemetry.addLine("Câmera Inicializada!");
        //telemetry.addLine("Aguardando início...");
        //telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            // Aqui é a prog que o robô vai executar ao iniciar
            // Tá em CM
            alignWithAprilTag();

            /*
            servoIntake.setPower(1);
            servoIntake2.setPower(1);
            sleep(9000);
            servoIntake.setPower(0);
            servoIntake2.setPower(0);
            sleep(400);
            servoIntake3.setPower(-1);
            shooter.setPower(0.2);
            sleep(5000);
            servoIntake3.setPower(0);
            shooter.setPower(0);
            sleep(3000);
            OmniDriveByCM(80, 0, 0, 0.4);
            sleep(2000);
            shooter.setPower(1);
            sleep(1500);
            servoOutake.setPosition(-0.2);
            sleep(3000);
            servoOutake.setPosition(0.7);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            */

        }
    }
    public void OmniDriveByCM(double frente, double lateral, double giro, double power) {
        // Calcular quantidade de rotações necessárias para cada motor
        // usando a cinemática omni do robô triangular

        double offset = Math.toRadians(30);

        double theta1 = roda_frente + offset;
        double theta2 = roda_esquerda + offset;
        double theta3 = roda_direita + offset;

        // Calcular componentes de movimento para cada motor
        double move1 = (-Math.sin(theta1) * frente - Math.cos(theta1) * lateral + giro);
        double move2 = (-Math.sin(theta2) * frente - Math.cos(theta2) * lateral + giro);
        double move3 = (-Math.sin(theta3) * frente - Math.cos(theta3) * lateral + giro);

        // Converter para counts do encoder
        int target1 = (int)(move1 * DRIVE_COUNTS_PER_CM);
        int target2 = (int)(move2 * DRIVE_COUNTS_PER_CM);
        int target3 = (int)(move3 * DRIVE_COUNTS_PER_CM);

        // Posições alvo
        int targetFrente = motorFrente.getCurrentPosition() + target1;
        int targetEsquerda = motorEsquerda.getCurrentPosition() + target2;
        int targetDireita = motorDireita.getCurrentPosition() + target3;

        // Definir posições alvo
        motorFrente.setTargetPosition(targetFrente);
        motorEsquerda.setTargetPosition(targetEsquerda);
        motorDireita.setTargetPosition(targetDireita);

        // Mudar para modo RUN_TO_POSITION
        motorFrente.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorEsquerda.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorDireita.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Definir potência
        motorFrente.setPower(power);
        motorEsquerda.setPower(power);
        motorDireita.setPower(power);

        // Aguardar até que todos os motores terminem
        while (opModeIsActive() &&
                (motorFrente.isBusy() || motorEsquerda.isBusy() || motorDireita.isBusy())) {

            telemetry.addData("Frente Busy", motorFrente.isBusy());
            telemetry.addData("Esquerda Busy", motorEsquerda.isBusy());
            telemetry.addData("Direita Busy", motorDireita.isBusy());
            telemetry.addData("Frente Pos", motorFrente.getCurrentPosition());
            telemetry.addData("Esquerda Pos", motorEsquerda.getCurrentPosition());
            telemetry.addData("Direita Pos", motorDireita.getCurrentPosition());
            telemetry.update();

            sleep(10);
        }

        motorFrente.setPower(0);
        motorEsquerda.setPower(0);
        motorDireita.setPower(0);

        telemetry.addData("Status", "Movimento completado");
        telemetry.update();
    }

    public void OmniDrive(double frente, double lateral, double giro){

        double offset = Math.toRadians(30);

        double theta1 = roda_frente + offset;
        double theta2 = roda_esquerda + offset;
        double theta3 = roda_direita + offset;

        double power1 = (-Math.sin(theta1) * frente - Math.cos(theta1) * lateral + giro) * vel;
        double power2 = (-Math.sin(theta2) * frente - Math.cos(theta2) * lateral + giro) * vel;
        double power3 = (-Math.sin(theta3) * frente - Math.cos(theta3) * lateral + giro) * vel;

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

    public void alignWithAprilTag() {
        long startTime = System.currentTimeMillis();
        long maxAlignmentTime = 10000; // 10 segundos máximo

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < maxAlignmentTime) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            AprilTagDetection targetTag = null;

            // Procurar pela tag alvo
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == TARGET_TAG_ID) {
                    targetTag = tag;
                    break;
                }
            }

            if (targetTag != null) {
                // Calcular erros
                double rangeError = (targetTag.pose.z - DESIRED_DISTANCE_METERS);
                double strafeError = targetTag.pose.x;

                // Extrair ângulo de rotação
                Orientation rot = Orientation.getOrientation(targetTag.pose.R,
                        org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC,
                        org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YXZ,
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

                double headingError = rot.firstAngle;

                // Verificar se está dentro da tolerância
                if (Math.abs(rangeError) < DISTANCE_TOLERANCE &&
                        Math.abs(strafeError) < LATERAL_TOLERANCE &&
                        Math.abs(headingError) < ANGLE_TOLERANCE) {

                    telemetry.addLine("=== ALINHAMENTO CONCLUÍDO ===");
                    telemetry.addData("Distância", String.format("%.3f m", targetTag.pose.z));
                    telemetry.addData("Erro Lateral", String.format("%.3f m", strafeError));
                    telemetry.addData("Erro Angular", String.format("%.3f rad", headingError));
                    telemetry.update();

                    OmniDrive(0, 0, 0);
                    sleep(200);
                    break;
                }

                // Calcular potências com PID
                double frente_power = rangeError * KP_FRENTE;
                double lateral_power = -strafeError * KP_LATERAL;
                double giro_power = -headingError * KP_GIRO;

                // Aplicar movimento
                OmniDrive(frente_power, lateral_power, giro_power);

                telemetry.addLine("=== ALINHANDO COM APRIL TAG ===");
                telemetry.addData("Tag ID", targetTag.id);
                telemetry.addData("Distância (m)", String.format("%.3f", targetTag.pose.z));
                telemetry.addData("Erro Frente (m)", String.format("%.3f", rangeError));
                telemetry.addData("Erro Lateral (m)", String.format("%.3f", strafeError));
                telemetry.addData("Erro Angular (rad)", String.format("%.3f", headingError));
                telemetry.addData("Power Frente", String.format("%.3f", frente_power));
                telemetry.addData("Power Lateral", String.format("%.3f", lateral_power));
                telemetry.addData("Power Giro", String.format("%.3f", giro_power));
                telemetry.update();

            } else {
                telemetry.addLine("TAG NÃO DETECTADA!");
                telemetry.addData("Procurando ID", TARGET_TAG_ID);
                telemetry.update();
                OmniDrive(0, 0, 0);
            }

            sleep(20); // Pequeno delay para não sobrecarregar
        }

        // Garantir que o robô pare
        OmniDrive(0, 0, 0);

        if ((System.currentTimeMillis() - startTime) >= maxAlignmentTime) {
            telemetry.addLine("TIMEOUT: Alinhamento não foi possível");
            telemetry.update();
        }
    }
}
