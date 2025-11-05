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

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.IMU.Parameters;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; // <-- IMPORTANTE


import java.util.ArrayList;

@Autonomous(name = "First_Autonomous_CM")
public class FirstAuto extends LinearOpMode {

    // ===== CONSTANTES DE CONFIGURAÇÃO =====
    double vel = 0.2;

    private IMU imu;

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
    final double DESIRED_DISTANCE_METERS = 1.4; //4 cm

    final double KP_FRENTE = 2.5;
    final double KP_LATERAL = 2.6;
    final double KP_GIRO = 2.0;

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
    CRServo servoOutake;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private ElapsedTime runtime = new ElapsedTime();

    private double targetHeadingRads = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        targetHeadingRads = 0.0;

        // Obtenha a IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Crie os parâmetros COM a orientação
        Parameters parameters = new Parameters(orientationOnRobot);

        // Inicialize
        imu.initialize(parameters);

        // Inicialização dos motores
        motorFrente = hardwareMap.get(DcMotorEx.class, "frente");
        motorEsquerda = hardwareMap.get(DcMotorEx.class, "esquerda");
        motorDireita = hardwareMap.get(DcMotorEx.class, "direta");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        servoIntake = hardwareMap.get(CRServo.class, "servoIntake");
        servoIntake2 = hardwareMap.get(CRServo.class, "servoIntake2");
        servoIntake3 = hardwareMap.get(CRServo.class, "servoIntake3");
        servoOutake = hardwareMap.get(CRServo.class, "servoOutake");

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
        //telemetry.update()
        waitForStart();

        while(opModeIsActive()){
            // Aqui é a prog que o robô vai executar ao iniciar
            // Tá em CM
            // Esse é pro lado AZUL
            imu.resetYaw();
            alignWithAprilTag();
            sleep(1000);
            shooter.setPower(0.7);
            sleep(2000);
            Servoutake();
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            servoIntake.setPower(1);
            servoIntake2.setPower(1);
            sleep(3000);
            servoIntake.setPower(0);
            servoIntake2.setPower(0);
            sleep(400);
            servoIntake3.setPower(-1);
            shooter.setPower(0.2);
            sleep(2500);
            servoIntake3.setPower(0);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(1000);
            shooter.setPower(0.7);
            sleep(1000);
            Servoutake();
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            OmniDriveByCM(0, -80, 0, 0.8);

            //Esse é para o lado VERMELHO
            /*
            PELO AMOR DE DEUS, NÂO ESQUEÇA DE MUDAR O ID DA APRIL TAG
            imu.resetYaw();
            alignWithAprilTag();
            sleep(1000);
            shooter.setPower(0.7);
            sleep(2000);
            Servoutake();
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            servoIntake.setPower(1);
            servoIntake2.setPower(1);
            sleep(3000);
            servoIntake.setPower(0);
            servoIntake2.setPower(0);
            sleep(400);
            servoIntake3.setPower(-1);
            shooter.setPower(0.2);
            sleep(2500);
            servoIntake3.setPower(0);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            sleep(1000);
            shooter.setPower(0.7);
            sleep(1000);
            Servoutake();
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            OmniDriveByCM(0, 80, 0, 0.8);
            */
        }
    }
    public void OmniDriveByCM(double frenteCM, double lateralCM, double targetHeadingRads, double maxPower) {

        // ===== 1. CALCULAR ALVOS DE ENCODER =====
        double offset = Math.toRadians(30);
        double theta1 = roda_frente + offset;
        double theta2 = roda_esquerda + offset;
        double theta3 = roda_direita + offset;

        // Calcula o 'movimento' (distância em CM) para cada motor
        double move1 = (-Math.sin(theta1) * frenteCM - Math.cos(theta1) * lateralCM);
        double move2 = (-Math.sin(theta2) * frenteCM - Math.cos(theta2) * lateralCM);
        double move3 = (-Math.sin(theta3) * frenteCM - Math.cos(theta3) * lateralCM);

        // Converte para counts
        int targetCounts1 = (int)(move1 * DRIVE_COUNTS_PER_CM);
        int targetCounts2 = (int)(move2 * DRIVE_COUNTS_PER_CM);
        int targetCounts3 = (int)(move3 * DRIVE_COUNTS_PER_CM);

        // Posições alvo *absolutas*
        int targetFrente = motorFrente.getCurrentPosition() + targetCounts1;
        int targetEsquerda = motorEsquerda.getCurrentPosition() + targetCounts2;
        int targetDireita = motorDireita.getCurrentPosition() + targetCounts3;

        // ===== 2. CONFIGURAR PARA O LOOP MANUAL =====
        motorFrente.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorEsquerda.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorDireita.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Tolerância (em counts)
        int posTolerance = (int)(1.5 * DRIVE_COUNTS_PER_CM); // 1.5 cm

        // ===== 3. CALCULAR POTÊNCIA DE TRANSLACÃO =====
        // Normaliza o vetor de movimento para ter um 'maxPower'
        double totalDistance = Math.hypot(frenteCM, lateralCM);
        double moveFrentePower = 0;
        double moveLateralPower = 0;

        if (totalDistance > 0.1) { // Só se move se a distância for significativa
            moveFrentePower = (frenteCM / totalDistance) * maxPower;
            moveLateralPower = (lateralCM / totalDistance) * maxPower;
        }

        // ===== 4. LOOP DE CONTROLE (APENAS TRANSLACAO) =====
        while(opModeIsActive()) {

            // --- CHECAR SE TERMINOU (CONDIÇÃO DE PARADA) ---
            int posFrente = motorFrente.getCurrentPosition();
            int posEsquerda = motorEsquerda.getCurrentPosition();
            int posDireita = motorDireita.getCurrentPosition();

            int errorFrente = targetFrente - posFrente;
            int errorEsquerda = targetEsquerda - posEsquerda;
            int errorDireita = targetDireita - posDireita;

            // Checa se TODOS os motores estão dentro da tolerância
            if (Math.abs(errorFrente) < posTolerance &&
                    Math.abs(errorEsquerda) < posTolerance &&
                    Math.abs(errorDireita) < posTolerance) {
                break; // Saímos do loop, chegamos ao destino
            }

            // --- CÁLCULO DE GIRO DESATIVADO NESTA ETAPA ---
            double giroPower = 0.0; // Força o giro a ser zero durante a translação

            double power1 = (-Math.sin(theta1) * moveFrentePower - Math.cos(theta1) * moveLateralPower + giroPower);
            double power2 = (-Math.sin(theta2) * moveFrentePower - Math.cos(theta2) * moveLateralPower + giroPower);
            double power3 = (-Math.sin(theta3) * moveFrentePower - Math.cos(theta3) * moveLateralPower + giroPower);

            // Não é necessário normalizar, pois 'giroPower' é 0
            // e 'moveFrentePower'/'moveLateralPower' já foram normalizados por 'maxPower'.

            // Aplicar potência
            motorFrente.setPower(power1);
            motorEsquerda.setPower(power2);
            motorDireita.setPower(power3);

            telemetry.addData("Status", "Movendo para Posição");
            telemetry.addData("Err Frente", errorFrente);
            telemetry.addData("Err Esq", errorEsquerda);
            telemetry.addData("Err Dir", errorDireita);
            telemetry.update();
        }

        // ===== 5. PARAR MOTORES (ANTES DE CORRIGIR O GIRO) =====
        motorFrente.setPower(0);
        motorEsquerda.setPower(0);
        motorDireita.setPower(0);

        // Pequena pausa para garantir que o robô pare o movimento de translação
        sleep(50);

        // ===== 6. LOOP DE CORREÇÃO DE GIRO (NOVO) =====

        // Constantes para a correção de giro no local.
        // Recomendo um Kp_GIRO mais forte aqui (ex: 0.6)
        // O valor 0.15 que você usou pode ser muito lento ou fraco.
        double Kp_GIRO_CORRECAO = 0.1;
        double angleTolerance = Math.toRadians(1.5); // 1.5 graus de tolerância

        while(opModeIsActive()) {

            double currentHeading = getHeading();
            double headingError = targetHeadingRads - currentHeading;

            // Normaliza o erro para o intervalo [-PI, +PI]
            while (headingError > Math.PI)  headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            // Condição de parada
            if (Math.abs(headingError) < angleTolerance) {
                break; // Estamos alinhados
            }

            // Calcula a potência de giro
            double giroPower = -(headingError * Kp_GIRO_CORRECAO);

            // Adiciona uma potência mínima para vencer o atrito estático
            double minPower = 0.1;
            if (Math.abs(headingError) > angleTolerance) {
                giroPower = Math.signum(giroPower) * Math.max(minPower, Math.abs(giroPower));
            }

            // Limita a potência máxima de giro (para não girar muito rápido)
            double maxGiroPower = 0.4; // 40% de potência
            giroPower = Math.max(-maxGiroPower, Math.min(maxGiroPower, giroPower));

            // Aplicar potência de GIRO PURO
            // (Para giro no local, todos os motores recebem a mesma potência)
            motorFrente.setPower(giroPower);
            motorEsquerda.setPower(giroPower);
            motorDireita.setPower(giroPower);

            telemetry.addData("Status", "Corrigindo Ângulo");
            telemetry.addData("Target", Math.toDegrees(targetHeadingRads));
            telemetry.addData("Atual", Math.toDegrees(currentHeading));
            telemetry.addData("Erro (Graus)", Math.toDegrees(headingError));
            telemetry.addData("Giro Power", giroPower);
            telemetry.update();
        }

        // ===== 7. PARAR TUDO (FINAL) =====
        motorFrente.setPower(0);
        motorEsquerda.setPower(0);
        motorDireita.setPower(0);
    }
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
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
        long maxAlignmentTime = 10000;

        // Defina a velocidade máxima que você deseja para o alinhamento (ex: 70%)
        // Ajuste este valor para controlar a velocidade máxima do alinhamento.
        final double ALIGNMENT_SPEED_CAP = 0.4;

        // Constantes da cinemática (copiadas de OmniDrive)
        final double offset = Math.toRadians(30);
        final double theta1 = roda_frente + offset;
        final double theta2 = roda_esquerda + offset;
        final double theta3 = roda_direita + offset;

        // É uma boa prática definir o BRAKE mode UMA VEZ antes do loop
        motorFrente.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorDireita.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorEsquerda.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive() && (System.currentTimeMillis() - startTime) < maxAlignmentTime) {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();
            AprilTagDetection tag = null;

            for (AprilTagDetection t : detections) {
                if (t.id == TARGET_TAG_ID) {
                    tag = t;
                    break;
                }
            }

            if (tag != null) {
                // --- 1. Calcular Erros ---
                double rangeError = -(tag.pose.z - DESIRED_DISTANCE_METERS);
                double strafeError = tag.pose.x;
                Orientation rot = Orientation.getOrientation(
                        tag.pose.R,
                        org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC,
                        org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YXZ,
                        org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
                double headingError = rot.firstAngle;

                // Normalizar ângulo
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;

                // --- 2. Checar Condição de Parada ---
                if (Math.abs(rangeError) < DISTANCE_TOLERANCE &&
                        Math.abs(strafeError) < LATERAL_TOLERANCE &&
                        Math.abs(headingError) < ANGLE_TOLERANCE) {
                    // Chegamos ao destino
                    motorFrente.setPower(0);
                    motorEsquerda.setPower(0);
                    motorDireita.setPower(0);
                    break;
                }

                // --- 3. Calcular Potência (sem "caps" rígidos) ---
                // A potência é proporcional ao erro.
                double frente = rangeError * KP_FRENTE;
                double lateral = strafeError * KP_LATERAL;
                double giro = headingError * KP_GIRO;

                // --- 4. Aplicar Cinemática (copiado de OmniDrive, SEM 'vel') ---
                double power1 = (-Math.sin(theta1) * frente - Math.cos(theta1) * lateral + giro);
                double power2 = (-Math.sin(theta2) * frente - Math.cos(theta2) * lateral + giro);
                double power3 = (-Math.sin(theta3) * frente - Math.cos(theta3) * lateral + giro);

                // --- 5. Normalizar e Limitar Velocidade ---
                double maxPower = Math.max(Math.abs(power1), Math.abs(power2));
                maxPower = Math.max(maxPower, Math.abs(power3));

                // 'maxPower' é a maior potência calculada.
                // Se for > 1.0, precisamos normalizar (reduzir) todas proporcionalmente.
                if (maxPower > 1.0) {
                    power1 /= maxPower;
                    power2 /= maxPower;
                    power3 /= maxPower;
                }

                // Aplica o "teto" de velocidade de alinhamento
                power1 *= ALIGNMENT_SPEED_CAP;
                power2 *= ALIGNMENT_SPEED_CAP;
                power3 *= ALIGNMENT_SPEED_CAP;

                // --- 6. Aplicar Potência aos Motores ---
                motorFrente.setPower(power1);
                motorEsquerda.setPower(power2);
                motorDireita.setPower(power3);

            } else {
                // Se não vemos o tag, pare (importante!)
                motorFrente.setPower(0);
                motorEsquerda.setPower(0);
                motorDireita.setPower(0);
            }

            sleep(20); // Pausa para não sobrecarregar o loop (50hz)
        }

        // Garante que os motores parem no final
        motorFrente.setPower(0);
        motorEsquerda.setPower(0);
        motorDireita.setPower(0);
    }
    public void Servoutake(){
        servoOutake.setPower(-1);
        sleep(3500);
        servoOutake.setPower(0);
        sleep(2000);
        servoOutake.setPower(1);
        sleep(500);
    }
}