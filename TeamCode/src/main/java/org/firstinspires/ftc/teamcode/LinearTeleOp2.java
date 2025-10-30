package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.cam.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp(name = "First_Teleop2")
public class LinearTeleOp2 extends LinearOpMode {
    double vel = 0.4;
    double vel2 = 0;
    private static final double roda_frente = Math.toRadians(90);
    private static final double roda_esquerda = Math.toRadians(210);
    private static final double roda_direita = Math.toRadians(330);

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

    double fx = 475.144;
    double fy = 475.144;
    double cx = 302.239;
    double cy = 233.107;

    double tagsize = 0.14;

    final int TARGET_TAG_ID = 585;

    final double DESIRED_DISTANCE_METERS = 0.4; // (0.4 metros = 40 cm)

    final double KP_FRENTE = 0.7;
    final double KP_LATERAL = 1.0;
    final double KP_GIRO = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        boolean isPressed = false;
        boolean isPressed2 = false;
        boolean isPressed3 = false;

        motorFrente = hardwareMap.get(DcMotorEx.class, "frente");
        motorEsquerda = hardwareMap.get(DcMotorEx.class, "esquerda");
        motorDireita = hardwareMap.get(DcMotorEx.class, "direta");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        servoIntake = hardwareMap.get(CRServo.class, "servoIntake");
        servoIntake2 = hardwareMap.get(CRServo.class, "servoIntake2");
        servoIntake3 = hardwareMap.get(CRServo.class, "servoIntake3");

        servoOutake = hardwareMap.get(Servo.class, "servoOutake");

        motorFrente.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerda.setDirection(DcMotor.Direction.FORWARD);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrente.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorEsquerda.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            }
        });

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Pronto para iniciar!");
        telemetry.addLine("Calibração da Câmera e Tamanho do Tag Corretos?");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            AprilTagDetection targetTag = null;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == TARGET_TAG_ID) {
                    targetTag = tag;
                    break;
                }
            }

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

            else if(gamepad1.a && !isPressed3){
                isPressed3 = true;
                vel2 = vel;
                vel = 1.0;
            }
            else if(!gamepad1.a && isPressed3) {
                isPressed3 = false;
                vel = vel2;
            }

            else if(gamepad1.b && !isPressed3){
                isPressed3 = true;
                if (targetTag != null) {
                    double rangeError = (targetTag.pose.z - DESIRED_DISTANCE_METERS);
                    double strafeError = targetTag.pose.x;

                    Orientation rot = Orientation.getOrientation(targetTag.pose.R,
                            org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC,
                            org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YXZ,
                            org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);

                    double headingError = rot.firstAngle;

                    double frente_power = rangeError * KP_FRENTE;
                    double lateral_power = -strafeError * KP_LATERAL;
                    double giro_power = -headingError * KP_GIRO;

                    OmniDrive(frente_power, lateral_power, giro_power);

                    telemetry.addLine("Tag Alvo Detectado!");
                } else if (!gamepad1.b && isPressed3) {
                isPressed3 = false;
                }
            }

            //gamepad 2
            if(gamepad2.a){
                servoIntake.setPower(1);
                servoIntake2.setPower(1);
            }else if(!gamepad2.a){
                servoIntake.setPower(0);
                servoIntake2.setPower(0);
            }

            if(gamepad2.x){
                servoIntake3.setPower(-1);
                shooter.setPower(0.2);
            }else if(!gamepad2.x){
                servoIntake3.setPower(0);
                shooter.setPower(0);
            }

            if(gamepad2.b){
                shooter.setPower(1);
                sleep(3000);
                servoOutake.setPosition(0);

            }
            if(!gamepad2.b){
                servoOutake.setPosition(0.7);
            }

            vel = Math.min(Math.max(vel, 0.2), 1);

            double frente = gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double giro = gamepad1.right_stick_x;

            OmniDrive(frente, lateral, giro);

            telemetry.addData("Velocidade do robô:", vel);
            telemetry.addData("Servo:", servoOutake.getPosition());
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

