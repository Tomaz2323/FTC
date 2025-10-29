package org.firstinspires.ftc.teamcode.cam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.cam.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

@Autonomous(name = "Auto Sigue AprilTag")
public class FollowAT extends LinearOpMode {

    double vel;
    private static final double roda_frente = Math.toRadians(0);
    private static final double roda_esquerda = Math.toRadians(120);
    private static final double roda_direita = Math.toRadians(240);

    DcMotorEx motorFrente;
    DcMotorEx motorEsquerda;
    DcMotorEx motorDireita;

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

        motorFrente = hardwareMap.get(DcMotorEx.class, "frente");
        motorEsquerda = hardwareMap.get(DcMotorEx.class, "esquerda");
        motorDireita = hardwareMap.get(DcMotorEx.class, "direta");

        motorFrente.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerda.setDirection(DcMotor.Direction.FORWARD);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);

        motorFrente.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorEsquerda.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        vel = 0.6;

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

        while (opModeIsActive()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            AprilTagDetection targetTag = null;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == TARGET_TAG_ID) {
                    targetTag = tag;
                    break;
                }
            }

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
                telemetry.addData("Erro Dist (m)", "%.2f", rangeError);
                telemetry.addData("Erro Strafe (m)", "%.2f", strafeError);
                telemetry.addData("Erro Giro (rad)", "%.2f", headingError);

            } else {
                OmniDrive(0, 0, 0);

                telemetry.addLine("Procurando Tag ID: " + TARGET_TAG_ID);
            }

            telemetry.update();
        }

        OmniDrive(0, 0, 0);
    }

    public void OmniDrive(double frente, double lateral, double giro) {

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

        motorFrente.setPower(-power1);
        motorEsquerda.setPower(-power2);
        motorDireita.setPower(-power3);

        telemetry.addData("Power frente:", power1);
        telemetry.addData("Power esquerda:", power2);
        telemetry.addData("Power direita:", power3);
    }
}