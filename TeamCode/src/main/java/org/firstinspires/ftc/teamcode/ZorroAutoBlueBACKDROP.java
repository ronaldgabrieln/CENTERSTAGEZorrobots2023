package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "BLUEBACKDROP")
public class ZorroAutoBlueBACKDROP extends LinearOpMode {
    //Garras
    private Servo GarraIzq;
    private Servo GarraDer;
    //Sistema de poleas
    private DcMotor ElevadorDer;
    //Eje
    private Servo Muneca;
    private DcMotor Brazo;
    private ModernRoboticsI2cRangeSensor Distancias;
    //Mamadas
    OpenCvWebcam CAM = null;
    int direccion = 0;

    int TARGETID = -1;
    WebcamName NombreCamara;
    int IDMONITORVIEW;
    SampleMecanumDrive drive;
    Trajectory centerPlacement;
    Trajectory backdropCenter;
    Trajectory backdropRight;
    Trajectory backdropLeftAlignment;
    Trajectory backdropLeftAdvancement;
    Trajectory parkLeft;
    Trajectory parkRight;
    Trajectory eighteenInchAdvancementToSpikeMark;
    Trajectory twoInchAdvancementToSpikeMark;
    Trajectory centerPlacementTwoMhuahahaha;
    Trajectory moveBackdropToLeft;
    Trajectory centerPlacementThree;
    Trajectory strafeToCenter;
    Trajectory parkCenter;
    Trajectory adjustLeftBackdrop;

    @Override
    public void runOpMode() throws InterruptedException {
        //OPENCV
        NombreCamara = hardwareMap.get(WebcamName.class, "CAM");

        IDMONITORVIEW = hardwareMap.appContext.getResources().getIdentifier("IDMONITORVIEW", "ID", hardwareMap.appContext.getPackageName());
        CAM = OpenCvCameraFactory.getInstance().createWebcam(NombreCamara, IDMONITORVIEW);
        CAM.setPipeline(new sigmaPipeline());


        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-54, 9, Math.toRadians(00));
        drive.setPoseEstimate(startPose);

        centerPlacement =  drive.trajectoryBuilder(startPose)
                .forward(35)
                .build();
        centerPlacementThree =  drive.trajectoryBuilder(startPose)
                .forward(35)
                .build();
        centerPlacementTwoMhuahahaha =  drive.trajectoryBuilder(startPose)
                .forward(21)
                .build();
        //

        backdropCenter = drive.trajectoryBuilder(centerPlacementTwoMhuahahaha.end().plus(new Pose2d(0,0,Math.toRadians(-180))).plus(new Pose2d(0,0,Math.toRadians(90))), false)
                .back(42.5)
                .build();

        backdropLeftAlignment = drive.trajectoryBuilder(backdropCenter.end())
                .strafeRight(16)
                .build();

        eighteenInchAdvancementToSpikeMark = drive.trajectoryBuilder(centerPlacement.end().plus(new Pose2d(0,0,Math.toRadians(90))), false)
                .forward(28)
                .build();
        backdropLeftAdvancement = drive.trajectoryBuilder(eighteenInchAdvancementToSpikeMark.end().plus(new Pose2d(0,0,Math.toRadians(180))), false)
                .back(11)
                .build();
        twoInchAdvancementToSpikeMark = drive.trajectoryBuilder(centerPlacementThree.end() .plus(new Pose2d(0,0,Math.toRadians(90))), false)
                .forward(5)
                .build();
        backdropRight = drive.trajectoryBuilder(twoInchAdvancementToSpikeMark.end().plus(new Pose2d(0,0,Math.toRadians(-180))),false )
                .back(33.5)
                .build();
        moveBackdropToLeft = drive.trajectoryBuilder(backdropLeftAdvancement.end())
                .strafeRight(15)
                .build();
        adjustLeftBackdrop = drive.trajectoryBuilder(moveBackdropToLeft.end())
                .back(7)
                .build();
        parkRight = drive.trajectoryBuilder(backdropRight.end())
                .strafeRight(37)
                .build();
        strafeToCenter = drive.trajectoryBuilder(backdropCenter.end())
                .strafeLeft(15)
                .build();
        parkCenter = drive.trajectoryBuilder(strafeToCenter.end())
                .strafeRight(35)
                .build();
        parkLeft = drive.trajectoryBuilder(adjustLeftBackdrop.end())
                .strafeRight(18)
                .build();
        //Elevador
        ElevadorDer = hardwareMap.get(DcMotor.class, "P");
        //Servo de garritas kawai
        GarraDer = hardwareMap.get(Servo.class, "GD");
        GarraIzq = hardwareMap.get(Servo.class, "GI");
        Muneca = hardwareMap.get(Servo.class, "M");
        Brazo = hardwareMap.get(DcMotor.class, "B");
        Brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        GarraDer.setDirection(Servo.Direction.REVERSE);
        Brazo.setDirection(DcMotorSimple.Direction.REVERSE);
        //Sensor
        Distancias = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor");

        GarraIzq.setPosition(0);
        GarraDer.setPosition(0);

        telemetry.addData("Optico", Distancias.rawOptical());
        telemetry.addData("Infrarojo", Distancias.rawUltrasonic());
        CAM.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                CAM.startStreaming(1024,  576, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Brazo: ", Brazo.getCurrentPosition());
                if (direccion == 0) {
                    //Left
                    CAM.stopStreaming();
                    CAM.stopRecordingPipeline();
                    drive.followTrajectory(centerPlacement);
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(eighteenInchAdvancementToSpikeMark);
                    Muneca.setPosition(1);
                    sleep(2000);
                    GarraDer.setPosition(1);
                    sleep(2000);
                    powerBrazo(350, .2);
                    Muneca.setPosition(.86);
                    drive.turn(Math.toRadians(180));
                    drive.followTrajectory(backdropLeftAdvancement);
                    drive.followTrajectory(moveBackdropToLeft);
                    Muneca.setPosition(.9);
                    sleep(200);
                    drive.followTrajectory(adjustLeftBackdrop);
                    GarraIzq.setPosition(1);
                    sleep(2000);
                    drive.followTrajectory(parkLeft);
                } else if (direccion == 1) {
                    //Center
                    CAM.stopStreaming();
                    CAM.stopRecordingPipeline();
                    drive.followTrajectory(centerPlacementTwoMhuahahaha);
                    Muneca.setPosition(1);
                    drive.turn(Math.toRadians(-180));
                    sleep(1000);
                    GarraDer.setPosition(1);
                    sleep(1000);
                    Muneca.setPosition(0);
                    sleep(500);
                    drive.turn(Math.toRadians(90));
                    powerBrazo(300, .2);
                    drive.followTrajectory(backdropCenter);
                    Muneca.setPosition(.86);
                    sleep(2000);
                    drive.followTrajectory(strafeToCenter);
                    Muneca.setPosition(.9);
                    sleep(200);
                    GarraIzq.setPosition(1);
                    sleep(2000);
                    Muneca.setPosition(.3);
                    drive.followTrajectory(parkCenter);
                } else {
                    //Right
                    CAM.stopStreaming();
                    CAM.stopRecordingPipeline();
                    drive.followTrajectory(centerPlacementThree);
                    Muneca.setPosition(1);
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(twoInchAdvancementToSpikeMark);
                    GarraDer.setPosition(1);
                    sleep(200);
                    Muneca.setPosition(.86);
                    drive.turn(Math.toRadians(-180));
                    powerBrazo(330, .2);
                    drive.followTrajectory(backdropRight);
                    GarraIzq.setPosition(1);
                    sleep(500);
                    drive.followTrajectory(parkRight);
                }
            break;
        }
    }

    private void powerBrazo(int ticks, double speed) {
        Brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
        Brazo.setTargetPosition(ticks);
        //
        Brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        Brazo.setPower(speed);
        while(Brazo.isBusy() && opModeIsActive()) {
            telemetry.addData("Brazo: ", Brazo.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
    class sigmaPipeline extends OpenCvPipeline {
        Mat YcbCr = new Mat();
        Mat leftCrop;
        Mat centerCrop;
        Mat rightCrop;

        double leftavgfin;
        double centeravgfin;
        double rightavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(0.0, 255.0, 0.0);
        Rect leftRect;
        Rect centRect;
        Rect rightRect;

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YcbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("sigmaPipeline Running");

            leftRect = new Rect(1, 200, 340, 281);
            centRect = new Rect(342, 200, 340, 281);
            rightRect = new Rect(683, 200, 340, 281);


            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, centRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2);

            leftCrop = YcbCr.submat(leftRect);
            centerCrop = YcbCr.submat(centRect);
            rightCrop = YcbCr.submat(rightRect);


            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(centerCrop,centerCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar centavg = Core.mean(centerCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            centeravgfin = centavg.val[0];
            rightavgfin = rightavg.val[0];
            telemetry.addData("leftblue: ", leftavgfin);
            telemetry.addData("centerblue: ", centeravgfin);
            telemetry.addData("rightblue: ", rightavgfin);




            if(leftavgfin > centeravgfin && leftavgfin > rightavgfin) {
                direccion = 0;
                telemetry.addLine("LEFT");

            } else if (centeravgfin > leftavgfin && centeravgfin > rightavgfin) {
                direccion = 1;
                telemetry.addLine("CENTER");

            } else {
                direccion = 2;
                telemetry.addLine("RIGHT");
            }


            telemetry.update();
            return(output);
        }

    }
}
