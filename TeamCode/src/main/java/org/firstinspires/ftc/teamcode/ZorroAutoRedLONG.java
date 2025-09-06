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

@Autonomous(name = "REDLONG")
public class ZorroAutoRedLONG extends LinearOpMode {
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
    Trajectory leftPlacement, leftbackfromPurplePixel, leftToStack, leftStrafetoStack, leftGoToDoor, leftGoToUpperArea, leftBackDropStrafe, leftAdvanceBackdrop, leftPegate, leftPark;
    Trajectory rightStrafeToWhiteStack, rightPlacement,rightAdjustToPurpleRight, rightToStack, rightToDoor, rightToBackdrop, rightLeavePixel, rightBackFromBackdrop;
    Trajectory centerPlacement, centerStack, centerToDoor, centerToBackdrop, centerLeavePixel, centerBackFromBackdrop;

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

        leftPlacement =  drive.trajectoryBuilder(startPose)
                .forward(35)
                .build();
        leftbackfromPurplePixel= drive.trajectoryBuilder(leftPlacement.end().plus(new Pose2d(0,0,Math.toRadians(90))), false)
                .forward(5)
                .build();
        leftToStack = drive.trajectoryBuilder(leftbackfromPurplePixel.end().plus(new Pose2d(0,0,Math.toRadians(180))), false)
                .back(11)
                .build();
        leftStrafetoStack = drive.trajectoryBuilder(leftToStack.end())
                .strafeLeft(3)
                .build();
        leftPegate = drive.trajectoryBuilder(leftStrafetoStack.end())
                .back(2.5)
                .build();
        leftGoToDoor = drive.trajectoryBuilder(leftPegate.end())
                .strafeLeft(25)
                .build();
        leftGoToUpperArea = drive.trajectoryBuilder(leftGoToDoor.end())
                .forward(85)
                .build();
        leftBackDropStrafe = drive.trajectoryBuilder(leftGoToUpperArea.end())
                .strafeRight(35)
                .build();
        leftAdvanceBackdrop = drive.trajectoryBuilder(leftBackDropStrafe.end().plus(new Pose2d(0, 0, Math.toRadians(180))), false)
                .back(29)
                .build();
        leftPark = drive.trajectoryBuilder(leftAdvanceBackdrop.end())
                .forward(5)
                .build();
        //Right
        rightPlacement =  drive.trajectoryBuilder(startPose)
                .forward(35)
                .build();
        rightAdjustToPurpleRight= drive.trajectoryBuilder(rightPlacement.end().plus(new Pose2d(0,0,Math.toRadians(-90))), false)
                .forward(3)
                .build();
        rightStrafeToWhiteStack= drive.trajectoryBuilder(rightAdjustToPurpleRight.end())
                .strafeLeft(23)
                .build();
        rightToStack = drive.trajectoryBuilder(rightStrafeToWhiteStack.end())
                .back(24.6)
                .build();
        rightToDoor = drive.trajectoryBuilder(rightToStack.end())
                .forward(85)
                .build();
        rightToBackdrop = drive.trajectoryBuilder(rightToDoor.end())
                .strafeRight(25.5)
                .build();
        rightLeavePixel = drive.trajectoryBuilder(rightToBackdrop.end().plus(new Pose2d(0,0,Math.toRadians(180))), false)
                .back(27.5)
                .build();
        rightBackFromBackdrop = drive.trajectoryBuilder(rightLeavePixel.end())
                .forward(5)
                .build();
        //Center
        centerPlacement = drive.trajectoryBuilder(startPose)
                .forward(52.5)
                .build();
        centerStack = drive.trajectoryBuilder(centerPlacement.end().plus(new Pose2d(0,0, Math.toRadians(-90))), false)
                .back(22)
                .build();
        centerToDoor = drive.trajectoryBuilder(centerStack.end())
                .forward(85)
                .build();
        centerToBackdrop = drive.trajectoryBuilder(centerToDoor.end())
                .strafeRight(30)
                .build();
        centerLeavePixel = drive.trajectoryBuilder(centerToBackdrop.end().plus(new Pose2d(0,0,Math.toRadians(-180))), false)
                .back(27)
                .build();
        centerBackFromBackdrop = drive.trajectoryBuilder(centerLeavePixel.end())
                .forward(5)
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
                    drive.followTrajectory(rightPlacement);
                    Muneca.setPosition(1);
                    drive.turn(Math.toRadians(-90));
                    drive.followTrajectory(rightAdjustToPurpleRight);
                    Muneca.setPosition(1);
                    sleep(1000);
                    GarraDer.setPosition(1);
                    sleep(1000);
                    Muneca.setPosition(.82);
                    drive.followTrajectory(rightStrafeToWhiteStack);
                    drive.followTrajectory(rightToStack);
                    GarraDer.setPosition(0);
                    sleep(1000);
                    drive.followTrajectory(rightToDoor);
                    drive.followTrajectory(rightToBackdrop);
                    powerBrazo(300,.3);
                    drive.turn(Math.toRadians(180));
                    drive.followTrajectory(rightLeavePixel);
                    GarraIzq.setPosition(1);
                    sleep(1000);
                    GarraDer.setPosition(1);
                    drive.followTrajectory(rightBackFromBackdrop);
                } else if (direccion == 1) {
                    //Center
                    CAM.stopStreaming();
                    CAM.stopRecordingPipeline();
                    drive.followTrajectory(centerPlacement);
                    Muneca.setPosition(1);
                    sleep(1000);
                    GarraDer.setPosition(1);
                    sleep(700);
                    Muneca.setPosition(.84);
                    sleep(700);
                    drive.turn(Math.toRadians(-90));
                    drive.followTrajectory(centerStack);
                    GarraDer.setPosition(0);
                    sleep(1000);
                    drive.followTrajectory(centerToDoor);
                    drive.followTrajectory(centerToBackdrop);
                    powerBrazo(300,.3);
                    drive.turn(Math.toRadians(-180));
                    drive.followTrajectory(centerLeavePixel);
                    GarraIzq.setPosition(1);
                    sleep(1000);
                    GarraDer.setPosition(1);
                    sleep(1000);
                    drive.followTrajectory(centerBackFromBackdrop);
                } else {
                    //Right
                    CAM.stopStreaming();
                    CAM.stopRecordingPipeline();
                    drive.followTrajectory(leftPlacement);
                    Muneca.setPosition(1);
                    sleep(700);
                    drive.turn(Math.toRadians(90));
                    drive.followTrajectory(leftbackfromPurplePixel);
                    GarraDer.setPosition(1);
                    sleep(700);
                    Muneca.setPosition(0);
                    sleep(700);
                    drive.turn(Math.toRadians(180));
                    drive.followTrajectory(leftToStack);
                    Muneca.setPosition(0.84);
                    sleep(700);
                    drive.followTrajectory(leftStrafetoStack);
                    drive.followTrajectory(leftPegate);
                    GarraDer.setPosition(0);
                    sleep(1500);
                    drive.followTrajectory(leftGoToDoor);
                    drive.followTrajectory(leftGoToUpperArea);
                    drive.followTrajectory(leftBackDropStrafe);
                    drive.turn(Math.toRadians(180));
                    powerBrazo(300,.3);
                    drive.followTrajectory(leftAdvanceBackdrop);
                    GarraIzq.setPosition(1);
                    sleep(1500);
                    GarraDer.setPosition(1);
                    sleep(1500);
                    drive.followTrajectory(leftPark);
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
        Scalar lowerColor = new Scalar(100.0,100.0,100.0);
        Scalar upperColor = new Scalar(255.0,255.0,255.0);
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


            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(centerCrop,centerCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);

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
