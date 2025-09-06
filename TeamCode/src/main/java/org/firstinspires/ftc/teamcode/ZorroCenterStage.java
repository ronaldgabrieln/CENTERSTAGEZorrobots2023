package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name="ZorroCenterStage", group = "CENTER")

public class ZorroCenterStage extends OpMode {

    DcMotor EnfrenteIzq;
    DcMotor AbajoDer;
    DcMotor AbajoIzq;
    DcMotor EnfrenteDer;
    //Garras
    private Servo GarraIzq;
    private Servo GarraDer;
    private DcMotorEx Brazo;
    private Servo Muneca;

    Servo Avion;
    double GS = .2; // Velocidad que incrementa mientras se siga presionando botones del gamepad
    // Si se deja de presionar botones se actualiza a volver a ser 0;
    boolean usingGilbertoSpeed = false;
    double rightSidePower;
    double leftSidePower;
    boolean cambiadoDerecha;
    boolean cambiadoIzquierda;
    boolean cappedVelocity = true;
    //Control de PID que hace que el brazo meta fuerza si siente que la gravedad lo esta jalando
    private PIDController PIDBrazo;
    public static double p = 0.03, i = 0, d = 0.0002;
    public static double f = 0.1;
    public static int target = 0;
    private final double ticksPerDegree = 537.6 / 360.0;
    //
    boolean modificado;
    public void init() {
        PIDBrazo = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        target = 0;
        EnfrenteDer = hardwareMap.get(DcMotor.class, "ED");
        EnfrenteIzq = hardwareMap.get(DcMotor.class, "EI");
        AbajoDer = hardwareMap.get(DcMotor.class, "AD");
        AbajoIzq = hardwareMap.get(DcMotor.class, "AI");

        //Servo de garritas kawai
        GarraDer = hardwareMap.get(Servo.class, "GD");
        GarraIzq = hardwareMap.get(Servo.class, "GI");
        Muneca = hardwareMap.get(Servo.class, "M");

        GarraDer.setDirection(Servo.Direction.REVERSE);
        //Avion
        Avion = hardwareMap.get(Servo.class, "AS");
        Brazo = hardwareMap.get(DcMotorEx.class, "B");

        AbajoDer.setDirection(DcMotorSimple.Direction.FORWARD);
        AbajoIzq.setDirection(DcMotorSimple.Direction.REVERSE);
        EnfrenteIzq.setDirection(DcMotorSimple.Direction.REVERSE);
        EnfrenteDer.setDirection(DcMotorSimple.Direction.FORWARD);

        AbajoDer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        AbajoIzq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EnfrenteIzq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EnfrenteDer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("SigmaBot: ", "ESTADO FUNCIONAL");
        Brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Brazo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.update();

        GarraIzq.setPosition(0);
        GarraDer.setPosition(0);
        cambiadoDerecha = false;
        cambiadoIzquierda = false;
        modificado = false;
    }
    
    public void loop() {
        telemetry.addData("Posicion left X", gamepad1.left_stick_x);
        telemetry.addData("Posicion left Y", gamepad1.left_stick_y);
        telemetry.addData("Posicion right X", gamepad1.right_stick_x);
        telemetry.addData("Muneca", Muneca.getPosition());
        telemetry.addData("Brazo", Brazo.getCurrentPosition());
        telemetry.addData("Garrader", GarraDer.getPosition());
        telemetry.addData("GarraIzq", GarraIzq.getPosition());
        //tanque Horizontal, Vertical y Rotacion

        if(gamepad2.left_trigger != 0) {
            cappedVelocity = false;
        } else {
            cappedVelocity = true;
        }
        if(gamepad1.dpad_right) {
            usingGilbertoSpeed = true;
            power(GS, -GS, -GS, GS);
        } else if(gamepad1.dpad_left) {
            power(-GS, GS, GS,-GS);
            usingGilbertoSpeed = true;
        } else if(gamepad1.dpad_up) {
            power(GS, GS, GS, GS);
            usingGilbertoSpeed = true;
        } else if (gamepad1.dpad_down) {
            power(-GS, -GS, -GS, -GS);
            usingGilbertoSpeed = true;
        } else {
            rightSidePower = -gamepad1.right_stick_y;
            leftSidePower = -gamepad1.left_stick_y;

            if(cappedVelocity) {
                telemetry.addLine("CAPPED VELOCITY");
                rightSidePower *= .65;
                leftSidePower *=  .65;
            }

            power(leftSidePower, rightSidePower, leftSidePower, rightSidePower);

            usingGilbertoSpeed = false;
        }


        //Garras
        if(gamepad2.x) {
            GarraDer.setPosition(0);
            GarraIzq.setPosition(0);
        } else if(gamepad2.y) {
            GarraDer.setPosition(1);
            GarraIzq.setPosition(1);
        } else if(gamepad2.a) {

        }


        //Derecha
        if(gamepad2.right_bumper && !cambiadoDerecha) {
            cambiadoDerecha = true;
            if(GarraDer.getPosition() > 0) {
                GarraDer.setPosition(0);
            } else {
                GarraDer.setPosition(1);
            }
        } else if(!gamepad2.right_bumper && cambiadoDerecha) {
            cambiadoDerecha = false;
        }
        //Izquierda
        if(gamepad2.left_bumper && !cambiadoIzquierda) {
            cambiadoIzquierda = true;
            if(GarraIzq.getPosition() > 0) {
                GarraIzq.setPosition(0);
            } else {
                GarraIzq.setPosition(1);
            }
        } else if(!gamepad2.left_bumper && cambiadoIzquierda) {
            cambiadoIzquierda = false;
        }
        //Avion
        if(gamepad1.right_bumper) {
            Avion.setPosition(0);
        } else {
            Avion.setPosition(.8);
        }

        target = (int)(target + gamepad2.right_stick_y * 6);
        Muneca.setPosition(Muneca.getPosition() + (gamepad2.left_stick_y) * .035);
        PIDBrazo.setPID(p, i, d);
        int armPos = Brazo.getCurrentPosition();
        double pid = PIDBrazo.calculate(armPos,target);
        double ff = Math.cos(Math.toRadians(target / ticksPerDegree)) * f;

        double powerBrazo = pid + ff;
        Brazo.setPower(powerBrazo);
        telemetry.addData("Position", armPos);
        telemetry.addData("target", target);
        telemetry.addData("p", p);
        telemetry.addData("i", i);
        telemetry.addData("d", d);
        telemetry.addData("f", f);
        //SlopeSpeed
        if(GS < .5 && usingGilbertoSpeed == true) {
            GS += .01;// aumento cada reinicio del loop
        } else if (usingGilbertoSpeed == false){
            GS = 0.1;
        }

        telemetry.update();
    }


    public void power(double EI, double ED, double AI, double AD) {
        EnfrenteDer.setPower(ED);
        EnfrenteIzq.setPower(EI);
        AbajoDer.setPower(AD);
        AbajoIzq.setPower(AI);
    }

}
