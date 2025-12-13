package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;

import kotlin.Unit;

@TeleOp(name="PIDShooterTester", group="Linear OpMode")
public class PIDShooterTester extends LinearOpMode {
    //Electronic Variables
    //Motors
    private DcMotorEx topMotor;
    private DcMotorEx bottomMotor;
    private DcMotor sorter = null;
    private DcMotor intake;
    private DcMotor LFront;
    private DcMotor RFront;
    private DcMotor LBack;
    private DcMotor RBack;
    //Servos
    private Servo outtakeFeeder;
    @Override
    public void runOpMode() throws InterruptedException {
        //References to Electronics
        topMotor = hardwareMap.get(DcMotorEx.class, "top");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottom");
        outtakeFeeder = hardwareMap.get(Servo.class, "feeder");
        //Variables
        double velocity = 800;
        boolean shooterOn = true;
        boolean OnOffShooter = false;
        boolean changeValue = true;
        String driveMode = "INTAKE";
        int slot = 0;
        float pValue;
        float iValue;
        float fValue;
        //Setup for Electronics
        topMotor.setDirection(DcMotor.Direction.REVERSE);
        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(25,3,1.25,5.5));
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(25,3,1.25,5.5));
        outtakeFeeder.setPosition(0);
        while(opModeIsActive()) {
            PIDFCoefficients pidfCoefficients = topMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
            //Shooter
            //Increase Velocity
            if(gamepad2.left_trigger != 0)
            {
                velocity = 900;
            }
            else
            {
                velocity = 825;
            }
            //Shooter On O
            if(!OnOffShooter)
            {
                if(gamepad2.right_bumper)
                {
                    OnOffShooter = true;
                    shooterOn = !shooterOn;
                }
            }
            else
            {
                if(!gamepad2.right_bumper)
                {
                    OnOffShooter = false;
                }
            }
            //Feed Shooter
            if(gamepad2.right_trigger >= .6)
            {
                outtakeFeeder.setPosition(1);

            } else if (gamepad2.right_trigger > 0) {
                outtakeFeeder.setPosition(0.6);
            } else {
                outtakeFeeder.setPosition(0.1);
            }
            //Applies power to the shooter
            if(shooterOn)
            {
                topMotor.setVelocity(velocity);
                bottomMotor.setVelocity(topMotor.getVelocity());
            }
            else
            {
                if(OnOffShooter)
                {
                    topMotor.setVelocity(0);
                    bottomMotor.setVelocity(0);
                    topMotor.setVelocity(-10);
                    bottomMotor.setVelocity(-10);
                    sleep(400);
                    topMotor.setVelocity(0);
                    bottomMotor.setVelocity(0);
                }
            }

            if(changeValue)
            {
                if(gamepad2.dpad_up)
                {
                    changeValue = false;
                    pidfCoefficients.p += .1;
                } else if (gamepad2.dpad_down) {
                    changeValue = false;
                    pidfCoefficients.p -= .1;
                }else if (gamepad2.dpad_right) {
                    changeValue = false;
                    pidfCoefficients.d += .1;
                }else if (gamepad2.dpad_left) {
                    changeValue = false;
                    pidfCoefficients.d -= .1;
                }else if (gamepad2.y) {
                    changeValue = false;
                    pidfCoefficients.f += .1;
                }else if (gamepad2.a) {
                    changeValue = false;
                    pidfCoefficients.f -= .1;
                }else if (gamepad2.b) {
                    changeValue = false;
                    pidfCoefficients.i += .1;
                }else if (gamepad2.x) {
                    changeValue = false;
                    pidfCoefficients.i -= .1;
                }
            }
            else
            {
                if(!gamepad2.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_right && !gamepad2.dpad_left && !gamepad2.y && !gamepad2.a && !gamepad2.b && !gamepad2.x)
                {
                    changeValue = true;
                }
            }

            topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            //Telemetry
            //Shooter
            telemetry.addData("Velocity: ", velocity);
            telemetry.addData("Shooter On: ", shooterOn);
            telemetry.addData("PIDFValues: ", pidfCoefficients);
            telemetry.addData("Top Motor Current: ", topMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("Bottom Motor Current: ", bottomMotor.getCurrent(CurrentUnit.MILLIAMPS));
            //Drive Mode
            telemetry.addData("Drive Mode: ", driveMode);
            telemetry.update();
        }

    }
}

