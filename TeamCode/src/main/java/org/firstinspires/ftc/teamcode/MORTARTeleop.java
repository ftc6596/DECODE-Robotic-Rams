package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@TeleOp(name="MORTARTeleop", group="Linear OpMode")
public class MORTARTeleop extends LinearOpMode {
    //Electronic Variables
    //Extras
    private Limelight3A limelight;
    //Motors
    private DcMotorEx topMotor;
    private DcMotorEx bottomMotor;
    private DcMotorEx sorter = null;
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
        //Servos
        outtakeFeeder = hardwareMap.get(Servo.class, "feeder");
        //Motors
        topMotor = hardwareMap.get(DcMotorEx.class, "top");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottom");
        sorter = hardwareMap.get(DcMotorEx.class, "sorter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        LFront  = hardwareMap.get(DcMotor.class, "leftfront");
        RFront = hardwareMap.get(DcMotor.class, "rightfront");
        LBack  = hardwareMap.get(DcMotor.class, "leftback");
        RBack = hardwareMap.get(DcMotor.class, "rightback");
        //LimeLght3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        //Variables
        //Booleans
        boolean shooterOn = true;
        boolean OnOffShooter = false;
        boolean nextSlot = false;
        boolean ableToSwitchMode = true;
        boolean autoAiming = false;
        boolean ableToAim = true;
        boolean shootingAll = false;
        //Modes
        String driveMode = "FAST";
        //Numbers
        double velocity = 800;
        int slot = 0;
        //Setup for Electronics
        //Motors
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFront.setDirection(DcMotor.Direction.FORWARD);
        LBack.setDirection(DcMotor.Direction.FORWARD);
        RFront.setDirection(DcMotor.Direction.FORWARD);
        RBack.setDirection(DcMotor.Direction.REVERSE);
        topMotor.setDirection(DcMotor.Direction.REVERSE);
        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        //PIDF Coefficients
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(27.5,0,1.25,14));
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(27.5,0,1.25,14));
        sorter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(12.5,.5,3,15));
        //Get the flicker set
        outtakeFeeder.setPosition(0.4);
        while(opModeIsActive()) {
            //LimeLight3A Uses
            LLResult result = limelight.getLatestResult();
            if(result != null) {
                if (result.isValid()) {
                    //Goes through all the Tags
                    for(LLResultTypes.FiducialResult tag : result.getFiducialResults())
                    {
                        int tagId = tag.getFiducialId();
                        //Checks if the tag is not the Obelisk
                        if (tagId != 21 && tagId != 22 && tagId != 23)
                        {
                            double x = result.getTx();//Tag X Position
                            double a = result.getTa();//Tag Area
                            //Auto Aiming Code
                            if(autoAiming)
                            {
                                if(x > 1.1)
                                {
                                    TurnLeft(LFront, RFront, LBack, RBack, x);
                                } else if (x < -1.1) {
                                    TurnRight(LFront, RFront, LBack, RBack, x);
                                } else {
                                    Stop(LFront, RFront, LBack, RBack);
                                    autoAiming = false;
                                }
                            }
                            //Auto-Velocity Code
                            velocity = 850 - ((195 * a) - 20);
                        }
                    }
                }
                else if (autoAiming)
                {
                    //Stops if there is no April Tag
                    Stop(LFront, RFront, LBack, RBack);
                    autoAiming = false;
                }
                else {
                    //Manual Flywheel speed just in case ;)
                    if (gamepad2.left_trigger != 0)
                    {
                        velocity = 830;
                    }
                    else
                    {
                        velocity = 635;
                    }
                }
            }
            else if (autoAiming)
            {
                //Stops if there is no April Tag
                Stop(LFront, RFront, LBack, RBack);
                autoAiming = false;
            }
            else {
                //Manual Flywheel speed just in case ;)
                if (gamepad2.left_trigger != 0)
                {
                    velocity = 830;
                }
                else
                {
                    velocity = 635;
                }
            }
            //Driving
            if(!autoAiming)
            {
                ApplyInputToMotors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, driveMode, LFront, RFront, LBack, RBack);
            }
            //Aiming
            if(ableToAim)
            {
                if(gamepad1.left_trigger != 0 && !autoAiming)
                {
                    ableToAim = false;

                    autoAiming = true;
                } else if (gamepad1.left_trigger != 0 && autoAiming) {
                    ableToAim = false;

                    autoAiming = false;
                }
            }
            else
            {
                if(gamepad1.left_trigger == 0)
                {
                    ableToAim = true;
                }
            }
            //Shooter
            if (gamepad2.a)
            {
                ShootAllBalls(this, outtakeFeeder, sorter, slot);
            }
            //Feed Shooter
            if (!sorter.isBusy())
            {
                if (gamepad2.right_trigger > 0) {
                    outtakeFeeder.setPosition(0.75);
                } else {
                    outtakeFeeder.setPosition(0);
                }
            }
            else
            {
                if (gamepad2.right_trigger > 0) {
                    outtakeFeeder.setPosition(0.75);
                } else {
                    outtakeFeeder.setPosition(0);
                }
            }
            //Shooter On Off
            if(!OnOffShooter)
            {
                if(gamepad2.x)
                {
                    OnOffShooter = true;
                    shooterOn = !shooterOn;
                }
            }
            else
            {
                if(!gamepad2.x)
                {
                    OnOffShooter = false;
                }
            }
            //Intake
            if(gamepad1.right_bumper)
            {
                intake.setPower(-1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(1);
            } else
            {
                intake.setPower(0);
            }

            //Change Drive Mode
            if(ableToSwitchMode)
            {
                if(gamepad1.a && driveMode.equals("FAST"))
                {
                    ableToSwitchMode = false;
                    driveMode = "SLOW";
                } else if (gamepad1.a && driveMode.equals("SLOW")) {
                    ableToSwitchMode = false;
                    driveMode = "FAST";
                }
            }
            else
            {
                if(!gamepad1.a)
                {
                    ableToSwitchMode = true;
                }
            }

            //Moves the sorter to the next slot with no skipping
            if(!nextSlot)
            {
                if(gamepad2.dpad_right)
                {
                    nextSlot = true;
                    slot = RotateMotorToNextSlotEncoder(sorter, slot, false);
                } else if (gamepad2.y) {
                    nextSlot = true;
                    RotateMotorToNextHalfSlotEncoder(sorter);
                } else if (gamepad2.dpad_left) {
                    nextSlot = true;
                    slot = RotateMotorToNextSlotEncoder(sorter, slot, true);
                }
            }
            else
            {
                if(!gamepad2.dpad_right && !gamepad2.y && !gamepad2.dpad_left)
                {
                    nextSlot = false;
                }
            }

            //Apply Powers
            //Sorter Power
            if(gamepad2.dpad_up)//Manual Control
            {
                sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sorter.setPower(.1);
            } else if (gamepad2.dpad_down)//Manual Control
            {
                sorter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sorter.setPower(-.1);
            } else if (gamepad2.b)//Reset Sorter Encoder
            {
                sorter.setTargetPosition(0);
                sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            else
            {
                if(sorter.getMode().equals(DcMotor.RunMode.RUN_USING_ENCODER))//Stops supplying power to Sorter when in manual Mode
                {
                    sorter.setPower(0);
                }
                else//Always in Run to Position Mode
                {
                    if(sorter.getCurrentPosition() != sorter.getTargetPosition())//Applies power to the sorter when not at desired position
                    {
                        sorter.setPower(1);
                    }
                    else
                    {
                        sorter.setPower(0);
                    }
                }
            }
            //Applies power to the shooter
            if(shooterOn)
            {
                topMotor.setVelocity(velocity);
                bottomMotor.setVelocity(velocity);
            }
            else
            {//Turn off shooter
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


            //Telemetry
            //Shooter
            telemetry.addData("Velocity: ", velocity);
            telemetry.addData("Shooter On: ", shooterOn);
            //Sorter
            telemetry.addData("Current Slot: ", slot);
            telemetry.addData("Sorter Position: ", sorter.getCurrentPosition());
            telemetry.addData("PIDF Values: ", sorter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            //Drive Mode
            telemetry.addData("Drive Mode: ", driveMode);
            telemetry.update();
        }

    }

    //Applies input to the Drive Motors
    public static void ApplyInputToMotors(double Ly, double Lx, double Rx, String driveMode, DcMotor LFront, DcMotor RFront, DcMotor LBack, DcMotor RBack)
    {
        //Inputs
        if(driveMode.equals("SLOW"))
        {
            Ly = Ly * .5;
            Lx = Lx * .5;
        }
        //Computing Powers
        double LeftFrontWheel = Ly + Lx - Rx;
        double RightFrontWheel = Ly - Lx + Rx;
        double LeftBackWheel = Ly - Lx - Rx;
        double RightBackWheel = Ly + Lx + Rx;

        //Applying Power the Drive Train
        LFront.setPower(LeftFrontWheel);
        RFront.setPower(RightFrontWheel);
        LBack.setPower(LeftBackWheel);
        RBack.setPower(RightBackWheel);
    }

    //Turn the Robot Left
    public static void TurnRight(DcMotor LFront, DcMotor RFront, DcMotor LBack, DcMotor RBack, double x)
    {
        double power = .12;
        //Applying Power the Drive Train
        LFront.setPower(power);
        RFront.setPower(-power);
        LBack.setPower(power);
        RBack.setPower(-power);
    }

    //Turn the Robot Right
    public static void TurnLeft(DcMotor LFront, DcMotor RFront, DcMotor LBack, DcMotor RBack, double x)
    {
        double power = .12;
        //Applying Power the Drive Train
        LFront.setPower(-power);
        RFront.setPower(power);
        LBack.setPower(-power);
        RBack.setPower(power);
    }

    //Stop the Robot
    public static void Stop(DcMotor LFront, DcMotor RFront, DcMotor LBack, DcMotor RBack)
    {
        //Applying Power the Drive Train
        LFront.setPower(0);
        RFront.setPower(0);
        LBack.setPower(0);
        RBack.setPower(0);
    }

    //Changes Slot Ball State
    public static void ChangeBallSlotColor(ArrayList<String> slots, String color, int slot)
    {
        slots.set(slot, color);
    }
    //Rotates to a specified slot
    public static void RotateMotorToSlot(DcMotor sorter, int slot)
    {
        if(slot >= 2)
        {
            sorter.setTargetPosition(256);
        } else if (slot <= 0) {
            sorter.setTargetPosition(0);
        }
        else
        {
            sorter.setTargetPosition(128);
        }

    }
    //Check and/or resets encoder
    public static boolean CheckNextAngle(DcMotor sorter)
    {
        if(sorter.getTargetPosition() + 128 > 380)
        {
            sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return true;
        }

        return false;
    }
    //Rotates to a specified angle
    public static void RotateMotorToAngle(DcMotor sorter, int angle)
    {
        sorter.setTargetPosition(angle);
    }
    //Rotates to the next slot using encoder ticks
    public static int RotateMotorToNextSlotEncoder(DcMotor sorter, int slot, boolean inverse)
    {
        sorter.setPower(1);
        if(!inverse)
        {
            sorter.setTargetPosition(sorter.getTargetPosition() + 128);

            int nextSlot = slot + 1;
            if(nextSlot >= 3)
            {
                nextSlot = 0;
            }

            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            return nextSlot;
        }
        else
        {
            sorter.setTargetPosition(sorter.getTargetPosition() - 128);

            int nextSlot = slot - 1;
            if(nextSlot <= -1)
            {
                nextSlot = 2;
            }

            sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            return nextSlot;
        }
    }
    //Rotates to the next slot using encoder ticks
    public static void RotateMotorToNextHalfSlotEncoder(DcMotor sorter)
    {
        sorter.setTargetPosition(sorter.getTargetPosition() + 66);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //Rotates to the nest slot
    public static int RotateMotorToNextSlot(DcMotor sorter, int currentSlot) throws InterruptedException {
        int nextSlot = currentSlot + 1;
        if(nextSlot == 2)
        {
            sorter.setTargetPosition(260);
        }
        else if(nextSlot == 1)
        {
            sorter.setTargetPosition(130);
        }
        else if(nextSlot == 3)
        {
            sorter.setTargetPosition(380);

            return 0;
        }

        return nextSlot;
    }
    //Shoots all three balls in sequential order
    public static void ShootAllBalls(LinearOpMode opmode, Servo outtakeFeeder, DcMotor sorter, int slot)
    {
        outtakeFeeder.setPosition(0.75);
        opmode.sleep(250);
        outtakeFeeder.setPosition(0);
        opmode.sleep(300);
        slot = RotateMotorToNextSlotEncoder(sorter, slot, false);
        opmode.sleep(500);
        outtakeFeeder.setPosition(0.75);
        opmode.sleep(250);
        outtakeFeeder.setPosition(0);
        opmode.sleep(300);
        slot = RotateMotorToNextSlotEncoder(sorter, slot, false);
        opmode.sleep(500);
        outtakeFeeder.setPosition(0.75);
        opmode.sleep(250);
        outtakeFeeder.setPosition(0);

    }
}

