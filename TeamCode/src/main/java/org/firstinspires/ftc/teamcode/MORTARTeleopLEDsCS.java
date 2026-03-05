package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import java.util.ArrayList;
@Disabled
@TeleOp(name="MORTARTeleopLEDsCS", group="Linear OpMode")
public class MORTARTeleopLEDsCS extends LinearOpMode {
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
    //I2C
    private GoBildaPrismDriver prismDriver;
    private RevColorSensorV3 colorsensor;
    //Servos
    private CRServo outtakeFeeder;
    private Rev2mDistanceSensor feederSensor;
    //Animations
    PrismAnimations.Solid slot0 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot1 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot2 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot3 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot4 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot5 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot6 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot7 = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid slot8 = new PrismAnimations.Solid(Color.YELLOW);
    @Override
    public void runOpMode() throws InterruptedException {
        //References to Electronics
        //Servos
        outtakeFeeder = hardwareMap.get(CRServo.class, "feeder");
        feederSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distancesensor");
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
        //I2C
        prismDriver = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        colorsensor = hardwareMap.get(RevColorSensorV3.class, "colorsensor");
        //Variables
        //Booleans
        boolean shooterOn = true;
        boolean OnOffShooter = false;
        boolean nextSlot = false;
        boolean ableToSwitchMode = true;
        boolean autoAiming = false;
        boolean ableToAim = true;
        boolean canReadBall = true;
        boolean shootingAll = false;
        //Modes
        String driveMode = "FAST";
        String SorterMode = "Intake";
        //Numbers
        double velocity = 800;
        double lastDistance = 0;
        int currentslot = 0;
        //Array
        ArrayList<String> currentartifacts = new ArrayList<String>(3);
        currentartifacts.add("none");
        currentartifacts.add("none");
        currentartifacts.add("none");
        //Setup for Electronics
        //Motors
        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFront.setDirection(DcMotor.Direction.FORWARD);
        LBack.setDirection(DcMotor.Direction.FORWARD);
        RFront.setDirection(DcMotor.Direction.REVERSE);
        RBack.setDirection(DcMotor.Direction.REVERSE);
        topMotor.setDirection(DcMotor.Direction.REVERSE);
        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
        waitForStart();
        //PIDF Coefficients
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15,0,0,13.895));
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15,0,0,13.895));
        sorter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(12.5,.5,3,15));
        //Get the flicker set
        outtakeFeeder.setPower(0);
        updateleds(prismDriver, currentartifacts.get(0), currentartifacts.get(1), currentartifacts.get(2), slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
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
                        if (!(tagId == 21 || tagId == 22 || tagId == 23))
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
                            velocity = 850 - ((195 * a) - 40);
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
            //Color/LEDs
            String color = "None";
            int r = colorsensor.red();
            int g = colorsensor.green();
            int b = colorsensor.blue();
            int rgb = r+g+b;
            double rPercent = (double) r /rgb;
            double gPercent = (double) g /rgb;
            double bPercent = (double) b /rgb;
            if (SorterMode.equals("Intake"))
            {
                if((sorter.getCurrentPosition() > sorter.getTargetPosition() - 10 && sorter.getCurrentPosition() < sorter.getTargetPosition() + 10) && (currentartifacts.get(0).equals("none") || currentartifacts.get(1).equals("none") ||currentartifacts.get(2).equals("none")))
                {
                    if((rPercent > .12 && rPercent < .25) && (gPercent > .4 && gPercent < .65) && (bPercent > .3 && bPercent < .5) && !currentartifacts.get(currentslot).equals("green")){
                        color = "green";
                        currentartifacts.set(currentslot, "green");
                        if(!currentartifacts.get(0).equals("none") && !currentartifacts.get(1).equals("none") && !currentartifacts.get(2).equals("none"))
                        {
                            currentslot = RotateMotorToNextHalfSlotEncoder(sorter, currentslot);
                            SorterMode = "Shooting";
                            setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                        }
                        else {
                            currentslot = RotateMotorToNextSlotEncoder(sorter, currentslot, false);
                        }
                        UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    }
                    else if ((rPercent > .1 && rPercent < .35) && (gPercent > .1 && gPercent < .35) && (bPercent > .35 && bPercent < .65) && !currentartifacts.get(currentslot).equals("purple")) {
                        color = "purple";
                        currentartifacts.set(currentslot, "purple");
                        if(!currentartifacts.get(0).equals("none") && !currentartifacts.get(1).equals("none") && !currentartifacts.get(2).equals("none"))
                        {
                            currentslot = RotateMotorToNextHalfSlotEncoder(sorter, currentslot);
                            SorterMode = "Shooting";
                            setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                        }
                        else {
                            currentslot = RotateMotorToNextSlotEncoder(sorter, currentslot, false);
                        }
                        UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    }
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
                currentslot = ShootAllBalls(this, outtakeFeeder, sorter, currentslot, currentartifacts);
                SorterMode = "Intake";
                setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
            }
            //Feed Shooter
            if (gamepad2.right_trigger > 0) {
                outtakeFeeder.setPower(.2);
                if(((lastDistance-feederSensor.getDistance(DistanceUnit.INCH)) > .5 || feederSensor.getDistance(DistanceUnit.INCH) < 3.2) && SorterMode.equals("Shooting"))
                {
                    if(!currentartifacts.get(currentslot).equals("none"))
                    {
                        currentartifacts.set(currentslot, "none");
                        currentslot = RotateMotorToNextSlotEncoder(sorter, currentslot, false);
                        UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                        nextSlot = false;
                    }
                    else if((currentartifacts.get(0).equals("none") && currentartifacts.get(1).equals("none") && currentartifacts.get(2).equals("none")))
                    {
                        currentslot = RotateMotorToNextHalfSlotEncoder(sorter, currentslot);
                        SorterMode = "Intake";
                        setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                        UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                        nextSlot = false;
                    }
                } else if (feederSensor.getDistance(DistanceUnit.INCH) > 3.0) {
                    nextSlot = true;
                }

            }else if (gamepad2.left_trigger > 0) {
                outtakeFeeder.setPower(-.25);
            } else {
                outtakeFeeder.setPower(0);
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
                    currentslot = RotateMotorToNextSlotEncoder(sorter, currentslot, false);
                    UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                } else if (gamepad2.y) {
                    nextSlot = true;
                    currentslot = RotateMotorToNextHalfSlotEncoder(sorter, currentslot);
                    if (SorterMode.equals("Intake"))
                    {
                        SorterMode = "Shooting";
                    } else {
                        SorterMode = "Intake";
                    }

                    setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                } else if (gamepad2.dpad_left) {
                    nextSlot = true;
                    currentslot = RotateMotorToNextSlotEncoder(sorter, currentslot, true);
                    UpdateLEDs(currentartifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
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
                        sorter.setPower(.75);
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

            lastDistance = feederSensor.getDistance(DistanceUnit.INCH);


            //Telemetry
            telemetry.addData("Distance: ", feederSensor.getDistance(DistanceUnit.INCH));
            //Shooter
            telemetry.addData("Velocity: ", velocity);
            telemetry.addData("Shooter On: ", shooterOn);
            //Sorter
            telemetry.addData("Current Slot: ", currentslot);
            telemetry.addData("Sorter Position: ", sorter.getCurrentPosition());
            telemetry.addData("Sorter Mode: ", SorterMode);
            telemetry.addData("Sensed Color: ", color);
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
        double LeftFrontWheel = Ly - Lx - Rx;
        double RightFrontWheel = Ly + Lx + Rx;
        double LeftBackWheel = Ly + Lx - Rx;
        double RightBackWheel = Ly - Lx + Rx;

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
        sorter.setPower(.75);
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
    public static int RotateMotorToNextHalfSlotEncoder(DcMotor sorter, int slot)
    {
        sorter.setTargetPosition(sorter.getTargetPosition() + 66);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int nextSlot = slot - 1;
        if(nextSlot <= -1)
        {
            nextSlot = 2;
        }
        return nextSlot;
    }
    //Shoots all three balls in sequential order
    public static int ShootAllBalls(LinearOpMode opmode, CRServo outtakeFeeder, DcMotor sorter, int slot, ArrayList<String> artifacts)
    {
        artifacts.set(slot, "none");
        outtakeFeeder.setPower(1);
        opmode.sleep(250);
        slot = RotateMotorToNextSlotEncoder(sorter, slot, false);
        artifacts.set(slot, "none");
        opmode.sleep(450);
        opmode.sleep(250);
        slot = RotateMotorToNextSlotEncoder(sorter, slot, false);
        artifacts.set(slot, "none");
        opmode.sleep(450);
        opmode.sleep(250);
        slot = RotateMotorToNextHalfSlotEncoder(sorter, slot);
        outtakeFeeder.setPower(0);
        return slot;
    }
    //Sets up the LEDs per the intake mode
    public static void setupleds(String SorterMode, PrismAnimations.Solid slot0, PrismAnimations.Solid slot1, PrismAnimations.Solid slot2, PrismAnimations.Solid slot3, PrismAnimations.Solid slot4, PrismAnimations.Solid slot5, PrismAnimations.Solid slot6, PrismAnimations.Solid slot7, PrismAnimations.Solid slot8)
    {

        if (SorterMode.equals("Intake")) {
            slot0.setBrightness(25);
            slot0.setStartIndex(9);
            slot0.setStopIndex(11);
            slot3.setBrightness(25);
            slot3.setStartIndex(9);
            slot3.setStopIndex(11);
            slot6.setBrightness(25);
            slot6.setStartIndex(12);
            slot6.setStopIndex(14);

            slot1.setBrightness(25);
            slot1.setStartIndex(0);
            slot1.setStopIndex(2);
            slot4.setBrightness(25);
            slot4.setStartIndex(0);
            slot4.setStopIndex(2);
            slot7.setBrightness(25);
            slot7.setStartIndex(15);
            slot7.setStopIndex(17);

            slot2.setBrightness(25);
            slot2.setStartIndex(3);
            slot2.setStopIndex(5);
            slot5.setBrightness(25);
            slot5.setStartIndex(3);
            slot5.setStopIndex(5);
            slot8.setBrightness(25);
            slot8.setStartIndex(6);
            slot8.setStopIndex(8);
        } else {
            slot0.setStartIndex(0);
            slot0.setStopIndex(1);
            slot3.setBrightness(25);
            slot3.setStartIndex(2);
            slot3.setStopIndex(3);
            slot6.setBrightness(25);
            slot6.setStartIndex(4);
            slot6.setStopIndex(5);

            slot1.setBrightness(25);
            slot1.setStartIndex(6);
            slot1.setStopIndex(7);
            slot4.setBrightness(25);
            slot4.setStartIndex(8);
            slot4.setStopIndex(9);
            slot7.setBrightness(25);
            slot7.setStartIndex(10);
            slot7.setStopIndex(11);

            slot2.setBrightness(25);
            slot2.setStartIndex(12);
            slot2.setStopIndex(13);
            slot5.setBrightness(25);
            slot5.setStartIndex(14);
            slot5.setStopIndex(15);
            slot8.setBrightness(25);
            slot8.setStartIndex(16);
            slot8.setStopIndex(17);
        }
    }
    //The logic to Update the LEDs
    public static void updateleds(GoBildaPrismDriver prismDriver, String slot0Color, String slot1Color, String slot2Color, PrismAnimations.Solid slot0, PrismAnimations.Solid slot1, PrismAnimations.Solid slot2, PrismAnimations.Solid slot3, PrismAnimations.Solid slot4, PrismAnimations.Solid slot5, PrismAnimations.Solid slot6, PrismAnimations.Solid slot7, PrismAnimations.Solid slot8)
    {
        if(slot0Color.equals("green"))
        {
            slot0.setBrightness(25);
            slot3.setBrightness(25);
            slot6.setBrightness(25);
            slot0.setPrimaryColor(Color.GREEN);
            slot3.setPrimaryColor(Color.GREEN);
            slot6.setPrimaryColor(Color.GREEN);
        } else if (slot0Color.equals("purple")) {
            slot0.setBrightness(25);
            slot3.setBrightness(25);
            slot6.setBrightness(25);
            slot0.setPrimaryColor(Color.PURPLE);
            slot3.setPrimaryColor(Color.PURPLE);
            slot6.setPrimaryColor(Color.PURPLE);
        }
        else {
            slot0.setBrightness(0);
            slot3.setBrightness(0);
            slot6.setBrightness(0);
        }

        if(slot1Color.equals("green"))
        {
            slot1.setBrightness(25);
            slot4.setBrightness(25);
            slot7.setBrightness(25);
            slot1.setPrimaryColor(Color.GREEN);
            slot4.setPrimaryColor(Color.GREEN);
            slot7.setPrimaryColor(Color.GREEN);
        } else if (slot1Color.equals("purple")) {
            slot1.setBrightness(25);
            slot4.setBrightness(25);
            slot7.setBrightness(25);
            slot1.setPrimaryColor(Color.PURPLE);
            slot4.setPrimaryColor(Color.PURPLE);
            slot7.setPrimaryColor(Color.PURPLE);
        }
        else {
            slot1.setBrightness(0);
            slot4.setBrightness(0);
            slot7.setBrightness(0);
        }
        if(slot2Color.equals("green"))
        {
            slot2.setBrightness(25);
            slot5.setBrightness(25);
            slot8.setBrightness(25);
            slot2.setPrimaryColor(Color.GREEN);
            slot5.setPrimaryColor(Color.GREEN);
            slot8.setPrimaryColor(Color.GREEN);
        } else if (slot2Color.equals("purple")) {
            slot2.setBrightness(25);
            slot5.setBrightness(25);
            slot8.setBrightness(25);
            slot2.setPrimaryColor(Color.PURPLE);
            slot5.setPrimaryColor(Color.PURPLE);
            slot8.setPrimaryColor(Color.PURPLE);
        }
        else {
            slot2.setBrightness(0);
            slot5.setBrightness(0);
            slot8.setBrightness(0);
        }

        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, slot0);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, slot1);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_2, slot2);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, slot3);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, slot4);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, slot5);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_6, slot6);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_7, slot7);
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_8, slot8);
    }
    //Update Code for the LEDs
    public static void UpdateLEDs(ArrayList<String> currentartifacts, int currentslot, GoBildaPrismDriver prismDriver, PrismAnimations.Solid slot0, PrismAnimations.Solid slot1, PrismAnimations.Solid slot2, PrismAnimations.Solid slot3, PrismAnimations.Solid slot4, PrismAnimations.Solid slot5, PrismAnimations.Solid slot6, PrismAnimations.Solid slot7, PrismAnimations.Solid slot8)
    {
        int currentslotPlus1 = (currentslot+1)%3;
        int currentslotPlus2 = (currentslot+2)%3;
        updateleds(prismDriver, currentartifacts.get(currentslot), currentartifacts.get(currentslotPlus1), currentartifacts.get(currentslotPlus2), slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
    }
}

