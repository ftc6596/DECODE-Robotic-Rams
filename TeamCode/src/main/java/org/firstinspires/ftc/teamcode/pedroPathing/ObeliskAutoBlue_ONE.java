package org.firstinspires.ftc.teamcode.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import java.util.ArrayList;

@Autonomous(name="ObeliskAutoBlue_ONE", group="Linear OpMode")
public class ObeliskAutoBlue_ONE extends OpMode {
    //Electronic Variables
    //Extra
    private Limelight3A limelight;
    //Motors
    private DcMotorEx topMotor;
    private DcMotorEx bottomMotor;
    private DcMotorEx sorter = null;
    private DcMotor intake;
    //Servos
    private Servo outtakeFeeder;
    private boolean ableToResetTimer = true;
    private boolean ableToResetStayTimer = true;
    private boolean nextSlot = true;
    private int currentslot = 0;
    private boolean sorted = false;
    //I2C
    private GoBildaPrismDriver prismDriver;
    private RevColorSensorV3 colorsensor;
    //Modes
    String SorterMode = "Shooting";

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
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private long secondShot = 1100;
    private long thirdShot = secondShot * 2;
    private int pathState;

    ArrayList<String> currentArtifacts = new ArrayList<>(3);

    private final Pose startPose = new Pose(32.75, 129, Math.toRadians(270));// Start Pose of our robot.
    private final Pose ReadMotifPose = new Pose(61, 91, Math.toRadians(270));
    private final Pose scorePose = new Pose(55, 91, Math.toRadians(314)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1SetUpPose = new Pose(40, 82, Math.toRadians(180));
    private final Pose pickup1EndPose = new Pose(13, 80, Math.toRadians(180));
    private final Pose pickup2SetUpPose = new Pose(40, 56, Math.toRadians(180));
    private final Pose pickup2EndPose = new Pose(5, 57, Math.toRadians(180));
    private final Pose OpenGatePose = new Pose(10, 60, Math.toRadians(270));
    private final Pose NotPushBall = new Pose(40, 60, Math.toRadians(270));
    private final Pose endingPose = new Pose(32, 66, Math.toRadians(270)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(14, 42, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path readMotif;
    private Path scorePreload;
    private PathChain  setup1, pickup1, scorePickup1, setup2, pickup2, openGate, notPushBall, scorePickup2, ending;
    boolean foundMotif = false;
    int motifId = -1;
    ArrayList<String> motif = new ArrayList<>();
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        readMotif = new Path(new BezierLine(startPose, ReadMotifPose));
        readMotif.setLinearHeadingInterpolation(startPose.getHeading(), ReadMotifPose.getHeading());


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        scorePreload = new Path(new BezierLine(ReadMotifPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(ReadMotifPose.getHeading(), scorePose.getHeading());
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        setup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2SetUpPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2SetUpPose.getHeading(), .1)
                .build();
        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2SetUpPose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2SetUpPose.getHeading(), pickup2EndPose.getHeading())
                .setVelocityConstraint(4.0)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        openGate = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, OpenGatePose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), OpenGatePose.getHeading(), .2)
                .build();
        notPushBall = follower.pathBuilder()
                .addPath(new BezierLine(OpenGatePose, NotPushBall))
                .setLinearHeadingInterpolation(OpenGatePose.getHeading(), NotPushBall.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(NotPushBall, scorePose))
                .setLinearHeadingInterpolation(NotPushBall.getHeading(), scorePose.getHeading())
                .build();

        setup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1SetUpPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1SetUpPose.getHeading(), .1)
                .build();
        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1SetUpPose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1SetUpPose.getHeading(), pickup1EndPose.getHeading())
                .setVelocityConstraint(4.0)
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        ending = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endingPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endingPose.getHeading(), .1)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(readMotif);
                setPathState(1);
                break;
            case 1:
                if (!foundMotif)
                {
                    LLResult result = limelight.getLatestResult();

                    if(result != null)
                    {
                        if(result.isValid())
                        {
                            for(LLResultTypes.FiducialResult tag : result.getFiducialResults())
                            {
                                motifId = tag.getFiducialId();
                                foundMotif = true;

                                currentslot = Sort(sorter, motifId, 0, currentslot);
                                UpdateLEDs(currentArtifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                            }
                        }
                    }
                }

                if(!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    ShootAllBalls(3, setup1, 0);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                intake.setPower(-1);
                if(!follower.isBusy()) {
                    follower.followPath(pickup1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(ableToResetStayTimer)
                {
                    pathTimer.resetTimer();
                    ableToResetStayTimer = false;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() >= 2000) {
                    intake.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(openGate,false);
                    currentslot = RotateMotorToNextHalfSlotEncoder(sorter, currentslot);
                    SorterMode = "Shooting";
                    setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    UpdateLEDs(currentArtifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    sorted = false;
                    ableToResetStayTimer = true;
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    intake.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(notPushBall,true);
                    ableToResetStayTimer = true;
                    setPathState(6);
                }
                break;
            case 6:
                if(ableToResetStayTimer)
                {
                    pathTimer.resetTimer();
                    ableToResetStayTimer = false;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() >= 1500) {
                    intake.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    ableToResetStayTimer = true;
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    intake.setPower(0);
                    ShootAllBalls(8, setup2, 6);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                intake.setPower(-1);
                if(!follower.isBusy()) {
                    follower.followPath(pickup2,true);
                    sorted = false;
                    setPathState(9);
                }
                break;
            case 9:
                if(ableToResetStayTimer)
                {
                    pathTimer.resetTimer();
                    ableToResetStayTimer = false;
                }
                if(!follower.isBusy() && pathTimer.getElapsedTime() >= 2000) {
                    intake.setPower(1);
                    follower.followPath(scorePickup2,true);
                    currentslot = RotateMotorToNextHalfSlotEncoder(sorter, currentslot);
                    SorterMode = "Shooting";
                    setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    UpdateLEDs(currentArtifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    ableToResetStayTimer = true;
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!sorted)
                {
                    currentslot = Sort(sorter, motifId, 1, currentslot);
                    UpdateLEDs(currentArtifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                    sorted = true;
                }
                if(!follower.isBusy()) {
                    intake.setPower(0);
                    ShootAllBalls(11, ending, 0);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }



    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

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
            if((sorter.getCurrentPosition() > sorter.getTargetPosition() - 8 && sorter.getCurrentPosition() < sorter.getTargetPosition() + 8) && (currentArtifacts.get(0).equals("none") || currentArtifacts.get(1).equals("none") ||currentArtifacts.get(2).equals("none")))
            {
                if((rPercent > .12 && rPercent < .25) && (gPercent > .4 && gPercent < .65) && (bPercent > .3 && bPercent < .5) && !currentArtifacts.get(currentslot).equals("green")){
                    color = "green";
                    currentArtifacts.set(currentslot, "green");
                    if(!currentArtifacts.get(0).equals("none") && !currentArtifacts.get(1).equals("none") && !currentArtifacts.get(2).equals("none"))
                    {

                    }
                    else {
                        currentslot = RotateMotorToNextSlotEncoder(sorter, currentslot, false);
                    }
                    UpdateLEDs(currentArtifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                }
                else if ((rPercent > .1 && rPercent < .35) && (gPercent > .1 && gPercent < .35) && (bPercent > .35 && bPercent < .65) && !currentArtifacts.get(currentslot).equals("purple")) {
                    color = "purple";
                    currentArtifacts.set(currentslot, "purple");
                    if(!currentArtifacts.get(0).equals("none") && !currentArtifacts.get(1).equals("none") && !currentArtifacts.get(2).equals("none"))
                    {

                    }
                    else {
                        currentslot = RotateMotorToNextSlotEncoder(sorter, currentslot, false);
                    }
                    UpdateLEDs(currentArtifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
                }
            }
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Slot: ", currentslot);
        telemetry.addData("Found Motif: ", foundMotif);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        currentArtifacts.add("green");
        currentArtifacts.add("purple");
        currentArtifacts.add("purple");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        topMotor = hardwareMap.get(DcMotorEx.class, "top");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottom");
        outtakeFeeder = hardwareMap.get(Servo.class, "feeder");
        sorter = hardwareMap.get(DcMotorEx.class, "sorter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //I2C
        prismDriver = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        colorsensor = hardwareMap.get(RevColorSensorV3.class, "colorsensor");

        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(8.1,0,0,14));
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(8.1,0,0,14));
        sorter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(12.5,.5,3,15));

        limelight.pipelineSwitch(0);
        limelight.start();

        setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
        UpdateLEDs(currentArtifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        outtakeFeeder.setPosition(0.05);
        //Applies power to the shooter
        topMotor.setVelocity(815);
        bottomMotor.setVelocity(815);
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPower(1);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}



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
            sorter.setTargetPosition(260);
        } else if (slot <= 0) {
            sorter.setTargetPosition(0);
        }
        else
        {
            sorter.setTargetPosition(130);
        }

        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //Check and/or resets encoder
    public static boolean CheckNextAngle(DcMotor sorter)
    {
        if(sorter.getCurrentPosition() + 130 > 380)
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

    public static int Sort(DcMotor sorter, int motifId, int ballIndexId, int currentslot)
    {
        if(motifId == 21)
        {
            if(ballIndexId == 0)
            {
                currentslot = currentslot;
            }
            else if(ballIndexId == 1)
            {
                sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                currentslot = (currentslot+1)%3;
            }
            else if(ballIndexId == 2)
            {
                sorter.setTargetPosition(sorter.getTargetPosition() - 128);
                currentslot = (currentslot+2)%3;
            }
        } else if (motifId == 22) {
            if(ballIndexId == 0)
            {
                sorter.setTargetPosition(sorter.getTargetPosition() - 128);
                currentslot = (currentslot+2)%3;
            }
            else if(ballIndexId == 1)
            {
                currentslot = currentslot;
            }
            else if(ballIndexId == 2)
            {
                sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                currentslot = (currentslot+1)%3;
            }
        }
        else if (motifId == 23) {
            if(ballIndexId == 0)
            {
                sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                currentslot = (currentslot+1)%3;
            }
            else if(ballIndexId == 1)
            {
                sorter.setTargetPosition(sorter.getTargetPosition() - 128);
                currentslot = (currentslot+2)%3;
            }
            else if(ballIndexId == 2)
            {
                currentslot = currentslot;
            }
        }
        return currentslot;
    }

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

    //Shoot All Balls
    public void ShootAllBalls(int nextPath, PathChain nextPathChain, int ballIndexId) {
        if(ableToResetTimer)
        {
            pathTimer.resetTimer();
            ableToResetTimer = false;
            nextSlot = true;
        }

        if(pathTimer.getElapsedTime() > 3250)
        {
            currentslot = RotateMotorToNextHalfSlotEncoder(sorter, currentslot);
            ableToResetTimer = true;
            follower.followPath(nextPathChain,true);
            SorterMode = "Intake";
            setupleds(SorterMode, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
            UpdateLEDs(currentArtifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
            setPathState(nextPath);
        }  else if (((pathTimer.getElapsedTime() > 1010 && pathTimer.getElapsedTime() < 1500) || (pathTimer.getElapsedTime() > 1010 + secondShot && pathTimer.getElapsedTime() < 1500 + secondShot)) && nextSlot) {
            currentslot = RotateMotorToNextSlotEncoder(sorter, currentslot, false);
            nextSlot = false;
            UpdateLEDs(currentArtifacts, currentslot, prismDriver, slot0, slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8);
        } else if ((pathTimer.getElapsedTime() > 760 && pathTimer.getElapsedTime() < 1010) || (pathTimer.getElapsedTime() > 760 + secondShot && pathTimer.getElapsedTime() < 1010 + secondShot) || (pathTimer.getElapsedTime() > 760 + thirdShot && pathTimer.getElapsedTime() < 1010 + thirdShot)) {
            outtakeFeeder.setPosition(0.05);
        } else if ((pathTimer.getElapsedTime() > 500 && pathTimer.getElapsedTime() < 760) || (pathTimer.getElapsedTime() > 500 + secondShot && pathTimer.getElapsedTime() < 760 + secondShot) || (pathTimer.getElapsedTime() > 500 + thirdShot && pathTimer.getElapsedTime() < 760 + thirdShot)) {
            outtakeFeeder.setPosition(1);
            currentArtifacts.set(currentslot, "none");
            nextSlot = true;
        }
    }
}