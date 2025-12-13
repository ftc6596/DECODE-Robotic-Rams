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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@Autonomous(name="BackAutoRed", group="Linear OpMode")
public class BackAutoRed extends OpMode {
    //Electronic Variables
    //Extra
    private Limelight3A limelight;
    //Motors
    private DcMotorEx topMotor;
    private DcMotorEx bottomMotor;
    private DcMotor sorter = null;
    private DcMotor intake;
    //Servos
    private Servo outtakeFeeder;
    private boolean ableToResetTimer = true;
    private boolean nextSlot = true;
    private int slot = 0;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private long secondShot = 2300;
    private long thirdShot = secondShot * 2;
    private int pathState;

    private final Pose startPose = new Pose(96, 8, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(81, 87, Math.toRadians(226)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1APose = new Pose(113.5, 86, Math.toRadians(0));
    private final Pose pickup1BPose = new Pose(122, 86, Math.toRadians(0));
    private final Pose pickup1CPose = new Pose(132, 86, Math.toRadians(0));// Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(14, 68, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(14, 42, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain grabPickup1, grabPickup1A, grabPickup1B, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    boolean foundMotif = false;

    ArrayList<String> motif = new ArrayList<>();
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1APose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1APose.getHeading(), .1)
                .build();
        grabPickup1A = follower.pathBuilder()
                .addPath(new BezierLine(pickup1APose, pickup1BPose))
                .setLinearHeadingInterpolation(pickup1APose.getHeading(), pickup1BPose.getHeading(), .1)
                .build();
        grabPickup1B = follower.pathBuilder()
                .addPath(new BezierLine(pickup1BPose, pickup1CPose))
                .setLinearHeadingInterpolation(pickup1BPose.getHeading(), pickup1CPose.getHeading(), .1)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1CPose, scorePose))
                .setLinearHeadingInterpolation(pickup1CPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading(), .1)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading(), .1)
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */
                    if(ableToResetTimer)
                    {
                        pathTimer.resetTimer();
                        ableToResetTimer = false;
                    }

                    if(pathTimer.getElapsedTime() > 7100)
                    {
                        ableToResetTimer = true;
                        follower.followPath(grabPickup1,true);
                        setPathState(2);
                    } else if (((pathTimer.getElapsedTime() > 2150 && pathTimer.getElapsedTime() < 2800) || (pathTimer.getElapsedTime() > 2150 + secondShot && pathTimer.getElapsedTime() < 2800 + secondShot) || (pathTimer.getElapsedTime() > 2150 + thirdShot && pathTimer.getElapsedTime() < 2800 + thirdShot)) && nextSlot) {
                        if((pathTimer.getElapsedTime() > 6550 && pathTimer.getElapsedTime() < 7100))
                        {
                            sorter.setTargetPosition(sorter.getTargetPosition() + 64);
                        }
                        else
                        {
                            sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                        }

                        nextSlot = false;
                        slot = RotateMotorToNextSlot(sorter, slot);
                    } else if ((pathTimer.getElapsedTime() > 1600 && pathTimer.getElapsedTime() < 2150) || (pathTimer.getElapsedTime() > 1600 + secondShot && pathTimer.getElapsedTime() < 2150 + secondShot) || (pathTimer.getElapsedTime() > 1600 + thirdShot && pathTimer.getElapsedTime() < 2150 + thirdShot)) {
                        outtakeFeeder.setPosition(.4);
                    } else if ((pathTimer.getElapsedTime() > 1050 && pathTimer.getElapsedTime() < 1600) || (pathTimer.getElapsedTime() > 1050 + secondShot && pathTimer.getElapsedTime() < 1600 + secondShot) || (pathTimer.getElapsedTime() > 1050 + thirdShot && pathTimer.getElapsedTime() < 1600 + thirdShot)) {
                        outtakeFeeder.setPosition(1);
                    } else if ((pathTimer.getElapsedTime() > 500 && pathTimer.getElapsedTime() < 1050) || (pathTimer.getElapsedTime() > 500 + secondShot && pathTimer.getElapsedTime() < 1050 + secondShot) || (pathTimer.getElapsedTime() > 500 + thirdShot && pathTimer.getElapsedTime() < 1050 + thirdShot)) {
                        outtakeFeeder.setPosition(.75);
                        nextSlot = true;
                    }
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                intake.setPower(-1);
                if (pathTimer.getElapsedTime() > 2800 && slot == 0) {
                    sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sorter.setPower(1);
                    slot = RotateMotorToNextSlot(sorter, slot);
                }
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTime() > 3000)
                    {
                        follower.followPath(grabPickup1A,true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (pathTimer.getElapsedTime() > 1800 && slot == 1) {
                    sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sorter.setPower(1);
                    slot = RotateMotorToNextSlot(sorter, slot);
                }
                if(!follower.isBusy()) {
                    /* Score Sample */

                    if(pathTimer.getElapsedTime() > 2500)
                    {
                        follower.followPath(grabPickup1B,true);
                        setPathState(4);
                    }

                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (pathTimer.getElapsedTime() > 1600 && slot == 2) {
                    sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sorter.setPower(1);
                    slot = RotateMotorToNextSlot(sorter, slot);
                }
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if(pathTimer.getElapsedTime() > 2500)
                    {
                        sorter.setTargetPosition(sorter.getTargetPosition() + 64);
                        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sorter.setPower(1);
                        slot = RotateMotorToNextSlot(sorter, slot);
                        intake.setPower(0);
                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup1,true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    if(ableToResetTimer)
                    {
                        pathTimer.resetTimer();
                        ableToResetTimer = false;
                    }

                    if(pathTimer.getElapsedTime() > 7100)
                    {
                        ableToResetTimer = true;
                    } else if (((pathTimer.getElapsedTime() > 2150 && pathTimer.getElapsedTime() < 2800) || (pathTimer.getElapsedTime() > 2150 + secondShot && pathTimer.getElapsedTime() < 2800 + secondShot) || (pathTimer.getElapsedTime() > 2150 + thirdShot && pathTimer.getElapsedTime() < 2800 + thirdShot)) && nextSlot) {
                        if((pathTimer.getElapsedTime() > 6550 && pathTimer.getElapsedTime() < 7100))
                        {
                            sorter.setTargetPosition(sorter.getTargetPosition() + 64);
                        }
                        else
                        {
                            sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                        }

                        nextSlot = false;
                        slot = RotateMotorToNextSlot(sorter, slot);
                    } else if ((pathTimer.getElapsedTime() > 1600 && pathTimer.getElapsedTime() < 2150) || (pathTimer.getElapsedTime() > 1600 + secondShot && pathTimer.getElapsedTime() < 2150 + secondShot) || (pathTimer.getElapsedTime() > 1600 + thirdShot && pathTimer.getElapsedTime() < 2150 + thirdShot)) {
                        outtakeFeeder.setPosition(.4);
                    } else if ((pathTimer.getElapsedTime() > 1050 && pathTimer.getElapsedTime() < 1600) || (pathTimer.getElapsedTime() > 1050 + secondShot && pathTimer.getElapsedTime() < 1600 + secondShot) || (pathTimer.getElapsedTime() > 1050 + thirdShot && pathTimer.getElapsedTime() < 1600 + thirdShot)) {
                        outtakeFeeder.setPosition(1);
                    } else if ((pathTimer.getElapsedTime() > 500 && pathTimer.getElapsedTime() < 1050) || (pathTimer.getElapsedTime() > 500 + secondShot && pathTimer.getElapsedTime() < 1050 + secondShot) || (pathTimer.getElapsedTime() > 500 + thirdShot && pathTimer.getElapsedTime() < 1050 + thirdShot)) {
                        outtakeFeeder.setPosition(.75);
                        nextSlot = true;
                    }
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
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

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Slot: ", slot);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        topMotor = hardwareMap.get(DcMotorEx.class, "top");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottom");
        outtakeFeeder = hardwareMap.get(Servo.class, "feeder");
        sorter = hardwareMap.get(DcMotor.class, "sorter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(25,3,1.25,5.5));
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(25,3,1.25,5.5));
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
        outtakeFeeder.setPosition(0.4);
        //Applies power to the shooter
        topMotor.setVelocity(635);
        bottomMotor.setVelocity(635);
        sorter.setTargetPosition(128);
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
    //Rotates to the nest slot
    public static int RotateMotorToNextSlot(DcMotor sorter, int currentSlot){
        int nextSlot = currentSlot + 1;
        if(nextSlot == 3)
        {
            return 0;
        }

        return nextSlot;
    }

    //Shoot All Balls
    public void ShootAllBalls() {
        if(ableToResetTimer)
        {
            pathTimer.resetTimer();
            ableToResetTimer = false;
        }

        if(pathTimer.getElapsedTime() > 7100)
        {
            ableToResetTimer = true;
            follower.followPath(grabPickup1,true);
            setPathState(2);
        } else if (((pathTimer.getElapsedTime() > 2150 && pathTimer.getElapsedTime() < 2800) || (pathTimer.getElapsedTime() > 2150 + secondShot && pathTimer.getElapsedTime() < 2800 + secondShot) || (pathTimer.getElapsedTime() > 2150 + thirdShot && pathTimer.getElapsedTime() < 2800 + thirdShot)) && nextSlot) {
            if((pathTimer.getElapsedTime() > 6550 && pathTimer.getElapsedTime() < 7100))
            {
                sorter.setTargetPosition(sorter.getTargetPosition() + 64);
            }
            else
            {
                sorter.setTargetPosition(sorter.getTargetPosition() + 128);
            }

            nextSlot = false;
            slot = RotateMotorToNextSlot(sorter, slot);
        } else if ((pathTimer.getElapsedTime() > 1600 && pathTimer.getElapsedTime() < 2150) || (pathTimer.getElapsedTime() > 1600 + secondShot && pathTimer.getElapsedTime() < 2150 + secondShot) || (pathTimer.getElapsedTime() > 1600 + thirdShot && pathTimer.getElapsedTime() < 2150 + thirdShot)) {
            outtakeFeeder.setPosition(.4);
        } else if ((pathTimer.getElapsedTime() > 1050 && pathTimer.getElapsedTime() < 1600) || (pathTimer.getElapsedTime() > 1050 + secondShot && pathTimer.getElapsedTime() < 1600 + secondShot) || (pathTimer.getElapsedTime() > 1050 + thirdShot && pathTimer.getElapsedTime() < 1600 + thirdShot)) {
            outtakeFeeder.setPosition(1);
        } else if ((pathTimer.getElapsedTime() > 500 && pathTimer.getElapsedTime() < 1050) || (pathTimer.getElapsedTime() > 500 + secondShot && pathTimer.getElapsedTime() < 1050 + secondShot) || (pathTimer.getElapsedTime() > 500 + thirdShot && pathTimer.getElapsedTime() < 1050 + thirdShot)) {
            outtakeFeeder.setPosition(.75);
            nextSlot = true;
        }
    }


}