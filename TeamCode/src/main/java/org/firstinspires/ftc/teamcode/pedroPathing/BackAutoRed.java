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

@Autonomous(name="BackAutoRed", group="OpMode")
public class BackAutoRed extends OpMode {
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
    private boolean nextSlot = true;
    private int slot = 0;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private long secondShot = 1750;
    private long thirdShot = secondShot * 2;
    private int pathState;

    private final Pose startPose = new Pose(84, 9.5, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(84, 16, Math.toRadians(250)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(138, 16, Math.toRadians(355));
    private final Pose pickup2Pose = new Pose(134, 16, Math.toRadians(0));
    private final Pose endingPose = new Pose(134, 16, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain grabPickup1, backup, grabPickup1B, scorePickup1, ending;
    boolean foundMotif = false;
    double velocity = 810;

    ArrayList<String> motif = new ArrayList<>();
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading(), .1)
                .build();
        backup = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading(), .1)
                .build();
        grabPickup1B = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup1Pose.getHeading(), .1)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();


        ending = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endingPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endingPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                if(pathTimer.getElapsedTime() > 450)
                {
                    setPathState(1);
                }
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                ShootAllBalls(2, grabPickup1, 0);
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                intake.setPower(-1);
                if (pathTimer.getElapsedTime() > 250 && slot == 0) {
                    sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sorter.setPower(1);
                    slot = RotateMotorToNextSlot(sorter, slot);
                }
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTime() > 500)
                    {
                        follower.followPath(backup,true);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (pathTimer.getElapsedTime() > 500 && slot == 2) {
                    sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sorter.setPower(1);
                    slot = RotateMotorToNextSlot(sorter, slot);
                }
                if(!follower.isBusy()) {
                    /* Score Sample */

                    if(pathTimer.getElapsedTime() > 750)
                    {
                        follower.followPath(grabPickup1B,true);
                        setPathState(4);
                    }

                }
                break;
            case 4:
                if (pathTimer.getElapsedTime() > 500 && slot == 2) {
                    sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sorter.setPower(1);
                    slot = RotateMotorToNextSlot(sorter, slot);
                }
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if(pathTimer.getElapsedTime() > 750)
                    {
                        sorter.setTargetPosition(sorter.getTargetPosition() + 64);
                        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sorter.setPower(1);
                        slot = RotateMotorToNextSlot(sorter, slot);

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup1,true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                intake.setPower(0);
                if(!follower.isBusy()) {
                    ShootAllBalls(6, ending, 0);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                intake.setPower(-1);
                if (pathTimer.getElapsedTime() > 250 && slot == 0) {
                    sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sorter.setPower(1);
                    slot = RotateMotorToNextSlot(sorter, slot);
                }
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    if(pathTimer.getElapsedTime() > 500)
                    {
                        follower.followPath(backup,true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (pathTimer.getElapsedTime() > 500 && slot == 2) {
                    sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sorter.setPower(1);
                    slot = RotateMotorToNextSlot(sorter, slot);
                }
                if(!follower.isBusy()) {
                    /* Score Sample */

                    if(pathTimer.getElapsedTime() > 750)
                    {
                        follower.followPath(grabPickup1B,true);
                        setPathState(8);
                    }

                }
                break;
            case 8:
                if (pathTimer.getElapsedTime() > 500 && slot == 2) {
                    sorter.setTargetPosition(sorter.getTargetPosition() + 128);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    sorter.setPower(1);
                    slot = RotateMotorToNextSlot(sorter, slot);
                }
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    if(pathTimer.getElapsedTime() > 750)
                    {
                        sorter.setTargetPosition(sorter.getTargetPosition() + 64);
                        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        sorter.setPower(1);
                        slot = RotateMotorToNextSlot(sorter, slot);

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(scorePickup1,true);
                        setPathState(9);
                    }
                }
                break;
            case 9:
                intake.setPower(0);
                if(!follower.isBusy()) {
                    ShootAllBalls(10, ending, 0);
                }
                break;
            case 10:
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

        LLResult result = limelight.getLatestResult();
        if(result != null) {
            if (result.isValid()) {
                //Goes through all the Tags
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int tagId = tag.getFiducialId();
                    //Checks if the tag is not the Obelisk
                    if (tagId != 21 && tagId != 22 && tagId != 23) {
                        double a = result.getTa();//Tag Area
                        //Auto-Velocity Code
                        velocity = 850 - ((195 * a) - 20);
                    }
                }
            }
        }

        topMotor.setVelocity(velocity);
        bottomMotor.setVelocity(velocity);
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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        topMotor = hardwareMap.get(DcMotorEx.class, "top");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottom");
        outtakeFeeder = hardwareMap.get(Servo.class, "feeder");
        sorter = hardwareMap.get(DcMotorEx.class, "sorter");
        intake = hardwareMap.get(DcMotor.class, "intake");

        sorter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(27.5,0,1.25,14.1));
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(27.5,0,1.25,14.1));
        sorter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(12.5,.5,3,15));

        limelight.pipelineSwitch(0);
        limelight.start();
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
        outtakeFeeder.setPosition(0);
        //Applies power to the shooter
        topMotor.setVelocity(velocity);
        bottomMotor.setVelocity(velocity);
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
    public void ShootAllBalls(int nextPath, PathChain nextPathChain, int ballIndexId) {
        if(ableToResetTimer)
        {
            pathTimer.resetTimer();
            ableToResetTimer = false;
            nextSlot = true;
        }

        if(pathTimer.getElapsedTime() > 4450)
        {
            sorter.setTargetPosition(sorter.getTargetPosition() + 64);
            ableToResetTimer = true;
            follower.followPath(nextPathChain,true);
            setPathState(nextPath);
        }  else if (((pathTimer.getElapsedTime() > 1400 && pathTimer.getElapsedTime() < 2050) || (pathTimer.getElapsedTime() > 1400 + secondShot && pathTimer.getElapsedTime() < 2050 + secondShot)) && nextSlot) {
            sorter.setTargetPosition(sorter.getTargetPosition() + 128);
            nextSlot = false;
            slot = RotateMotorToNextSlot(sorter, slot);
        } else if ((pathTimer.getElapsedTime() > 850 && pathTimer.getElapsedTime() < 1400) || (pathTimer.getElapsedTime() > 850 + secondShot && pathTimer.getElapsedTime() < 1400 + secondShot) || (pathTimer.getElapsedTime() > 850 + thirdShot && pathTimer.getElapsedTime() < 1400 + thirdShot)) {
            outtakeFeeder.setPosition(0);
        } else if ((pathTimer.getElapsedTime() > 500 && pathTimer.getElapsedTime() < 850) || (pathTimer.getElapsedTime() > 500 + secondShot && pathTimer.getElapsedTime() < 850 + secondShot) || (pathTimer.getElapsedTime() > 500 + thirdShot && pathTimer.getElapsedTime() < 850 + thirdShot)) {
            outtakeFeeder.setPosition(1);
            nextSlot = true;
        }
    }
}