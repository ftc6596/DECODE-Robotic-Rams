package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;

import kotlin.Unit;

@TeleOp(name="PIDFSorter", group="Linear OpMode")
public class PIDFSorter extends LinearOpMode {
    //Electronic Variables
    //Motors
    private DcMotorEx topMotor;
    private DcMotorEx bottomMotor;
    private DcMotorEx sorter;
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
        sorter = hardwareMap.get(DcMotorEx.class, "sorter");
        //Variables
        boolean changeValue = true;
        boolean nextSlot = false;
        int slot = 0;
        double pValue = 10;
        //Setup for Electronics
        sorter.setTargetPosition(0);
        sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sorter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(12.5,.5,3,15));
        sorter.setPower(1);
        waitForStart();
        while(opModeIsActive()) {
            PIDFCoefficients pidfCoefficients = sorter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

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

            //Moves the sorter to the next slot with no skipping
            if(!nextSlot)
            {
                if(gamepad2.right_bumper)
                {
                    nextSlot = true;
                    slot = RotateMotorToNextSlotEncoder(sorter, slot, false);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.left_bumper) {
                    nextSlot = true;
                    RotateMotorToNextHalfSlotEncoder(sorter);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.right_trigger != 0) {
                    nextSlot = true;
                    slot = RotateMotorToNextSlotEncoder(sorter, slot, true);
                    sorter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            else
            {
                if(!gamepad2.right_bumper && !gamepad2.left_bumper && gamepad2.right_trigger == 0)
                {
                    nextSlot = false;
                }
            }

            sorter .setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            //Telemetry
            telemetry.addData("Position: ", sorter.getCurrentPosition());
            telemetry.addData("PIDFValues: ", pidfCoefficients);
            telemetry.update();
        }

    }

    //Rotates to the next slot using encoder ticks
    public static int RotateMotorToNextSlotEncoder(DcMotor sorter, int slot, boolean inverse)
    {
        if(!inverse)
        {
            sorter.setTargetPosition(sorter.getTargetPosition() + 128);
            int nextSlot = slot + 1;
            if(nextSlot >= 3)
            {
                return 0;
            }

            return nextSlot;
        }
        else
        {
            sorter.setTargetPosition(sorter.getTargetPosition() - 128);
            int nextSlot = slot - 1;
            if(nextSlot <= -1)
            {
                return 2;
            }

            return nextSlot;
        }
    }

    public static void RotateMotorToNextHalfSlotEncoder(DcMotor sorter)
    {
        sorter.setTargetPosition(sorter.getTargetPosition() + 64);
    }
}

