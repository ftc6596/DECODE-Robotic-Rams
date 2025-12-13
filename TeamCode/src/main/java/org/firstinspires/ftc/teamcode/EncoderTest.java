package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;

@TeleOp(name="EncoderTest", group="Linear OpMode")
public class EncoderTest extends LinearOpMode {

    private AnalogInput analogInput0;
    private AnalogInput analogInput1;
    private DcMotor motorT;
    @Override
    public void runOpMode() throws InterruptedException {
        analogInput0 = hardwareMap.get(AnalogInput.class, "supplyVoltage");
        analogInput1 = hardwareMap.get(AnalogInput.class, "signalVoltage");
        motorT = hardwareMap.get(DcMotor.class, "motorT");
        waitForStart();
        motorT.setTargetPosition(0);
        motorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorT.setPower(1);
        motorT.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean nextSlot = false;
        boolean canReset = true;
        boolean resetting = false;
        int targetAngle = 120;
        double multiplier = 2;
        int slot = 0;
        while(opModeIsActive())
        {
            double voltage0 = analogInput0.getVoltage();
            double voltage1 = analogInput1.getVoltage();
            int currentAngle = (int)(voltage0 / voltage1 * 360);

            //Moves the sorter to the next slot with no skipping
            if(!nextSlot)
            {
                if(gamepad2.dpad_right)
                {
                    nextSlot = true;
                    slot = RotateMotorToNextSlotEncoder(motorT, slot, false);
                    motorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.y) {
                    nextSlot = true;
                    RotateMotorToNextHalfSlotEncoder(motorT);
                    motorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } else if (gamepad2.dpad_left) {
                    nextSlot = true;
                    slot = RotateMotorToNextSlotEncoder(motorT, slot, true);
                    motorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            else
            {
                if(!gamepad2.dpad_right && !gamepad2.y && !gamepad2.dpad_left)
                {
                    nextSlot = false;
                }
            }

            if(canReset)
            {
                if(gamepad2.b)
                {
                    canReset = false;
                    resetting = true;
                    motorT.setTargetPosition(0);
                    motorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorT.setTargetPosition((int)((targetAngle - currentAngle) * multiplier));
                    motorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            else
            {
                if(!gamepad2.b)
                {
                    canReset = true;
                }
            }

            if(resetting)
            {
                if((motorT.getCurrentPosition() == motorT.getTargetPosition() && !motorT.isBusy()))
                {
                    motorT.setTargetPosition(0);
                    motorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    resetting = false;
                }
            }
            telemetry.addData("Input 0 Output: ", voltage0);
            telemetry.addData("Input 1 Output: ", voltage1);

            telemetry.addData("Encoder Output: ", motorT.getCurrentPosition());
            telemetry.addData("Angle Output: ", currentAngle);

            telemetry.update();
        }
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
            sorter.setTargetPosition(260);
        } else if (slot <= 0) {
            sorter.setTargetPosition(0);
        }
        else
        {
            sorter.setTargetPosition(130);
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
    //Rotates to the next slot using encoder ticks
    public static void RotateMotorToNextHalfSlotEncoder(DcMotor sorter)
    {
        sorter.setTargetPosition(sorter.getTargetPosition() + 64);
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

}
