package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Disabled
@TeleOp(name="PIDShooter", group="OpMode")
public class PIDFShooter extends OpMode {

    public DcMotorEx topMotor;
    public DcMotorEx bottomMotor;
    public double highVelo = 885;
    public double lowVelo = 700;
    double curTargetVelo = highVelo;

    double P = 8.0;
    double F = 13.895;
    double[] stopSizes = {10.0, 1.0, 0.1, 0.01, 0.001};

    int stopIndex = 1;

    @Override
    public void init() {
        topMotor = hardwareMap.get(DcMotorEx.class, "top");
        bottomMotor = hardwareMap.get(DcMotorEx.class, "bottom");

        topMotor.setDirection(DcMotor.Direction.REVERSE);
        topMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        topMotor.setPower(1);
        bottomMotor.setPower(1);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        if(gamepad2.yWasPressed())
        {
            if(curTargetVelo == highVelo)
            {
                curTargetVelo = lowVelo;
            }
            else {
                curTargetVelo = highVelo;
            }
        }

        if(gamepad2.bWasPressed())
        {
            stopIndex = (stopIndex + 1) % stopSizes.length;
        }

        if(gamepad2.dpadLeftWasPressed())
        {
            F += stopSizes[stopIndex];
        }
        if(gamepad2.dpadRightWasPressed())
        {
            F -= stopSizes[stopIndex];
        }
        if(gamepad2.dpadUpWasPressed())
        {
            P += stopSizes[stopIndex];
        }
        if(gamepad2.dpadDownWasPressed())
        {
            P -= stopSizes[stopIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        topMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        bottomMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        topMotor.setVelocity(curTargetVelo);
        bottomMotor.setVelocity(curTargetVelo);

        double curVelo = topMotor.getVelocity();
        double error = curTargetVelo - curVelo;

        telemetry.addData("Target Velo: ", curTargetVelo);
        telemetry.addData("Current Velo: ", "%.2f", curVelo);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stopSizes[stopIndex]);
    }
}
