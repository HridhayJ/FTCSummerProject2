package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PDController", group="Self-Tuning")
public class PDController extends LinearOpMode {

    private DcMotor motor;
    private int targetPosition = 1000;

    @Override
    public void runOpMode() {

        // Initialize motor
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double previousTicks = 0.0;

        // Timer to measure loop delta time
        ElapsedTime timer = new ElapsedTime();
        double previousTime = 0.0;

        waitForStart();
        timer.reset();
        previousTime = timer.seconds();

        while (opModeIsActive()) {

            double currentTime = timer.seconds();
            double dt = currentTime - previousTime; // exact delta time
            previousTime = currentTime;

            // Read current encoder position
            double currentTicks = motor.getCurrentPosition();

            // Compute error and velocity
            double error = targetPosition - currentTicks;
            double velocity = (currentTicks - previousTicks) / dt; // ticks/sec
            double dotE = -velocity; // derivative of error

            // Self-tuning PD calculation
            double Kd = 0.0;
            double Kp = 0.0;

            if (error != 0) {
                double discriminant = dotE * dotE + 4 * error * error;
                Kd = (-dotE + Math.sqrt(discriminant)) / (2 * error);
                Kp = 0.25 * Kd * Kd; // critical damping
            }

            // Compute motor command
            double power = Kp * error + Kd * dotE;

            // Clamp motor power
            power = Math.max(Math.min(power, 1.0), -1.0);

            // Apply power
            motor.setPower(power);

            // Update previous ticks
            previousTicks = currentTicks;

            // Telemetry for debugging
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current", currentTicks);
            telemetry.addData("Error", error);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Kd", Kd);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
