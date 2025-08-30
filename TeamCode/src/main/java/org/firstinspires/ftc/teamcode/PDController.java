package org.firstinspires.ftc.teamcode;

// FTC SDK imports
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name="PDController", group="Self-Tuning")
public class PDController extends LinearOpMode {

    private DcMotorEx motor;
    public static int targetPosition = 0; // desired encoder ticks
    public static double cov = 1.0;
    public static double processNoise = 0.01;
    public static double meas = 1.0;

    @Override
    public void runOpMode() {

        // Initialize motor+
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double previousTicks = motor.getCurrentPosition();

        // Timer for delta time
        ElapsedTime timer = new ElapsedTime();
        double previousTime = timer.seconds();

        // Initialize Kalman filter (initial estimate = current position)
        KalmanFilter1D kalman = new KalmanFilter1D(previousTicks, 1.0, 0.01, 4.0);

        waitForStart();
        timer.reset();
        previousTime = timer.seconds();

        while (opModeIsActive()) {

            double currentTime = timer.seconds();
            double dt = currentTime - previousTime;
            previousTime = currentTime;

            if (dt <= 0.0) dt = 0.001;

            // Read raw encoder ticks
            double rawTicks = motor.getCurrentPosition();

            // Smooth ticks using Kalman filter
            double smoothTicks = kalman.update(rawTicks);

            // Compute error and velocity using smoothed ticks
            double error = targetPosition - smoothTicks;
            double velocity = (smoothTicks - previousTicks) / dt;
            double dotE = -velocity;

            // Self-tuning PD calculation
            double Kd = 0.0;
            double Kp = 0.0;
            if (error != 0) {
                double discriminant = dotE * dotE + 4 * error * error;
                Kd = (-dotE + Math.sqrt(discriminant)) / (2 * error);
                Kp = 0.25 * Kd * Kd; // critical damping
            }

            // Compute motor power
            double power = Kp * error + Kd * dotE;

            // Optional deadzone to prevent jitter
            if (Math.abs(error) < 2) power = 0;

            // Clamp power
            power = Math.max(Math.min(power, 1.0), -1.0);

            // Send command to motor
            motor.setPower(power);

            // Update previous ticks
            previousTicks = smoothTicks;

            // Telemetry
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Raw Ticks", rawTicks);
            telemetry.addData("Smooth Ticks", smoothTicks);
            telemetry.addData("Error", error);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Kd", Kd);
            telemetry.addData("Power", power);
            telemetry.update();

        }
    }
}
