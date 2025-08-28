package org.firstinspires.ftc.teamcode;

// FTC SDK imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="PDController", group="Self-Tuning")
public class PDController extends LinearOpMode {

    private DcMotorEx motor;
    private int targetPosition = 1000; // desired encoder ticks

    @Override
    public void runOpMode() {

        // Initialize motor
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

            sleep(5); // small delay to avoid overloading the loop
        }
    }

    // -------------------
    // 1D Kalman filter class
    // -------------------
    public class KalmanFilter1D {
        private double x; // estimated position
        private double P; // estimation error covariance
        private double Q; // process noise
        private double R; // measurement noise

        public KalmanFilter1D(double initialEstimate, double initialCovariance, double processNoise, double measurementNoise) {
            x = initialEstimate;
            P = initialCovariance;
            Q = processNoise;
            R = measurementNoise;
        }

        public double update(double measurement) {
            // Predict
            double x_pred = x;
            double P_pred = P + Q;

            // Update
            double K = P_pred / (P_pred + R);
            x = x_pred + K * (measurement - x_pred);
            P = (1 - K) * P_pred;

            return x;
        }

        public double getEstimate() {
            return x;
        }
    }
}
