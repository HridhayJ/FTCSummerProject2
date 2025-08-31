package org.firstinspires.ftc.teamcode;

// FTC SDK imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name="PDController", group="hi")
public class PDController extends LinearOpMode {
    private DcMotorEx motor;
    public static int targetPosition = 0; // desired encoder ticks
    public static double cov = 1.0;
    public static double pNoise = 0.01;
    public static double mNoise = 4.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // Initialize motor+
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        double previousTicks = motor.getCurrentPosition();

        // Timer for delta time
        ElapsedTime timer = new ElapsedTime();
        double previousTime = timer.seconds();

        // Initialize Kalman filter (initial estimate = current position)
        KalmanFilter1D kalman = new KalmanFilter1D(previousTicks, cov, pNoise, mNoise);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();

            double mPos = motor.getCurrentPosition();

            // Smooth ticks using Kalman filter
            double smoothTicks = kalman.update(mPos);

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

            double power = Kp * error + Kd * dotE;

            motor.setPower(power);

            // Update previous ticks
            previousTicks = smoothTicks;

            // Telemetry
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Raw Ticks", mPos);
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
