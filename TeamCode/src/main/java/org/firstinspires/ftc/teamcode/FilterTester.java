package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
@TeleOp(name="KalmanFilterTest", group="Test")
public class FilterTester extends LinearOpMode {

    private DcMotor motor;
    private KalmanFilter1D kf;
    public static double cov = 1.0;
    public static double pNoise = 0.01;
    public static double mNoise = 4.0;
    double initialPos;

    @Override
    public void runOpMode() {
        // Init motor
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initialPos = motor.getCurrentPosition();

        // Init Kalman filter (tune Q and R!)
        kf = new KalmanFilter1D(initialPos, cov, pNoise, mNoise);

        waitForStart();

        while (opModeIsActive()) {
            // Raw encoder value
            double z = motor.getCurrentPosition();

            // Filtered value
            double filtered = kf.update(z);

            // Display
            telemetry.addData("Raw Encoder", z);
            telemetry.addData("Filtered Encoder", filtered);
            telemetry.update();
        }
    }
}
