package org.firstinspires.ftc.teamcode;

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