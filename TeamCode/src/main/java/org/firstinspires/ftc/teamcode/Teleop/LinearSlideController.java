package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LinearSlideController {

    // Tunable in FTC Dashboard
    public static double kP = 0.005;
    public static double kI = 0.0;
    public static double kD = 0.0002;
    public static double kG = 0.08;          // constant power to fight gravity when up
    public static double UP_POS = 425;      // encoder ticks, tune this
    public static double DOWN_POS = 0;
    public static double MAX_POWER = 1.0;
    public static double I_LIMIT = 0.3;      // anti-windup cap on the i term's contribution

    private final DcMotor motor;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean up = false;
    private boolean lastButton = false;
    private double integral = 0;
    private double lastError = 0;
    private double target = 0;
    private double power = 0;

    public LinearSlideController(DcMotor motor) {
        this.motor = motor;
    }

    /** Call once in init. The slide must be fully down when this runs. */
    public void init() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // we're doing the PID ourselves
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
    }

    /** Call right after waitForStart() so the first dt isn't huge. */
    public void start() {
        timer.reset();
    }

    /** Call every loop. Pass in the toggle button (e.g. gamepad1.dpad_up). */
    public void update(boolean toggleButton) {
        // Toggle on button press (rising edge only)
        if (toggleButton && !lastButton) {
            up = !up;
            integral = 0; // reset I so old error doesn't carry over
        }
        lastButton = toggleButton;

        target = up ? UP_POS : DOWN_POS;

        // PID
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-3;

        double error = target - motor.getCurrentPosition();

        integral += error * dt;
        if (kI != 0) {
            double iMax = I_LIMIT / kI;
            integral = Math.max(-iMax, Math.min(iMax, integral));
        }

        double derivative = (error - lastError) / dt;
        lastError = error;

        double ff = up ? kG : 0; // gravity compensation only when raised
        power = kP * error + kI * integral + kD * derivative + ff;
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        motor.setPower(power);
    }

    public double getTarget() { return target; }
    public double getPosition() { return motor.getCurrentPosition(); }
    public double getPower() { return power; }
    public boolean isUp() { return up; }
}