package org.firstinspires.ftc.teamcode.Telop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Cobalt")
public class Cobalt extends LinearOpMode {

    // Drive motors
    private DcMotor LB; // 0C
    private DcMotor LF; // 1C
    private DcMotor RB; // 2C
    private DcMotor RF; // 3C

    // Mechanism motors
    private DcMotorEx Nectar;      // 0E
    private DcMotorEx Pollen;      // 1E
    private DcMotorEx LinearSlide; // 2E
    private DcMotorEx Intake;      // 3E

    // Drive power values
    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;

    // Whether each shooter is currently spun up (toggled by B / A / X)
    boolean nectarOn = false;
    boolean pollenOn = false;

    // Tunable in FTC Dashboard
    public static double NECTAR_VEL = 2000;
    public static double POLLEN_VEL = 2000;
    public static double INTAKE_VEL = 2000;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        // Send telemetry to both the Driver Station and the Dashboard
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        Nectar = hardwareMap.get(DcMotorEx.class, "NECTAR");
        Pollen = hardwareMap.get(DcMotorEx.class, "POLLEN");
        LinearSlide = hardwareMap.get(DcMotorEx.class, "LS");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake"); // was wrongly assigned to LinearSlide

        // Directions
        Nectar.setDirection(DcMotor.Direction.REVERSE);
        // If the robot drives wrong, reverse the left (or right) side here, e.g.:
        // LB.setDirection(DcMotor.Direction.REVERSE);
        // LF.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders, then run using encoders
        DcMotor[] driveMotors = {LB, LF, RB, RF};
        for (DcMotor m : driveMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // setVelocity() requires RUN_USING_ENCODER
        DcMotorEx[] velocityMotors = {Nectar, Pollen, Intake};
        for (DcMotorEx m : velocityMotors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("Здравствуйте!");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart(); // (was called twice)
        runtime.reset();

        while (opModeIsActive()) {
            MecanumDrive();
            Launchers();
            Intake();
            // LinearSlide();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Nectar target", NECTAR_VEL);
            telemetry.addData("Nectar actual", Nectar.getVelocity());
            telemetry.addData("Pollen target", POLLEN_VEL);
            telemetry.addData("Pollen actual", Pollen.getVelocity());
            telemetry.addData("LB", lbPower);
            telemetry.addData("LF", lfPower);
            telemetry.addData("RB", rbPower);
            telemetry.addData("RF", rfPower);
            telemetry.update();
        }

        // Stop everything when the OpMode ends
        Nectar.setVelocity(0);
        Pollen.setVelocity(0);
        Intake.setVelocity(0);
    }

    // Shooter control:
    //   B = spin up Nectar, A = spin up Pollen, X = stop both,
    //   Y (hold) = run both in reverse (unjam); releasing returns to the previous state.
    // No sleep() inside the loop, so driving never freezes.
    private void Launchers() {
        if (gamepad1.b) nectarOn = true;
        if (gamepad1.a) pollenOn = true;
        if (gamepad1.x) {
            nectarOn = false;
            pollenOn = false;
        }

        if (gamepad1.y) {
            Nectar.setVelocity(-NECTAR_VEL);
            Pollen.setVelocity(-POLLEN_VEL);
        } else {
            Nectar.setVelocity(nectarOn ? NECTAR_VEL : 0);
            Pollen.setVelocity(pollenOn ? POLLEN_VEL : 0);
        }
    }

    /*private void LinearSlide() {

    }*/

    private void Intake() {
        if (gamepad1.left_trigger > 0)
            Intake.setVelocity(INTAKE_VEL);
        else if (gamepad1.left_bumper)
            Intake.setVelocity(-INTAKE_VEL);
        else
            Intake.setVelocity(0);
    }

    private void MecanumDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        lbPower = forward - strafe + turn;
        lfPower = forward + strafe + turn;
        rbPower = forward + strafe - turn;
        rfPower = forward - strafe - turn;

        // Normalize
        double max = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                Math.max(Math.abs(lbPower), Math.abs(rbPower)));

        if (max > 1.0) {
            lbPower /= max;
            lfPower /= max;
            rbPower /= max;
            rfPower /= max;
        }

        LB.setPower(lbPower);
        LF.setPower(lfPower);
        RB.setPower(rbPower);
        RF.setPower(rfPower);
    }
}