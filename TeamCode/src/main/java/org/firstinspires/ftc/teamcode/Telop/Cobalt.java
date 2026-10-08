package org.firstinspires.ftc.teamcode.Telop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "Cobalt")
public class Cobalt extends LinearOpMode {

    private DcMotor LB; // 0C
    private DcMotor LF; // 1C
    private DcMotor RB; // 2C
    private DcMotor RF; // 3C

    // Drive power values
    private DcMotorEx Nectar; // 0E
    private DcMotorEx Pollen; // 1E
    private DcMotorEx Intake; // 2E

    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;
    boolean shootingNectar = false;
    boolean shootingPollen = false;

    // Shooter velocity (tunable in FTC Dashboard)
    public static double NECTAR_VEL = 2000;
    public static double POLLEN_VEL = 2000;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        Nectar = hardwareMap.get(DcMotorEx.class, "LS");

        // Directions
        Nectar.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Здравствуйте!");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            MecanumDrive();
            NectarLauncher();
            PollenLauncher();
            Intake();
            

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Shooter Velocity:", NECTAR_VEL);
            telemetry.addData("Intake:", POLLEN_VEL);
            telemetry.addData("LB:", lbPower);
            telemetry.addData("LF:", lfPower);
            telemetry.addData("RB:", rbPower);
            telemetry.addData("RF:", rfPower);
            telemetry.update();
        }
    }

    // Shooter control
    private void NectarLauncher() {

        if (gamepad1.b){
            Nectar.setVelocity(NECTAR_VEL);
        }
        if (gamepad1.x){
            Nectar.setVelocity(0);
            shootingNectar = false;
        }
        if (gamepad1.y&&!shootingNectar){
            Nectar.setVelocity(-NECTAR_VEL);
            shootingNectar=true;
            sleep(10);
            Nectar.setVelocity(0);
            shootingNectar = false;
        }
    }
    private void PollenLauncher() {

        if (gamepad1.a){
            Pollen.setVelocity(POLLEN_VEL);
        }
        if (gamepad1.x){
            Pollen.setVelocity(0);
            shootingPollen = false;
        }
        if (gamepad1.y&&!shootingPollen){
            Pollen.setVelocity(-POLLEN_VEL);
            shootingPollen=true;
            sleep(10);
            Pollen.setVelocity(0);
            shootingPollen = false;
        }
    }
    private void Intake() {
        if (gamepad1.left_trigger>0)
            Intake.setVelocity(2000);
        else if (gamepad1.left_bumper)
            Intake.setVelocity(-2000);
        else
            Intake.setVelocity(0);
        }

    private void MecanumDrive(){
        // Mecanum drive calculations
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
