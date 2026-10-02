package org.firstinspires.ftc.teamcode;

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

    private DcMotorEx Nectar; // 0E
    private DcMotorEx Pollen; // 1E

    boolean shootingNectar = false;
    boolean shootingPollen = false;

    // Shooter velocity (tunable in FTC Dashboard)
    public static double NECTAR_VEL = 2000;
    public static double POLLEN_VEL = 2000;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        Nectar = hardwareMap.get(DcMotorEx.class, "LS");
        Nectar.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            NectarLauncher();
            PollenLauncher();
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
}
