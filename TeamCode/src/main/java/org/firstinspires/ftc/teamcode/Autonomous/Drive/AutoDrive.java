package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import android.graphics.Path;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.ObjectIdentification.TensorFlowCubeDetection;
import org.firstinspires.ftc.teamcode.Configuration.Configuration;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;


/**
 * Created by Simon on 27/09/2018.
 */

public class AutoDrive {

    public Coordinates Coords = new Coordinates(0, 0, 0);
    private DcMotor[] Motors = new DcMotor[4];
    private float[] Encoders = new float[4];
    private float[] MotorPower = new float[4];

    private BNO055IMU imu = null;


    private static float Encoder = 1120f;
    private static float RobotCirumfrance = 1957.39f;
    private static float RobotOneDeg = RobotCirumfrance / 360f;
    private static float WheelCirumfrance = 320;
    private static float WheelCount = WheelCirumfrance / Encoder;

    public boolean BoxCheck = false;
    public int BoxPosition = 2;

    //initialization
    boolean RotateInit = false;

    private Telemetry tel;

    public AutoDrive(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, Telemetry tel, BNO055IMU imu2) {
        Motors[0] = FLM;
        Motors[1] = FRM;
        Motors[2] = BLM;
        Motors[3] = BRM;
        imu = imu2;
        this.tel = tel;
    }

    public AutoDrive(HardwareMap hardwaremap , Telemetry tel){
        this.tel = tel;
        Motors[0]  = hardwaremap.get(DcMotor.class, "front_left_drive");
        Motors[1] = hardwaremap.get(DcMotor.class, "front_right_drive");
        Motors[2] = hardwaremap.get(DcMotor.class,"rear_left_drive");
        Motors[3] = hardwaremap.get(DcMotor.class,"rear_right_drive");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwaremap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        Motors[0].setDirection(DcMotor.Direction.FORWARD);
        Motors[1].setDirection(DcMotor.Direction.REVERSE);
        Motors[2].setDirection(DcMotor.Direction.FORWARD);
        Motors[3].setDirection(DcMotor.Direction.REVERSE);

        Motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Update(ArrayList<Task> tasks) {
        UpdateTelemetry();
        UpdateEncoders();
        if (tasks.size() > 0) {
            if (tasks.get(0).CheckTask()) {
                switch (tasks.get(0).Context) {
                    case "Forward":
                        if (Forward(tasks.get(0).Value, tasks.get(0).Power)) {
                            tasks.remove(0);
                            return;
                        }
                        break;
                    case "Turning":
                        InitRotate(tasks.get(0).Value);
                        if (turn()){
                            tasks.remove(0);
                            RotateInit = false;
                            return;
                        }
                        break;
                    case "Strafing":
                        if (Strafe(tasks.get(0).Value, tasks.get(0).Power)) {
                            tasks.remove(0);
                            return;
                        }
                        break;
                    case "sleep":
                        if (sleep(tasks.get(0).disiredTime)) {
                            tasks.remove(0);
                            return;
                        }
                        break;
                    case "CubeDetection":
                        if (TensorFlow(tasks.get(0).tensorFlow, tasks, tasks.get(0).disiredTime)) {
                            tasks.remove(0);
                            return;
                        }
                        break;
                }
            } else {
                tasks.remove(0);
                return;
            }
        }
    }


    //Rotation Varables
    private Orientation angles = null;
    private float heading = 0;
    private float previousheading = heading;
    private float offset = 0;
    private float targetangle = 0;
    private boolean turnDirection = true;

    public void InitRotate(float degrestoturn) {
        if (!RotateInit) {
            //Put Init

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;


            targetangle = heading + degrestoturn;

            if (degrestoturn > 0) {
                turnDirection = true;
            } else {
                turnDirection = false;
            }

            RotateInit = true;
        }

    }

    public boolean Rotate(float Angle, float Power) {
        if (Math.abs(ConvertToAngle(GetAvarage())) < Math.abs(Angle)) {
            MotorPower[0] = Power;
            MotorPower[1] = -Power;
            MotorPower[2] = Power;
            MotorPower[3] = -Power;
            UpdateMotor(true);
            tel.addLine("Turning Running....");
            return false;
        } else {
            UpdateMotor(false);
            ResestMotors();
            UpdateEncoders();
            return true;
        }
    }

    //turning function
    public boolean turn() {
        tel.addLine("turning....");
        if (turnDirection) {
            if (heading + offset < targetangle - 5) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
                if (Math.abs(previousheading - heading) < 90) {
                } else {
                    offset += 360;
                }
                previousheading = heading;
                if (targetangle - (heading + offset) < 45) {
                    MotorPower[0] = (float) (0.5 * ((targetangle - (heading + offset)) / 360));
                    MotorPower[1] = (float) (-0.5 * ((targetangle - (heading + offset)) / 360));
                    MotorPower[2] = (float) (0.5 * ((targetangle - (heading + offset)) / 360));
                    MotorPower[3] = (float) (-0.5 * ((targetangle - (heading + offset)) / 360));
                    UpdateMotor(true);

                } else {
                    MotorPower[0] = (float) 0.5;
                    MotorPower[1] = (float) -0.5;
                    MotorPower[2] = (float) 0.5;
                    MotorPower[3] = (float) -0.5;
                    UpdateMotor(true);
                }
                return false;
            }
            else { UpdateMotor(false);
            return true;}
        } else if (!turnDirection) {
            if (heading + offset > targetangle - 5) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
                if (Math.abs(previousheading - heading) < 90) {
                } else {
                    offset -= 360;
                }
                previousheading = heading;
                if (targetangle - (heading + offset) < 45) {
                    MotorPower[0] = (float) (-0.5 * ((targetangle - (heading + offset)) / 180 / 8));
                    MotorPower[1] = (float) (0.5 * ((targetangle - (heading + offset)) / 180 / 8));
                    MotorPower[2] = (float) (-0.5 * ((targetangle - (heading + offset)) / 180 / 8));
                    MotorPower[3] = (float) (0.5 * ((targetangle - (heading + offset)) / 180 / 8));
                    UpdateMotor(true);
                } else {
                    MotorPower[0] = (float) -0.5;
                    MotorPower[1] = (float) 0.5;
                    MotorPower[2] = (float) -0.5;
                    MotorPower[3] = (float) 0.5;
                    UpdateMotor(true);
                }
                return false;
            }else {
                UpdateMotor(false);
                ResestMotors();
                return true;
            }

        }
        return false;
    }



    public boolean Forward(float Distance, float Power) {
        tel.addLine("Forward Running....");
        int direction = 1;
        if(Distance<0){
            direction = -1;
        }
        MotorPower[0] = Power*direction;
        MotorPower[1] = Power*direction;
        MotorPower[2] = Power*direction;
        MotorPower[3] = Power*direction;
        ForwardScale(Distance);
        if (GetAvaragePower() > 0.1) {
            UpdateMotor(true);
            UpdateEncoders();
            return false;
        } else {
            UpdateMotor(false);
            ResestMotors();
            UpdateEncoders();
            return true;
        }
    }

    public boolean Strafe(float Distance, float Power) {
        int direction = 1;
        if(Distance<0){
            direction = -1;
        }
        if (Math.abs(ConverForStrafe(GetAvarage())) < Math.abs(Distance)) {
            MotorPower[0] = Power*direction;
            MotorPower[1] = -Power*direction;
            MotorPower[2] = -Power*direction;
            MotorPower[3] = Power*direction;
            tel.addLine("Strafing running...");
            UpdateMotor(true);
            UpdateEncoders();
            return false;
        } else {
            UpdateMotor(false);
            ResestMotors();
            UpdateEncoders();
            return true;
        }
    }

    private boolean sleep(float disiredTime) {
        if (System.currentTimeMillis() / 1000 < disiredTime) {
            tel.addLine("Sleeping....");
            return false;
        }
        return true;
    }

    private boolean TensorFlow(TensorFlowCubeDetection tensorFlow, ArrayList<Task> tasks,
                               long time) {
        if (System.currentTimeMillis() / 1000 > time) {
            return true;
        } else {
            if (tensorFlow.GetCubePos() != 0) {
                BoxCheck = true;
                BoxPosition = tensorFlow.GetCubePos();
                return true;
            }
        }
        return false;
    }

    public void ForwardScale(float Distance) {
        float value = Math.abs(ConvertToMM(GetAvarage())) / Math.abs(Distance);
        value = 1 - value;
        if (Distance - 200 > 0) {
            value = value * (Distance / 200);
        }
        if (!(value > 1)) {
            MotorPower[0] = MotorPower[0] * value;
            MotorPower[1] = MotorPower[1] * value;
            MotorPower[2] = MotorPower[2] * value;
            MotorPower[3] = MotorPower[3] * value;
        }
    }

    public float GetAvaragePower() {
        float value = 0;
        for (int i = 0; i < MotorPower.length; i++) {
            value += Math.abs(MotorPower[i]);
        }
        return value / MotorPower.length;
    }

    public float GetAvarage() {
        float value = 0;
        for (int i = 0; i < Encoders.length; i++) {
            value += Math.abs(Encoders[i]);
        }
        return value / Encoders.length;
    }

    public float ConvertToMM(float Encoder) {
        return Encoder * WheelCount;
    }

    public float ConvertToAngle(float Encoder) {
        return (Encoder * WheelCount) / RobotOneDeg;
    }

    public float ConverForStrafe(float Encoder) {
        return (Encoder * WheelCount) * 0.86f;
    }

    private void UpdateMotor(boolean On) {
        if (On) {
            Motors[0].setPower(MotorPower[0]);
            Motors[1].setPower(MotorPower[1]);
            Motors[2].setPower(MotorPower[2]);
            Motors[3].setPower(MotorPower[3]);
        } else {
            Motors[0].setPower(0);
            Motors[1].setPower(0);
            Motors[2].setPower(0);
            Motors[3].setPower(0);
        }
    }

    private void UpdateEncoders() {
        Encoders[0] = Motors[0].getCurrentPosition();
        Encoders[1] = Motors[1].getCurrentPosition();
        Encoders[2] = Motors[2].getCurrentPosition();
        Encoders[3] = Motors[3].getCurrentPosition();
    }


    private void ResestMotors() {
        Motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    private void UpdateTelemetry() {
        tel.addLine("Avg Encoder: " + GetAvarage());
    }


}
