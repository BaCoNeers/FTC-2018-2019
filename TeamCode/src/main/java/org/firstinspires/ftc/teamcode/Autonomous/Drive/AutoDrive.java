package org.firstinspires.ftc.teamcode.Autonomous.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.ObjectIdentification.TensorFlowCubeDetection;
import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;

import java.util.ArrayList;


/**
 * Created by Simon on 27/09/2018.
 */

public class AutoDrive {

    public Coordinates Coords = new Coordinates(0, 0, 0);
    private DcMotor[] Motors = new DcMotor[4];
    private float[] Encoders = new float[4];
    private float[] MotorPower = new float[4];

    private static float Encoder = 1120f;
    private static float RobotCirumfrance = 1957.39f;
    private static float RobotOneDeg = RobotCirumfrance / 360f;
    private static float WheelCirumfrance = 320;
    private static float WheelCount = WheelCirumfrance / Encoder;


    //Mineral Varables
    public boolean BoxCheck = false;
    public int BoxPosition = 2;

    //Lift Varables
    enum LiftState {LiftBottom,LiftMiddle,LiftTop};
    LiftState PrimliftState = LiftState.LiftBottom;
    LiftState SecliftState = LiftState.LiftBottom;

    boolean PrevPrimState = false;
    long PrevPrimStateTime = 0;
    boolean PrevSecState = false;
    long PrevSecStateTime = 0;

    //initialization
    boolean RotateInit = false;

    private Telemetry tel;

    RoverRucusConfiguration config;

    public AutoDrive(RoverRucusConfiguration config,Telemetry tel) {
        this.config = config;
        Motors[0] = this.config.front_left_motor;
        Motors[1] = this.config.front_right_motor;
        Motors[2] = this.config.rear_left_motor;
        Motors[3] = this.config.rear_right_motor;
        this.tel = tel;
    }


    public void Update(ArrayList<Task> tasks) {
        UpdateTelemetry();
        UpdateEncoders();
        if (tasks.size() > 0) {
            if (tasks.get(0).CheckTask()) {
                tel.addLine("Max loop count: "+tasks.get(0).maxLoop);
                switch (tasks.get(0).Context) {
                    case "Forward":
                        if (Forward(tasks.get(0).Value, tasks.get(0).Power)) {
                            tasks.remove(0);
                            return;
                        }
                        break;
                    case "Turning":
                        //InitRotate(tasks.get(0).Value);
                        if (Rotate(tasks.get(0).Value,tasks.get(0).Power)){
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
                    case "Sleep":
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
                    case "Lift":
                        if(Lift(tasks.get(0).LiftState,tasks.get(0).Power )){
                            tasks.remove(0);
                            return;
                        }
                        break;
                    case "SetPosition":
                        if(SetPosition(tasks.get(0).disiredTime)){
                            tasks.remove(0);
                            return;
                        }
                        break;
                    case "Marker":
                        if(Marker(tasks.get(0).Value)){
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
/*
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
*/
    public boolean Rotate(float Angle, float Power) {
        int direction = 1;
        if(Angle<0){
            direction = -1;
        }
        if (Math.abs(ConvertToAngle(GetAvarage())) < Math.abs(Angle)) {
            MotorPower[0] = Power*direction;
            MotorPower[1] = -Power*direction;
            MotorPower[2] = Power*direction;
            MotorPower[3] = -Power*direction;
            UpdateMotor(true);
            UpdateEncoders();
            tel.addLine("Turning Running....");
            return false;
        } else {
            UpdateMotor(false);
            ResestMotors();
            UpdateEncoders();
            return true;
        }
    }
/*
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
*/
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
        if (System.nanoTime() < disiredTime) {
            tel.addLine("Sleeping....");
            return false;
        }
        return true;
    }

    private boolean SetPosition(float disiredTime) {
        if (System.nanoTime() < disiredTime) {
            config.prim_box_arm_servo.setPower(-0.3f);
            config.sec_box_arm_servo.setPower(0.3f);
            config.PrimHavServo.setPower(0.3f);
            config.SecHavServo.setPower(-0.3f);
            tel.addLine("Setting Position.....");
            return false;
        }
        else{
            config.prim_box_arm_servo.setPower(0);
            config.sec_box_arm_servo.setPower(0);
            config.PrimHavServo.setPower(0);
            config.SecHavServo.setPower(0);
        }
        return true;
    }


    boolean start = true;
    float count;
    private boolean Marker(float value){
        if(start){
            count = value;
            start = false;
        }
        else {
            count -= 1;
        }
        if(count>0){
            config.MarkerDrop.setPower(.3f);
            return false;
        }
        else {
            config.MarkerDrop.setPower(0);
            return true;
        }
    }

    private boolean TensorFlow(TensorFlowCubeDetection tensorFlow, ArrayList<Task> tasks,
                               long time) {
        if (System.nanoTime() < time) {
            BoxCheck = true;
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


    private boolean Lift(Boolean state, float Power){
        long CurrentTime = System.nanoTime();

        boolean PrimTimeElapsed = (PrevPrimStateTime + 200000000) < CurrentTime;
        boolean SecTImeElapsed = (PrevSecStateTime + 200000000) < CurrentTime;

        if(state && PrimliftState.equals(LiftState.LiftTop)){
            config.prim_lift_motor.setPower(0);
        }
        if(state && SecliftState.equals(LiftState.LiftTop)){
            config.sec_lift_motor.setPower(0);
        }
        if(!state && PrimliftState.equals(LiftState.LiftBottom)){
            config.prim_lift_motor.setPower(0);
        }
        if(!state && SecliftState.equals(LiftState.LiftBottom)){
            config.sec_lift_motor.setPower(0);
        }
        if(state && PrimliftState.equals(LiftState.LiftTop) && SecliftState.equals(LiftState.LiftTop)) {
            return true;
        }
        if(!state && PrimliftState.equals(LiftState.LiftBottom) && SecliftState.equals(LiftState.LiftBottom)) {
            return true;
        }

        boolean PrimStateLowToHigh = PrimTimeElapsed && !PrevPrimState && config.PrimLimitSwitch.getState();
        boolean PrimStateHighToLow = PrimTimeElapsed && PrevPrimState && !config.PrimLimitSwitch.getState();

        boolean SecStateLowToHigh = SecTImeElapsed && !PrevSecState && config.SecLimitSwitch.getState();
        boolean SecStateHighToLow = SecTImeElapsed && PrevSecState && !config.SecLimitSwitch.getState();

        if(PrimStateHighToLow || PrimStateLowToHigh) {
            PrevPrimState = config.PrimLimitSwitch.getState();
            PrevPrimStateTime = CurrentTime;
        }
        if(SecStateHighToLow || SecStateLowToHigh){
            PrevSecState = config.SecLimitSwitch.getState();
            PrevSecStateTime = CurrentTime;
        }

        if (state) {
            // Going up to top
            switch (PrimliftState) {
                case LiftBottom:
                    config.prim_lift_motor.setPower(-Power);
                    if (PrimStateLowToHigh) {
                        PrimliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.prim_lift_motor.setPower(-Power);
                    if (PrimStateHighToLow) {
                        PrimliftState = LiftState.LiftTop;
                    }
                    break;
                default:
                    break;
            }
            switch (SecliftState){
                case LiftBottom:
                    config.sec_lift_motor.setPower(Power);
                    if (SecStateLowToHigh) {
                        SecliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.sec_lift_motor.setPower(Power);
                    if (SecStateHighToLow) {
                        SecliftState = LiftState.LiftTop;
                    }
                    break;
                default:
                    break;
            }
        } else {
            //Going Down
            switch (PrimliftState){
                case LiftTop:
                    config.prim_lift_motor.setPower(Power);
                    if(PrimStateLowToHigh){
                        PrimliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.prim_lift_motor.setPower(Power);
                    if(PrimStateHighToLow){
                        PrimliftState = LiftState.LiftBottom;
                    }
                    break;
                default:
                    break;
            }
            switch (SecliftState){
                case LiftTop:
                    config.sec_lift_motor.setPower(-Power);
                    if (SecStateLowToHigh) {
                        SecliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.sec_lift_motor.setPower(-Power);
                    if (SecStateHighToLow) {
                        SecliftState = LiftState.LiftBottom;
                    }
                    break;
                default:
                    break;
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
