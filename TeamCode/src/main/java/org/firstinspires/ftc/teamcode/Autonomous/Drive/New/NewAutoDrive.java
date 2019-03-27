package org.firstinspires.ftc.teamcode.Autonomous.Drive.New;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.TrackableResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.NewAttempt.Task;
import org.firstinspires.ftc.teamcode.Autonomous.ObjectIdentification.TensorFlowCubeDetection;
import org.firstinspires.ftc.teamcode.Configuration.RoverRucusConfiguration;

import java.util.ArrayList;

public class NewAutoDrive {

    //motor variables
    private DcMotor[] Motors = new DcMotor[4];
    private float[] Encoders = new float[4];
    private float[] MotorPower = new float[4];

    //Robot size
    private static float Encoder = 1120f;
    private static float WheelCirumfrance = 320;
    private static float WheelCount = WheelCirumfrance / Encoder;
    

    //Lift Varables
    enum LiftState {LiftBottom,LiftMiddle,LiftTop}
    private LiftState PrimliftState = LiftState.LiftBottom;
    private LiftState SecliftState = LiftState.LiftBottom;
    private boolean PrevPrimState = false;
    private long PrevPrimStateTime = 0;
    private boolean PrevSecState = false;
    private long PrevSecStateTime = 0;


    private Telemetry tel;
    private RoverRucusConfiguration config;

    private TensorFlowCubeDetection tensorFlow = new TensorFlowCubeDetection();

    //IMU
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    //cube detection
    public boolean BoxCheck = false;
    public int CubePosition;
    boolean Looping = true;

    //Tasks
    public ArrayList<MainTask> Tasks = new ArrayList<>();


    public NewAutoDrive(RoverRucusConfiguration config, float wheelCircumgfranceMM, float encoderFullrotation, Telemetry tel, HardwareMap hardwaremap) {
        this.config = config;
        Motors[0] = this.config.front_left_motor;
        Motors[1] = this.config.front_right_motor;
        Motors[2] = this.config.rear_left_motor;
        Motors[3] = this.config.rear_right_motor;
        this.tel = tel;
        tensorFlow.Int(this.tel,hardwaremap);

        WheelCirumfrance = wheelCircumgfranceMM;
        Encoder = encoderFullrotation;
        WheelCount = WheelCirumfrance / Encoder;
    }


    public void InitialiseIMU(HardwareMap hardwareMap){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
    public boolean CalabrateIMU(){
        return (!imu.isGyroCalibrated());

    }


    public void Update() {
        UpdateTelemetry();
        UpdateEncoders();
        if (Tasks.size() > 0) {
            //check if task is stuck in loop
            MainTask currentTask = Tasks.get(0);
            if (currentTask.CheckTask()) {
                //check what type of task
                switch (currentTask.context) {
                    case "Forward":
                        if (Forward(currentTask.value, currentTask.power)) {
                            Tasks.remove(currentTask);
                            Status.add(currentTask.context + "finished");
                            return;
                        }
                        break;
                    case "Turning":
                        if(IMURotation(currentTask.value,currentTask.power)){
                            Tasks.remove(currentTask);
                            Status.add(currentTask.context + "finished");
                            return;
                        }
                        break;
                    case "Strafing":
                        if (Strafe(currentTask.value, currentTask.power)) {
                            Tasks.remove(currentTask);
                            Status.add(currentTask.context + "finished");
                            return;
                        }
                        break;
                    case "CubeDetection":
                        if (TensorFlow(currentTask.Left,currentTask.Middle,currentTask.Right)){
                            Tasks.remove(currentTask);
                            Status.add(currentTask.context + "finished");
                            return;
                        }
                        break;
                    case "Lift":
                        if(Lift(currentTask.LiftState,currentTask.power )){
                            Tasks.remove(currentTask);
                            Status.add(currentTask.context + "finished");
                            return;
                        }
                        break;
                    case "Marker":
                        if(Marker(currentTask.value)){
                            Tasks.remove(currentTask);
                            Status.add(currentTask.context + "finished");
                            return;
                        }
                        break;
                    case "Wait":
                        if(wait(currentTask.value)){
                            Tasks.remove(currentTask);
                            Status.add(currentTask.context + "finished");
                            return;
                        }
                        break;
                }
            } else {
                Tasks.remove(currentTask);
            }
        }
    }

    private boolean wait(float seconds){
        if(seconds < System.nanoTime()){
            return true;
        }
        return false;
    }

    private boolean IMURotation(float Angle, float power){
        tel.addLine("Turning.....");
        int direction = 1;
        if(Angle>0){
            direction = -1;
        }
        if(Math.abs(getAngle()) < Math.abs(Angle)){
            MotorPower[0] = power*direction;
            MotorPower[1] = -power*direction;
            MotorPower[2] = power*direction;
            MotorPower[3] = -power*direction;
            UpdateMotor(true);
            tel.addLine("Angle: "+getAngle());
            return false;
        }
        else{
            ResestMotors();
            UpdateEncoders();
            resetAngle();
            UpdateMotor(false);
            return true;
        }
    }

    public boolean Forward(float Distance, float power) {
        tel.addLine("Forward Running....");
        tel.addLine("Encoder: "+GetAvarage());
        int direction = 1;
        if(Distance<0){
            direction = -1;
        }
        MotorPower[0] = power*direction;
        MotorPower[1] = power*direction;
        MotorPower[2] = power*direction;
        MotorPower[3] = power*direction;
        UpdateEncoders();
        ForwardScale(Distance);
        if (GetAvaragepower() > 0.1) {
            UpdateMotor(true);
            return false;
        } else {
            UpdateMotor(false);
            ResestMotors();
            UpdateEncoders();
            return true;
        }
    }



    private boolean Strafe(float Distance, float power) {
        int direction = 1;
        if(Distance<0){
            direction = -1;
        }
        if (Math.abs(ConverForStrafe(GetAvarage())) < Math.abs(Distance)) {
            MotorPower[0] = power*direction;
            MotorPower[1] = -power*direction;
            MotorPower[2] = -power*direction;
            MotorPower[3] = power*direction;
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


    private boolean start = true;
    private float count;
    private boolean Marker(float value){
        if(start){
            count = value;
            start = false;
        }
        else {
            count -= 1;
        }
        if(count>0){
            //config.MarkerDrop.setPower(.3f);
            return false;
        }
        else {
            //config.MarkerDrop.setPower(0);
            return true;
        }
    }

    public void CubePosition(){
        tensorFlow.start();
        int count = 0;

        while(tensorFlow.GetCubePos() == 0){
            count++;
            if(count > 2000){
                Looping = false;
            }
        }

        Looping = false;
        BoxCheck = true;
        tensorFlow.running = false;
        CubePosition = tensorFlow.GetCubePos();
    }

    private boolean TensorFlow(ArrayList<MainTask> Left, ArrayList<MainTask> Middle, ArrayList<MainTask> Right) {
        switch (CubePosition){
            case 0:
                for(int i=Middle.size()-1;i>=0;i--){
                    Tasks.add(1,Middle.get(i));
                }
                break;
            case 1:

                for(int i=Left.size()-1;i>=0;i--) {
                    Tasks.add(1,Left.get(i));
                }

                break;
            case 2:
                for(int i=Middle.size()-1;i>=0;i--){
                    Tasks.add(1,Middle.get(i));
                }
                break;
            case 3:
                for(int i=Right.size()-1;i>=0;i--){
                    Tasks.add(1,Right.get(i));
                }
        }
        BoxCheck = true;
        return true;
    }


    private boolean Lift(Boolean state, float power){
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
                    config.prim_lift_motor.setPower(-power);
                    if (PrimStateLowToHigh) {
                        PrimliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.prim_lift_motor.setPower(-power);
                    if (PrimStateHighToLow) {
                        PrimliftState = LiftState.LiftTop;
                    }
                    break;
                default:
                    break;
            }
            switch (SecliftState){
                case LiftBottom:
                    config.sec_lift_motor.setPower(power);
                    if (SecStateLowToHigh) {
                        SecliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.sec_lift_motor.setPower(power);
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
                    config.prim_lift_motor.setPower(power);
                    if(PrimStateLowToHigh){
                        PrimliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.prim_lift_motor.setPower(power);
                    if(PrimStateHighToLow){
                        PrimliftState = LiftState.LiftBottom;
                    }
                    break;
                default:
                    break;
            }
            switch (SecliftState){
                case LiftTop:
                    config.sec_lift_motor.setPower(-power);
                    if (SecStateLowToHigh) {
                        SecliftState = LiftState.LiftMiddle;
                    }
                    break;
                case LiftMiddle:
                    config.sec_lift_motor.setPower(-power);
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

    private void ForwardScale(float Distance) {
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

    private float GetAvaragepower() {
        float value = 0;
        for (float power:MotorPower) {
            value += Math.abs(power);
        }
        return value / MotorPower.length;
    }

    private float GetAvarage() {
        float value = 0;
        for(float encoder:Encoders){
            value += Math.abs(encoder);
        }
        return value / Encoders.length;
    }

    private float ConvertToMM(float Encoder) {
        return Encoder * WheelCount;
    }

    private float ConverForStrafe(float Encoder) {
        return (Encoder * WheelCount) * 0.86f;
    }

    //http://stemrobotics.cs.pdx.edu/node/7265

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
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

    ArrayList<String> Status = new ArrayList<>();
    private void UpdateTelemetry() {
        for(int i=0;i<Status.size();i++){
            tel.addLine("Job: "+Status.get(i));
        }

    }


}
