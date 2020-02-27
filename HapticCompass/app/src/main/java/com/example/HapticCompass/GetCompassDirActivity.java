package com.example.HapticCompass;

import android.app.Activity;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.GeomagneticField;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.util.Log;
import android.view.WindowManager;

import com.google.android.gms.maps.model.LatLng;
import com.google.maps.android.SphericalUtil;

import java.util.Timer;
import java.util.TimerTask;

public class GetCompassDirActivity extends Activity implements SensorEventListener {
    public static SensorManager mSensorManager = null;
    public WindowManager mWindowManager = null;
    public PackageManager pm = null;
    public static double finalCompassDeg;

    // angular speeds from gyro
    private float[] gyro = new float[3];

    // rotation matrix from gyro data
    private float[] gyroMatrix = new float[9];

    // orientation angles from gyro matrix
    private float[] gyroOrientation = new float[3];

    // magnetic field vector
    private float[] magnet = new float[3];

    // accelerometer vector
    private float[] accel = new float[3];

    // orientation angles from accel and magnet
    private float[] accMagOrientation = new float[3];

    // final orientation angles from sensor fusion
    private float[] fusedOrientation = new float[3];

    // accelerometer and magnetometer based rotation matrix
    private float[] rotationMatrix = new float[9];

    // grav vector
    private float[] grav = new float[3];

    float[] rotMatrix = new float[9];
    float[] rotMatrixAdjusted = new float[9];
    float[] rotVals = new float[3];

    public static final float EPSILON = 0.000000001f;

    private static final float NS2S = 1.0f / 1000000000.0f;
    private float timestamp;
    private boolean initState = true;

    public static final int TIME_CONSTANT = 30;
    public static final float FILTER_COEFFICIENT = 0.98f;
    private Timer fuseTimer = new Timer();

    public Activity activity;
    //public TextView compass_text;
    private Location myPos;
    private LatLng dest;

    public GetCompassDirActivity(Activity _activity, Location _myPos, LatLng _dest){
        this.activity = _activity;
        //View compassView = View.inflate(this.activity.getApplicationContext(),R.layout.compass_view, null);
        //compass_text = compassView.findViewById(R.id.compass_text);
        myPos = _myPos;
        dest = _dest;
        pm = this.activity.getPackageManager();
        onBegin();
    }

    public void onBegin() {

        gyroOrientation[0] = 0.0f;
        gyroOrientation[1] = 0.0f;
        gyroOrientation[2] = 0.0f;

        // initialise gyroMatrix with identity matrix
        gyroMatrix[0] = 1.0f;
        gyroMatrix[1] = 0.0f;
        gyroMatrix[2] = 0.0f;
        gyroMatrix[3] = 0.0f;
        gyroMatrix[4] = 1.0f;
        gyroMatrix[5] = 0.0f;
        gyroMatrix[6] = 0.0f;
        gyroMatrix[7] = 0.0f;
        gyroMatrix[8] = 1.0f;

        // get sensorManager and initialise sensor listeners
        mSensorManager = (SensorManager) this.activity.getSystemService(SENSOR_SERVICE);
        mWindowManager = (WindowManager) this.activity.getSystemService(Context.WINDOW_SERVICE);
        initListeners();

        // wait for one second until gyroscope and magnetometer/accelerometer
        // data is initialised then schedule the complementary filter task
        fuseTimer.scheduleAtFixedRate(new calculateFusedOrientationTask(),
                1000, TIME_CONSTANT);

        Log.d("Sensors", "Created");

    }

    @Override
    public void onPause() {
        super.onPause();
        fuseTimer.cancel();
        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onStop(){
        super.onStop();
        fuseTimer.cancel();
        mSensorManager.unregisterListener(this);
    }

    private void initListeners() {
        mSensorManager.flush(this);
        if(mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) != null) {
            mSensorManager.registerListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                    SensorManager.SENSOR_DELAY_FASTEST);
            Log.d("Sensors", "Acc sensor OK");
        }
        else{
            Log.d("Sensors", "Acc sensor NOT FOUND!");
        }

        if (mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE) != null) {
            mSensorManager.registerListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                    SensorManager.SENSOR_DELAY_FASTEST);
            Log.d("Sensors", "Gyro sensor OK");
        }
        else{
            Log.d("Sensors", "Gyro sensor NOT FOUND!");
        }

        if (mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) != null) {
            mSensorManager.registerListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                    SensorManager.SENSOR_DELAY_FASTEST);
            Log.d("Sensors", "Magn sensor OK");
        }
        else{
            Log.d("Sensors", "Magn sensor NOT FOUND!");
        }

        if(mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) != null){
            mSensorManager.registerListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR),
                    SensorManager.SENSOR_DELAY_FASTEST);
            Log.d("Sensors", "RotVect sensor OK");
        }
        else
        {
            Log.d("Sensors", "RotVect sensor NOT FOUND!");
        }

        if(mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) != null){
            mSensorManager.registerListener(this,
                    mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY),
                    SensorManager.SENSOR_DELAY_NORMAL);
            Log.d("Sensors", "Grav sensor OK");
        }
        else
        {
            Log.d("Sensors", "Grav sensor NOT FOUND!");
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        switch(event.sensor.getType()) {
            case Sensor.TYPE_ACCELEROMETER:
                // copy new accelerometer data into accel array
                // then calculate new orientation
                System.arraycopy(event.values, 0, accel, 0, 3);
                calculateAccMagOrientation();
                break;

            case Sensor.TYPE_GYROSCOPE:
                // process gyro data
                gyroFunction(event);
                break;

            case Sensor.TYPE_GRAVITY:
                System.arraycopy(event.values, 0, grav, 0, 3);
                break;

            case Sensor.TYPE_ROTATION_VECTOR:
                SensorManager.getRotationMatrixFromVector(rotMatrix, event.values);
//                if (screenRotation == Surface.ROTATION_0) {
//                    worldAxisForDeviceAxisX = SensorManager.AXIS_X;
//                    worldAxisForDeviceAxisY = SensorManager.AXIS_Y;
//                } else if (screenRotation == Surface.ROTATION_90) {
//                    worldAxisForDeviceAxisX = SensorManager.AXIS_X;
//                    worldAxisForDeviceAxisY = SensorManager.AXIS_MINUS_Y;
//                } else if (screenRotation == Surface.ROTATION_180) {
//                    worldAxisForDeviceAxisX = SensorManager.AXIS_MINUS_X;
//                    worldAxisForDeviceAxisY = SensorManager.AXIS_MINUS_Y;
//                } else if (screenRotation == Surface.ROTATION_270) {
//                    worldAxisForDeviceAxisX = SensorManager.AXIS_MINUS_X;
//                    worldAxisForDeviceAxisY = SensorManager.AXIS_Y;
//                }
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                // copy new magnetometer data into magnet array
                System.arraycopy(event.values, 0, magnet, 0, 3);
                break;
        }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public void gyroFunction(SensorEvent event) {
        // don't start until first accelerometer/magnetometer orientation has been acquired
        if (accMagOrientation == null)
            return;

        // initialisation of the gyroscope based rotation matrix
        if(initState) {
            float[] initMatrix = new float[9];
            initMatrix = getRotationMatrixFromOrientation(accMagOrientation);
            float[] test = new float[3];
            SensorManager.getOrientation(initMatrix, test);
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix);
            initState = false;
        }

        // copy the new gyro values into the gyro array
        // convert the raw gyro data into a rotation vector
        float[] deltaVector = new float[4];
        if(timestamp != 0) {
            final float dT = (event.timestamp - timestamp) * NS2S;
            System.arraycopy(event.values, 0, gyro, 0, 3);
            getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f);
        }

        // measurement done, save current time for next interval
        timestamp = event.timestamp;

        // convert rotation vector into rotation matrix
        float[] deltaMatrix = new float[9];
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector);

        // apply the new rotation interval on the gyroscope based rotation matrix
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix);

        // get the gyroscope based orientation from the rotation matrix
        SensorManager.getOrientation(gyroMatrix, gyroOrientation);
    }

    private float[] getRotationMatrixFromOrientation(float[] o) { //Can be improved in terms of performance
        float[] xM = new float[9];
        float[] yM = new float[9];
        float[] zM = new float[9];

        float sinX = (float)Math.sin(o[1]);
        float cosX = (float)Math.cos(o[1]);
        float sinY = (float)Math.sin(o[2]);
        float cosY = (float)Math.cos(o[2]);
        float sinZ = (float)Math.sin(o[0]);
        float cosZ = (float)Math.cos(o[0]);

        // rotation about x-axis (pitch)
        xM[0] = 1.0f; xM[1] = 0.0f; xM[2] = 0.0f;
        xM[3] = 0.0f; xM[4] = cosX; xM[5] = sinX;
        xM[6] = 0.0f; xM[7] = -sinX; xM[8] = cosX;

        // rotation about y-axis (roll)
        yM[0] = cosY; yM[1] = 0.0f; yM[2] = sinY;
        yM[3] = 0.0f; yM[4] = 1.0f; yM[5] = 0.0f;
        yM[6] = -sinY; yM[7] = 0.0f; yM[8] = cosY;

        // rotation about z-axis (azimuth)
        zM[0] = cosZ; zM[1] = sinZ; zM[2] = 0.0f;
        zM[3] = -sinZ; zM[4] = cosZ; zM[5] = 0.0f;
        zM[6] = 0.0f; zM[7] = 0.0f; zM[8] = 1.0f;

        // rotation order is y, x, z (roll, pitch, azimuth)
        float[] resultMatrix = matrixMultiplication(xM, yM);
        resultMatrix = matrixMultiplication(zM, resultMatrix);
        return resultMatrix;
    }

    private float[] matrixMultiplication(float[] A, float[] B) {
        float[] result = new float[9];

        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6];
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7];
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8];

        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6];
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7];
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8];

        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6];
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7];
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8];

        return result;
    }

    private void getRotationVectorFromGyro(float[] gyroValues,
                                           float[] deltaRotationVector,
                                           float timeFactor)
    {
        float[] normValues = new float[3];

        // Calculate the angular speed of the sample
        float omegaMagnitude =
                (float)Math.sqrt(gyroValues[0] * gyroValues[0] +
                        gyroValues[1] * gyroValues[1] +
                        gyroValues[2] * gyroValues[2]);

        // Normalize the rotation vector if it's big enough to get the axis
        if(omegaMagnitude > EPSILON) {
            normValues[0] = gyroValues[0] / omegaMagnitude;
            normValues[1] = gyroValues[1] / omegaMagnitude;
            normValues[2] = gyroValues[2] / omegaMagnitude;
        }
        else{
            normValues[0] = gyroValues[0];
            normValues[1] = gyroValues[1];
            normValues[2] = gyroValues[2];
        }

        // Integrate around this axis with the angular speed by the timestep
        // in order to get a delta rotation from this sample over the timestep
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        float thetaOverTwo = omegaMagnitude * timeFactor;
        float sinThetaOverTwo = (float)Math.sin(thetaOverTwo);
        float cosThetaOverTwo = (float)Math.cos(thetaOverTwo);
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0];
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1];
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2];
        deltaRotationVector[3] = cosThetaOverTwo;
    }

    class calculateFusedOrientationTask extends TimerTask {
//        public TextView compass_text;
        public void run() {
            float oneMinusCoeff = 1.0f - FILTER_COEFFICIENT;
            fusedOrientation[0] =
                    FILTER_COEFFICIENT * gyroOrientation[0]
                            + oneMinusCoeff * accMagOrientation[0];

            fusedOrientation[1] =
                    FILTER_COEFFICIENT * gyroOrientation[1]
                            + oneMinusCoeff * accMagOrientation[1];

            fusedOrientation[2] =
                    FILTER_COEFFICIENT * gyroOrientation[2]
                            + oneMinusCoeff * accMagOrientation[2];

            // overwrite gyro matrix and orientation with fused orientation
            // to compensate gyro drift
            gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation);
            System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3);

            // By default, remap the axes as if the front of the
            // device screen was the instrument panel.

            int worldAxisForDeviceAxisX;
            int worldAxisForDeviceAxisY;

            // Adjust the rotation matrix for the device orientation
            //int screenRotation = mWindowManager.getDefaultDisplay().getRotation();

            if(( Math.abs(grav[2]) > Math.abs(grav[1]) ) &&
                    ( Math.abs(grav[2]) > Math.abs(grav[0]) )){
                worldAxisForDeviceAxisX = SensorManager.AXIS_X;
                worldAxisForDeviceAxisY = SensorManager.AXIS_Y;
            }
            else if(( Math.abs(grav[1]) > Math.abs(grav[0]) ) && ( Math.abs(grav[1]) > Math.abs(grav[2]) ) && grav[1] < 0){
                worldAxisForDeviceAxisX = SensorManager.AXIS_MINUS_Z;
                worldAxisForDeviceAxisY = SensorManager.AXIS_X;
            }
            else{
                worldAxisForDeviceAxisX = SensorManager.AXIS_X;
                worldAxisForDeviceAxisY = SensorManager.AXIS_Z;
            }

            SensorManager.remapCoordinateSystem(rotMatrix,  // gyroMatrix or rotMatrix
                    worldAxisForDeviceAxisX,
                    worldAxisForDeviceAxisY, rotMatrixAdjusted);

            // Transform rotation matrix into azimuth/pitch/roll
            SensorManager.getOrientation(rotMatrixAdjusted, rotVals);

            double bearing = calculateBearing(new LatLng(myPos.getLatitude(),myPos.getLongitude()),dest);
            Log.d("Sensors", "Bearing is: " + bearing);

            double azimuth = Math.toDegrees(rotVals[0]);

            GeomagneticField geoField = null;
            geoField = new GeomagneticField(
                    (float) myPos.getLatitude(),
                    (float) myPos.getLongitude(),
                    (float) myPos.getAltitude(),
                    System.currentTimeMillis()      );
            azimuth -= geoField.getDeclination(); // converts magnetic north into true north
            Log.d("Sensors", "Azimuth is: " + azimuth);

            if (azimuth < 0) {
                azimuth = azimuth + 360;
            }

            double compassDeg =  bearing - azimuth;

            // If the compassDeg is smaller than 0, add 360 to get the rotation clockwise.
            if (compassDeg < 0) {
                compassDeg = compassDeg + 360;
            }

            finalCompassDeg = Math.round(compassDeg);

//            Context c = App.getContext();
//            Intent myIntent = new Intent(activity,CompassArrowActivity.class);
//            activity.startActivity(myIntent);
//            myIntent.putExtra("compassDeg", finalCompassDeg);

//            runOnUiThread(new Runnable() {
//
//                @Override
//                public void run() {
//                    // get prompts.xml view
////                    LayoutInflater layoutInflater = LayoutInflater.from(App.getActivity());
////                    View view = layoutInflater.inflate(R.layout.input_dialog, null);
////                    compass_text = view.findViewById(R.id.compass_text);
//                    compass_text.setText(null);
//                    compass_text.setText(String.valueOf(finalCompassDeg));
//                    //Log.d("Sensors", "Compass Text Updated");
//                }
//            });

        }
    }

    private void calculateAccMagOrientation() {
        if(SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnet)) {
            SensorManager.getOrientation(rotationMatrix, accMagOrientation);
        }
    }

    public double calculateBearing(LatLng from, LatLng to){
            double bearing;

            bearing = SphericalUtil.computeHeading(from,to);
            if(bearing < 0){
                bearing = bearing + 360;
            }

            return bearing;
    }

}
