package com.example.HapticCompass;

import android.app.Activity;
import android.content.Intent;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.RotateAnimation;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import com.google.android.gms.maps.model.LatLng;

import java.text.DecimalFormat;
import java.util.ArrayList;

import static com.example.HapticCompass.MainActivity.currentPos_location;
import static com.example.HapticCompass.MainActivity.temp_dest;
import static com.example.HapticCompass.MainActivity.points;

public class CompassArrowActivity extends AppCompatActivity {
    private TextView compass_text;
    private TextView distance_text;
    private Button mBtGoBack;
    public double compassDeg;
    public ImageView compassArrow;
    public LatLng tempDest = temp_dest;
    private LatLng currentPos;
    private SensorManager sensorManager = null;
    private GetCompassDirActivity CompassActivity = null;
    // record the compass picture angle turned
    private float currentDegree = 0f;

    public boolean dead = false;
    public static boolean isVib_Right = false;
    public static boolean isVib_Left = false;

    private void goBackToMainAct(){
        Intent myIntent = new Intent(this, MainActivity.class);
        startActivity(myIntent);
    }

    @Override
    public void onPause(){
        super.onPause();
    }

    @Override
    public void onResume() {
        super.onResume();
    }

    @Override
    public void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);
//        LayoutInflater inflater = LayoutInflater.from(App.getContext());
//        View view = inflater.inflate(R.layout.compass_view, null);
        setContentView(R.layout.compass_view);
        mBtGoBack = (Button) findViewById(R.id.back);

        mBtGoBack.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //dead = true;
                //finish();
                goBackToMainAct();
            }
        });

        Bundle extras = getIntent().getExtras();
        if(extras != null){
            compassDeg = extras.getDouble("compassDeg");
            //currentPos_location = (Location) getIntent().getSerializableExtra("currentPos");
            currentPos = new LatLng(currentPos_location.getLatitude(), currentPos_location.getLongitude());
        }

        PolyPointListener polyPointListener = new PolyPointListener() {
            @Override
            public void onPolyPointReached(ArrayList<LatLng> _points) {
                if (_points.size() == 0)
                    return;
                else {
                    tempDest = _points.get(0);
                    Log.d("MyPos", tempDest.toString());
                    Toast.makeText(getApplicationContext(), "Inside onPolyPoint Callback",
                            Toast.LENGTH_LONG).show();

                    if (sensorManager != null) {
                        sensorManager.unregisterListener(CompassActivity);
                        sensorManager.flush(CompassActivity);
                        //CompassActivity.onStop();
                    }
                    GetCompassDirActivity CompassActivity = new GetCompassDirActivity(getParent(),
                            currentPos_location, tempDest);
                    sensorManager = GetCompassDirActivity.mSensorManager;
                    PolyPointParser polyPointParser = new PolyPointParser(getParent(),this::onPolyPointReached);
                    if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
                        polyPointParser.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, _points);
                    else
                        polyPointParser.execute(_points);
//                    polyPointParser.execute(_points);
                }
            }
        };

        if (sensorManager != null){
            sensorManager.unregisterListener(CompassActivity);
            CompassActivity.onStop();
        }
        GetCompassDirActivity CompassActivity = new GetCompassDirActivity( this,
                currentPos_location, tempDest);
        sensorManager = GetCompassDirActivity.mSensorManager;
        PolyPointParser polyPointParser = new PolyPointParser(this,polyPointListener);
        if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
            polyPointParser.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, points);
        else
            polyPointParser.execute(points);
//        polyPointParser.execute(points);

        compass_text = findViewById(R.id.compass_text);
        distance_text = findViewById(R.id.distance);
        compassArrow = findViewById(R.id.imageViewCompass);

        compassDeg = GetCompassDirActivity.finalCompassDeg;
        Log.d("CompassAct", "Compass Act. running");
        //compass_text.setText(String.valueOf(compassDeg));

//        runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                compass_text.setText(String.valueOf(compassDeg));
//            }
//        });

    }

    @Override
    public void onStop() {
        dead = true;
        super.onStop();
    }

    // Step 1 - This interface defines the type of messages I want to communicate to my owner
    public interface PolyPointListener {
        // These methods are the different events and
        // need to pass relevant arguments related to the event triggered
        void onPolyPointReached(ArrayList<LatLng> restOfPoint);
    }

    public class PolyPointParser extends AsyncTask< ArrayList<LatLng>, Double, ArrayList<LatLng>> {
        private PolyPointListener polyPointListener;
        private Activity mActivity;
        private int vib_counter = 0;
        private static final int VIB_COUNTER_LIMIT = 250;

        // Define the Haptic Icon patterns for right-wise and left-wise direction feedback
//        long[] vib_pattern_right = {0, 100, 200, 200};  // [initial delay, vibration time, pause time etc.] (in milliseconds)
//        long[] vib_pattern_left = {0, 100, 100, 100, 100, 100};
        long[] vib_pattern_right = {0, 300, 50};  // [initial delay, vibration time, pause time etc.] (in milliseconds)
        long[] vib_pattern_left = {0, 100, 50, 100};

        // Define the Intensity level of Haptic Feedback
        int vibIntensity = 30; // value between 1-255

//        private List<Double> update_elements = new ArrayList<Double>();
//        private LatLng myPos = MainActivity.this.currentPos;

        private PolyPointParser(Activity _activity, PolyPointListener _polyPointListener){
            this.polyPointListener = _polyPointListener;
            mActivity = _activity;
        }

        @Override
        protected void onPostExecute(ArrayList<LatLng> restOfPoint){
            final Activity activity = mActivity;
            if (activity != null) {
                //polyPointListener.onPolyPointReached(restOfPoint);
                Log.d("PolyParser", "PolyPointParser execution finished successfully");
            }
        }

        @Override
        protected void onProgressUpdate(Double... _update_elements) {
            super.onProgressUpdate(_update_elements);
            double compassDeg_local = _update_elements[0];
            double distance = _update_elements[1];


            final Activity activity = mActivity;
            if (activity != null) {
                compass_text = activity.findViewById(R.id.compass_text);
                distance_text = activity.findViewById(R.id.distance);

                compass_text.setText(null);
                compass_text.setText(compassDeg_local + "°");
                DecimalFormat df = new DecimalFormat("##.###");
                distance_text.setText(null);
                distance_text.setText(df.format(distance) + "m");

                RotateAnimation rotateAnimation = new RotateAnimation(currentDegree, (float) compassDeg_local,
                        Animation.RELATIVE_TO_SELF,0.5f,
                        Animation.RELATIVE_TO_SELF,0.5f);
                rotateAnimation.setDuration(210);
                rotateAnimation.setFillAfter(true);
                ImageView imageView = activity.findViewById(R.id.imageViewCompass);
                imageView.startAnimation(rotateAnimation);
                currentDegree = (float) compassDeg_local;

                // Vibration action
                if(vib_counter == 0){

                    if((compassDeg_local > 30.0) && (compassDeg_local <= 180.0)){
                        Log.d("Vibration", "Right is ON");
                        if(!CompassArrowActivity.isVib_Right && !CompassArrowActivity.isVib_Left && MainActivity.middle_path_clear){
                            CompassArrowActivity.isVib_Right = true;
                            int duration = 400; // in milliseconds
//                            HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(duration);
                            HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(vib_pattern_right, vibIntensity);
                            if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
                                hapticsFeedbackTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
                            else
                                hapticsFeedbackTask.execute();
                            Log.d("Vibration", "Vibrating right...");
                        }
                    }
                    else if((compassDeg_local < (360.0 - 30)) && (compassDeg_local > 180.0)){
                        Log.d("Vibration", "Left is ON");
                        if(!CompassArrowActivity.isVib_Right && !CompassArrowActivity.isVib_Left && MainActivity.middle_path_clear){
                            CompassArrowActivity.isVib_Left = true;
                            int duration = 400; // in milliseconds
//                            HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(duration);
                            HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(vib_pattern_left, vibIntensity);
                            if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
                                hapticsFeedbackTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
                            else
                                hapticsFeedbackTask.execute();
                            Log.d("Vibration", "Vibrating left...");
                        }
                    }
                    else
                        Log.d("Vibration", "Vibration is OFF");
                }

                vib_counter += 1;
                if(vib_counter > VIB_COUNTER_LIMIT)
                    vib_counter = 0;

            }
        }

        @Override
        protected ArrayList<LatLng> doInBackground(ArrayList<LatLng>... _points) {
            ArrayList<LatLng> listOfPoints = _points[0];

            while(listOfPoints.size() > 0){
                LatLng nextPoint = listOfPoints.get(0);
                listOfPoints.remove(0);
//            List<Double> update_elements = new ArrayList<Double>();
                float[] distance_array = new float[1];
                currentPos = new LatLng(currentPos_location.getLatitude(), currentPos_location.getLongitude());
//            double distance = SphericalUtil.computeDistanceBetween(currentPos,nextPoint);
                Location.distanceBetween(currentPos.latitude,currentPos.longitude,
                        nextPoint.latitude, nextPoint.longitude, distance_array);

                int counter = 0;
                while(distance_array[0] > 2){

                    if(isCancelled())
                        break;
                    if(dead)
                        this.cancel(true);
                    currentPos = new LatLng(currentPos_location.getLatitude(), currentPos_location.getLongitude());
                    Location.distanceBetween(currentPos.latitude,currentPos.longitude,
                            nextPoint.latitude, nextPoint.longitude, distance_array);
//                //distance = SphericalUtil.computeDistanceBetween(currentPos,nextPoint);
                    Log.d("distance", "Distance is: " + distance_array[0]);
                    Log.d("current location","Current Location(Lat/Lon) is: "
                            + currentPos.latitude + " / " + currentPos.longitude);
                    if(counter % 50 == 0){
                        publishProgress(GetCompassDirActivity.finalCompassDeg,(double)distance_array[0]);
                    }
                    counter += 1;
                    if(counter > 100)
                        counter = 1;
                }
            }

            return listOfPoints;
        }
    }
}