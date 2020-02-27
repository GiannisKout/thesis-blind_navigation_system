package com.example.HapticCompass;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.os.Looper;
import android.text.InputType;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
// Add an import statement for the client library.
import com.google.android.gms.common.api.ApiException;
import com.google.android.gms.location.LocationCallback;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationResult;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.google.android.libraries.places.api.Places;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.MapFragment;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.PointOfInterest;
import com.google.android.libraries.places.api.model.AutocompleteSessionToken;
import com.google.android.libraries.places.api.model.Place;
import com.google.android.libraries.places.api.net.FetchPlaceRequest;
import com.google.android.libraries.places.api.net.FindAutocompletePredictionsRequest;
import com.google.android.libraries.places.api.net.PlacesClient;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class MainActivity extends Activity implements OnMapReadyCallback {

    public static final int MY_PERMISSIONS_REQUEST_LOCATION = 99;
    public LatLng currentPos;
    public static LatLng temp_dest;
    public static Location currentPos_location;
    public LatLng dest = null;
    public static ArrayList<LatLng> points = new ArrayList<>();
    public static boolean middle_path_clear = true;

    private LatLngBounds bounds;
    private Button button;
    private Button get_Dir_button;
    public  Button get_Compass_View;
    private TextView resultText;
    //public TextView distance_text;
    private GoogleMap mMap;
    private String apiKey = "AIzaSyATrVQwe8wEcJV14Mq3Dv-eINfwDdcaa_w";
    /*private FetchPlaceRequest request;
    private static final String TAG = "MyActivity";*/
    private PlacesClient placesClient;
    private AutocompleteSessionToken token;
    private Polyline mPolyline;
    ArrayList<LatLng> markerPoints= new ArrayList<>();

    private GetCompassDirActivity CompassActivity = null;
    private static SensorManager sensorManager = null;
    FusedLocationProviderClient fusedLocationProviderClient = null;
    LocationRequest mLocationRequest = null;

    ///// TCP Communication ///////
    TCP_Client mTCP_Client = null;
    private int clickCounter = 0;

    ///////////////////////////////

    // Defines several constants used when transmitting messages between the
    // service and the UI.
    private interface MessageConstants {
        public static final int MESSAGE_READ = 0;
        public static final int MESSAGE_WRITE = 1;
        public static final int MESSAGE_TOAST = 2;

        // ... (Add other message types here as needed.)
    }


    @Override
    public void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        App.setContext(this);
        App.setActivity(this);
        setContentView(R.layout.activity_main);
        MapFragment mapFragment = (MapFragment) getFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        // Initialize Places.
        Places.initialize(getApplicationContext(), apiKey);
        // Create a new Places client instance.
        placesClient = Places.createClient(this);

        token = AutocompleteSessionToken.newInstance();
        //RectangularBounds bounds = RectangularBounds.newInstance(first, sec);

                // components from main.xml
        button = findViewById(R.id.dest_button);
        get_Dir_button = findViewById(R.id.dir_button);
        get_Compass_View = (Button) findViewById(R.id.bCompassview);
        resultText = findViewById(R.id.result);
        //distance_text = findViewById(R.id.distance);

        if(get_Compass_View == null)
            Toast.makeText(getApplicationContext(), "NULL Compass View", Toast.
                    LENGTH_LONG).show();
        get_Compass_View.setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View view){
                launchCompassActivity();
            }
        });

        get_Dir_button.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View view) {
                if(dest != null && currentPos != null){
                    // Getting URL to the Google Directions API
                    mMap.clear();
                    markerPoints.clear();
                    String url = getDirectionsUrl(currentPos, dest, apiKey);

                    DownloadTask downloadTask = new DownloadTask();

                    // Start downloading json data from Google Directions API
                    downloadTask.execute(url);
                }
                else{
                    Toast.makeText(getApplicationContext(), "Dest or Current Position not Ready", Toast.
                            LENGTH_LONG).show();
                }

            }
        });

        button.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View view) {
                showInputDialog();
            }
        });

        // Initialize the TCP communication //////////////////////////////////////////////////////////////

        final Button button_TCP = (Button) findViewById(R.id.button_tcp);

        button_TCP.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                clickCounter += 1;
                if (clickCounter == 1) {
                    //Log.d("Button", "Button clicked time: " + clickCounter);
                    new ConnectTask().execute();
                }
                else if (clickCounter % 2 == 0) {
                        if (mTCP_Client != null){
                            mTCP_Client.sendMessage("end");
                            //mTCP_Client = null;
                            //Log.d("TCP", "end message sent to server");
                        }
                }
                else{
                    new ConnectTask().execute();
                }
                //Log.d("Button", "Button clicked time: " + clickCounter);
            }
        });

        // END of TCP communication //////////////////////////////////////////////////////////////
    }

    ///////////////// TCP main implementation ///////////////////////////////////////////////////////////////

    public class ConnectTask extends AsyncTask<String, String, TCP_Client> {

        @Override
        protected TCP_Client doInBackground(String... message) {

            //we create a TCPClient object
            mTCP_Client = new TCP_Client(new TCP_Client.OnMessageReceived() {
                @Override
                //here the messageReceived method is implemented
                public void messageReceived(String message) {
                    //this method calls the onProgressUpdate
                    publishProgress(message);
                }
            });
            mTCP_Client.run();

            return null;
        }

        @Override
        protected void onProgressUpdate(String... values) {
            super.onProgressUpdate(values);
            long[] vib_pattern_red = {0, 600, 10};  // [initial delay, vibration time, pause time etc.] (in milliseconds)
            long[] vib_pattern_green = {0, 100, 10, 300, 10, 100, 10, 300};  // [initial delay, vibration time, pause time etc.] (in milliseconds)
            //long[] vib_pattern_unknown = {0, 600, 10};  // [initial delay, vibration time, pause time etc.] (in milliseconds)
            long[] vib_pattern_right = {0, 300, 50};  // [initial delay, vibration time, pause time etc.] (in milliseconds)
            long[] vib_pattern_left = {0, 100, 50, 100};
            // Define the Intensity level of Haptic Feedback
            int vibIntensity = 30; // value between 1-255

            //response received from server
            //Log.d("test", "response " + values[0]);
            Log.d("RESPONSE FROM SERVER", "Server talking: " + values[0]);

            if (values[0].equals("0")){ // when server sends signal for closing the socket
                Toast.makeText(App.getContext(), "Server: " + values[0],
                  Toast.LENGTH_SHORT).show();
                middle_path_clear = true;
                mTCP_Client.stopClient();
            }else if (values[0].equals("1")){ // when red light detected
                Toast.makeText(App.getContext(), "Red Light",
                        Toast.LENGTH_SHORT).show();
                middle_path_clear = false;
                mTCP_Client.sendMessage("Transmission OK!");
                //Log.d("Vibrate", "Vibrating...");
                HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(vib_pattern_red, vibIntensity);
                if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
                    hapticsFeedbackTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
                else
                    hapticsFeedbackTask.execute();

            }else if (values[0].equals("2")){ // when green light detected
                Toast.makeText(App.getContext(), "Green Light",
                        Toast.LENGTH_SHORT).show();
                mTCP_Client.sendMessage("Transmission OK!");
                middle_path_clear = false;
                HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(vib_pattern_green, vibIntensity);
                if(Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
                    hapticsFeedbackTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
                else
                    hapticsFeedbackTask.execute();
            }else if (values[0].equals("3")){ // when neither red nor green light detected
                Toast.makeText(App.getContext(), "Algorithm cannot decide..",
                        Toast.LENGTH_SHORT).show();
                middle_path_clear = false;
                mTCP_Client.sendMessage("Transmission OK!");
            }else if (values[0].equals("4")) { // when obstacle detected and needs to go LEFT
                Toast.makeText(App.getContext(), "obstacle ahead, GO LEFT",
                        Toast.LENGTH_SHORT).show();
                middle_path_clear = false;
                mTCP_Client.sendMessage("Transmission OK!");
                HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(vib_pattern_left, vibIntensity);
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
                    hapticsFeedbackTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
                else
                    hapticsFeedbackTask.execute();
            }else if (values[0].equals("5")) { // when obstacle detected and needs to go RIGHT
                Toast.makeText(App.getContext(), "obstacle ahead, GO RIGHT",
                        Toast.LENGTH_SHORT).show();
                mTCP_Client.sendMessage("Transmission OK!");
                middle_path_clear = false;
                HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(vib_pattern_right, vibIntensity);
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
                    hapticsFeedbackTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
                else
                    hapticsFeedbackTask.execute();
            }else if (values[0].equals("6")) { // when obstacle detected and cannot go ANYWHERE
                Toast.makeText(App.getContext(), "obstacle ahead, STUCK",
                        Toast.LENGTH_SHORT).show();
                mTCP_Client.sendMessage("Transmission OK!");
                middle_path_clear = false;
//                HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(vib_pattern_green, vibIntensity);
//                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
//                    hapticsFeedbackTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
//                else
//                    hapticsFeedbackTask.execute();
            }else if (values[0].equals("7")) { // when NO OBSTACLE is detected
                //Toast.makeText(App.getContext(), "Middle path clear!!!",
                    //Toast.LENGTH_SHORT).show();
                //mTCP_Client.sendMessage("Transmission OK!");
                middle_path_clear = true;
//                HapticsFeedbackTask hapticsFeedbackTask = new HapticsFeedbackTask(vib_pattern_green, vibIntensity);
//                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB)
//                    hapticsFeedbackTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
//                else
//                    hapticsFeedbackTask.execute();
            }

        }
    }

    // END //////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void onStop() {
        super.onStop();

    }

    @Override
    public void onDestroy() {
        super.onDestroy();

        // Unregister all sensor listeners in this callback so they don't
        // continue to use resources when the app is paused.
        if (sensorManager != null)
            sensorManager.unregisterListener(CompassActivity);

        mTCP_Client.sendMessage("end");
    }

    private void launchCompassActivity(){
        double compassView = 0;
        Intent intent = new Intent(this, CompassArrowActivity.class);
        if(CompassActivity != null)
            compassView = GetCompassDirActivity.finalCompassDeg;
        intent.putExtra("compassDeg", compassView);
        if(currentPos_location != null)
            intent.putExtra("currentPos", currentPos_location);
        if(temp_dest != null){
            intent.putExtra("tempDest", temp_dest);
            intent.putExtra("tempDest_lat", temp_dest.latitude);
            intent.putExtra("tempDest_long", temp_dest.longitude);
        }
        startActivity(intent);
    }

    @Override
    public void onMapReady(GoogleMap map) {
        mMap = map;
        enableMyLocation();
        mMap.setOnMyLocationButtonClickListener(onMyLocationButtonClickListener);
        LatLng home = new LatLng(38.288493, 21.788689);
        float zoom = 10; // City
        mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(home, zoom));
        Toast.makeText(getApplicationContext(), "Map Ready",
                Toast.LENGTH_LONG).show();
        setPoiClick(mMap);
        if(markerPoints != null){
            markerPoints.clear();
        }
        mMap.addMarker(new MarkerOptions()
                .position(home)
                .title("VVR Group"));

        mMap.setOnMapLongClickListener(new GoogleMap.OnMapLongClickListener() {
            @Override
            public void onMapLongClick(LatLng latLng) {
                dest = latLng;
                DecimalFormat df = new DecimalFormat("##.####");
                resultText.setText(df.format(dest.latitude));
                resultText.append(", ");
                resultText.append(df.format(dest.longitude));

                if(dest != null && currentPos != null){
                    // Getting URL to the Google Directions API
                    mMap.clear();
                    markerPoints.clear();
                    String url = getDirectionsUrl(currentPos, dest, apiKey);

                    DownloadTask downloadTask = new DownloadTask();

                    // Start downloading json data from Google Directions API
                    downloadTask.execute(url);
                }
                else{
                    Toast.makeText(getApplicationContext(), "Dest or Current Position not Reasy", Toast.
                            LENGTH_LONG).show();
                }

            }
        });

        mMap.setOnMapClickListener(new GoogleMap.OnMapClickListener() {
            @Override
            public void onMapClick(LatLng latLng) {

                if(markerPoints.size() == 1 && markerPoints.get(0) == currentPos){
                    Log.d("MarkerPoints", "MarkerPoints OK");
                }
                else if(markerPoints.size() > 1) {
                    markerPoints.clear();
                    mMap.clear();
                }

                // Adding new item to the ArrayList
                markerPoints.add(latLng);

                // Creating MarkerOptions
                MarkerOptions options = new MarkerOptions();

                // Setting the position of the marker
                options.position(latLng);

                if (markerPoints.size() == 1) {
                    options.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_BLUE));
                } else if (markerPoints.size() == 2) {
                    options.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_RED));
                }

                // Add new marker to the Google Map Android API V2
                mMap.addMarker(options);

                // Checks, whether start and end locations are captured
                if (markerPoints.size() >= 2) {
                    LatLng origin = (LatLng) markerPoints.get(0);
                    dest = (LatLng) markerPoints.get(1);

                    LatLngBounds.Builder new_bounds_builder = new LatLngBounds.Builder();
                    new_bounds_builder.include(origin);
                    new_bounds_builder.include(dest);
                    bounds = new_bounds_builder.build();
                    mMap.animateCamera(CameraUpdateFactory.newLatLngBounds(bounds, 100));

                    // Getting URL to the Google Directions API
                    String url = getDirectionsUrl(origin, dest, apiKey);

                    DownloadTask downloadTask = new DownloadTask();

                    // Start downloading json data from Google Directions API
                    downloadTask.execute(url);
                }

            }
        });
    }

    private void setPoiClick(final GoogleMap map) {
        map.setOnPoiClickListener(new GoogleMap.OnPoiClickListener() {
            @Override
            public void onPoiClick(PointOfInterest poi) {
                Marker poiMarker = map.addMarker(new MarkerOptions()
                        .position(poi.latLng)
                        .title(poi.name));
                poiMarker.showInfoWindow();
            }
        });
    }

    private void enableMyLocation() {
        if (ContextCompat.checkSelfPermission(this,
                Manifest.permission.ACCESS_FINE_LOCATION)
                == PackageManager.PERMISSION_GRANTED &&
                ContextCompat.checkSelfPermission(this,
                        android.Manifest.permission.ACCESS_COARSE_LOCATION)
                        == PackageManager.PERMISSION_GRANTED) {
            mMap.setMyLocationEnabled(true);
            mMap.getUiSettings().setMyLocationButtonEnabled(true);
        } else {
            ActivityCompat.requestPermissions(this, new String[]
                            {Manifest.permission.ACCESS_FINE_LOCATION,
                                    Manifest.permission.ACCESS_COARSE_LOCATION},
                    MY_PERMISSIONS_REQUEST_LOCATION);
        }
    }

    private GoogleMap.OnMyLocationButtonClickListener onMyLocationButtonClickListener =
            new GoogleMap.OnMyLocationButtonClickListener() {
                @Override
                public boolean onMyLocationButtonClick() {
                    Toast.makeText(getApplicationContext(), "MyLocation button clicked", Toast.LENGTH_SHORT).show();
                    // Return false so that we don't consume the event and the default behavior still occurs
                    // (the camera animates to the user's current position).
                    mMap.clear();
                    markerPoints.clear();
                    fusedLocationProviderClient =
                            LocationServices.getFusedLocationProviderClient(MainActivity.this);
                    if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED && checkSelfPermission(Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
                        ActivityCompat.requestPermissions(MainActivity.this, new String[]
                                        {Manifest.permission.ACCESS_FINE_LOCATION,
                                                Manifest.permission.ACCESS_COARSE_LOCATION},
                                MY_PERMISSIONS_REQUEST_LOCATION);
                    }
                    mLocationRequest = new LocationRequest();
                    mLocationRequest.setInterval(1000);
                    mLocationRequest.setFastestInterval(500);
                    mLocationRequest.setPriority(LocationRequest.PRIORITY_BALANCED_POWER_ACCURACY);
                    //Task task = fusedLocationProviderClient.getLastLocation();
                    fusedLocationProviderClient.requestLocationUpdates(mLocationRequest,
                            mLocationCallback,Looper.getMainLooper());
                    if(Looper.myLooper() == null)
                        Log.d("Loooper", "Looper is Null");
//                    task.addOnSuccessListener(new OnSuccessListener<Location>() {
//                        @Override
//                        public void onSuccess(Location location) {
//                            if(location!=null) {
//                                Log.d("AndroidClarified", location.getLatitude() + " " + location.getLongitude());
//                                currentPos_location = location;
//                                currentPos = new LatLng(location.getLatitude(),location.getLongitude());
//                                mMap.addMarker(new MarkerOptions()
//                                        .position(currentPos)
//                                        .title("My Position")).showInfoWindow();
//                            }
//                        }
//                    });

                    return false;
                }
            };

    LocationCallback mLocationCallback = new LocationCallback() {
        @Override
        public void onLocationResult(LocationResult locationResult) {
            List<Location> locationList = locationResult.getLocations();
            if (locationList.size() > 0) {
                //The last location in the list is the newest
                Location location = locationList.get(locationList.size() - 1);
                Log.i("MapsActivity", "Location: " + location.getLatitude() + " " + location.getLongitude());
                currentPos_location = location;
                currentPos = new LatLng(currentPos_location.getLatitude(), currentPos_location.getLongitude());
                if(markerPoints.size() < 2){
                    if(markerPoints.size() != 0 && markerPoints.get(0) != currentPos){
                        markerPoints.add(currentPos);
                        mMap.addMarker(new MarkerOptions()
                                .position(currentPos)
                                .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN))
                                .title("My Position")).showInfoWindow();
                    }
                    else if(markerPoints.size() == 0){
                        markerPoints.add(currentPos);
                        mMap.addMarker(new MarkerOptions()
                                .position(currentPos)
                                .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN))
                                .title("My Position")).showInfoWindow();
                    }
                }
            }
        }
    };

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                            String[] permissions,
                                            int[] grantResults) {
        // Check if location permissions are granted and if so enable the
        // location data layer.
        switch (requestCode) {
            case MY_PERMISSIONS_REQUEST_LOCATION:
                if (grantResults.length > 0
                        && grantResults[0]
                        == PackageManager.PERMISSION_GRANTED) {
                    enableMyLocation();
                    break;
                }
        }
    }

    protected void showInputDialog() {

        // get prompts.xml view
        LayoutInflater layoutInflater = LayoutInflater.from(MainActivity.this);
        View promptView = layoutInflater.inflate(R.layout.input_dialog, null);
        AlertDialog.Builder alertDialogBuilder = new AlertDialog.Builder(MainActivity.this);
        alertDialogBuilder.setView(promptView);
        alertDialogBuilder.setTitle("Destination Picker");


        // Set up the input
        final EditText input = promptView.findViewById(R.id.edittext);
        // Specify the type of input expected; this, for example, sets the input as a password, and will mask the text
        input.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_NUMBER_VARIATION_NORMAL);
        // setup a dialog window
        alertDialogBuilder.setCancelable(false)
                .setPositiveButton("OK", new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int id) {
                        //resultText.setText("Hello, " + input.getText().toString());
                        // Define a Place ID.
                        String placeId = input.getText().toString();
                        // Specify the fields to return.
                        List<Place.Field> placeFields = Arrays.asList(Place.Field.LAT_LNG);
                        FindAutocompletePredictionsRequest request =
                                FindAutocompletePredictionsRequest.builder()
                                        .setSessionToken(token)
                                        .setQuery(input.getText().toString())
                                        .build();

                        placesClient.findAutocompletePredictions(request)
                                .addOnSuccessListener(
                                        (response) -> {
                                            Log.i("generic", "number of results in search places response"
                                                    +response.getAutocompletePredictions().size());
                                            //StringBuilder sb = new StringBuilder();
                                            /*for (AutocompletePrediction prediction :
                                                    response.getAutocompletePredictions()) {
                                                sb.append(prediction.getPrimaryText(null).toString());
                                                sb.append("\n");
                                            }*/
                                            resultText.setText(response.getAutocompletePredictions().get(0).getPrimaryText(null).toString());
                                            resultText.append(", ");
                                            resultText.append(response.getAutocompletePredictions().get(0).getSecondaryText(null).toString());

                                            FetchPlaceRequest fetchPlaceRequest = FetchPlaceRequest.
                                                    newInstance(response.getAutocompletePredictions().get(0).getPlaceId(),
                                                            placeFields);
                                            placesClient.fetchPlace(fetchPlaceRequest).addOnSuccessListener((destination)-> {
                                                Place place = destination.getPlace();
                                                dest = place.getLatLng();
                                                //Log.i(TAG, "Place found: " + place.getName());
                                            }).addOnFailureListener((exception) -> {
                                                if (exception instanceof ApiException) {
                                                    ApiException apiException = (ApiException) exception;
                                                    int statusCode = apiException.getStatusCode();
                                                    // Handle error with given status code.
                                                    //Log.e(TAG, "Place not found: " + exception.getMessage());
                                                }
                                            });


                                        })
                                .addOnFailureListener((exception) -> {
                                    //exception.printStackTrace();
                                    resultText.setText("TRY AGAIN.. :(");
                                });
                    }
                })
                .setNegativeButton("Cancel",
                        new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int id) {
                                dialog.cancel();
                            }
                        });

        // create an alert dialog
        AlertDialog alert = alertDialogBuilder.create();
        alert.show();
    }

    private class DownloadTask extends AsyncTask<String, Void, String> {

        @Override
        protected String doInBackground(String... url) {
            String data = "";

            try {
                data = downloadUrl(url[0]);
            } catch (Exception e) {
                Log.d("Background Task", e.toString());
            }
            return data;
        }

        protected void onPostExecute(String result) {
            super.onPostExecute(result);

            ParserTask parserTask = new ParserTask();
            //parserTask.execute(result);
            parserTask.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR, result);

        }


    }

    private String getDirectionsUrl(LatLng origin, LatLng dest, String apiKey) {

        // Origin of route
        String str_origin = "origin=" + origin.latitude + "," + origin.longitude;

        // Destination of route
        String str_dest = "destination=" + dest.latitude + "," + dest.longitude;

        // Sensor enabled
        String sensor = "sensor=false";
        String mode = "mode=walking";

        // Building the parameters to the web service
        String parameters = str_origin + "&" + str_dest + "&" + sensor + "&" + mode
                + "&" + "key=" + apiKey;

        // Output format
        String output = "json";

        // Building the url to the web service
        String url = "https://maps.googleapis.com/maps/api/directions/" + output + "?" + parameters;
        Log.d("SOS!", url);
        Toast.makeText(getApplicationContext(), "GetDirections OK", Toast.
                LENGTH_LONG).show();

        return url;
    }

    private String downloadUrl(String strUrl) throws IOException {
        String data = "";
        InputStream iStream = null;
        HttpURLConnection urlConnection = null;
        try {
            URL url = new URL(strUrl);

            urlConnection = (HttpURLConnection) url.openConnection();

            urlConnection.connect();

            iStream = urlConnection.getInputStream();

            BufferedReader br = new BufferedReader(new InputStreamReader(iStream));

            StringBuffer sb = new StringBuffer();

            String line = "";
            while ((line = br.readLine()) != null) {
                sb.append(line);
            }

            data = sb.toString();

            br.close();

        } catch (Exception e) {
            Log.d("Exception", e.toString());
        } finally {
            iStream.close();
            urlConnection.disconnect();
        }
        return data;
    }

    private class ParserTask extends AsyncTask<String, Integer, List<List<HashMap<String,String>>>>  {

        // Parsing the data in non-ui thread
        @Override
        protected List<List<HashMap<String, String>>> doInBackground(String... jsonData) {

            JSONObject jObject;
            List<List<HashMap<String,String>>> routes = null;

            try {
                jObject = new JSONObject(jsonData[0]);
                DirectionsJSONParser parser = new DirectionsJSONParser();

                routes = parser.parse(jObject);
                bounds = parser.boundsParse(jObject);
            } catch (Exception e) {
                e.printStackTrace();
            }

            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(getApplicationContext(), "ParserTask in action...", Toast.
                            LENGTH_LONG).show();
                }
            });
            return routes;
        }

        @Override
        protected void onPostExecute(List<List<HashMap<String, String>>> result) {
            PolylineOptions lineOptions = null;
            MarkerOptions markerOptions = new MarkerOptions();

            Log.d("Points","Pending...");
            Toast.makeText(getApplicationContext(), "Points Pending...", Toast.
                    LENGTH_LONG).show();

            if(points != null)
                points.clear();
            else {
                Log.d("Points", "NO Points Found");
                Toast.makeText(getApplicationContext(), "NO Points Found", Toast.
                        LENGTH_LONG).show();
            }

            for (int i = 0; i < result.size(); i++) {

                lineOptions = new PolylineOptions();

                List<HashMap<String, String>> path = result.get(i);

                for (int j = 0; j < path.size(); j++) {
                    HashMap point = path.get(j);

                    double lat = Double.parseDouble(point.get("lat").toString());
                    double lng = Double.parseDouble(point.get("lng").toString());
                    LatLng position = new LatLng(lat, lng);

                    points.add(position);
                }
                if(points != null){
                    lineOptions.addAll(points);
                    lineOptions.width(12);
                    lineOptions.color(Color.RED);
                    lineOptions.geodesic(true);
                    Toast.makeText(getApplicationContext(), "points are OK", Toast.
                            LENGTH_LONG).show();
                }
                else
                    Toast.makeText(getApplicationContext(), "points are null", Toast.
                            LENGTH_LONG).show();

            }

            // Drawing polyline in the Google Map for the i-th route
            if(lineOptions != null) {
                if (mPolyline != null) {
                    mPolyline.remove();
                }
                if (markerPoints != null) {
                    if (markerPoints.size() < 2) {
                        markerPoints.clear();
                        markerPoints.add(currentPos);
                        markerPoints.add(dest);
                    }
                }
                mPolyline = mMap.addPolyline(lineOptions);

                /**
                 * For the start location, the color of marker is GREEN and
                 * for the end location, the color of marker is RED.
                 */
                markerOptions.position(markerPoints.get(0))
                        .title("Origin")
                        .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN));
                mMap.addMarker(markerOptions);
                markerOptions.position(markerPoints.get(1))
                        .title("Destination")
                        .icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_RED));
                mMap.addMarker(markerOptions);
                if (bounds != null)
                    mMap.animateCamera(CameraUpdateFactory.newLatLngBounds(bounds, 100));
                bounds = null;

                temp_dest = points.get(0);
                Log.d("MyPosInit",temp_dest.toString());
//                PolyPointListener polyPointListener = new PolyPointListener() {
//                    @Override
//                    public void onPolyPointReached(ArrayList<LatLng> _points) {
//                        if (_points.size() == 0)
//                            return;
//                        else {
//                            temp_dest = _points.get(0);
//                            Log.d("MyPos", temp_dest.toString());
//                            _points.remove(0);
//
//                            if (sensorManager != null) {
//                                sensorManager.unregisterListener(CompassActivity);
//                                //CompassActivity.onStop();
//                            }
//                            GetCompassDirActivity CompassActivity = new GetCompassDirActivity(MainActivity.this,
//                                    currentPos_location, temp_dest);
//                            sensorManager = CompassActivity.mSensorManager;
//                            PolyPointParser polyPointParser = new PolyPointParser(this::onPolyPointReached);
//                            polyPointParser.execute(_points);
//                        }
//                    }
//                };
//
//                if (sensorManager != null){
//                    sensorManager.unregisterListener(CompassActivity);
//                    CompassActivity.onStop();
//                }
//                GetCompassDirActivity CompassActivity = new GetCompassDirActivity( MainActivity.this,
//                        currentPos_location, temp_dest);
//                sensorManager = CompassActivity.mSensorManager;
//                PolyPointParser polyPointParser = new PolyPointParser(polyPointListener);
//                polyPointParser.execute(points);

            }else
                Toast.makeText(getApplicationContext(),"No route is found", Toast.LENGTH_LONG).show();
        }

    }

//    // Step 1 - This interface defines the type of messages I want to communicate to my owner
//    public interface PolyPointListener {
//        // These methods are the different events and
//        // need to pass relevant arguments related to the event triggered
//        void onPolyPointReached(ArrayList<LatLng> restOfPoint);
//    }
//
//    public class PolyPointParser extends AsyncTask< ArrayList<LatLng>, Double, ArrayList<LatLng>>{
//        private PolyPointListener polyPointListener;
//        //private LatLng myPos = MainActivity.this.currentPos;
//
//        private PolyPointParser(PolyPointListener polyPointListener){
//            this.polyPointListener = polyPointListener;
//        }
//
//        @Override
//        protected void onPostExecute(ArrayList<LatLng> restOfPoint){
//            // your stuff
//            polyPointListener.onPolyPointReached(restOfPoint);
//        }
//
//        @Override
//        protected void onProgressUpdate(Double... _distance) {
//            super.onProgressUpdate(_distance);
//            DecimalFormat df = new DecimalFormat("##.###");
//            distance_text.setText(df.format(_distance[0]));
//        }
//
//        @Override
//        protected ArrayList<LatLng> doInBackground(ArrayList<LatLng>... _points) {
//            ArrayList<LatLng> listOfPoints = _points[0];
//            LatLng nextPoint = listOfPoints.get(0);
//            Double distance = SphericalUtil.computeDistanceBetween(currentPos,nextPoint);
//
//            int counter = 0;
//            while(distance > 2){
//                distance = SphericalUtil.computeDistanceBetween(currentPos,nextPoint);
//                Log.d("Distance is: ", Double.toString(distance));
//                if(counter % 50 == 0)
//                    publishProgress(distance);
//                counter += 1;
//                if(counter > 100)
//                    counter = 1;
//            }
//
//            return listOfPoints;
//        }
//    }

}