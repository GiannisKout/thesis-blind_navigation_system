package com.example.HapticCompass;

import android.content.Context;
import android.os.AsyncTask;
import android.os.Build;
import android.os.VibrationEffect;
import android.os.Vibrator;
import android.util.Log;

public class HapticsFeedbackTask extends AsyncTask<Void, Void, Void> {
    private int milliseconds;
    private long[] pattern = null;
    private int amplitude;

    public HapticsFeedbackTask(int _milliseconds, int _amplitude){
        this.milliseconds = _milliseconds;
        this.amplitude = _amplitude;
    }

    public HapticsFeedbackTask(long[] _pattern, int _amplitude){
        this.pattern = _pattern;
        this.amplitude = _amplitude;
    }

    @Override
    protected Void doInBackground(Void... voids) {
        if(isCancelled())
            return null;
        Vibrator vibrator = (Vibrator) App.getActivity().getSystemService(Context.VIBRATOR_SERVICE);
        if(vibrator.hasVibrator()){
            Log.d("Can Vibrate", "YES");
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O){
                if(pattern != null)
                    vibrator.vibrate(VibrationEffect.createWaveform(pattern, new int[]{amplitude}, -1));
                else
                    vibrator.vibrate(VibrationEffect.createOneShot(milliseconds, amplitude));
            }else{
                //deprecated since API 26
                if(pattern != null)
                    vibrator.vibrate(pattern, -1);
                else
                    vibrator.vibrate(this.milliseconds);
            }
        }
        else
            Log.d("Can Vibrate", "NO");
        return null;
    }

    @Override
    protected void onPostExecute(Void voids){
        CompassArrowActivity.isVib_Right = false;
        CompassArrowActivity.isVib_Left = false;
    }
}
