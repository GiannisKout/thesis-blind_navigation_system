package com.example.HapticCompass;

import android.app.Activity;
import android.app.Application;
import android.content.Context;

public class App extends Application {

    private static Context mContext;
    private static Activity mActivity;

    public static Context getContext() {
        return mContext;
    }

    public static void setContext(Context mContext) {
        App.mContext = mContext;
    }

    public static Activity getActivity() {
        return mActivity;
    }

    public static void setActivity(Activity mActivity) {
        App.mActivity = mActivity;
    }

}
