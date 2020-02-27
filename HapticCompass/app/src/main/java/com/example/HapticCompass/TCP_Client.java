package com.example.HapticCompass;
import android.os.AsyncTask;
import android.os.Build;
import android.util.Log;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;

public class TCP_Client {

    public static final String TAG = TCP_Client.class.getSimpleName();
    public static final String SERVER_IP = "192.168.43.67"; //server IP address
    public static final int SERVER_PORT = 63123;
    // message to send to the server
    private String mServerMessage = null;
    // sends message received notifications


    private OnMessageReceived mMessageListener = null;
    // while this is true, the server will continue running
    public boolean mRun = false;
    // used to send messages
    private PrintWriter mBufferOut = null;
    // used to read messages from the server
    private BufferedReader mBufferIn = null;

    /**
     * Constructor of the class. OnMessagedReceived listens for the messages received from server
     */
    public TCP_Client(OnMessageReceived listener) {
        mMessageListener = listener;
    }

    /**
     * Sends the message entered by client to the server
     *
     * @param message text entered by client
     */
    public void sendMessage(final String message) {
        Runnable runnable = new Runnable() {
            @Override
            public void run() {
                if (mBufferOut != null) {
                    mBufferOut.flush();
                    Log.d(TAG, "Sending: " + message);
                    mBufferOut.println(message);

                    if (message.equals("end")){
                        stopClient();
                    }
                    //Toast.makeText(App.getContext(), "Sending to server:" + message,
                      //      Toast.LENGTH_SHORT).show();
                }
            }
        };
        Thread thread = new Thread(runnable);
        thread.start();
    }

    /**
     * Close the connection and release the members
     */
    public void stopClient() {

        mRun = false;

        if (mBufferOut != null) {
            mBufferOut.flush();
            mBufferOut.close();
        }

        mMessageListener = null;
        mBufferIn = null;
        mBufferOut = null;
        mServerMessage = null;
    }

    public void run() {

        mRun = true;

        try {
            //here you must put your computer's IP address.
            InetAddress serverAddr = InetAddress.getByName(SERVER_IP);

            Log.d("TCP Client", "C: Connecting...");

            //create a socket to make the connection with the server
            Socket socket = new Socket(serverAddr, SERVER_PORT);

            try {

                //buffer used for sending the message to the server
                mBufferOut = new PrintWriter(new BufferedWriter(new OutputStreamWriter(socket.getOutputStream())), true);

                //buffer used for receiving the message which the server sends back
                mBufferIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));

                //in this while the client listens for the messages sent by the server
                while (mRun) {

                    mServerMessage = mBufferIn.readLine();

                    if (mServerMessage != null && mMessageListener != null) {
                        //call the method messageReceived from MyActivity class

                        if (mServerMessage.length() > 1)
                        {
                            mServerMessage = mServerMessage.substring(mServerMessage.length() - 1);
                        }
                        else
                        {
                            continue;
                        }

                        mMessageListener.messageReceived(mServerMessage);

                        //Log.d("RESPONSE FROM SERVER", "Server talking: " + mServerMessage + "'");
                    }

                }

            } catch (Exception e) {
                Log.e("TCP", "S: Error", e);
            } finally {
                //the socket must be closed. It is not possible to reconnect to this socket
                // after it is closed, which means a new socket instance has to be created.
                socket.close();
            }

        } catch (Exception e) {
            Log.e("TCP", "C: Error", e);
        }

    }

    //Declare the interface. The method messageReceived(String message) will must be implemented in the Activity
    //class at on AsyncTask doInBackground
    public interface OnMessageReceived {
        public void messageReceived(String message);
    }
}
