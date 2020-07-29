package com.example.boostboxapp;

import androidx.appcompat.app.AppCompatActivity;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.SimpleExpandableListAdapter;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.UUID;

public class DashboardActivity extends AppCompatActivity {

    private final static String TAG = DashboardActivity.class.getSimpleName();

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothGatt bluetoothGatt;

    private BLEService mBluetoothLeService;
    private BluetoothGattService arduinoService;
    private BluetoothGattCharacteristic upDownErrorChar;
    private BluetoothGattCharacteristic leftRightErrorChar;
    private BluetoothGattCharacteristic setAngleLRChar;
    private BluetoothGattCharacteristic setAngleUDChar;

    private int angleToSend;

    private boolean mConnected = false;

    // Code to manage Service lifecycle.
    private final ServiceConnection mServiceConnection = new ServiceConnection() {

        @Override
        public void onServiceConnected(ComponentName componentName, IBinder service) {
            Log.i(TAG, "Service connection 1...");
            mBluetoothLeService = ((BLEService.LocalBinder) service).getService();
            Log.i(TAG, "Service connection...");
            if (!mBluetoothLeService.initialize()) {
                Log.e(TAG, "Unable to initialize Bluetooth");
                finish();
            }
            // Automatically connects to the device upon successful start-up initialization.
            mBluetoothLeService.connect(GattConfigs.ARDUINO_ADDRESS);
        }

        @Override
        public void onServiceDisconnected(ComponentName componentName) {
            Log.i(TAG, "We are here");
            mBluetoothLeService = null;
        }
    };

    // Handles various events fired by the Service.
    // ACTION_GATT_CONNECTED: connected to a GATT server.
    // ACTION_GATT_DISCONNECTED: disconnected from a GATT server.
    // ACTION_GATT_SERVICES_DISCOVERED: discovered GATT services.
    // ACTION_DATA_AVAILABLE: received data from the device.  This can be a result of read
    //                        or notification operations.
    private final BroadcastReceiver mGattUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            Log.i(TAG, "We received a broadcast.");
            if (BLEService.ACTION_GATT_CONNECTED.equals(action)) {
                mConnected = true;
                Log.i(TAG, "Connected");
            } else if (BLEService.ACTION_GATT_DISCONNECTED.equals(action)) {
                mConnected = false;
                Log.i(TAG, "Disconnected");
            } else if (BLEService.ACTION_GATT_SERVICES_DISCOVERED.equals(action)) {
                // Show all the supported services and characteristics on the user interface.
                Log.i(TAG, "There are services.");
                displayGattServices(mBluetoothLeService.getSupportedGattServices());
            } else if (BLEService.ACTION_DATA_AVAILABLE.equals(action)) {
                Log.i(TAG, "There is data");
                //Log.i(TAG, "The data is: " + intent.getStringExtra(BLEService.EXTRA_DATA));
                displayDataOnScreen(intent.getStringExtra(BLEService.EXTRA_UUID), intent.getStringExtra(BLEService.EXTRA_DATA));
            }
        }
    };

    // Demonstrates how to iterate through the supported GATT Services/Characteristics.
    // In this sample, we populate the data structure that is bound to the ExpandableListView
    // on the UI.
    private void displayGattServices(List<BluetoothGattService> gattServices) {
        Log.i(TAG, "Gatt services will be displayed");
        if (gattServices == null) return;
        for (BluetoothGattService gattService : gattServices) {
            if (gattService.getUuid().toString().equals(GattConfigs.ARDUINO_SERVICE_UUID)) {
                // The arduino is publishing only one service
                arduinoService = gattService;
            }
            Log.i(TAG, "The UUID of this service is " + gattService.getUuid().toString());
        }
        if (arduinoService == null) {
            Log.i(TAG, "Arduino service was not found");
            return;
        }

        // Get all characteristics of the Arduino service and store it in the respective attribute:
        List<BluetoothGattCharacteristic> gattCharacteristics = arduinoService.getCharacteristics();
        for (BluetoothGattCharacteristic gattCharacteristic : gattCharacteristics) {
            Log.i(TAG, "The UUID of this characteristic is "
                    + gattCharacteristic.getUuid().toString());
            if (gattCharacteristic.getUuid().toString().equals(GattConfigs.UPDOWNERROR_CHAR_UUID)) {
                upDownErrorChar = gattCharacteristic;
            } else if (gattCharacteristic.getUuid().toString().equals(GattConfigs.LEFTRIGHTERROR_CHAR_UUID)) {
                leftRightErrorChar = gattCharacteristic;
            } else if (gattCharacteristic.getUuid().toString().equals(GattConfigs.LRANGLESET_CHAR_UUID)) {
                setAngleLRChar = gattCharacteristic;
            } else if (gattCharacteristic.getUuid().toString().equals(GattConfigs.UDANGLESET_CHAR_UUID)) {
                setAngleUDChar = gattCharacteristic;
            }
        }
        if (upDownErrorChar == null) {
            Log.i(TAG, "UpDownError Char was not found");
            return;
        }
        Log.i(TAG, "The current characteristic has UUID: " + upDownErrorChar.getUuid().toString());
        //mBluetoothLeService.setCharacteristicNotification(upDownErrorChar, true);

        mBluetoothLeService.readCharacteristic(upDownErrorChar);
        //mBluetoothLeService.setCharacteristicNotification(upDownErrorChar, true);
    }

    public void displayDataOnScreen(String uuid, String dataToDisplay) {
        double data = Double.parseDouble(dataToDisplay);
        long angle = Math.round(Math.toDegrees(Math.asin(data)));
        if (uuid.equals(GattConfigs.UPDOWNERROR_CHAR_UUID)) {
            TextView udAngleDisplay = (TextView)findViewById(R.id.udErrorDisplay);
            SeekBar lrSeekBar = (SeekBar)findViewById(R.id.seekBarLR);
            udAngleDisplay.setText("" + angle);
            lrSeekBar.setProgress((int)angle + 90);
        } else if (uuid.equals(GattConfigs.LEFTRIGHTERROR_CHAR_UUID)) {
            TextView lrAngleDisplay = (TextView)findViewById(R.id.lrErrorDisplay);
            VerticalSeekBar udSeekBar = (VerticalSeekBar)findViewById(R.id.seekBarUD);
            udSeekBar.setProgress((int)angle + 90);
            lrAngleDisplay.setText("" + angle);
        }

    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_dashboard);

        final Intent dashboardIntent = getIntent();
        angleToSend = 0;

        Intent gattServiceIntent = new Intent(this, BLEService.class);
        bindService(gattServiceIntent, mServiceConnection, Context.BIND_AUTO_CREATE);
        Log.i(TAG, "Service bound");
        Log.i(TAG, GattConfigs.ARDUINO_ADDRESS);

        Button readDataButton = (Button)findViewById(R.id.readDataButton);
        readDataButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (mBluetoothLeService != null && upDownErrorChar != null) {
                    mBluetoothLeService.readCharacteristic(upDownErrorChar);
                } else {
                    Log.w(TAG, "Cannot read, no BLE connection or char not available!");
                }
            }
        });
        Button setAngleButton = (Button)findViewById(R.id.setAngleButton);
        final TextView lrAngleText = (TextView)findViewById(R.id.lrAngleSetter);
        final TextView udAngleText = (TextView)findViewById(R.id.udAngleSetter);
        setAngleButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.i(TAG, "We have clicked");
                if (mBluetoothLeService == null || setAngleLRChar == null || setAngleUDChar == null) {
                    Log.w(TAG, "No BLE connection or char not available!");
                    return;
                }
                mBluetoothLeService.queueUpWrite(setAngleLRChar, Integer.parseInt(lrAngleText.getText().toString()));
                mBluetoothLeService.queueUpWrite(setAngleUDChar, Integer.parseInt(udAngleText.getText().toString()));
            }
        });

        Button disconnectButton = (Button)findViewById(R.id.disconnectButton);
        disconnectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if (mBluetoothLeService != null) {
                    mBluetoothLeService.disconnect();
                } else {
                    Log.w(TAG, "Cannot disconnect!");
                }
            }
        });

        //mBluetoothLeService.connect(GattConfigs.ARDUINO_ADDRESS);
    }


    @Override
    protected void onResume() {
        super.onResume();
        registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        if (mBluetoothLeService != null) {
            final boolean result = mBluetoothLeService.connect(GattConfigs.ARDUINO_ADDRESS);
            Log.d(TAG, "Connect request result=" + result);
        }
    }

    @Override
    protected void onPause() {
        super.onPause();
        unregisterReceiver(mGattUpdateReceiver);
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        unbindService(mServiceConnection);
        mBluetoothLeService = null;
    }


    private static IntentFilter makeGattUpdateIntentFilter() {
        final IntentFilter intentFilter = new IntentFilter();
        intentFilter.addAction(BLEService.ACTION_GATT_CONNECTED);
        intentFilter.addAction(BLEService.ACTION_GATT_DISCONNECTED);
        intentFilter.addAction(BLEService.ACTION_GATT_SERVICES_DISCOVERED);
        intentFilter.addAction(BLEService.ACTION_DATA_AVAILABLE);
        return intentFilter;
    }
}