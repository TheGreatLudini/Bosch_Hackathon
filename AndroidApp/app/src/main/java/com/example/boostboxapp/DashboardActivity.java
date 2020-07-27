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
import android.widget.SimpleExpandableListAdapter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.UUID;

public class DashboardActivity extends AppCompatActivity {

    private final static String TAG = DashboardActivity.class.getSimpleName();
    private static final String ARDUINO_SERVICE_UUID = "aa461740-dc53-4624-97bd-0fee7b1212bb";
    private static final String UPDOWNERROR_UUID = "a71f3fc6-f97c-4659-9ca2-76a67dede2e3";
    private static final String UPDOWNERROR_DESCRIPTOR_UUID = "00002902-0000-1000-8000-00805f9b34fb";

    public static final String EXTRAS_ARDUINO_NAME = "ARDUINO_NAME";
    public static final String EXTRAS_ARDUINO_ADDRESS = "ARDUINO_ADDRESS";

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothGatt bluetoothGatt;

    private BLEService mBluetoothLeService;
    private BluetoothGattService arduinoService;
    private BluetoothGattCharacteristic upDownErrorChar;

    private String arduinoName;
    private String arduinoAddress;

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
            mBluetoothLeService.connect(arduinoAddress);
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
                Log.i(TAG, "The data is: " + intent.getStringExtra(BLEService.EXTRA_DATA));

            }
        }
    };

    // Demonstrates how to iterate through the supported GATT Services/Characteristics.
    // In this sample, we populate the data structure that is bound to the ExpandableListView
    // on the UI.
    private void displayGattServices(List<BluetoothGattService> gattServices) {
        if (gattServices == null) return;
        for (BluetoothGattService gattService : gattServices) {
            if (gattService.getUuid().toString().equals(ARDUINO_SERVICE_UUID)) {
                arduinoService = gattService;
            }
            Log.i(TAG, "The UUID of this service is " + gattService.getUuid().toString());
        }
        if (arduinoService == null) {
            Log.i(TAG, "Arduino service was not found");
            return;
        }
        List<BluetoothGattCharacteristic> gattCharacteristics = arduinoService.getCharacteristics();
        for (BluetoothGattCharacteristic gattCharacteristic : gattCharacteristics) {
            Log.i(TAG, "The UUID of this characteristic is " + gattCharacteristic.getUuid().toString());
            if (gattCharacteristic.getUuid().toString().equals(UPDOWNERROR_UUID)) {
                upDownErrorChar = gattCharacteristic;
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

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_dashboard);

        final Intent dashboardIntent = getIntent();
        arduinoName = dashboardIntent.getStringExtra(EXTRAS_ARDUINO_NAME);
        arduinoAddress = dashboardIntent.getStringExtra(EXTRAS_ARDUINO_ADDRESS);

        Intent gattServiceIntent = new Intent(this, BLEService.class);
        bindService(gattServiceIntent, mServiceConnection, Context.BIND_AUTO_CREATE);
        Log.i(TAG, "Service bound");
        Log.i(TAG, arduinoAddress);

        //mBluetoothLeService.connect(arduinoAddress);
    }


    @Override
    protected void onResume() {
        super.onResume();
        registerReceiver(mGattUpdateReceiver, makeGattUpdateIntentFilter());
        if (mBluetoothLeService != null) {
            final boolean result = mBluetoothLeService.connect(arduinoAddress);
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

    public void readData(View view) {
        if (mBluetoothLeService != null && upDownErrorChar != null) {
            mBluetoothLeService.readCharacteristic(upDownErrorChar);
        } else {
            Log.w(TAG, "Cannot read, no BLE connection or char not available!");
        }

    }

    public void disconnectDevice(View view) {
        if (mBluetoothLeService != null) {
            mBluetoothLeService.disconnect();
        } else {
            Log.w(TAG, "Cannot disconnect!");
        }
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