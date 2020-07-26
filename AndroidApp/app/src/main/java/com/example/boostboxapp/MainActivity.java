package com.example.boostboxapp;

import androidx.appcompat.app.AppCompatActivity;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;

public class MainActivity extends AppCompatActivity {

    private final static String TAG = MainActivity.class.getSimpleName();

    private BluetoothAdapter bluetoothAdapter;

    private static final int REQUEST_ENABLE_BT = 1;
    private static final String ARDUINO_NAME = "Schraubenmaster4000"; // BLE name of the arduino
    private static final String ARDUINO_ADDRESS = "4C:11:AE:C8:5A:B2"; // BLE address of the arduino



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

    }

    public void connectArduino(View view) {
        final BluetoothManager bluetoothManager =
                (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = bluetoothManager.getAdapter();

        // Ensures Bluetooth is available on the device and it is enabled. If not,
        // displays a dialog requesting user permission to enable Bluetooth.
        if (bluetoothAdapter == null || !bluetoothAdapter.isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }
        Log.i(TAG, "Bluetooth enabled, starting Dashboard now.");
        final Intent dashboardIntent = new Intent(this, DashboardActivity.class);
        dashboardIntent.putExtra(DashboardActivity.EXTRAS_ARDUINO_NAME, ARDUINO_NAME);
        dashboardIntent.putExtra(DashboardActivity.EXTRAS_ARDUINO_ADDRESS, ARDUINO_ADDRESS);
        startActivity(dashboardIntent);


    }
}