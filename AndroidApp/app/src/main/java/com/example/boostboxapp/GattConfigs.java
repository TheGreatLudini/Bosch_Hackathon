package com.example.boostboxapp;

/**
 * This class contains all addresses, uuids and names of BLE devices, services and characteristics
 * that are used in the app
 */

public class GattConfigs {
    public static final String ARDUINO_NAME = "Schraubenmaster4000"; // Common name of Arduino
    public static final String ARDUINO_ADDRESS = "4C:11:AE:C8:5A:B2"; // BLE address of Arduino

    public static final String ARDUINO_SERVICE_UUID = "aa461740-dc53-4624-97bd-0fee7b1212bb";
    public static final String UPDOWNERROR_CHAR_UUID = "a71f3fc6-f97c-4659-9ca2-76a67dede2e3";
    public static final String UPDOWNERROR_DESCRIPTOR_UUID = "00002902-0000-1000-8000-00805f9b34fb";
    public static final String LRANGLESET_CHAR_UUID = "951a4e8a-16a8-46a7-8962-0d5dc72881b5";
    public static final String UDANGLESET_CHAR_UUID = "05d47d9d-6295-491d-81ac-375395100e1e";


}
