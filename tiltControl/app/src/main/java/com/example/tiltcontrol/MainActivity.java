package com.example.tiltcontrol;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.widget.TextView;

import com.anychart.AnyChart;
import com.anychart.AnyChartView;
import com.anychart.chart.common.dataentry.DataEntry;
import com.anychart.chart.common.dataentry.ValueDataEntry;

import com.anychart.charts.Scatter;
import com.anychart.core.scatter.series.Marker;
import com.anychart.enums.MarkerType;
import com.anychart.enums.TooltipDisplayMode;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;


public class MainActivity extends AppCompatActivity {

    private TextView mTextViewResult;
    private AnyChartView anyChartView;

    String baseUrl = "http://192.168.4.1";
    OkHttpClient client = new OkHttpClient();

    List<Double> angles = new ArrayList<>();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        anyChartView = (AnyChartView) findViewById(R.id.any_chart_view);

        Crosshairs myCross = new Crosshairs(anyChartView);
    }

    public class Crosshairs {
        private List<DataEntry> currentAngle = new ArrayList<>();
        Scatter scatter;
        AnyChartView myView;
        public void drawCrosshairs(AnyChartView myView){

        }
        public void changeCoordinate(double x, double y){
            currentAngle.remove(0);
            currentAngle.add(new ValueDataEntry(x,y));
        }
        public Crosshairs(AnyChartView View) {
            myView = View;
            currentAngle.add(new ValueDataEntry(0.0,0.0));
            scatter = AnyChart.scatter();
            scatter.animation(true);
            scatter.title("Zielen und Vorwärts");
            scatter.xScale()
                    .minimum(-1d)
                    .maximum(1d);
            scatter.yScale()
                    .minimum(-1d)
                    .maximum(1d);
            scatter.tooltip().displayMode(TooltipDisplayMode.UNION);
            Marker marker = scatter.marker(currentAngle);
            marker.type(MarkerType.CIRCLE)
                    .size(8d);
            myView.setChart(scatter);
            myView.bringToFront();
        }
    }

    void getTilt(final Crosshairs myCrosshairs){
        String url = baseUrl + "/getLocalTilt";
        Request request = new Request.Builder()
                .url(url)
                .build();
        client.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                e.printStackTrace();
            }

            @Override
            public void onResponse(Call call, Response response) throws IOException {
                if (response.isSuccessful()) {
                    final String myResponse = response.body().string();
                    String[] parts = myResponse.split("_");
                    angles.remove(1);
                    angles.remove(0);
                    angles.add(Double.parseDouble(parts[0]));
                    angles.add(Double.parseDouble(parts[1]));

                }
            }
        });
    }
    void setAngles(double x, double y){
        String url = baseUrl + "setAngles=" + Double.toString(x) + "_" + Double.toString(y);
        Request request = new Request.Builder()
                .url(url)
                .build();
        client.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                e.printStackTrace();
            }

            @Override
            public void onResponse(Call call, Response response) throws IOException {
                if (response.isSuccessful()) {
                    final String myResponse = response.body().string();
                }
            }
        });
    }
}