package com.example.tiltcontrol;

import androidx.appcompat.app.AppCompatActivity;

import android.graphics.BitmapFactory;
import android.graphics.drawable.Drawable;
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
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
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

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        anyChartView = (AnyChartView) findViewById(R.id.any_chart_view);
        mTextViewResult = findViewById(R.id.test_view_result);

        OkHttpClient client = new OkHttpClient();
        String url = "http://192.168.4.1/testValue"; //http://192.168.4.1/testValue
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
                    MainActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {

                            mTextViewResult.setText(myResponse);
                        }
                    });
                }
            }
        });

        Crosshairs myCross = new Crosshairs(anyChartView);

    }

    private class Crosshairs {
        private List<DataEntry> myData = new ArrayList<>();
        Scatter scatter;
        AnyChartView myView;
        public void drawCrosshairs(AnyChartView myView){

        }
        public void changeCoordinate(double x, double y){
            myData.remove(0);
            myData.add(new ValueDataEntry(x,y));
        }
        public Crosshairs(AnyChartView View) {
            myView = View;
            myData.add(new ValueDataEntry(0.0,0.0));
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
            Marker marker = scatter.marker(myData);
            marker.type(MarkerType.CIRCLE)
                    .size(8d);
            myView.circle(300, 150, 70);
            myView.setChart(scatter);
            myView.bringToFront();



        }

    }

    private class HttpHandler {

    }


}