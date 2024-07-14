package org.bavovnar.gateway;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import org.bavovnar.core.SensorData;
import org.bavovnar.utils.InstantConverter;
import org.eclipse.paho.client.mqttv3.*;
import org.tinylog.Logger;

import java.time.Instant;
import java.util.UUID;

public class MQTTClient {

    private static final String MQTT_TOPIC = "horizon";
    private static final String SENSOR_DATA_TOPIC = "/sensor-data";
//    private static final String MQTT_URL = "tcp://192.168.1.58:1883";  // home
    private static final String MQTT_URL = "tcp://192.168.1.47:1883"; // flat
    private IMqttClient client;
    private Gson gson;

    public MQTTClient() {
        try {
            this.client = initMqttClient();
            this.gson = new GsonBuilder()
                    .registerTypeAdapter(Instant.class, new InstantConverter())
                    .create();

        } catch (MqttException e) {
            throw new RuntimeException(e);
        }
    }

    public void send(SensorData data) throws Exception {
        String payload = gson.toJson(data);

        if (System.currentTimeMillis() % 100 == 0)
            Logger.info("Payload [ " + payload.getBytes().length + " ] > " + payload);

        MqttMessage message = new MqttMessage(payload.getBytes());
        client.publish(SENSOR_DATA_TOPIC, message);
    }

    private  IMqttClient initMqttClient() throws MqttException {

        String publisherId = UUID.randomUUID().toString();
        IMqttClient client = new MqttClient(MQTT_URL, publisherId);

        MqttConnectOptions options = new MqttConnectOptions();
        options.setAutomaticReconnect(true);
        options.setMaxInflight(100);
        options.setCleanSession(false);
        options.setConnectionTimeout(5);
        client.connect(options);

        return client;
    }
}
