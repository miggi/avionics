package org.bavovnar.mqtt;

import com.google.gson.GsonBuilder;
import org.bavovnar.core.SensorData;
import org.bavovnar.core.intertial.Instruments;
import org.bavovnar.utils.InstantConverter;
import org.eclipse.paho.client.mqttv3.*;
import com.google.gson.Gson;
import java.time.Instant;
import java.util.UUID;

public class MQTTClient {
    private static final String MQTT_TOPIC = "horizon";
    private static final String SENSOR_DATA_TOPIC = "/sensor-data";
    private static final String MQTT_URL = "tcp://192.168.1.58:1883";
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
    public void sendMqtt(SensorData data) throws Exception {
        String altitude_topic =  MQTT_TOPIC + "/altitude";
        String temp_topic = MQTT_TOPIC + "/temperature";
        String pressure_topic = MQTT_TOPIC + "/pressure";
        String gyroX = MQTT_TOPIC + "/gx";
        String gyroY = MQTT_TOPIC + "/gy";
        String gyroZ = MQTT_TOPIC + "/gz";

        System.out.println("Sending to MQTT: " + data);

        client.publish(altitude_topic, newMessage(data.getAltitude()));
        client.publish(temp_topic, newMessage(data.getTemperature()));
        client.publish(pressure_topic, newMessage(data.getPressure()));
        client.publish(gyroX, newMessage(data.getGyroX()));
        client.publish(gyroY, newMessage(data.getGyroY()));
        client.publish(gyroZ, newMessage(data.getGyroZ()));
    }

    public void send(SensorData data) throws Exception {
        String payload = gson.toJson(data);
        MqttMessage message = new MqttMessage(payload.getBytes());

        client.publish(SENSOR_DATA_TOPIC, message);
    }
    public void send(Instruments data) throws Exception {
        String payload = gson.toJson(data);

        MqttMessage message = new MqttMessage(payload.getBytes());
        client.publish(SENSOR_DATA_TOPIC, message);
    }
    private static MqttMessage newMessage(Double payload) {
        String value = String.format("%.4f", payload);

        MqttMessage msg = new MqttMessage(value.getBytes());
        msg.setQos(0);
        msg.setRetained(true);
        return msg;
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
