package org.bavovnar.core;

import org.bavovnar.core.legacy.Data3f;
import org.bavovnar.navigation.Quaternion;
import org.tinylog.Logger;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.Scanner;
import java.util.stream.Collectors;

public class Utils {

    public static double roundThreeDigits(double d) {
        return Math.round(d * 1000) / 1000.0d;
    }

    public static double roundFourDigits(double d) {
        return Math.round(d * 10000) / 10000.0d;
    }

    public static float roundFourDigits(float d) {
        return Math.round(d * 10000) / 10000.0f;
    }

    public static float roundThreeDigits(float d) {
        return Math.round(d * 1000) / 1000.0f;
    }

    public static short roundThreeDigits(short d) {
        return (short) (Math.round(d * 1000) / 1000.0f);
    }

//    public static String toString(float[] arr) {
//        return Stream.of(arr)
//                .map(Object::toString)
//                .collect(Collectors.joining(","));
//    }

    public static String toString(float[] arr) {
       return Arrays.toString(arr).replaceAll("\\[|\\]", "").trim();
    }

    public static float[] fromString(String line) {
        Float[] wrappers = Arrays.stream(line.split(","))
                .map(Float::parseFloat)
                .collect(Collectors.toList())
                .toArray(Float[]::new);

        float[] result = new float[wrappers.length];
        for (int i = 0; i < wrappers.length; i++) {
            result[i] = wrappers[i];
        }

        return result;
    }

    public static float[] readFromFile(String prefix, String path) {
        float[] result = new float[]{0.0f, 0.0f, 0.0f};

        try (Scanner scanner = new Scanner(new File(path))) {
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                if (line.startsWith(prefix)) {
                    String[] tokens = line.split(prefix);
                    if (tokens.length > 1)
                        result = fromString(tokens[1]);
                }
            }
            return result;
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }

    public static Data3f roundToFourDigits(Data3f data) {
        return
                new Data3f(
                        roundFourDigits(data.getX()),
                        roundFourDigits(data.getY()),
                        roundFourDigits(data.getZ())
                );
    }

    public static Quaternion roundToFourDigits(Quaternion q) {
        return
                new Quaternion(
                        roundFourDigits(q.w),
                        roundFourDigits(q.x),
                        roundFourDigits(q.y),
                        roundFourDigits(q.z)
                );
    }

    public static void createDirectory(String directoryPath) {
        Path path = Paths.get(directoryPath);
        try {
            if (!Files.exists(path)) {
                Files.createDirectories(path);
                Logger.info("Initialised directory: " + path);
            }
        } catch (IOException e) {
            Logger.error("Failed to create directory. " + e.getMessage(), e);
        }
    }

    public static void createFile(String filePath, String data) {
        Path path = Paths.get(filePath);
        try {
            Files.write(path, data.getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            Logger.error("Failed to write to file: " + e.getMessage(), e);
        }
    }

}
