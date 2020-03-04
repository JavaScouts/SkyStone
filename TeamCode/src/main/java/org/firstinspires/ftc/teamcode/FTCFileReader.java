package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.CharBuffer;
import java.util.HashMap;

public class FTCFileReader {

    private HashMap<String, Integer> map = new HashMap<String, Integer>();
    private File f;
    private java.io.FileReader fr;
    FTCFileReader(String fileName) {
        f = new File(Environment.getExternalStorageDirectory().getPath()+"/"+fileName);
        try {
            fr = new FileReader(f);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        map.put("STONE_1",1);
        map.put("STONE_2",2);
        map.put("STONE_3",3);
        map.put("STONE_4",4);
        map.put("STONE_5",5);
        map.put("STONE_6",6);
        map.put("WALL_TO_STONE",7);
        map.put("STRAFE_CORRECTION",8);
        map.put("DROP_CORRECTION",9);
        map.put("DROPOFF_LOCATION_1",10);
        map.put("DROPOFF_LOCATION_2",11);
        map.put("DROPOFF_LOCATION_3",12);

    }

    public String getKey(String key) {
        int line = map.get(key);
        try {
            fr = new FileReader(f);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        try {
            return getLine(line);
        } catch(IOException e) {
            e.printStackTrace();
            return "";
        }
    }

    public String getLine(int line_number) throws IOException {

        StringBuilder result = new StringBuilder();
        int c;
        int pos = 0;
        while((c=fr.read()) != -1 && pos < line_number) {
            if(c == 10) {
                Log.d("CUR STR",result.toString());
                pos++;
                if(pos != line_number) {
                    result = new StringBuilder();
                }
            }
            Log.d("CUR CHAR",String.valueOf((char)c));
            result.append((char) c);
        }
        return result.toString();
    }

    public String getPath() {
        return f.getAbsolutePath();
    }

}
