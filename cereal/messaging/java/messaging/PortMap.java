package messaging;
//java

import org.yaml.snakeyaml.Yaml;
import org.yaml.snakeyaml.constructor.Constructor;

import java.io.IOException;
import java.io.InputStream;
import java.util.Map;

public class PortMap{
    public Map<String, Service> services;

    public PortMap load(){
        Yaml yaml = new Yaml(new Constructor(PortMap.class));
        PortMap portmap;
        InputStream inputStream = PortMap.class.getResourceAsStream("/services.yaml");
        portmap = (PortMap) yaml.load(inputStream);
        try {
            inputStream.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        return portmap;
    }
}
