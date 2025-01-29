# Single pin version of the HCSR04 driver module

The HC_SR04 module (see the (elecfreak datasheet](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)) already has [a driver in Zephyr](https://docs.zephyrproject.org/latest/build/dts/api/bindings/sensor/hc-sr04.html).

There exist multiple variants of the HC_SR04 module, including the [Grove-Ultrasonic ranger V2.0](https://wiki.seeedstudio.com/Grove-Ultrasonic_Ranger/) that uses the same pin for both trigger and echo.

The Zephyr HC_SR04 driver can't be used as it is. The present module is base on the Zephyr HC_SR04 driver but reconfigures the trigger pin in input mode after the trigger signal has been sent.


## Usage

This is a west module.
You can integrate it in your application by adding the project to your west manifest:

```yaml
manifest:
  projects:
    - name: hcsr04-trig 
      url: https://github.com/ptournoux/hcsr04-trig.git
      revision: main
      path: modules/lib/hcsr04-trig
```

Enable the sensor by defining its location and properties in the devicetree (overlay).


```c
&gpioa {
    hcsr04trig {
        compatible = "hcsr04trig";
        label = "HCSR04TRIG_0";
        trigger-gpios = <&gpioa 5 GPIO_ACTIVE_HIGH>;  // TRIG: PA5 (D13)
        status = "okay";
    };
};
```

Example of reading sensor values:

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>


const struct device *const dev = DEVICE_DT_GET_ONE(hcsr04trig);


int main(void)
{

    if (dev == NULL) {
        printk("Error: HCSR04TRIG device not found\n");
        return -1;
    }

    if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return -90;
	}

    printk("HCSR04TRIG device found and ready\n");

    while (1) {

        struct sensor_value distance;
        int ret = sensor_sample_fetch(dev);
        if (ret < 0) {
            printk("Error: sensor_sample_fetch failed: %d\n", ret);
            continue;
        }

        ret = sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &distance);
        if (ret < 0) {
            printk("Error: sensor_channel_get failed: %d\n", ret);
            continue;
        }

        printk("Distance: %d.%d m\n", distance.val1,distance.val2);

	    k_sleep(K_MSEC(1000)); // Sample every 1 second
    }

}

```

## Code Sample

A sample application using this module is available [here](https://github.com/ptournoux/hcsr04-trig-sample.git).

## Writing Out-Of-Tree Drivers
Since this was kind of a pain to figure out, here are a few helpful links if you want to do this:
* https://github.com/teamspatzenhirn/pmw3389_zephyr_driver
* https://jdelaney.me/posts/zephyr-oot-modules/
* https://interrupt.memfault.com/blog/building-drivers-on-zephyr
* https://blog.golioth.io/adding-an-out-of-tree-sensor-driver-to-zephyr/
* https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/application_development/out_of_tree_driver