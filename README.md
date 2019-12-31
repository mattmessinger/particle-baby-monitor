# particle-baby-monitor
This is the companion source code for a [Particle](https://www.particle.io/)-based Baby Car Monitor blog that I posted [here](https://medium.com/@matthew.messinger/building-an-iot-car-baby-monitor-2412bdafe487?source=friends_link&sk=45e6806bcf4317e9c5a24120b5518fe5)

This code is written for the [Particle Boron LTE](https://store.particle.io/products/boron-lte).

The baby monitor uses an accelerometer to detect when the vehicle is in motion (and when it is not) and uses a noise detector to determine when the baby is crying. If the vehicle is not moving for a certain amount of time and a cry is detected then it will use a cellular modem to send a message to a server — which can then be used to notify a call tree.

The basic detection algorithm is fairly simple:
1. Use the accelerometer to determine that the vehicle is moving.
2. Once the vehicle has been stopped for T-stop seconds start monitoring for a cry sound. 
3. If there is a cry sound while monitoring then trigger alert.
4. After T-monitoring seconds stop monitoring.
