# embedded-systems

#### The goal of the final project was to create a prototype for a thermostat that interacts with a cloud server.

#### We simulated sending data to the server by just printing the thermomstat state (target temp, current temp, etc.) to the standatd output
#### We simulated heating and cooling by touching the sensor with hot/cold objects.

#### This implementation attempts to carefully organize the code by peripheral. There is a clean separation of GPIO, I2C, UART, and Timer functions.
#### This goal was to make the code readable and maintainable. While the whole program is small, we want to make it as easy as possible to extend functionality.
#### The comments are written so as to describe the intent of the developer without proving an exhaustive explanation of what the code "does".
#### These are principles that are appropriate for most projects, and the thermostat source code benefits from sticking to them.

#### We could improve the prototype by implementing "cooling". So far, the red LED flashes when the heat is ON, but there is no LED for air conditioning.

#### The TI CS-3220 documentation is very informative and the sample code covers a variety of use cases. These will be used to support future developent.

