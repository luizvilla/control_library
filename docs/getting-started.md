# Getting started

The control library is in the `src/` directory.
It has been written in c++. It has mainly been developped to be used with 
The [OwnTech Power API](https://github.com/owntech-foundation/Core) based on [Zephyr](https://www.zephyrproject.org/)
and integrated with [PlatformIO](https://platformio.org/).

Here we show step-by-step an example on how to use a `Pid()` assuming the installation 
step is done [link](link).


The use of the `Pid` is based on **4 steps**.

1. Pid object instanciation (declaration).
2. Pid initialisation.
3. Optional parameter update through getters and setters.
4. Pid execution.


## Pid object instanciation.

For each `controller` like (`Pid`, `Rst`, `Pr`) we have to define a parameter structure.


```c++
static float32_t kp = 0.5F;
static float32_t Ti = 7.5175e-5F;
static float32_t Td = 0.0F;
static float32_t N = 0.0F;
static float32_t upper_bound = 1.0F;
static float32_t lower_bound = 0.0F;
static float32_t Ts = 100.0e-6F;
```
```c++
static PidParams pid_params{Ts, kp, Ti, Td, N, lower_bound, upper_bound};
static Pid pid;
```
## Pid initialization.

Initialize the controller with the parameter structure.

```c++
pid.init(pid_params);
```

## Optional parameter update through getters and setters.

After initialization, the PID parameters and saturation bounds can be read and
updated individually.

```c++
pid.setKp(0.8F);
pid.setTi(1.0e-4F);
pid.setTd(0.0F);
pid.setN(0.0F);
pid.setTs(100.0e-6F);
pid.setLowerBound(0.0F);
pid.setUpperBound(1.0F);

float32_t kp_value = pid.getKp();
float32_t upper = pid.getUpperBound();
```

## Pid execution.

Once configured, update the measurement and the reference, then compute the
new command.

```c++
pid.setMeasurement(y);
pid.setReference(yref);
pid.calculate();

float32_t duty = pid.getOutput();
```
