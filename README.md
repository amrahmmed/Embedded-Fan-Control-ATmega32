# 🚀 Intelligent Fan Control System – ATmega32 + L293D

**Temperature-Based PWM Motor Drive with Embedded Safety Features**  
*A microcontroller-based embedded system simulating principles used in Siemens, Schneider, and El Sewedy applications.*

---

## 🔧 Key Features
- 🎯 **Precision PWM Motor Control** using L293D H-Bridge
- 🌡️ **LM35 Temperature Sensor** input via 10-bit ADC
- 🚨 **Overheat Protection**: Buzzer alarm triggered at >75°C
- 🛑 **Emergency Stop** using INT1 External Interrupt
- 🧠 **Real-Time Logic**: Dynamic fan speed adjustment based on sensor input

---

## Hardware Overview

| Component          | Purpose                             | Industry Relevance                |
|--------------------|-------------------------------------|-----------------------------------|
| **ATmega32**       | MCU: Reads, controls, reacts        | Embedded controllers in VFD/PLC   |
| **L293D**          | Motor driver (H-Bridge)             | Power electronics switching       |
| **LM35**           | Temperature sensor                  | Environmental feedback systems    |
| **NPN Transistor** | Alarm trigger switch                | Fail-safe switching mechanism     |
| **Buzzer**         | Overheat alarm                      | Safety signaling in automation    |

---

## System Behavior

- **<25°C** → Fan OFF  
- **25–75°C** → Speed increases linearly  
- **>75°C** → Fan MAX + Buzzer ON  
- **INT1 Button** → Toggle fan ON/OFF anytime (safe interrupt-driven logic)

---

## 📊 Performance Metrics

| Metric             | Value                  |
|--------------------|------------------------|
| ADC Resolution     | 10-bit (0.25°C/LSB)    |
| PWM Frequency      | ~7.8kHz (Fast PWM)     |
| ISR Response Time  | <10ms                  |
| System Latency     | <100ms                 |

---

## 🧪 Simulation

- ✅ Built in Proteus 8 Professional
- 🔁 Includes fan ramping, interrupt toggle, buzzer logic
- 📹 **https://drive.google.com/file/d/1vSwFpfCyjpoMp__uvldNZat6sVOvMLXD/view?usp=sharing**

---

## 🧠 Industrial Relevance

> This project simulates *real-world embedded motor control logic* used in:
- **Siemens Sinamics Drives** – PWM scaling, overheat response
- **Schneider Altivar VFDs** – Sensor-to-actuator loops
- **El Sewedy Smart Panels** – Efficient thermal management + failsafe triggers

---

## 💡 Potential Extensions

- MODBUS/RS485 integration
- LCD or UART serial feedback
- 3-phase BLDC adaptation
- PID speed control refinement

---

## 📁 Repository Structure

