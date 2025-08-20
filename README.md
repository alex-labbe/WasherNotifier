# WasherNotifier
An STM32-based washer notifier that detects washing machine vibrations with the onboard accelerometer and sends a cloud-powered notification (email/push) when the load is done.

This project implements a **washing machine activity notifier** using the  
**STM32 B-L475E-IOT01A Discovery Kit**. By monitoring vibration patterns  
with the onboard **LSM6DSL accelerometer**, the device detects when a load  
is **in progress** and when it has **finished**. Once the machine is quiet  
for a defined period, the STM32 triggers a **notification** by sending a  
message to a remote server (AWS EC2) which then relays an **email or push  
notification** to the user.
