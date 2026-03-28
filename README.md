# 🚀 Real-Time GPS Tracker with SOS & Call Monitoring

A compact and powerful **IoT-based GPS Tracker** built using ESP32, GSM, and GNSS modules.  
This device allows you to **track location in real-time**, send **SOS alerts**, and even **listen to surrounding audio remotely**.

---

## 📍 Features

- 🌍 Real-Time GPS Tracking (updates every 60 sec)
- 🚨 SOS Button (SMS + Auto Call)
- 📞 Auto Call Receive (Live Audio Monitoring)
- 📊 Trip Analytics (Distance, Speed, Route History)
- 🌐 Free IoT Dashboard Integration
- 🔧 Easy Configuration (No Coding Required for End Users)

---

## 🧰 Hardware Used

- XIAO ESP32 C3
- SIM800L GSM Module
- GP-02 GNSS (GPS) Module
- Rechargeable Battery
- Microphone Module
- Push Buttons (SOS + Config)
- Misc Components (Resistors, Capacitors, PCB, etc.)

---

## ⚙️ How It Works

1. GPS module fetches real-time location data
2. ESP32 processes and sends data via GSM (SIM800L)
3. Data is pushed to IoT platform for live tracking
4. SOS button triggers:
   - SMS with location
   - Auto call to emergency contacts
5. Incoming calls are auto-answered for live listening
