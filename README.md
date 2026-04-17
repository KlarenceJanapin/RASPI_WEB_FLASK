# USV Control System

## Project Structure




### 1. On Laptop:
```bash
# Install dependencies
pip install -r requirements_laptop.txt

# Run the server
python laptop_server.py



# Install dependencies
pip install -r requirements_pi.txt

# Edit the laptop IP address in pi_telemetry_sender.py
nano pi_telemetry_sender.py
# Change LAPTOP_IP = "192.168.1.100" to your laptop's actual IP

# Run the sender
python pi_telemetry_sender.py

usv_project/
│
├── laptop_server.py          # Flask server (runs on laptop)
├── pi_telemetry_sender.py    # Telemetry sender (runs on Raspberry Pi)
├── requirements_laptop.txt   # Laptop dependencies
├── requirements_pi.txt       # Raspberry Pi dependencies
└── README.md                 # Setup instructions
