Command to debug run the RC Car ESP code with new changes ONELINER

mpremote connect COM4 fs cp main.py :main.py; `
mpremote connect COM4 fs cp jetson_communication.py :jetson_communication.py; `
mpremote connect COM4 fs cp motor_movement.py :motor_movement.py; `
mpremote connect COM4 run main.py


`mpremote connect COM4 run main.py` for just running the program