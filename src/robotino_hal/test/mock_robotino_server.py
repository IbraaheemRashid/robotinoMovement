#!/usr/bin/env python3

from flask import Flask, jsonify, request
import threading
import time
import math
from dataclasses import dataclass
from typing import List

@dataclass
class RobotinoState:
    voltage: float = 24.0
    charging: bool = False
    
    x: float = 0.0
    y: float = 0.0
    phi: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    
    distance_sensors: List[float] = None
    
    def __post_init__(self):
        if self.distance_sensors is None:
            self.distance_sensors = [200.0] * 9

class MockRobotinoServer:
    def __init__(self, host: str = 'localhost', port: int = 8081):
        self.app = Flask(__name__)
        self.host = host
        self.port = port
        self.state = RobotinoState()
        self.running = False
        
        self.setup_routes()
        self.update_thread = threading.Thread(target=self.update_state)
        self.update_thread.daemon = True
    
    def setup_routes(self):
        @self.app.route('/data/powermanagement', methods=['GET'])
        def get_power():
            return jsonify({
                'voltage': self.state.voltage / 350.0,
                'charging': self.state.charging
            })
        
        @self.app.route('/data/odometry', methods=['GET'])
        def get_odometry():
            return jsonify([
                self.state.x,
                self.state.y,
                self.state.phi,
                self.state.vx,
                self.state.vy,
                self.state.omega,
                int(time.time() * 1000)  # seq
            ])
        
        @self.app.route('/data/distancesensorarray', methods=['GET'])
        def get_sensors():
            return jsonify([int(x) for x in self.state.distance_sensors])
        
        @self.app.route('/data/omnidrive', methods=['POST'])
        def set_velocity():
            try:
                data = request.get_json(force=True)
                if isinstance(data, list) and len(data) == 3:
                    self.state.vx = float(data[0])
                    self.state.vy = float(data[1])
                    self.state.omega = float(data[2])
                    return jsonify({'status': 'ok'})
                return jsonify({'status': 'error', 'message': 'Invalid data format'}), 400
            except Exception as e:
                return jsonify({'status': 'error', 'message': str(e)}), 400

    def update_state(self):
        last_update = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_update
            
            self.state.x += self.state.vx * dt
            self.state.y += self.state.vy * dt
            self.state.phi += self.state.omega * dt
            
            self.state.phi = math.atan2(math.sin(self.state.phi), 
                                      math.cos(self.state.phi))
            
            if not self.state.charging:
                self.state.voltage -= 0.0001 * dt
                self.state.voltage = max(22.0, self.state.voltage)
            
            for i in range(len(self.state.distance_sensors)):
                noise = (math.sin(current_time * 2 + i) * 10)
                self.state.distance_sensors[i] = 200.0 + noise
            
            last_update = current_time
            time.sleep(0.01)

    def start(self):
        self.running = True
        self.update_thread.start()
        self.app.run(host=self.host, port=self.port)

    def stop(self):
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join()

if __name__ == '__main__':
    server = MockRobotinoServer()
    try:
        server.start()
    except KeyboardInterrupt:
        server.stop()