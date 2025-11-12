import socket
import time
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

HOST = '0.0.0.0'
PORT = 5000

class measure_latency():
    def __init__(self):
        self.command_list = []
        self.latency_list = []
        self.last_timestamp = None

    def get_data(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((HOST, PORT))
            s.listen()
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                while True:
                    data = conn.recv(1024)
                    if not data:
                        break
                    message = data.decode().strip()
                    if '|' in message:
                        command, timestamp = message.split('|')
                        sent_time = float(timestamp)
                        received_time = time.time()
                        latency_ms = (received_time - sent_time) * 1000
                        print(f"Command: {command}, Latency: {latency_ms:.2f} ms", flush=True)

                        self.command_list.append(command)
                        self.latency_list.append(latency_ms)

    def get_metrics(self):
        """Calculate and return latency statistics"""
        if not self.latency_list:
            return {}
            
        latencies = np.array(self.latency_list)
        return {
            'mean': np.mean(latencies),
            'median': np.median(latencies),
            'std_dev': np.std(latencies),
            'min': np.min(latencies),
            'max': np.max(latencies),
            'count': len(latencies)
        }

    def save_metrics(self, save_path='latency_metrics.npy', plot_path='latency_plot.png'):
        """Save latency data and visualization"""
        # Save raw latency data
        np.save(save_path, np.array(self.latency_list))
        
        # Create visualization
        plt.figure(figsize=(12, 6))
        
        # Time series plot
        plt.subplot(1, 2, 1)
        plt.plot(self.latency_list, marker='o', alpha=0.7)
        plt.xlabel('Command Sequence')
        plt.ylabel('Latency (ms)')
        plt.title('Latency Over Time')
        plt.grid(True)
        
        # Histogram
        plt.subplot(1, 2, 2)
        plt.hist(self.latency_list, bins=15, edgecolor='black', alpha=0.7)
        plt.xlabel('Latency (ms)')
        plt.ylabel('Frequency')
        plt.title('Latency Distribution')
        plt.grid(True)
        
        plt.tight_layout()
        plt.savefig(plot_path)
        plt.close()
        
        print(f"Metrics saved to {Path(save_path).resolve()}")
        print(f"Visualization saved to {Path(plot_path).resolve()}")


if __name__ == "__main__":
    ml = measure_latency()
    
    try:
        ml.get_data()  
    except KeyboardInterrupt:
        print("\nData collection interrupted")
    
    metrics = ml.get_metrics()
    if metrics:
        print("\nLatency Statistics:")
        print(f"  Commands processed: {metrics['count']}")
        print(f"  Average latency: {metrics['mean']:.2f} ms")
        print(f"  Median latency: {metrics['median']:.2f} ms")
        print(f"  Standard deviation: {metrics['std_dev']:.2f} ms")
        print(f"  Minimum latency: {metrics['min']:.2f} ms")
        print(f"  Maximum latency: {metrics['max']:.2f} ms")
    
    # Save results
    if ml.latency_list:
        ml.save_metrics()
