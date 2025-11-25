//
//  DrillPosesTCPClient.swift
//  SVD_ROS_Comms
//
//  TCP client for receiving drill poses from ROS server (Port 5001)
//

import Foundation
import Network
import SwiftUI
import simd

class DrillPosesTCPClient: ObservableObject {
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "DrillPosesTCPClient")
    
    private var host: String
    private var port: UInt16
    
    @Published var statusMessage: String = "Waiting for connection..."
    @Published var statusColor: Color = .gray
    @Published var isConnected: Bool = false
    
    var onDrillPosesReceived: (([DrillSite]) -> Void)?
    
    init(host: String, port: UInt16) {
        self.host = host
        self.port = port
    }
    
    func connect() {
        let nwHost = NWEndpoint.Host(host)
        let nwPort = NWEndpoint.Port(rawValue: port)!
        
        print("Attempting TCP connection to \(host):\(port) (Drill Poses)")
        
        connection = NWConnection(host: nwHost, port: nwPort, using: .tcp)
        
        connection?.stateUpdateHandler = { [weak self] newState in
            DispatchQueue.main.async {
                print("Drill Poses connection state changed: \(newState)")
                switch newState {
                case .ready:
                    self?.statusMessage = "Connected (Drill Poses)"
                    self?.statusColor = .green
                    self?.isConnected = true
                    print("TCP connected to \(self?.host ?? ""):\(self?.port ?? 0) (Drill Poses)")
                case .failed(let error):
                    self?.statusMessage = "Failed: \(error.localizedDescription)"
                    self?.statusColor = .red
                    self?.isConnected = false
                    print("TCP connection failed (Drill Poses): \(error)")
                    // Auto-reconnect after 2 seconds
                    self?.attemptReconnect()
                case .cancelled:
                    self?.statusMessage = "Connection cancelled"
                    self?.statusColor = .red
                    self?.isConnected = false
                    print("TCP connection cancelled (Drill Poses)")
                case .waiting(let error):
                    self?.statusMessage = "Waiting..."
                    self?.statusColor = .orange
                    print("TCP connection waiting (Drill Poses): \(error)")
                    // Auto-reconnect after 2 seconds
                    self?.attemptReconnect()
                case .preparing:
                    self?.statusMessage = "Preparing..."
                    self?.statusColor = .orange
                    print("TCP connection preparing (Drill Poses)")
                case .setup:
                    self?.statusMessage = "Setting up..."
                    self?.statusColor = .orange
                    print("TCP connection setup (Drill Poses)")
                @unknown default:
                    print("Unknown connection state (Drill Poses): \(newState)")
                }
            }
        }
        
        connection?.start(queue: queue)
        receive()
    }
    
    func disconnect() {
        print("Disconnecting from TCP server (Drill Poses)")
        connection?.cancel()
        connection = nil
        DispatchQueue.main.async {
            self.statusMessage = "Disconnected"
            self.statusColor = .red
            self.isConnected = false
        }
    }
    
    private func attemptReconnect() {
        DispatchQueue.main.asyncAfter(deadline: .now() + 2.0) { [weak self] in
            guard let self = self, !self.isConnected else { return }
            print("Attempting to reconnect (Drill Poses)...")
            self.connect()
        }
    }
    
    private func receive() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 8192) { [weak self] data, _, isComplete, error in
            if let data = data, !data.isEmpty {
                let message = String(decoding: data, as: UTF8.self)
                print("Received drill poses from server: \(message.prefix(100))...")
                self?.parseMessage(message)
            }
            
            if let error = error {
                print("Receive error (Drill Poses): \(error)")
                DispatchQueue.main.async {
                    self?.statusMessage = "Error: \(error.localizedDescription)"
                    self?.statusColor = .red
                    self?.isConnected = false
                    self?.attemptReconnect()
                }
            }
            
            if isComplete {
                print("Connection closed by server (Drill Poses)")
                DispatchQueue.main.async {
                    self?.statusMessage = "Disconnected"
                    self?.statusColor = .red
                    self?.isConnected = false
                    self?.attemptReconnect()
                }
                self?.connection?.cancel()
                self?.connection = nil
            } else if error == nil {
                // Continue receiving
                self?.receive()
            }
        }
    }
    
    private func parseMessage(_ message: String) {
        // Expected format: POSES|x,y,z,qx,qy,qz,qw|x,y,z,qx,qy,qz,qw|...
        let lines = message.split(separator: "\n")
        
        for line in lines {
            let trimmed = line.trimmingCharacters(in: .whitespacesAndNewlines)
            guard trimmed.hasPrefix("POSES|") else {
                print("Received non-POSES message: \(trimmed)")
                continue
            }
            
            let poseString = String(trimmed.dropFirst(6))  // Remove "POSES|"
            let poseStrings = poseString.split(separator: "|")
            
            print("Parsing \(poseStrings.count) pose(s)")
            
            var drillSites: [DrillSite] = []
            
            for poseStr in poseStrings {
                let components = poseStr.split(separator: ",").compactMap { Float($0) }
                guard components.count == 7 else {
                    print("Invalid pose format (expected 7 components, got \(components.count)): \(poseStr)")
                    continue
                }
                
                let position = SIMD3<Float>(components[0], components[1], components[2])
                let orientation = simd_quatf(
                    ix: components[3],
                    iy: components[4],
                    iz: components[5],
                    r: components[6]
                )
                
                drillSites.append(DrillSite(position: position, orientation: orientation))
            }
            
            if !drillSites.isEmpty {
                DispatchQueue.main.async { [weak self] in
                    self?.onDrillPosesReceived?(drillSites)
                }
                print("Successfully parsed \(drillSites.count) drill site(s) from ROS")
            }
        }
    }
    
    func send(_ message: String) {
        guard let connection = connection, connection.state == .ready else {
            print("Cannot send (Drill Poses): connection not ready")
            return
        }
        
        let data = Data(message.utf8)
        connection.send(content: data, completion: .contentProcessed { error in
            if let error = error {
                print("Send error (Drill Poses): \(error)")
            } else {
                print("Sent (Drill Poses): \(message.trimmingCharacters(in: .whitespacesAndNewlines))")
            }
        })
    }
}

