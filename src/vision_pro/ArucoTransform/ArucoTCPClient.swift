//
//  ArucoTCPClient.swift
//  ArucoTransform
//
//  TCP client for receiving drill poses from ROS server
//

import Foundation
import Network
import SwiftUI
import simd

class ArucoTCPClient: ObservableObject {
    private var connection: NWConnection?
    private let queue = DispatchQueue(label: "ArucoTCPClient")
    
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
        
        print("🔵 Attempting TCP connection to \(host):\(port)")
        
        connection = NWConnection(host: nwHost, port: nwPort, using: .tcp)
        
        connection?.stateUpdateHandler = { [weak self] newState in
            DispatchQueue.main.async {
                print("🔵 Connection state changed: \(newState)")
                switch newState {
                case .ready:
                    self?.statusMessage = "Connected to \(self?.host ?? ""):\(self?.port ?? 0)"
                    self?.statusColor = .green
                    self?.isConnected = true
                    print("✅ TCP connected to \(self?.host ?? ""):\(self?.port ?? 0)")
                case .failed(let error):
                    self?.statusMessage = "Failed: \(error.localizedDescription)"
                    self?.statusColor = .red
                    self?.isConnected = false
                    print("❌ TCP connection failed: \(error)")
                case .cancelled:
                    self?.statusMessage = "Connection cancelled"
                    self?.statusColor = .red
                    self?.isConnected = false
                    print("⚠️ TCP connection cancelled")
                case .waiting(let error):
                    self?.statusMessage = "Waiting..."
                    self?.statusColor = .orange
                    print("⏳ TCP connection waiting: \(error)")
                case .preparing:
                    self?.statusMessage = "Preparing..."
                    self?.statusColor = .orange
                    print("🔧 TCP connection preparing")
                case .setup:
                    self?.statusMessage = "Setting up..."
                    self?.statusColor = .orange
                    print("🔧 TCP connection setup")
                @unknown default:
                    print("❓ Unknown connection state: \(newState)")
                }
            }
        }
        
        connection?.start(queue: queue)
        receive()
    }
    
    func disconnect() {
        connection?.cancel()
        connection = nil
        DispatchQueue.main.async {
            self.statusMessage = "Disconnected"
            self.statusColor = .red
            self.isConnected = false
        }
    }
    
    private func receive() {
        connection?.receive(minimumIncompleteLength: 1, maximumLength: 8192) { [weak self] data, _, isComplete, error in
            if let data = data, !data.isEmpty {
                let message = String(decoding: data, as: UTF8.self)
                // 🆕 ISSUE #5: Add debug logging like working app
                print("📥 Received from server: \(message.trimmingCharacters(in: .whitespacesAndNewlines))")
                self?.parseMessage(message)
            }
            
            if let error = error {
                print("❌ Receive error: \(error)")
                DispatchQueue.main.async {
                    self?.statusMessage = "Error: \(error.localizedDescription)"
                    self?.statusColor = .red
                    self?.isConnected = false
                }
            }
            
            if isComplete {
                print("🔌 Connection closed by server")
                DispatchQueue.main.async {
                    self?.statusMessage = "Server closed connection"
                    self?.statusColor = .red
                    self?.isConnected = false
                }
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
                print("⚠️ Received non-POSES message: \(trimmed)")
                continue
            }
            
            let poseString = String(trimmed.dropFirst(6))  // Remove "POSES|"
            let poseStrings = poseString.split(separator: "|")
            
            print("🔍 Parsing \(poseStrings.count) pose(s)")
            
            var drillSites: [DrillSite] = []
            
            for poseStr in poseStrings {
                let components = poseStr.split(separator: ",").compactMap { Float($0) }
                guard components.count == 7 else {
                    print("⚠️ Invalid pose format (expected 7 components, got \(components.count)): \(poseStr)")
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
                print("📍 Successfully parsed \(drillSites.count) drill site(s) from ROS")
            }
        }
    }
    
    func send(_ message: String) {
        guard let connection = connection, connection.state == .ready else {
            print("⚠️ Cannot send: connection not ready")
            return
        }
        
        let data = Data(message.utf8)
        connection.send(content: data, completion: .contentProcessed { error in
            if let error = error {
                print("❌ Send error: \(error)")
            } else {
                print("✅ Sent: \(message.trimmingCharacters(in: .whitespacesAndNewlines))")
            }
        })
    }
}


